// MIT License
//
// Copyright (c) 2022 Joe Roback <joe.roback@gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once

#include "../racechrono-canbus.hpp"
#include "../utils/timer.hpp"

#include <atomic>

#include <BLE2902.h>
#include <BLECharacteristic.h>
#include <BLEDevice.h>

#if defined(CONFIG_NIMBLE_ENABLED)
#include <host/ble_gatt.h>
#include <host/ble_hs.h>
#include <host/ble_hs_mbuf.h>
#endif

namespace racechrono
{

/**
 * RaceChrono Bluetooth LE device.
 */
class device final
    : public BLEServerCallbacks
    , public BLECharacteristicCallbacks
{
    CPP_NOCOPY(device);
    CPP_NOMOVE(device);

public:
    ~device() noexcept override = default;

    /**
     * Get instance (singleton)
     */
    static device& get() noexcept;

    /**
     * @return true is connected to RaceChrono app
     */
    __always_inline bool connected() const noexcept
    {
        // Null-checked: the display task reads this from the moment it starts, which
        // is before start() has created the server.
        return _server != nullptr && _server->getConnectedCount() > 0;
    }

    /**
     * @return frames handed to the BLE stack since boot, one per frame off the queue.
     * This is the number comparable to the controller's forwarded count; the two
     * should track exactly. Cumulative, so a caller derives a rate from its own
     * previous sample -- see canbus::controller::counters_t.
     */
    __always_inline uint32_t frames() const noexcept
    {
        return _ble_offered.load(std::memory_order_relaxed);
    }

    /**
     * @return notifications the controller actually put on the air, cumulative,
     * counted from the BLE_GAP_EVENT_NOTIFY_TX dispatch into onStatus().
     *
     * So the healthy reading is **equal to frames()**, and the shortfall between the
     * two is the diagnostic: frames offered to the host that never reached the air.
     *
     * It read twice frames() while the frame path still went through
     * BLECharacteristic::notify(), which reports a second success synchronously at the
     * end of the call to mean the host accepted the frame into its queue. The two are
     * indistinguishable from inside onStatus() -- both report SUCCESS_NOTIFY with code
     * 0 -- so going straight at the host left only the event that means something. A
     * build with RC_WRAPPER_NOTIFY defined reads 2:1 again.
     */
    __always_inline uint32_t notifies() const noexcept
    {
        return _ble_count.load(std::memory_order_relaxed);
    }

    /**
     * @return frames the BLE stack refused, cumulative. notify() returns void and
     * logs at a level a release build does not print, so without this a congested
     * link loses frames exactly as silently as the 8-entry queue used to.
     */
    __always_inline uint32_t lost() const noexcept
    {
        return _ble_lost.load(std::memory_order_relaxed);
    }

    /**
     * @return frames dropped because nobody was subscribed, cumulative. Distinct from
     * lost(): this is not congestion, it is a phone that is connected and listening to
     * nothing, which is the failure that looks like success on every other indicator.
     */
    __always_inline uint32_t unsubscribed() const noexcept
    {
        return _ble_nosub.load(std::memory_order_relaxed);
    }

    /**
     * @return the esp_err_t behind the most recent refusal, 0 if there has never been one
     */
    __always_inline uint32_t last_error() const noexcept
    {
        return _ble_err.load(std::memory_order_relaxed);
    }

    /**
     * @return true once the client has subscribed to the frame characteristic.
     * Connecting and subscribing are separate steps and only the first one is visible
     * on the phone, so they are separate questions here too.
     */
    __always_inline bool subscribed() const noexcept
    {
#if defined(CONFIG_NIMBLE_ENABLED)
        return _subscribed.load(std::memory_order_relaxed);
#else
        // Bluedroid raises no onSubscribe(), and the 0x2902 descriptor cannot answer
        // it either: the constructor pre-arms it so a client that never writes the
        // descriptor still gets frames. Fall back to the weaker question.
        return connected();
#endif
    }

    /**
     * @return negotiated connection interval in units of 1.25 ms, 0 until the link
     * reports one. This is the ceiling on frames per second: the radio only talks at
     * connection events, so a phone that settles on 30 ms caps the stream regardless
     * of how fast the bus is.
     */
    __always_inline uint16_t interval() const noexcept
    {
        return _conn_interval.load(std::memory_order_relaxed);
    }

    /**
     * @return how many clients are connected. Should be 1 whenever anything is: the
     * frame path notifies once per subscriber, so a second connection silently doubles
     * the BLE traffic for the same bus -- half the link's capacity spent sending every
     * frame twice, with every other counter looking healthy.
     */
    __always_inline uint16_t peers() const noexcept
    {
        return _server != nullptr ? static_cast<uint16_t>(_server->getConnectedCount()) : 0U;
    }

    /**
     * @return negotiated ATT MTU, 0 until the phone negotiates one. A frame is at most
     * 12 bytes, so the 23-byte default is already enough; it is here because a
     * truncating link would otherwise corrupt payloads without saying so.
     */
    __always_inline uint16_t mtu() const noexcept
    {
        return _mtu.load(std::memory_order_relaxed);
    }

    /**
     * @return true is Bluetooth LE stack is started
     */
    __always_inline bool started() const noexcept
    {
        return _server != nullptr;
    }

    /**
     * Start Bluetooth LE stack.
     * @return true is sucessful; otherwise false.
     */
    bool start(BLECharacteristicCallbacks* callbacks) noexcept;

    /**
     * Send \p data of size \p len over Bluetooth LE stack as an LE notification.
     *
     * On NimBLE this goes straight at the host and past BLECharacteristic::notify(),
     * which costs two String assignments and a semaphore for every frame: setValue()
     * copies the bytes into a String under a mutex, notify() copies that String again
     * into a local one, and only then builds the mbuf the host actually wanted. Three
     * heap allocations to move twelve bytes that were already flat in memory, on a path
     * that runs a thousand times a second.
     *
     * The wrapper is still what defines the characteristic, tracks the subscription and
     * delivers onStatus() -- only the per-frame call is bypassed.
     */
    __always_inline void send(uint8_t* data, size_t len) noexcept
    {
        if (!_client_connected)
        {
            return;
        }

        _ble_offered.fetch_add(1, std::memory_order_relaxed);

#if defined(CONFIG_NIMBLE_ENABLED) && !defined(RC_WRAPPER_NOTIFY)
        if (RCLIKELY(_frames_handle != null_handle))
        {
            uint16_t conn = _conn_handle.load(std::memory_order_relaxed);

            if (conn == BLE_HS_CONN_HANDLE_NONE || !_subscribed.load(std::memory_order_relaxed))
            {
                // Nowhere to send it: counted apart from congestion, because the fix is
                // on the phone rather than on the board.
                _ble_nosub.fetch_add(1, std::memory_order_relaxed);
                return;
            }

            os_mbuf* om = ble_hs_mbuf_from_flat(data, len);

            if (om == nullptr)
            {
                // Out of mbufs before the host was even asked. Same exhaustion as a
                // refused notification, one step earlier, so it is counted the same.
                _ble_lost.fetch_add(1, std::memory_order_relaxed);
                _ble_err.store(BLE_HS_ENOMEM, std::memory_order_relaxed);
                return;
            }

            // Consumes the mbuf whatever the outcome, so there is nothing to free here.
            int rc = ble_gatts_notify_custom(conn, _frames_handle, om);

            if (RCUNLIKELY(rc != 0))
            {
                _ble_lost.fetch_add(1, std::memory_order_relaxed);
                _ble_err.store(static_cast<uint32_t>(rc), std::memory_order_relaxed);
            }

            // Success is not counted here. BLE_GAP_EVENT_NOTIFY_TX still arrives -- the
            // server dispatches it by attribute handle, not by who called the host --
            // so onStatus() counts what actually reached the air, which is the number
            // worth having.
            return;
        }
#endif

        // Bluedroid, or a NimBLE build where the characteristic handle never turned up
        // -- nothing has subscribed yet, in practice.
        _canbus_frames->setValue(data, len);
        // The outcome is counted in onStatus(), not here: notify() returns void,
        // and offering a frame to a congested stack is not the same as sending it.
        _canbus_frames->notify();
    }

    /**
     * BLE callback for when a client (RaceChrono app) connects
     */
    void onConnect(BLEServer*) override;

    /**
     * BLE callback for when a client (RaceChrono app) disconnects
     */
    void onDisconnect(BLEServer*) override;

    // The board builds against NimBLE, not Bluedroid: core 3.3.11 ships
    // CONFIG_NIMBLE_ENABLED for the ESP32-S3 and the Arduino BLE classes are a thin
    // wrapper over whichever host is configured. The two hosts hand these callbacks
    // different arguments, so the declarations have to follow.
#if defined(CONFIG_NIMBLE_ENABLED)

    /**
     * BLE callback carrying the connection descriptor, which is where the initial
     * connection interval and the handle a parameter request needs both live.
     */
    void onConnect(BLEServer*, ble_gap_conn_desc* desc) override;

    /**
     * BLE callback for the negotiated ATT MTU.
     */
    void onMtuChanged(BLEServer*, ble_gap_conn_desc* desc, uint16_t mtu) override;

    /**
     * BLE callback for the negotiated connection parameters. The phone has the final
     * say on these, so being told is the only way to know the link's ceiling.
     */
    void onConnParamsUpdate(uint16_t conn_handle, uint16_t interval, uint16_t latency,
        uint16_t timeout, uint8_t status) override;

    /**
     * BLE callback for the client subscribing to, or unsubscribing from, the frame
     * characteristic. NimBLE owns the 0x2902 descriptor itself and refuses to notify
     * anything until this has fired, which makes it the fact worth tracking.
     */
    void onSubscribe(BLECharacteristic*, ble_gap_conn_desc* desc, uint16_t sub) override;

#endif

    /**
     * BLE callback for the outcome of every notify(). The one place a refused frame
     * is visible: notify() itself returns void.
     */
    void onStatus(BLECharacteristic*, Status s, uint32_t code) override;

    /**
     * print any bluetooth stats
     */
#if defined(DEBUG)
    void stats() noexcept;
#else
    void stats() noexcept {}
#endif

private:
    // What the Arduino BLE wrapper uses for "this handle is not known yet". It is not
    // exported, and it is the value getHandle() returns before NimBLE has registered
    // the GATT table -- which is a live trap, because the host will transmit a
    // notification addressed to it perfectly happily and no phone will ever see one.
    static constexpr uint16_t null_handle = 0xffffU;

    // RaceChrono BLE service UUID
    static constexpr uint16_t racechrono_service_uuid = 0x1ff8;

    // RaceChrono uses two BLE characteristics:
    // 0x01 to be notified of data received for those PIDs
    // 0x02 to request which PIDs to send and how frequently
    static constexpr uint16_t can_bus_characteristic_uuid = 0x1;
    static constexpr uint16_t pid_characteristic_uuid = 0x2;

    explicit device() noexcept
        : _server(nullptr)
        , _service(nullptr)
        , _pid_requests(nullptr)
        , _canbus_frames(nullptr)
        , _2902_desc{}
        , _frames_handle(null_handle)
        , _conn_handle(BLE_HS_CONN_HANDLE_NONE)
        , _client_connected(false)
        , _stats_timer{}
        , _ble_offered(0U)
        , _ble_count(0U)
        , _ble_lost(0U)
        , _ble_nosub(0U)
        , _ble_err(0U)
        , _conn_interval(0U)
        , _mtu(0U)
        , _subscribed(false)
        , _ble_last(0U)
    {
        _2902_desc.setNotifications(true);
    }

private:
    BLEServer* _server;
    BLEService* _service;
    BLECharacteristic* _pid_requests;
    BLECharacteristic* _canbus_frames;
    BLE2902 _2902_desc;
    /// attribute handle of the frame characteristic, for the direct notify path.
    /// Resolved in onSubscribe() rather than at creation: NimBLE fills the wrapper's
    /// copy in while it registers the GATT table, so reading it any earlier returns
    /// null_handle. null_handle here means send() falls back to the wrapper.
    uint16_t _frames_handle;
    /// the peer's connection handle, from onConnect(); BLE_HS_CONN_HANDLE_NONE when
    /// nothing is connected. One peer, deliberately -- see peers().
    std::atomic<uint16_t> _conn_handle;
    bool _client_connected;
    utils::timer _stats_timer;
    /// cumulative: frames handed to the stack, one per frame off the queue
    std::atomic<uint32_t> _ble_offered;
    /// cumulative: notifications the controller actually transmitted, counted from
    /// BLE_GAP_EVENT_NOTIFY_TX. Reads 1:1 against offered on a healthy link, and the
    /// shortfall is what never made it onto the air
    std::atomic<uint32_t> _ble_count;
    /// cumulative: notifications the stack refused, congestion being the usual reason
    std::atomic<uint32_t> _ble_lost;
    /// cumulative: notifications dropped because nothing was subscribed
    std::atomic<uint32_t> _ble_nosub;
    /// the esp_err_t behind the most recent refusal
    std::atomic<uint32_t> _ble_err;
    /// negotiated connection interval, in units of 1.25 ms
    std::atomic<uint16_t> _conn_interval;
    /// negotiated ATT MTU
    std::atomic<uint16_t> _mtu;
    /// set from onSubscribe(): the app has enabled notifications on the frame char
    std::atomic<bool> _subscribed;
    /// what stats() saw last time, so it can print a rate without zeroing the counter
    uint32_t _ble_last;
};

} // namespace racechrono

extern racechrono::device& RCDEV;
