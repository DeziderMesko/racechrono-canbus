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
#include "../racechrono-canbus.hpp"
#include "../logging/logging.hpp"

#include <cstdio>

// core 3.x no longer pulls esp_mac.h in transitively
#include <esp_mac.h>

#include "device.hpp"

namespace racechrono
{

device& device::get() noexcept
{
    static device instance;
    return instance;
}

bool device::start(BLECharacteristicCallbacks* callbacks) noexcept
{
    bootln("Bluetooth LE starting...");

    char name[32];

    uint8_t mac[6];
    esp_err_t ret = esp_read_mac(mac, ESP_MAC_BT);

    if (ret == ESP_OK)
    {
        bootln("Bluetooth LE MAC: %02x:%02x:%02x:%02x:%02x:%02x",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        snprintf(name, sizeof(name), "RaceChrono %02X:%02X:%02X", mac[3], mac[4], mac[5]);
    }
    else
    {
        bootln("ERROR: Unable to determine bluetooth MAC address! reason: %d", ret);
        snprintf(name, sizeof(name), "RaceChrono DIY");
    }

    BLEDevice::init(name);
    BLEDevice::setPower(BLE_PWR_LVL);

    _server = BLEDevice::createServer();
    _server->setCallbacks(this);

    _service = _server->createService(racechrono_service_uuid);
    _pid_requests = _service->createCharacteristic(pid_characteristic_uuid, BLECharacteristic::PROPERTY_WRITE);
    _pid_requests->setCallbacks(callbacks);
    _canbus_frames = _service->createCharacteristic(can_bus_characteristic_uuid,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    // On NimBLE this is a no-op that logs a deprecation notice -- the host creates the
    // 0x2902 descriptor itself for any characteristic with NOTIFY. Kept because a
    // Bluedroid build still needs it.
    _canbus_frames->addDescriptor(&_2902_desc);
    // An observer on the frame path that does not change it: onStatus() is the only
    // place the outcome of a notify() is visible, and onSubscribe() the only place the
    // app's subscription is.
    _canbus_frames->setCallbacks(this);
    _service->start();

    BLEAdvertising* advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(_service->getUUID());
    advertising->setScanResponse(false);
    BLEDevice::startAdvertising();

    bootln("Bluetooth LE started!");

    return true;
}

void device::onConnect(BLEServer*)
{
    infoln("Bluetooth LE client connected!");
    _client_connected = true;
}

void device::onDisconnect(BLEServer*)
{
    // once connection is made, BLE stops advertising, so on disconnect, start advertising again..
    infoln("Bluetooth LE client disconnected!");
    _client_connected = false;
    _conn_interval.store(0U, std::memory_order_relaxed);
    _mtu.store(0U, std::memory_order_relaxed);
    _subscribed.store(false, std::memory_order_relaxed);
    BLEDevice::startAdvertising();
}

#if defined(CONFIG_NIMBLE_ENABLED)

void device::onConnect(BLEServer*, ble_gap_conn_desc* desc)
{
    _conn_interval.store(desc->conn_itvl, std::memory_order_relaxed);

    infoln("Bluetooth LE connected: interval %u (%.2f ms) latency %u timeout %u ms",
        desc->conn_itvl, desc->conn_itvl * 1.25f, desc->conn_latency,
        desc->supervision_timeout * 10U);
}

void device::onMtuChanged(BLEServer*, ble_gap_conn_desc*, uint16_t mtu)
{
    _mtu.store(mtu, std::memory_order_relaxed);
    infoln("Bluetooth LE MTU %u", mtu);
}

void device::onConnParamsUpdate(uint16_t, uint16_t interval, uint16_t latency,
    uint16_t timeout, uint8_t status)
{
    if (status == 0)
    {
        _conn_interval.store(interval, std::memory_order_relaxed);
    }

    // interval is in 1.25 ms units, timeout in 10 ms units. The interval is the number
    // that decides everything downstream: the radio only speaks at connection events,
    // so frames per second has a hard ceiling of a few per interval whatever the bus is
    // doing.
    infoln("Bluetooth LE conn interval %u (%.2f ms) latency %u timeout %u ms status %u",
        interval, interval * 1.25f, latency, timeout * 10U, status);
}

void device::onSubscribe(BLECharacteristic*, ble_gap_conn_desc*, uint16_t sub)
{
    // bit 0 is notifications, bit 1 indications; RaceChrono asks for the former
    bool on = (sub & 0x0001) != 0;
    _subscribed.store(on, std::memory_order_relaxed);
    infoln("Bluetooth LE subscribe 0x%04x -> notifications %s", sub, on ? "on" : "off");
}

#endif // CONFIG_NIMBLE_ENABLED

void device::onStatus(BLECharacteristic*, Status s, uint32_t code)
{
    switch (s)
    {
    case SUCCESS_NOTIFY:
        _ble_count.fetch_add(1, std::memory_order_relaxed);
        break;

    case ERROR_NOTIFY_DISABLED:
    case ERROR_NO_SUBSCRIBER:
    case ERROR_NO_CLIENT:
        // Not congestion: there was nowhere to send it. Counted apart from lost frames
        // because the fix is on the phone, not on the board.
        _ble_nosub.fetch_add(1, std::memory_order_relaxed);
        break;

    default:
        // ERROR_GATT, and anything the library adds later. ble_gatts_notify_custom()
        // refuses when the host runs out of mbufs, which is what a link too slow for the
        // bus looks like from here.
        _ble_lost.fetch_add(1, std::memory_order_relaxed);
        _ble_err.store(code, std::memory_order_relaxed);
        break;
    }
}

#if defined(DEBUG)
void device::stats() noexcept
{
    if (logging::logger::get().level() >= logging::log_level::info)
    {
        unsigned long delta = _stats_timer.elapsed(CONFIG_RC_STATS_TIMEOUT);

        if (delta > 0UL)
        {
            uint32_t total = _ble_count.load(std::memory_order_relaxed);
            uint32_t count = total - exchange(_ble_last, total);
            float msg_per_sec = (static_cast<float>(count) / static_cast<float>(delta)) * 1e6f;
            infoln(" Bluetooth LE msg/s: %.2f", msg_per_sec);
            infoln(" Bluetooth LE sent: %lu lost: %lu nosub: %lu err: %ld",
                (unsigned long)total,
                (unsigned long)_ble_lost.load(std::memory_order_relaxed),
                (unsigned long)_ble_nosub.load(std::memory_order_relaxed),
                (long)_ble_err.load(std::memory_order_relaxed));
            infoln(" Bluetooth LE link: %s sub %s interval %.2f ms mtu %u",
                connected() ? "up" : "down",
                subscribed() ? "yes" : "no",
                _conn_interval.load(std::memory_order_relaxed) * 1.25f,
                _mtu.load(std::memory_order_relaxed));
        }
    }
}
#endif

} // namespace racechrono

racechrono::device& RCDEV = racechrono::device::get();
