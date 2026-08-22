// MIT License
//
// Copyright (c) 2026 Dezider Mesko <dezo.mesko@pm.me>
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

#include <atomic>

#include <esp_timer.h>

#include "decoder.hpp"

namespace canbus
{

/**
 * decoder for the Yamaha R9 (2026).
 *
 * No CAN id on this bike is confirmed, so unlike decoder_bmwg8x this decoder has no
 * static id table and no per-id rates of its own. What it has instead is an allow-list
 * filled in at run time from RaceChrono's own requests: the app sends a deny-all and
 * then one allow per id it has a channel for, each with the notify interval it wants
 * that id at. Both are honoured here.
 *
 * Honouring them is what makes the BLE link's ceiling stop mattering. The link starts
 * refusing frames somewhere around 590 msg/s against the R9's ~1050 (see
 * motocan/bluecan/README.md), but a handful of allowed ids at the intervals a phone
 * asks for is tens of messages a second, not a thousand. The ids the app never asked
 * for are dropped in the CAN ISR, before they cost a queue slot or an mbuf.
 *
 * Two things are deliberately *not* done:
 *
 * - **Until an app says otherwise, everything is forwarded.** That is the boot state,
 *   it is what makes the board usable as a bench sniffer with no phone in the room,
 *   and it is what every measurement in bluecan/ was taken against. A deny-by-default
 *   would have quietly changed what those tools measure.
 *
 * - **An allow-all request is not rate-limited.** There is no per-id slot to hold a
 *   timestamp for an id nobody has named, and learning ids inside the ISR would mean
 *   writing this table from two contexts at once. Allow-all is the sniffer case
 *   anyway; the interval it carries is read, logged and then ignored.
 *
 * Once motocan/canbus-mapping.md has confirmed ids, a static table with measured
 * per-id rates could live here as well, modelled on decoder_bmwg8x -- but the app's
 * request would still be the thing that decides what crosses the radio.
 */
class decoder_yamahar9
    : public decoder
{
    CPP_NOCOPY(decoder_yamahar9);
    CPP_NOMOVE(decoder_yamahar9);

    /// One allowed id. RaceChrono asks for the ids it has channels for, so this is
    /// sized off the bus rather than off the protocol: the R1-family bus carries about
    /// sixteen arbitration ids in total (motocan/canbus-mapping.md), and a phone
    /// cannot usefully ask for more ids than the bike broadcasts.
    struct slot
    {
        uint32_t id;           ///< arbitration id, written by the BLE task
        uint32_t interval_us;  ///< 0 means every frame, written by the BLE task
        int64_t  last_us;      ///< last frame forwarded, written by the ISR only
    };

    static constexpr size_t max_slots = 32;

public:
    static decoder_yamahar9& get() noexcept
    {
        static decoder_yamahar9 instance;
        return instance;
    }

    ~decoder_yamahar9() noexcept override
    {
    }

    twai_timing_config_t timing() const noexcept override
    {
        return TWAI_TIMING_CONFIG_500KBITS();
    }

    twai_filter_config_t filter() const noexcept override
    {
        // No hardware filter. It is configured once in controller::install(), long
        // before the app connects and says what it wants, and its code/mask pair
        // cannot express an arbitrary set of ids in any case. The allow-list below
        // does the job a few instructions later, in the same ISR.
        return TWAI_FILTER_CONFIG_ACCEPT_ALL();
    }

    /**
     * called from the CAN ISR for every standard frame received.
     *
     * No allocation and no blocking lock: a linear scan of at most max_slots aligned
     * comparisons, and at most one systimer read. esp_timer_get_time() is safe here --
     * it is in IRAM and reads a counter register.
     */
    bool should_decode(uint32_t id) noexcept override
    {
        if (_allow_all.load(std::memory_order_relaxed))
        {
            return true;
        }

        // acquire against the release store in allow_id(): a slot is only visible
        // once every one of its fields has been written.
        const size_t count = _count.load(std::memory_order_acquire);

        for (size_t i = 0; i < count; i++)
        {
            slot& s = _slots[i];

            if (s.id != id)
            {
                continue;
            }

            if (s.interval_us == 0U)
            {
                return true;
            }

            const int64_t now = esp_timer_get_time();

            // >= rather than >, so an interval the bus happens to land exactly on is
            // forwarded rather than held back a whole period.
            if (now - s.last_us >= static_cast<int64_t>(s.interval_us))
            {
                s.last_us = now;
                return true;
            }

            return false;
        }

        return false;
    }

    int filter_size() const noexcept override
    {
        if (_allow_all.load(std::memory_order_relaxed))
        {
            return -1;
        }

        return static_cast<int>(_count.load(std::memory_order_relaxed));
    }

    uint32_t filter_overflow() const noexcept override
    {
        return _overflow.load(std::memory_order_relaxed);
    }

protected:
    uint16_t rate(uint32_t) const noexcept override
    {
        // the base class's frame-count divider is unused here: this decoder gates on
        // elapsed time instead, because the app states its wish in milliseconds and
        // the native rate of an id on this bike is exactly what is not known yet.
        return rate_default;
    }

    void deny_all() noexcept override
    {
        _allow_all.store(false, std::memory_order_relaxed);
        // The overflow count describes the list that is being thrown away, so it goes
        // with it. Left standing it would mark every later session -- the panel drawing
        // "flt 3!" in red and the fault latch re-arming every second -- for a channel
        // that fits and is not being dropped. The fault itself stays latched from the
        // moment it was first seen, which is the part that is meant to survive.
        _overflow.store(0U, std::memory_order_relaxed);
        // Emptying the list is one release store. A concurrent ISR either sees the old
        // count and forwards one more frame, or sees zero and forwards none; both are
        // correct answers to a deny that arrived mid-frame.
        _count.store(0U, std::memory_order_release);
    }

    void allow_all(uint16_t interval_ms) noexcept override
    {
        // read and logged by the caller, then ignored on purpose -- see the class
        // comment. There is nowhere to keep a timestamp for an id nobody has named.
        (void) interval_ms;
        _allow_all.store(true, std::memory_order_relaxed);
    }

    void allow_id(uint32_t id, uint16_t interval_ms) noexcept override
    {
        const uint32_t interval_us = static_cast<uint32_t>(interval_ms) * 1000U;
        const size_t count = _count.load(std::memory_order_relaxed);

        // already listed: update the interval in place. The ISR reading the old value
        // for one more frame is not worth ordering against.
        for (size_t i = 0; i < count; i++)
        {
            if (_slots[i].id == id)
            {
                _slots[i].interval_us = interval_us;
                return;
            }
        }

        if (count >= max_slots)
        {
            // Counted, not just logged. A release build has no console, and an id the
            // rider defined a channel for that silently never arrives is the worst
            // possible way for this to fail.
            _overflow.fetch_add(1, std::memory_order_relaxed);
            warnln("ID request ALLOW ID 0x%03x DROPPED, allow-list full (%u)", id, max_slots);
            return;
        }

        _slots[count].id = id;
        _slots[count].interval_us = interval_us;
        _slots[count].last_us = 0;

        // Publish last, and with release ordering: the ISR must never see a slot whose
        // id is set but whose interval still holds whatever was there before.
        _count.store(count + 1U, std::memory_order_release);
    }

private:
    explicit decoder_yamahar9() noexcept
        : decoder(0)
        , _slots{}
        , _count(0U)
        , _allow_all(true)
        , _overflow(0U)
    {
    }

    /// written by the BLE task, read by the CAN ISR
    slot _slots[max_slots];
    std::atomic<size_t> _count;
    std::atomic<bool> _allow_all;
    std::atomic<uint32_t> _overflow;
};

} // namespace canbus

#if defined(CONFIG_CANBUS_DECODER_YAMAHAR9)
canbus::decoder& CANDEC = canbus::decoder_yamahar9::get();
#endif
