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

#include <atomic>

#include "decoder.hpp"

namespace canbus
{

/**
 * decoder for the Yamaha R9 (2026).
 *
 * There is nothing to decode yet: no CAN ID on this bike is confirmed, so this
 * decoder deliberately has no ID table and no allow-list. Every standard frame the
 * controller receives is forwarded raw to RaceChrono, which decodes channels
 * app-side from CAN ID + bit offset + length + equation.
 *
 * That is affordable: the R1-family bus carries roughly sixteen arbitration IDs at
 * about 1050 msg/s in total, well inside what the ESP32-S3 and the BLE link handle -
 * the bench measured 3.2 M frames over an hour at ~850/s with none lost.
 *
 * RaceChrono still drives the stream over its allow/deny protocol, and that is
 * honoured coarsely: a deny-all stops the forwarding, any allow request - whether
 * for one ID or for all of them - starts it again. Sending the app a few IDs it did
 * not ask for costs nothing, since it ignores IDs it has no channel for, and it is
 * what makes the device usable as a sniffer before the mapping table has a single
 * confirmed row.
 *
 * Once ../canbus-mapping.md has confirmed IDs, a real table with per-ID rates
 * belongs here, modelled on decoder_bmwg8x.
 */
class decoder_yamahar9
    : public decoder
{
    CPP_NOCOPY(decoder_yamahar9);
    CPP_NOMOVE(decoder_yamahar9);

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
        // no hardware filter: which IDs matter is exactly what is unknown
        return TWAI_FILTER_CONFIG_ACCEPT_ALL();
    }

    /// called from the CAN ISR for every received frame
    bool should_decode(uint32_t) noexcept override
    {
        return _forward.load(std::memory_order_relaxed);
    }

protected:
    uint16_t rate(uint32_t) const noexcept override
    {
        return rate_default;
    }

    void deny_all() noexcept override
    {
        _forward.store(false, std::memory_order_relaxed);
    }

    void allow_all() noexcept override
    {
        _forward.store(true, std::memory_order_relaxed);
    }

    void allow_id(uint32_t) noexcept override
    {
        // no allow-list: any request from the app opens the whole bus
        _forward.store(true, std::memory_order_relaxed);
    }

private:
    explicit decoder_yamahar9() noexcept
        : decoder(0)
        , _forward(true)
    {
    }

    /// written by the BLE task, read by the CAN ISR
    std::atomic<bool> _forward;
};

} // namespace canbus

#if defined(CONFIG_CANBUS_DECODER_YAMAHAR9)
canbus::decoder& CANDEC = canbus::decoder_yamahar9::get();
#endif
