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
#pragma once

#include "../racechrono-canbus.hpp"
#include "../canbus/frame.hpp"
#include "../utils/timer.hpp"

#include <freertos/task.h>

namespace ui
{

/**
 * The on-board TFT. Singleton, like every other device here.
 *
 * Two hard rules, both bought with bench measurements (see motocan/bluecan/bench-link.md):
 *
 *   1. the panel is only ever touched from this module's own task, pinned to core 0.
 *      Repainting from the CAN path on core 1 destroyed 1200 of 2000 frames with every
 *      error counter on both nodes reading zero -- the frames arrived and rotted in the
 *      RX queue while the CPU pushed pixels
 *   2. a repaint only redraws the lines whose text actually changed. A full repaint of
 *      this panel is ~198 ms, because Adafruit_GFX clocks text out one character at a
 *      time; change-detected it is ~160 us. Core 0 is also where the BLE stack lives,
 *      so relocating the cost without cutting it would only move the damage onto the
 *      link to RaceChrono
 *
 * The only entry point on the hot path is note(), called by the core 0 drain task for
 * each frame it forwards. It does no drawing -- it feeds the ID census and the ring of
 * recent frames that the display task renders on its own schedule.
 */
class display final
{
    CPP_NOCOPY(display);
    CPP_NOMOVE(display);

public:
    static display& get() noexcept;

    ~display() noexcept = default;

    /**
     * Bring up the panel and start the refresh task on core 0. Safe to call from
     * core 1: the panel itself is initialised inside the task, so core 1 never
     * issues an SPI transaction to it.
     */
    bool start() noexcept;

    /**
     * Record a frame for the ID census and the recent-frame ring.
     *
     * Called from the core 0 drain task, once per forwarded frame. Cost is a linear
     * scan of at most id_slots entries -- at the R9's ~1050 msg/s that is noise, and
     * it deliberately does not run in the ISR, where it would not be.
     *
     * The census therefore counts what reaches RaceChrono, not what reaches the
     * controller: frames the decoder rejects never enter the queue. With the R9
     * decoder that is the whole bus, since it has no allow-list, but if RaceChrono
     * ever sends a deny-all the census goes quiet while the controller's own frame
     * counter keeps climbing. That difference is visible on the BUS page.
     */
    void note(canbus::frame const& f) noexcept;

    /**
     * print what the panel is showing, and what it costs.
     *
     * The panel is the one part of this firmware a shell cannot see, which would
     * make the repaint cost -- the number the whole design of this module turns on
     * -- the one number nobody can check without standing over the board. So the
     * display task logs it alongside the CAN and BLE stats, on the same cadence.
     * DEBUG-only, like the rest of them.
     */
#if defined(DEBUG)
    void stats() noexcept;
#else
    void stats() noexcept {}
#endif

private:
    explicit display() noexcept;

    static void task(void*);
    void run() noexcept;

    void refresh() noexcept;
    void poll_buttons() noexcept;
    void sample_rates() noexcept;

    /// draw one line, but only if its text differs from what is already on the glass
    void field(int y, uint8_t size, uint16_t color, int slot, const char* text) noexcept;
    /// format one content row, space-padded to the full width so the previous line
    /// cannot leave a tail behind: text is drawn over its own background, and only
    /// the cells a character occupies get repainted
    void row(int idx, uint16_t color, const char* fmt, ...) noexcept __attribute__((format(printf, 4, 5)));
    /// blank the content area and invalidate every cache slot
    void wipe() noexcept;

    void page_bus() noexcept;
    void page_ids() noexcept;
    void page_last() noexcept;

private:
    /// pages, in the order D1 cycles through them
    enum page : uint8_t
    {
        page_bus_id = 0,
        page_ids_id = 1,
        page_last_id = 2,
        page_count = 3,
    };

    /// arbitration ids tracked by the census. The R1-family bus carries ~16
    /// (motocan/canbus-mapping.md), and 20 rows is what the panel can show.
    static constexpr int id_slots = 20;
    /// recent frames kept for the LAST page
    static constexpr int ring_slots = 6;
    /// change-detection cache: one entry per drawable line
    static constexpr int cache_slots = 14;
    static constexpr int cache_len = 44;

    // --- written by the drain task, read by the display task. Both live on core 0
    // and neither takes a lock: every field is a counter or a byte of payload, so a
    // torn read costs one stale line for 200 ms and nothing else. A mutex here would
    // put the CAN drain behind the panel, which is the one thing this module exists
    // to avoid.
    uint32_t _ids[id_slots];
    uint32_t _hits[id_slots];
    uint32_t _hits_prev[id_slots];
    uint16_t _rates[id_slots];
    volatile int _id_used;
    uint32_t _id_overflow;

    canbus::frame _ring[ring_slots];
    volatile uint32_t _ring_pos;

    // --- display task only
    TaskHandle_t _handle;
    uint8_t _page;
    bool _lit;
    bool _hold;
    uint32_t _draw_us;
    uint32_t _draw_us_max;
    /// _draw_us as of the last one-second sample. What the panel shows has to change
    /// no faster than once a second, or the line reporting the repaint cost is the
    /// reason there is one
    uint32_t _draw_us_shown;
    /// characters actually redrawn on the last pass. Each one costs ~600 us of SPI,
    /// so this is the panel's whole cost model in a single number
    uint32_t _dirty;
    uint32_t _min_heap;

    /// cumulative counters as of the last rate sample, and the rates derived from them
    uint32_t _rx_prev;
    uint32_t _fwd_prev;
    uint32_t _ble_prev;
    uint32_t _rx_rate;
    uint32_t _fwd_rate;
    uint32_t _ble_rate;
    uint32_t _rate_ms;
    utils::timer _stats_timer;

    char _cache[cache_slots][cache_len];
};

} // namespace ui

extern ui::display& DISP;
