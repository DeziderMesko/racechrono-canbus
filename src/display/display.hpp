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

#include <atomic>

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

    /**
     * Drive the status pixel from the counters this task already reads.
     *
     * Runs every pass of the loop, not every repaint: the light is an animation and
     * the panel is not, and it must keep working in the light position where the
     * panel does not repaint at all. Costs a comparison unless the colour changed.
     */
    void update_status_led() noexcept;

    /**
     * Say on the console what colour the light is on, and why.
     *
     * DEBUG only, and called from two places: the stats cadence, and the moment the
     * light changes rung. The second is what makes it usable as an instrument --
     * plug a phone in and the line arrives with it, rather than up to five seconds
     * later. See tools/led-state.py in motocan.
     */
#if defined(DEBUG)
    void report_status_led() noexcept;
#else
    void report_status_led() noexcept {}
#endif

    /// draw one line, but only if its text or its colour differs from what is already
    /// on the glass -- a colour change with no character change still has to redraw,
    /// or a row that keeps its digits but changes hue leaves the old hue behind on
    /// every digit that happened not to change that pass
    void field(int y, uint8_t size, uint16_t color, int slot, const char* text, int col = 0) noexcept;
    /// format one content row, space-padded to the full width so the previous line
    /// cannot leave a tail behind: text is drawn over its own background, and only
    /// the cells a character occupies get repainted
    void row(int idx, uint16_t color, const char* fmt, ...) noexcept __attribute__((format(printf, 4, 5)));
    /// draw one coloured segment of a row at column *col*, sized for content rows
    /// (text size 1), and advance *col* past it. Used to give a row's label and its
    /// value different colours without hand-assigning a cache slot to each one: the
    /// slot is whichever one is next in _slot, which is reset to the same starting
    /// value at the top of every refresh() -- so as long as a page draws its segments
    /// in the same order every pass, each one lands on the same slot every time.
    void seg(int y, int& col, uint16_t color, const char* fmt, ...) noexcept __attribute__((format(printf, 5, 6)));
    /// like seg(), but pads its text with spaces out to column *cols* first. For the
    /// last segment on a row, so a value that shrinks cannot leave the previous,
    /// longer value's tail on the glass.
    void seg_fill(int y, int& col, uint16_t color, const char* fmt, ...) noexcept __attribute__((format(printf, 5, 6)));
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

    /// what D0 cycles through. The middle position is the riding one: the backlight
    /// and the repaint are the two expensive things on this board and the status
    /// light is the one worth keeping, at ~30 us an update.
    enum light : uint8_t
    {
        light_both = 0,
        light_pixel = 1,
        light_dark = 2,
        light_count = 3,
    };

    /// arbitration ids tracked by the census. The R1-family bus carries ~16
    /// (motocan/canbus-mapping.md). The IDS page shows fewer than this -- see
    /// id_cells in page_ids() -- and says so when it is hiding any.
    static constexpr int id_slots = 20;
    /// recent frames kept for the LAST page
    static constexpr int ring_slots = 6;
    /// change-detection cache: one entry per drawable line, split into two pools.
    /// Slots 1..9 and cache_slots-1 are hand-assigned to page_ids(), page_last() and
    /// the footer, which each draw one colour per line and can keep a fixed slot per
    /// row. Slots from slot_dynamic_base up are page_bus() and the header's pool:
    /// those rows mix a yellow label with a white or status-coloured value, so they
    /// are built from a variable number of seg()/seg_fill() calls instead, each
    /// claiming the next free slot in _slot. That counter resets to slot_dynamic_base
    /// at the top of every refresh(), so a pool-B slot is stable across repaints only
    /// because the sequence of seg() calls that produces it never changes shape.
    static constexpr int slot_dynamic_base = 20;
    static constexpr int cache_slots = 70;
    static constexpr int cache_len = 44;

    // --- written by the drain task, read by the display task. Both live on core 0
    // and neither takes a lock: every field is a counter or a byte of payload, so a
    // torn read costs one stale line for 200 ms and nothing else. A mutex here would
    // put the CAN drain behind the panel, which is the one thing this module exists
    // to avoid.
    //
    // The two indices are the exception, and they are atomic rather than volatile:
    // each one publishes the slot written just before it, and volatile orders nothing
    // against a plain store, so the compiler is free to sink the slot's contents past
    // it. Released here, acquired by every reader, which is the whole synchronisation
    // in this module.
    uint32_t _ids[id_slots];
    uint32_t _hits[id_slots];
    uint32_t _hits_prev[id_slots];
    uint16_t _rates[id_slots];
    std::atomic<int> _id_used;
    uint32_t _id_overflow;

    canbus::frame _ring[ring_slots];
    std::atomic<uint32_t> _ring_pos;

    // --- display task only
    TaskHandle_t _handle;
    uint8_t _page;
    uint8_t _light;
    bool _hold;
    /// the fault overlay, once armed. Latching, because dropped and lost are
    /// cumulative counters and the moment one of them moves is the moment nobody is
    /// looking. Only a reboot clears it; the BUS page says which one it was.
    bool _fault;
    /// millis() of the last non-zero forwarded-frame rate. What separates a slow
    /// link from a stopped one, which no other indicator on this board can do.
    uint32_t _flow_ms;
    /// RCDEV.lost() as of the last sample. The fault latch wants the growth, not the
    /// total: a test connection asks for every id and is not rate-limited, so
    /// refusals during one are the documented behaviour rather than a fault, and the
    /// count they leave behind must not arm the light for the session after it.
    uint32_t _lost_prev;
    /// what the light is doing, packed so a change is one comparison:
    /// rung << 16 | hz << 8 | fault. The pixel is the one part of this firmware a
    /// shell cannot see at all -- the panel at least reports its own repaint cost --
    /// so in a DEBUG build it says what colour it is on, both on the stats cadence
    /// and the moment it changes.
    uint32_t _led_state;
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
    /// the colour each slot was last drawn in, so a hue change is detected even when
    /// every character in the slot happens to be unchanged
    uint16_t _cache_color[cache_slots];
    /// the column each slot was last drawn at, so a slot whose own text and colour are
    /// unchanged still redraws when an earlier slot on the same row shifts it sideways
    int _cache_col[cache_slots];
    /// next free slot in pool B (see slot_dynamic_base), reset at the top of every
    /// refresh()
    int _slot;
};

} // namespace ui

extern ui::display& DISP;
