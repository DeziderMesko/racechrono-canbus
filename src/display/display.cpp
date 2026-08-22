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

#include "../canbus/controller.hpp"
#include "../canbus/decoder.hpp"
#include "../led/led.hpp"
#include "../logging/logging.hpp"
#include "../racechrono/device.hpp"

#include "display.hpp"

#if defined(CONFIG_DISPLAY_TFT)

#include <cstdarg>

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <hal/twai_ll.h>

namespace
{

// The panel has exactly one owner: ui::display's task. It is a file-static rather
// than a member so that no header outside this file pulls in Adafruit_GFX, and so
// that nothing else can reach it.
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// 240x135 in landscape, 6x8 pixels per character at size 1: 40 columns, and enough
// rows for a header, nine lines of content and a footer.
constexpr int screen_w = 240;
constexpr int screen_h = 135;
constexpr int cols = 40;
// header is drawn at text size 2, so half as many columns fit across the glass
constexpr int head_cols = 20;
constexpr int content_y = 20;
constexpr int row_h = 11;
constexpr int rows = 9;
constexpr int footer_y = 122;

/// how often the BUS page is repainted. Five hertz is faster than anyone reads and
/// nearly free: with nothing changing a pass costs ~280 us, and each character that
/// does change costs ~650 us on top.
constexpr uint32_t refresh_bus_ms = 200;
/// how often the census and frame pages are repainted. Deliberately slower: nearly
/// every character on them changes every time, so at 5 Hz the LAST page alone would
/// spend half of core 0 on SPI to tell a human something they cannot read that fast.
constexpr uint32_t refresh_slow_ms = 1000;
/// how often the buttons are sampled. The refresh interval is far too coarse for a
/// button -- a quick press between two repaints would simply not exist.
constexpr uint32_t poll_ms = 25;

/// bytes, not words: ESP-IDF's StackType_t is a byte. The bench sketch's display
/// task ran the same drawing code in 4 KB with ~2.2 KB to spare.
constexpr uint32_t stack_size = 4096;
StaticTask_t task_buffer;
StackType_t task_stack[stack_size];

/// D0 is the BOOT strapping pin: pulled up, and reads LOW when pressed. D1 and D2
/// are pulled down and read HIGH. They do not share a polarity -- see
/// motocan/bluecan/hardware.md.
constexpr int btn_light = 0;
constexpr int btn_page = 1;
constexpr int btn_hold = 2;

const char* bus_state(uint32_t status)
{
    if (status & TWAI_LL_STATUS_BS) return "BUS-OFF";
    if (status & TWAI_LL_STATUS_ES) return "ERRORS";
    return "RUN";
}

#if defined(CONFIG_STATUS_LED)

/// how long the forwarded rate must read zero before the light calls it stopped
/// rather than slow. Two seconds is four samples of a rate that updates once a
/// second, and forty times the gap between frames at the ~20 msg/s per id a
/// recording session actually produces.
constexpr uint32_t flow_dead_ms = 2000;
/// the flatline pulse: one dim blip this often, so a stopped link still looks alive
/// enough to be worth reading rather than dead enough to be mistaken for off
constexpr uint32_t flatline_ms = 2000;
/// how long a blip, a flatline pulse or a fault flash lasts
constexpr uint32_t flash_ms = 60;
/// what a blip falls back to between flashes. Not zero: the hue is the state, and
/// dropping it entirely would make a slow link indistinguishable from a dark board.
constexpr uint8_t blip_floor = 64;
/// the fault overlay's period
constexpr uint32_t fault_ms = 1000;

/// 0-255 triangle over one period. A triangle rather than a sine because it costs
/// two branches and a multiply, and at this brightness the difference is invisible.
uint8_t breathe(uint32_t now, uint32_t period_ms) noexcept
{
    const uint32_t phase = now % period_ms;
    const uint32_t half = period_ms / 2U;
    const uint32_t up = phase < half ? phase : period_ms - phase;
    return static_cast<uint8_t>((up * 255U) / half);
}

/// square wave, on for the first half of the period
__always_inline uint8_t blink(uint32_t now, uint32_t period_ms) noexcept
{
    return (now % period_ms) < (period_ms / 2U) ? 255U : 0U;
}

/// a short flash on a dim floor, at the rate frames are actually being forwarded.
/// Clamped at both ends: below 1 Hz it would be indistinguishable from the flatline
/// it exists to contrast with, and above 5 Hz from a solid colour.
__always_inline uint8_t blip(uint32_t now, uint32_t rate) noexcept
{
    uint32_t hz = rate / 8U;
    if (hz < 1U) { hz = 1U; }
    if (hz > 5U) { hz = 5U; }

    const uint32_t period = 1000U / hz;
    return (now % period) < flash_ms ? 255U : blip_floor;
}

/// scale one component of a full-scale colour by a 0-255 pattern level
__always_inline uint8_t level(uint8_t component, uint8_t lvl) noexcept
{
    return static_cast<uint8_t>((static_cast<uint32_t>(component) * lvl) / 255U);
}

#endif // CONFIG_STATUS_LED

} // namespace

namespace ui
{

display& display::get() noexcept
{
    static display instance;
    return instance;
}

display::display() noexcept
    : _ids{}
    , _hits{}
    , _hits_prev{}
    , _rates{}
    , _id_used(0)
    , _id_overflow(0U)
    , _ring{}
    , _ring_pos(0U)
    , _handle(nullptr)
    , _page(page_bus_id)
    , _light(light_both)
    , _hold(false)
    , _fault(false)
    , _flow_ms(0U)
    , _lost_prev(0U)
    , _draw_us(0U)
    , _draw_us_max(0U)
    , _draw_us_shown(0U)
    , _dirty(0U)
    , _min_heap(0xFFFFFFFFU)
    , _rx_prev(0U)
    , _fwd_prev(0U)
    , _ble_prev(0U)
    , _rx_rate(0U)
    , _fwd_rate(0U)
    , _ble_rate(0U)
    , _rate_ms(0U)
    , _stats_timer{}
    , _cache{}
{
}

bool display::start() noexcept
{
    bootln("Display starting...");

    _handle = xTaskCreateStaticPinnedToCore(
        task,
        "display",
        stack_size,
        this,
        tskIDLE_PRIORITY + 1,
        task_stack,
        &task_buffer,
        0
    );

    if (_handle == nullptr)
    {
        errorln("ERROR: display task creation failed!");
        return false;
    }

    bootln("Display started!");

    return true;
}

void display::note(canbus::frame const& f) noexcept
{
    uint32_t id = f.id;
    // Relaxed: this task is the only writer of either index.
    int used = _id_used.load(std::memory_order_relaxed);
    bool known = false;

    for (int i = 0; i < used; i++)
    {
        if (_ids[i] == id)
        {
            _hits[i]++;
            known = true;
            break;
        }
    }

    if (!known)
    {
        if (used < id_slots)
        {
            // fill the slot before publishing it, so the display task cannot read a
            // half-written entry: _id_used is what makes the slot visible, and the
            // release is what stops the four stores above being sunk past it
            _ids[used] = id;
            _hits[used] = 1;
            _hits_prev[used] = 0;
            _rates[used] = 0;
            _id_used.store(used + 1, std::memory_order_release);
        }
        else
        {
            _id_overflow++;
        }
    }

    uint32_t pos = _ring_pos.load(std::memory_order_relaxed);
    _ring[pos % ring_slots] = f;
    _ring_pos.store(pos + 1, std::memory_order_release);
}

void display::task(void* arg)
{
    static_cast<display*>(arg)->run();
}

void display::run() noexcept
{
    // The panel is initialised here rather than in start(), which runs on core 1.
    // One task owns the SPI bus to it, and this is that task.
    pinMode(TFT_I2C_POWER, OUTPUT);
    digitalWrite(TFT_I2C_POWER, HIGH);
    pinMode(TFT_BACKLITE, OUTPUT);
    digitalWrite(TFT_BACKLITE, HIGH);

    pinMode(btn_light, INPUT_PULLUP);
    pinMode(btn_page, INPUT_PULLDOWN);
    pinMode(btn_hold, INPUT_PULLDOWN);

    tft.init(screen_h, screen_w);
    // Adafruit_ST7789 defaults conservatively. The panel is good for 40 MHz and a
    // repaint is almost entirely SPI clocking, so this halves it for nothing. It is
    // not the fix -- change detection is -- just cheaper work.
    tft.setSPISpeed(40000000);
    tft.setRotation(3);
    tft.fillScreen(ST77XX_BLACK);
    wipe();

#if defined(CONFIG_STATUS_LED)
    // Here rather than lazily on the first update: this is the call that allocates
    // the RMT channel behind the pixel, and start-up is where an allocation belongs
    // on a board whose free heap is a measurement.
    LED.status_begin();
#endif

    _rate_ms = millis();

    uint32_t last_paint = 0;

    for (;;)
    {
        poll_buttons();

        uint32_t now = millis();

        // Sampled even while the panel is dark or held: the rates are a measurement
        // of the bus, not of the display, and a held screen that resumed with a
        // minute-long average would be reporting the wrong thing.
        if (now - _rate_ms >= 1000)
        {
            sample_rates();
        }

        // Every pass, and before the repaint: the light is the instrument that still
        // works in the two positions where the panel does not, and 25 ms is the
        // animation's frame time.
        update_status_led();

        uint32_t interval = _page == page_bus_id ? refresh_bus_ms : refresh_slow_ms;

        if (_light == light_both && !_hold && (now - last_paint) >= interval)
        {
            last_paint = now;
            uint32_t t0 = micros();
            _dirty = 0;
            refresh();
            _draw_us = micros() - t0;
            if (_draw_us > _draw_us_max)
            {
                _draw_us_max = _draw_us;
            }
        }

        stats();

        vTaskDelay(pdMS_TO_TICKS(poll_ms));
    }
}

#if defined(DEBUG)
void display::stats() noexcept
{
    if (logging::logger::get().level() >= logging::log_level::info)
    {
        if (_stats_timer.elapsed(CONFIG_RC_STATS_TIMEOUT) > 0UL)
        {
            const char* name = _page == page_ids_id ? "IDS"
                             : _page == page_last_id ? "LAST"
                                                     : "BUS";

            const char* lightness = _light == light_both ? ""
                                  : _light == light_pixel ? " (panel off)"
                                                          : " (dark)";

            infoln("      Display page: %s%s%s", name, lightness,
                   _hold ? " (held)" : "");
            infoln("      Display draw: %lu us, worst %lu us, %lu chars",
                   (unsigned long)_draw_us, (unsigned long)_draw_us_max,
                   (unsigned long)_dirty);
            infoln("     Display stack: %lu bytes free",
                   (unsigned long)(_handle ? uxTaskGetStackHighWaterMark(_handle) : 0));
            infoln("        Census ids: %d, %lu over",
                   _id_used.load(std::memory_order_acquire),
                   (unsigned long)_id_overflow);
        }
    }
}
#endif

void display::poll_buttons() noexcept
{
    bool light = digitalRead(btn_light) == LOW;
    bool next = digitalRead(btn_page) == HIGH;
    bool hold = digitalRead(btn_hold) == HIGH;

    static bool p_light = false;
    static bool p_next = false;
    static bool p_hold = false;

    if (light && !p_light)
    {
        // Three positions: panel and pixel, pixel alone, dark. Backlight off also
        // stops the repaint, which makes the middle position the cheapest experiment
        // on the board -- everything else keeps running, and the pixel costs ~30 us
        // an update, so any difference in the frame counters afterwards is the
        // panel's cost and nothing else. The third is for riding at night.
        _light = (_light + 1U) % light_count;
        digitalWrite(TFT_BACKLITE, _light == light_both ? HIGH : LOW);
    }

    if (next && !p_next)
    {
        _page = (_page + 1) % page_count;
        _hold = false;
        wipe();
    }

    if (hold && !p_hold)
    {
        _hold = !_hold;
        // Freezes the glass, not the counting: the ISR, the queue and the census all
        // carry on, so a held screen is a snapshot rather than a pause.
        if (_hold)
        {
            refresh();
        }
    }

    p_light = light;
    p_next = next;
    p_hold = hold;
}

void display::sample_rates() noexcept
{
    uint32_t now = millis();
    uint32_t elapsed = now - _rate_ms;
    _rate_ms = now;

    if (elapsed == 0)
    {
        return;
    }

    canbus::controller::counters_t c = CANCTLR.counters();
    uint32_t ble = RCDEV.frames();

    _rx_rate = ((c.frames - _rx_prev) * 1000UL) / elapsed;
    _fwd_rate = ((c.forwarded - _fwd_prev) * 1000UL) / elapsed;
    _ble_rate = ((ble - _ble_prev) * 1000UL) / elapsed;
    _rx_prev = c.frames;
    _fwd_prev = c.forwarded;
    _ble_prev = ble;

    int used = _id_used.load(std::memory_order_acquire);
    for (int i = 0; i < used; i++)
    {
        uint32_t hits = _hits[i];
        _rates[i] = static_cast<uint16_t>(((hits - _hits_prev[i]) * 1000UL) / elapsed);
        _hits_prev[i] = hits;
    }

    _draw_us_shown = _draw_us;

    uint32_t heap = ESP.getFreeHeap();
    if (heap < _min_heap)
    {
        _min_heap = heap;
    }

#if defined(CONFIG_STATUS_LED)
    // The status light's two pieces of state are sampled here, at 1 Hz, rather than
    // on the light's own 25 ms tick. The fault latches, so a second's delay in
    // noticing costs nothing, and this keeps the tick off RCDEV.peers() -- the one
    // term in it that reaches into the NimBLE server rather than reading an atomic.
    const uint32_t lost = RCDEV.lost();

    // A dropped frame, an allow-list too small for what the app asked for and a
    // second subscriber are faults whenever they happen: the first is always ours,
    // the second silently loses a channel the rider defined, the third doubles the
    // radio's work for the same bus.
    if (!_fault
        && (c.dropped != 0U || CANDEC.filter_overflow() != 0U || RCDEV.peers() > 1U))
    {
        _fault = true;
    }

    // A refusal is only a fault while the allow-list is in force. An allow-all --
    // what RaceChrono sends while channels are being configured -- has no per-id slot
    // to rate-limit against, so it offers the whole bus to a radio measured at about
    // 590 msg/s and refusals are the expected result. Measured on the bench the day
    // this landed: a test connection at ~919 msg/s left 15001 behind. Latching on the
    // total would light the fault for every session after that one, so this latches
    // on growth, and only once the filter is active.
    if (!_fault && CANDEC.filter_size() >= 0 && lost > _lost_prev)
    {
        _fault = true;
    }

    _lost_prev = lost;

    if (_fwd_rate != 0U)
    {
        _flow_ms = now;
    }
#endif
}

void display::update_status_led() noexcept
{
#if defined(CONFIG_STATUS_LED)
    if (_light == light_dark)
    {
        LED.status_end();
        return;
    }

    const uint32_t now = millis();
    const bool running = CANCTLR.running();
    const uint32_t status = running ? CANCTLR.status() : 0U;

    // One LED, so precedence is strict. Hue says where the chain is, motion says
    // frames are moving, and every term below is a counter the BUS page already
    // reads -- nothing is measured for the light's sake.
    //
    // What is deliberately absent is vehicle data. This firmware forwards raw frames
    // and never decodes a signal, which is why an unfinished canbus-mapping.md never
    // blocks it; a shift light would need a confirmed mapping, a decoder here and a
    // scale to trust, which is the sniffer role that was settled the other way.
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t lvl;

    if (!running)
    {
        // The controller is not up. In practice this is the second and a half
        // between the display task starting and setup() reaching CANCTLR.start(),
        // because a failure there restarts the board -- so read it as booting.
        r = 255; g = 255; b = 255;
        lvl = breathe(now, 2000U);
    }
    else if (status & TWAI_LL_STATUS_BS)
    {
        // Bus-off. Nearly unreachable in listen-only, where this node never
        // transmits and its error counters are frozen, and kept for the day that
        // assumption is wrong: two comparisons is a fair price.
        r = 255; g = 0; b = 0;
        lvl = 255;
    }
    else if (status & TWAI_LL_STATUS_ES)
    {
        r = 255; g = 140; b = 0;
        lvl = blink(now, 1000U);
    }
    else if (!RCDEV.subscribed())
    {
        // Nobody is taking frames -- either nothing is connected, or something is
        // connected and has not subscribed. One colour for both: they differ in what
        // the rider would do about it not at all, and the header on the BUS page
        // separates them for anyone standing over the board.
        r = 0; g = 0; b = 255;
        lvl = breathe(now, 2000U);
    }
    else
    {
        // Yellow while RaceChrono is configuring channels and asking for every id,
        // green once it is recording and asking for the handful it has channels for.
        // The difference matters: an allow-all is not rate-limited, so it is also the
        // one state in which this link is expected to refuse frames.
        const bool unfiltered = CANDEC.filter_size() < 0;

        r = unfiltered ? 255 : 0;
        g = 255;
        b = 0;

        // Slow and stopped are the two states no other indicator on this board can
        // tell apart, and a diagnostic connector that fell off is the second one.
        lvl = (now - _flow_ms) >= flow_dead_ms
            ? ((now % flatline_ms) < flash_ms ? blip_floor : 0U)
            : blip(now, _fwd_rate);
    }

    // The fault is an overlay, not a rung. Made a rung it would replace the hue and
    // hide whether frames were still flowing at the moment that is most worth
    // knowing; flashed over it, "recording, and something was lost" is one glance
    // from "recording".
    if (_fault && (now % fault_ms) < flash_ms)
    {
        r = 255; g = 0; b = 0;
        lvl = 255;
    }

    LED.status(level(r, lvl), level(g, lvl), level(b, lvl));
#endif
}

void display::field(int y, uint8_t size, uint16_t color, int slot, const char* text) noexcept
{
    char* cache = _cache[slot];

    size_t len = strlen(text);
    if (len > cache_len - 1)
    {
        len = cache_len - 1;
    }

    // Character-level diffing, not line-level. A character costs ~600 us to draw:
    // Adafruit_GFX renders text a pixel at a time, so a 40-column line is 24 ms --
    // more than a tenth of the refresh interval, spent on core 0 next to the BLE
    // stack. Line-level detection was enough on the bench sketch and is not enough
    // here, because the line that reports the repaint cost changes on every pass and
    // would then be paying 24 ms to report that it had paid 24 ms.
    //
    // A length change means a differently-shaped line, and after wipe() the cache
    // holds a sentinel: either way, redraw all of it.
    bool whole = strlen(cache) != len;

    for (size_t i = 0; i < len; )
    {
        if (!whole && cache[i] == text[i])
        {
            i++;
            continue;
        }

        size_t j = i;
        while (j < len && (whole || cache[j] != text[j]))
        {
            j++;
        }

        tft.setTextSize(size);
        tft.setTextColor(color, ST77XX_BLACK);
        tft.setCursor(static_cast<int16_t>(i * 6 * size), y);
        tft.write(reinterpret_cast<const uint8_t*>(text) + i, j - i);
        _dirty += j - i;

        i = j;
    }

    memcpy(cache, text, len);
    cache[len] = '\0';
}

void display::row(int idx, uint16_t color, const char* fmt, ...) noexcept
{
    char buf[cache_len];

    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n < 0)
    {
        n = 0;
    }

    // Pad to the full line. Characters are drawn over their own background, so a
    // shorter line would leave the tail of the longer one it replaced on the glass.
    for (int i = (n < cols ? n : cols); i < cols; i++)
    {
        buf[i] = ' ';
    }
    buf[cols] = '\0';

    field(content_y + idx * row_h, 1, color, idx + 1, buf);
}

void display::wipe() noexcept
{
    tft.fillRect(0, content_y - 2, screen_w, screen_h - (content_y - 2), ST77XX_BLACK);

    // Blank the caches too, or the next pass compares against text that is no longer
    // on the screen, decides nothing changed, and leaves the area empty.
    for (int i = 1; i < cache_slots; i++)
    {
        _cache[i][0] = '\x01';
        _cache[i][1] = '\0';
    }
}

void display::refresh() noexcept
{
    // Header: the one line that is the same on every page, because it answers the
    // question asked most often -- is the phone actually getting anything.
    //
    // Three states, not two. A phone that is connected but has not subscribed to the
    // frame characteristic looks exactly like a working link on the phone, in the
    // advertising name, and in every other counter here, and receives nothing. It is
    // the failure worth spending a colour on.
    char head[cache_len];
    bool up = RCDEV.connected();
    bool sub = RCDEV.subscribed();
    const char* state = !up ? "adv" : sub ? "SUB" : "BLE";
    uint16_t state_c = !up ? ST77XX_YELLOW : sub ? ST77XX_GREEN : ST77XX_RED;

    // Connection interval, in tenths of a millisecond: the link reports it in 1.25 ms
    // units, and it is the ceiling on everything downstream, since the radio only
    // speaks at connection events. 7.5 ms is as fast as BLE goes; 30 ms is a phone
    // that will not keep up with the bus whatever the firmware does.
    unsigned long tenths = RCDEV.interval() * 125UL / 10UL;
    int n;
    if (up && tenths != 0UL)
    {
        n = snprintf(head, sizeof(head), "bluecan %s %lu.%lums",
                     state, tenths / 10UL, tenths % 10UL);
    }
    else
    {
        n = snprintf(head, sizeof(head), "bluecan %s", state);
    }

    // field() draws each character over its own background but does not pad, so a
    // shorter header would leave the tail of a longer one on the glass.
    for (; n < head_cols && n < static_cast<int>(sizeof(head)) - 1; n++)
    {
        head[n] = ' ';
    }
    head[n] = '\0';

    field(0, 2, state_c, 0, head);

    switch (_page)
    {
    case page_ids_id:
        page_ids();
        break;
    case page_last_id:
        page_last();
        break;
    default:
        page_bus();
        break;
    }

    char foot[cache_len];
    const char* name = _page == page_ids_id ? "IDS" : _page == page_last_id ? "LAST" : "BUS";
    snprintf(foot, sizeof(foot), "%-4s %-9s D0 lit D1 page D2 hold",
             name, _hold ? "**HOLD**" : "");
    field(footer_y, 1, _hold ? ST77XX_YELLOW : ST77XX_CYAN, cache_slots - 1, foot);
}

void display::page_bus() noexcept
{
    canbus::controller::counters_t c = CANCTLR.counters();
    // Nothing is read off the peripheral until install() has ungated its clock: this
    // task starts several seconds before that and would otherwise be sampling a
    // controller still held in reset. Zero is the honest reading meanwhile -- both
    // error counters are frozen there in listen-only mode anyway.
    bool running = CANCTLR.running();
    uint32_t status = running ? CANCTLR.status() : 0U;
    const char* state = running ? bus_state(status) : "DOWN";
    uint16_t state_color = !running || (status & TWAI_LL_STATUS_BS) ? ST77XX_RED
                         : (status & TWAI_LL_STATUS_ES)            ? ST77XX_YELLOW
                                                                   : ST77XX_GREEN;

    row(0, state_color, "CAN 500k LISTEN-ONLY %s", state);
    row(1, ST77XX_WHITE, "rx  %9lu %5lu/s", (unsigned long)c.frames, (unsigned long)_rx_rate);
    // What the app asked for, next to what it got. The IDS page says which ids crossed
    // the filter, but not whether the board is filtering at all -- and "the phone never
    // sent its allow-list" and "the phone asked for three ids" look identical from
    // every other counter on this panel. ALL is the boot state and the bench state.
    int flt = CANDEC.filter_size();
    // An allow-list too small for what the app asked for means a channel the rider
    // defined never arrives, and every other counter here would look healthy while it
    // happened. It cannot fit its own line, so it takes a character.
    uint32_t flt_over = CANDEC.filter_overflow();
    char flt_txt[8];
    if (flt < 0)
    {
        snprintf(flt_txt, sizeof(flt_txt), "ALL");
    }
    else
    {
        snprintf(flt_txt, sizeof(flt_txt), "%d%s", flt, flt_over ? "!" : "");
    }
    row(2, flt_over          ? ST77XX_RED
         : flt == 0          ? ST77XX_YELLOW
                             : ST77XX_WHITE,
        "fwd %9lu %5lu/s flt %s",
        (unsigned long)c.forwarded, (unsigned long)_fwd_rate, flt_txt);
    // lost is the BLE half of the drop counter two lines down, and it exists for the
    // same reason: BLECharacteristic::notify() returns void and logs a refusal at a
    // level a release build never prints, so a link too slow for the bus would
    // otherwise lose frames in complete silence.
    unsigned long ble_lost = RCDEV.lost();
    // peers should never exceed one. The frame path notifies once per subscriber, so a
    // second connection doubles the BLE traffic for the same bus while every other
    // counter here keeps looking healthy.
    unsigned long peers = RCDEV.peers();
    row(3, (ble_lost || peers > 1) ? ST77XX_RED
         : RCDEV.connected()       ? ST77XX_WHITE
                                   : ST77XX_YELLOW,
        "ble %8lu %4lu/s lost %lu p%lu",
        (unsigned long)RCDEV.frames(), (unsigned long)_ble_rate, ble_lost, peers);
    row(4, c.queued > (c.capacity / 2) ? ST77XX_YELLOW : ST77XX_WHITE,
        "queue %4lu peak %4lu/%lu",
        (unsigned long)c.queued, (unsigned long)c.peak, (unsigned long)c.capacity);

    // The line that matters most and is easiest to miss. A dropped frame is silent
    // everywhere else: the bus shows no error, because the loss was ours.
    row(5, c.dropped ? ST77XX_RED : ST77XX_GREEN,
        "drop %lu  err %lu", (unsigned long)c.dropped, (unsigned long)c.errors);

    // Frame classes the ISR throws away. A sniffer cannot claim a class is empty by
    // discarding it, so they are counted and shown -- every mapped R9 id is 11-bit,
    // and this is what would say otherwise.
    row(6, (c.extended || c.remote) ? ST77XX_YELLOW : ST77XX_WHITE,
        "ext %lu rtr %lu tec %lu rec %lu",
        (unsigned long)c.extended, (unsigned long)c.remote,
        (unsigned long)(running ? CANCTLR.tec() : 0U),
        (unsigned long)(running ? CANCTLR.rec() : 0U));

    UBaseType_t stack = _handle ? uxTaskGetStackHighWaterMark(_handle) : 0;
    row(7, ST77XX_WHITE, "heap %luk min %luk stk %lu",
        (unsigned long)(ESP.getFreeHeap() / 1024), (unsigned long)(_min_heap / 1024),
        (unsigned long)stack);

    // What a repaint costs, on the glass it costs it on. A full repaint of this panel
    // is ~198 ms and a change-detected one ~160 us; if max ever climbs back towards
    // the former, the change detection has stopped working.
    row(8, ST77XX_WHITE, "draw %luus max %luus up %lus",
        (unsigned long)_draw_us_shown, (unsigned long)_draw_us_max,
        (unsigned long)(millis() / 1000));
}

void display::page_ids() noexcept
{
    int used = _id_used.load(std::memory_order_acquire);

    // The census holds id_slots entries and this page has room for fewer, so there are
    // two ways to be hiding an id. Both have to raise the marker, or a truncated list
    // reads as the whole bus -- which is the one question this page exists to answer.
    static constexpr int id_cells = (rows - 1) * 2;

    row(0, ST77XX_CYAN, "ids seen %d%s", used,
        (_id_overflow || used > id_cells) ? " (+more)" : "");

    // Two columns of ids, newest-discovered last. This is the first question the
    // device exists to answer: which CAN family the R9 belongs to is read off the id
    // list, not off any decoded value.
    for (int r = 0; r < rows - 1; r++)
    {
        int left = r * 2;
        int right = left + 1;
        char cell[2][20];

        for (int col = 0; col < 2; col++)
        {
            int i = col == 0 ? left : right;
            if (i < used)
            {
                snprintf(cell[col], sizeof(cell[col]), "%03lX %7lu %3u/s",
                         (unsigned long)_ids[i], (unsigned long)_hits[i], _rates[i]);
            }
            else
            {
                cell[col][0] = '\0';
            }
        }

        row(r + 1, ST77XX_WHITE, "%-19s %-19s", cell[0], cell[1]);
    }
}

void display::page_last() noexcept
{
    uint32_t pos = _ring_pos.load(std::memory_order_acquire);

    row(0, ST77XX_CYAN, "last frames  (newest first)");

    for (int r = 0; r < ring_slots; r++)
    {
        if (pos == 0 || static_cast<uint32_t>(r) >= pos)
        {
            row(r + 1, ST77XX_WHITE, "%s", "");
            continue;
        }

        canbus::frame const& f = _ring[(pos - 1 - r) % ring_slots];

        char bytes[24];
        int n = 0;
        for (uint8_t b = 0; b < f.info.dlc && b < 8; b++)
        {
            n += snprintf(bytes + n, sizeof(bytes) - n, "%02X", f.data.u8[b]);
        }
        bytes[n] = '\0';

        row(r + 1, ST77XX_WHITE, "%03lX [%u] %s",
            (unsigned long)f.id, static_cast<unsigned>(f.info.dlc), bytes);
    }
}

} // namespace ui

#else // !CONFIG_DISPLAY_TFT

namespace ui
{

// Boards without the TFT still compile and run; they simply have nowhere to draw.

display& display::get() noexcept
{
    static display instance;
    return instance;
}

display::display() noexcept
    : _ids{}
    , _hits{}
    , _hits_prev{}
    , _rates{}
    , _id_used(0)
    , _id_overflow(0U)
    , _ring{}
    , _ring_pos(0U)
    , _handle(nullptr)
    , _page(page_bus_id)
    , _light(light_dark)
    , _hold(false)
    , _fault(false)
    , _flow_ms(0U)
    , _lost_prev(0U)
    , _draw_us(0U)
    , _draw_us_max(0U)
    , _draw_us_shown(0U)
    , _dirty(0U)
    , _min_heap(0xFFFFFFFFU)
    , _rx_prev(0U)
    , _fwd_prev(0U)
    , _ble_prev(0U)
    , _rx_rate(0U)
    , _fwd_rate(0U)
    , _ble_rate(0U)
    , _rate_ms(0U)
    , _stats_timer{}
    , _cache{}
{
}

bool display::start() noexcept { return true; }
void display::note(canbus::frame const&) noexcept {}
#if defined(DEBUG)
void display::stats() noexcept {}
#endif
void display::task(void*) {}
void display::run() noexcept {}
void display::refresh() noexcept {}
void display::poll_buttons() noexcept {}
void display::sample_rates() noexcept {}
void display::update_status_led() noexcept {}
void display::field(int, uint8_t, uint16_t, int, const char*) noexcept {}
void display::row(int, uint16_t, const char*, ...) noexcept {}
void display::wipe() noexcept {}
void display::page_bus() noexcept {}
void display::page_ids() noexcept {}
void display::page_last() noexcept {}

} // namespace ui

#endif // CONFIG_DISPLAY_TFT

ui::display& DISP = ui::display::get();
