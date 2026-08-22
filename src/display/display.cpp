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

/// the "bluecan" wordmark's colour. Darker than ST77XX_BLUE (0x001F), which at full
/// saturation reads closer to electric blue than the calmer navy the name is meant
/// to evoke sitting next to a white or yellow status word.
constexpr uint16_t color_blue_dim = 0x1A96;

#if defined(CONFIG_STATUS_LED)

/// how long the forwarded rate must read zero before the light calls it stopped
/// rather than slow. Two seconds is four samples of a rate that updates once a
/// second, and forty times the gap between frames at the ~20 msg/s per id a
/// recording session actually produces.
constexpr uint32_t flow_dead_ms = 2000;
/// how often the flatline's pair of flashes repeats, how long each one lasts, and
/// how far the second trails the first
constexpr uint32_t flatline_ms = 2000;
constexpr uint32_t flatline_flash_ms = 90;
constexpr uint32_t flatline_gap_ms = 240;
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

/// the blip rate for a forwarded-frame rate. Clamped at both ends: below 1 Hz it
/// would be indistinguishable from the flatline it exists to contrast with, and
/// above 5 Hz from a solid colour.
__always_inline uint8_t blip_hz(uint32_t rate) noexcept
{
    uint32_t hz = rate / 8U;
    if (hz < 1U) { hz = 1U; }
    if (hz > 5U) { hz = 5U; }
    return static_cast<uint8_t>(hz);
}

/// a short flash on a dim floor, at that rate
__always_inline uint8_t blip(uint32_t now, uint8_t hz) noexcept
{
    const uint32_t period = 1000U / hz;
    return (now % period) < flash_ms ? 255U : blip_floor;
}

/// scale one component of a full-scale colour by a 0-255 pattern level
__always_inline uint8_t level(uint8_t component, uint8_t lvl) noexcept
{
    return static_cast<uint8_t>((static_cast<uint32_t>(component) * lvl) / 255U);
}

/// the flatline: two flashes, at full level, every flatline_ms.
///
/// The first version of this was one flash_ms blip at the blips' dim floor, which is
/// a 3% duty cycle at a quarter of a brightness already capped at 24/255 -- on the
/// bench it read as a pixel that was simply off. That is the worst possible way for
/// this rung to fail, because "connected and nothing is arriving" is the state that
/// means the diagnostic connector fell off the bike. Two flashes rather than one
/// because a single tick is hard to tell from a slow heartbeat, and this rung exists
/// precisely to be told apart from one.
__always_inline uint8_t flatline(uint32_t now) noexcept
{
    const uint32_t phase = now % flatline_ms;
    const bool lit = phase < flatline_flash_ms
                  || (phase >= flatline_gap_ms && phase < flatline_gap_ms + flatline_flash_ms);
    return lit ? 255U : 0U;
}

/// what the light is on, for the DEBUG line that reports it
const char* rung_name(uint8_t rung) noexcept
{
    switch (rung)
    {
    case 0:  return "white breathe  (CAN controller not up -- booting)";
    case 1:  return "red solid      (bus-off)";
    case 2:  return "amber blink    (error-passive)";
    case 3:  return "blue breathe   (nothing subscribed)";
    case 4:  return "yellow         (subscribed, flt ALL -- configuring)";
    default: return "green          (subscribed, filtered -- recording)";
    }
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
    , _led_state(0xFFFFFFFFU)
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
    , _cache_color{}
    , _slot(0)
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
            report_status_led();
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

#if defined(DEBUG)
void display::report_status_led() noexcept
{
#if defined(CONFIG_STATUS_LED)
    if (_light == light_dark)
    {
        infoln("      Status light: off -- D0 is on the dark position");
        return;
    }

    const uint8_t rung = static_cast<uint8_t>((_led_state >> 16) & 0xFFU);
    const uint8_t hz = static_cast<uint8_t>((_led_state >> 8) & 0xFFU);

    char pattern[32];
    if (rung < 4U)
    {
        pattern[0] = '\0';
    }
    else if (hz == 0U)
    {
        snprintf(pattern, sizeof(pattern), " FLATLINE -- no frames 2 s+");
    }
    else
    {
        snprintf(pattern, sizeof(pattern), " blips %u Hz", (unsigned)hz);
    }

    infoln("      Status light: %s%s%s", rung_name(rung), pattern,
           _fault ? "  + FAULT flash" : "");
#endif
}
#endif

void display::update_status_led() noexcept
{
#if defined(CONFIG_STATUS_LED)
    if (_light == light_dark)
    {
        LED.status_end();
#if defined(DEBUG)
        if (_led_state != 0xFFFFFFFFU)
        {
            _led_state = 0xFFFFFFFFU;
            report_status_led();
        }
#endif
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
    uint8_t rung;
    /// blip rate actually used, or 0 for a rung that is not blipping
    uint8_t hz = 0U;

    if (!running)
    {
        // The controller is not up. In practice this is the second and a half
        // between the display task starting and setup() reaching CANCTLR.start(),
        // because a failure there restarts the board -- so read it as booting.
        r = 255; g = 255; b = 255;
        lvl = breathe(now, 2000U);
        rung = 0U;
    }
    else if (status & TWAI_LL_STATUS_BS)
    {
        // Bus-off. Nearly unreachable in listen-only, where this node never
        // transmits and its error counters are frozen, and kept for the day that
        // assumption is wrong: two comparisons is a fair price.
        r = 255; g = 0; b = 0;
        lvl = 255;
        rung = 1U;
    }
    else if (status & TWAI_LL_STATUS_ES)
    {
        r = 255; g = 140; b = 0;
        lvl = blink(now, 1000U);
        rung = 2U;
    }
    else if (!RCDEV.subscribed())
    {
        // Nobody is taking frames -- either nothing is connected, or something is
        // connected and has not subscribed. One colour for both: they differ in what
        // the rider would do about it not at all, and the header on the BUS page
        // separates them for anyone standing over the board.
        r = 0; g = 0; b = 255;
        lvl = breathe(now, 2000U);
        rung = 3U;
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
        rung = unfiltered ? 4U : 5U;

        // Slow and stopped are the two states no other indicator on this board can
        // tell apart, and a diagnostic connector that fell off is the second one.
        if ((now - _flow_ms) >= flow_dead_ms)
        {
            lvl = flatline(now);
        }
        else
        {
            hz = blip_hz(_fwd_rate);
            lvl = blip(now, hz);
        }
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

#if defined(DEBUG)
    const uint32_t state = (static_cast<uint32_t>(rung) << 16)
                         | (static_cast<uint32_t>(hz) << 8)
                         | (_fault ? 1U : 0U);

    if (state != _led_state)
    {
        _led_state = state;
        report_status_led();
    }
#endif
#endif
}

void display::field(int y, uint8_t size, uint16_t color, int slot, const char* text, int col) noexcept
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
    // A length change or a colour change forces the whole slot to redraw. Length,
    // because a differently-shaped line (or the sentinel wipe() leaves behind) cannot
    // be diffed character-by-character. Colour, because two draws can share every
    // character at some position while wanting a different hue there -- e.g. the
    // header's "bluecan " prefix is identical whether the state word after it is
    // "SUB" or "BLE" -- and a plain character diff would then skip repainting
    // characters that are still showing the previous call's colour, leaving the line
    // part one hue and part the other.
    bool whole = strlen(cache) != len || _cache_color[slot] != color;

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
        tft.setCursor(static_cast<int16_t>((col + i) * 6 * size), y);
        tft.write(reinterpret_cast<const uint8_t*>(text) + i, j - i);
        _dirty += j - i;

        i = j;
    }

    memcpy(cache, text, len);
    cache[len] = '\0';
    _cache_color[slot] = color;
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

void display::seg(int y, int& col, uint16_t color, const char* fmt, ...) noexcept
{
    char buf[cache_len];

    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    int slot = _slot++;
    field(y, 1, color, slot, buf, col);
    col += static_cast<int>(strlen(buf));
}

void display::seg_fill(int y, int& col, uint16_t color, const char* fmt, ...) noexcept
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

    int avail = cols - col;
    if (avail < 0)
    {
        avail = 0;
    }
    if (avail > static_cast<int>(sizeof(buf)) - 1)
    {
        avail = static_cast<int>(sizeof(buf)) - 1;
    }

    // Same reasoning as row()'s padding: this is the last segment on the row, so it
    // has to erase whatever a longer previous value left behind.
    for (int i = (n < avail ? n : avail); i < avail; i++)
    {
        buf[i] = ' ';
    }
    buf[avail] = '\0';

    int slot = _slot++;
    field(y, 1, color, slot, buf, col);
    col += avail;
}

void display::wipe() noexcept
{
    tft.fillRect(0, content_y - 2, screen_w, screen_h - (content_y - 2), ST77XX_BLACK);

    // Blank the caches too, or the next pass compares against text that is no longer
    // on the screen, decides nothing changed, and leaves the area empty. The colour
    // cache is reset alongside it for the same reason field() checks it at all: a
    // sentinel colour no real call ever uses forces the first post-wipe draw to be
    // whole, regardless of what text or hue happened to be cached there before.
    for (int i = 1; i < cache_slots; i++)
    {
        _cache[i][0] = '\x01';
        _cache[i][1] = '\0';
        _cache_color[i] = ST77XX_BLACK;
    }
}

void display::refresh() noexcept
{
    _slot = slot_dynamic_base;

    // Header: the one line that is the same on every page, because it answers the
    // question asked most often -- is the phone actually getting anything.
    //
    // Three states, not two. A phone that is connected but has not subscribed to the
    // frame characteristic looks exactly like a working link on the phone, in the
    // advertising name, and in every other counter here, and receives nothing. It is
    // the failure worth spending a colour on.
    //
    // Drawn as two segments rather than one string: "bluecan" is the wordmark and
    // stays a fixed navy regardless of state, while the state word after it carries
    // the status colour. One field() call for the whole line would work too, but only
    // because field() now forces a whole redraw on any colour change -- see the
    // comment there. Splitting it keeps the wordmark's slot static across every
    // repaint, which is one fewer redraw of glass that never changes.
    bool up = RCDEV.connected();
    bool sub = RCDEV.subscribed();
    const char* state = !up ? "adv" : sub ? "SUB" : "BLE";
    uint16_t state_c = !up ? ST77XX_YELLOW : sub ? ST77XX_GREEN : ST77XX_RED;

    // Connection interval, in tenths of a millisecond: the link reports it in 1.25 ms
    // units, and it is the ceiling on everything downstream, since the radio only
    // speaks at connection events. 7.5 ms is as fast as BLE goes; 30 ms is a phone
    // that will not keep up with the bus whatever the firmware does.
    unsigned long tenths = RCDEV.interval() * 125UL / 10UL;
    char state_buf[cache_len];
    int n;
    if (up && tenths != 0UL)
    {
        n = snprintf(state_buf, sizeof(state_buf), " %s %lu.%lums",
                     state, tenths / 10UL, tenths % 10UL);
    }
    else
    {
        n = snprintf(state_buf, sizeof(state_buf), " %s", state);
    }

    if (n < 0)
    {
        n = 0;
    }

    // "bluecan" is 7 columns wide at text size 2; the state segment starts right
    // after it and is padded out to the header's full head_cols width so a shorter
    // state word cannot leave the tail of a longer one on the glass.
    constexpr int wordmark_cols = 7;
    for (; n < head_cols - wordmark_cols && n < static_cast<int>(sizeof(state_buf)) - 1; n++)
    {
        state_buf[n] = ' ';
    }
    state_buf[n] = '\0';

    field(0, 2, color_blue_dim, _slot++, "bluecan", 0);
    field(0, 2, state_c, _slot++, state_buf, wordmark_cols);

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

    // Every row below is a sequence of seg()/seg_fill() calls rather than one row()
    // call: labels (rx, fwd, flt, ...) are always yellow, values are white unless
    // they are the one number on the row worth a warning colour, and the last call on
    // each row is seg_fill() so a value that shrinks cannot leave a longer previous
    // one's tail on the glass. See the slot_dynamic_base comment in display.hpp for
    // why none of these calls need an explicit slot number.
    //
    // Each row's leading label is printed "%-6s": 6 is one past "queue", the longest
    // of them, so every row's first value starts at the same column regardless of its
    // own label's length. Without this, rx/fwd (2- and 3-letter labels, a %9lu field)
    // and ble (a 3-letter label that used a narrower %8lu field) put their leading
    // digit one column apart, which is what made the BLE row's value look shifted.
    constexpr int label_w = 6;
    int col = 0;
    int y = content_y + 0 * row_h;
    seg_fill(y, col, state_color, "CAN 500k LISTEN-ONLY %s", state);

    col = 0;
    y = content_y + 1 * row_h;
    seg(y, col, ST77XX_YELLOW, "%-*s", label_w, "rx");
    seg_fill(y, col, ST77XX_WHITE, "%9lu %5lu/s", (unsigned long)c.frames, (unsigned long)_rx_rate);

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
    uint16_t flt_color = flt_over ? ST77XX_RED : flt == 0 ? ST77XX_YELLOW : ST77XX_WHITE;

    col = 0;
    y = content_y + 2 * row_h;
    seg(y, col, ST77XX_YELLOW, "%-*s", label_w, "fwd");
    seg(y, col, ST77XX_WHITE, "%9lu %5lu/s", (unsigned long)c.forwarded, (unsigned long)_fwd_rate);
    seg(y, col, ST77XX_YELLOW, " flt");
    seg_fill(y, col, flt_color, " %s", flt_txt);

    // lost is the BLE half of the drop counter two lines down, and it exists for the
    // same reason: BLECharacteristic::notify() returns void and logs a refusal at a
    // level a release build never prints, so a link too slow for the bus would
    // otherwise lose frames in complete silence.
    unsigned long ble_lost = RCDEV.lost();
    // peers should never exceed one. The frame path notifies once per subscriber, so a
    // second connection doubles the BLE traffic for the same bus while every other
    // counter here keeps looking healthy.
    unsigned long peers = RCDEV.peers();
    // This row stays the wordmark's light blue rather than yellow/white, on request --
    // it is the BLE identity line, not a generic counter row -- but a real fault
    // (a dropped notify, or a second subscriber) still overrides it with red, and a
    // link that is not even connected still shows yellow, same as before the split.
    bool connected = RCDEV.connected();

    col = 0;
    y = content_y + 3 * row_h;
    seg(y, col, ST77XX_CYAN, "%-*s", label_w, "ble");
    // Same %9lu/%5lu widths as rx/fwd above -- this used to be %8lu/%4lu, one column
    // narrower, which is exactly what put this row's own value one column to the left
    // of theirs.
    seg(y, col, connected ? ST77XX_CYAN : ST77XX_YELLOW,
        "%9lu %5lu/s", (unsigned long)RCDEV.frames(), (unsigned long)_ble_rate);
    seg(y, col, ST77XX_CYAN, " lost");
    seg(y, col, ble_lost ? ST77XX_RED : ST77XX_CYAN, " %lu", ble_lost);
    seg(y, col, ST77XX_CYAN, " p");
    seg_fill(y, col, peers > 1 ? ST77XX_RED : ST77XX_CYAN, "%lu", peers);

    col = 0;
    y = content_y + 4 * row_h;
    seg(y, col, ST77XX_YELLOW, "%-*s", label_w, "queue");
    seg(y, col, c.queued > (c.capacity / 2) ? ST77XX_YELLOW : ST77XX_WHITE, "%4lu", (unsigned long)c.queued);
    seg(y, col, ST77XX_YELLOW, " peak");
    seg_fill(y, col, ST77XX_WHITE, " %4lu/%lu", (unsigned long)c.peak, (unsigned long)c.capacity);

    // The line that matters most and is easiest to miss. A dropped frame is silent
    // everywhere else: the bus shows no error, because the loss was ours.
    uint16_t drop_color = c.dropped ? ST77XX_RED : ST77XX_GREEN;

    col = 0;
    y = content_y + 5 * row_h;
    seg(y, col, ST77XX_YELLOW, "%-*s", label_w, "drop");
    seg(y, col, drop_color, "%lu", (unsigned long)c.dropped);
    seg(y, col, ST77XX_YELLOW, "  err");
    seg_fill(y, col, drop_color, " %lu", (unsigned long)c.errors);

    // Frame classes the ISR throws away. A sniffer cannot claim a class is empty by
    // discarding it, so they are counted and shown -- every mapped R9 id is 11-bit,
    // and this is what would say otherwise.
    uint16_t anomaly_color = (c.extended || c.remote) ? ST77XX_YELLOW : ST77XX_WHITE;

    col = 0;
    y = content_y + 6 * row_h;
    seg(y, col, ST77XX_YELLOW, "%-*s", label_w, "ext");
    seg(y, col, anomaly_color, "%lu", (unsigned long)c.extended);
    seg(y, col, ST77XX_YELLOW, " rtr");
    seg(y, col, anomaly_color, " %lu", (unsigned long)c.remote);
    seg(y, col, ST77XX_YELLOW, " tec");
    seg(y, col, ST77XX_WHITE, " %lu", (unsigned long)(running ? CANCTLR.tec() : 0U));
    seg(y, col, ST77XX_YELLOW, " rec");
    seg_fill(y, col, ST77XX_WHITE, " %lu", (unsigned long)(running ? CANCTLR.rec() : 0U));

    UBaseType_t stack = _handle ? uxTaskGetStackHighWaterMark(_handle) : 0;

    // Two spaces before "min" and "stk" rather than one -- this is the busiest row on
    // the page (three label/value pairs in 40 columns) and the tighter spacing read
    // as a run-on: "min 147k303k 1872" rather than three distinguishable fields.
    col = 0;
    y = content_y + 7 * row_h;
    seg(y, col, ST77XX_YELLOW, "%-*s", label_w, "heap");
    seg(y, col, ST77XX_WHITE, "%luk", (unsigned long)(ESP.getFreeHeap() / 1024));
    seg(y, col, ST77XX_YELLOW, "  min");
    seg(y, col, ST77XX_WHITE, " %luk", (unsigned long)(_min_heap / 1024));
    seg(y, col, ST77XX_YELLOW, "  stk");
    seg_fill(y, col, ST77XX_WHITE, " %lu", (unsigned long)stack);

    // What a repaint costs, on the glass it costs it on. A full repaint of this panel
    // is ~198 ms and a change-detected one ~160 us; if max ever climbs back towards
    // the former, the change detection has stopped working. Shown in milliseconds --
    // microsecond precision is noise nobody standing over the board can read.
    unsigned long draw_shown_ds = (unsigned long)_draw_us_shown / 100UL;
    unsigned long draw_max_ds = (unsigned long)_draw_us_max / 100UL;
    // mm:ss, not hhh:mm:ss -- this row has no room left for hours once draw and max
    // are spelled out in full, and a ride is over long before uptime needs a third
    // field anyway.
    unsigned long up_s = millis() / 1000UL;
    unsigned long up_m = up_s / 60UL;
    unsigned long up_sec = up_s % 60UL;

    col = 0;
    y = content_y + 8 * row_h;
    seg(y, col, ST77XX_YELLOW, "%-*s", label_w, "draw");
    seg(y, col, ST77XX_WHITE, "%lu.%lums", draw_shown_ds / 10UL, draw_shown_ds % 10UL);
    seg(y, col, ST77XX_YELLOW, " max");
    seg(y, col, ST77XX_WHITE, " %lu.%lums", draw_max_ds / 10UL, draw_max_ds % 10UL);
    seg(y, col, ST77XX_YELLOW, " up");
    seg_fill(y, col, ST77XX_WHITE, " %lu:%02lu", up_m, up_sec);
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
    , _led_state(0xFFFFFFFFU)
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
    , _cache_color{}
    , _slot(0)
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
#if defined(DEBUG)
void display::report_status_led() noexcept {}
#endif
void display::field(int, uint8_t, uint16_t, int, const char*, int) noexcept {}
void display::row(int, uint16_t, const char*, ...) noexcept {}
void display::seg(int, int&, uint16_t, const char*, ...) noexcept {}
void display::seg_fill(int, int&, uint16_t, const char*, ...) noexcept {}
void display::wipe() noexcept {}
void display::page_bus() noexcept {}
void display::page_ids() noexcept {}
void display::page_last() noexcept {}

} // namespace ui

#endif // CONFIG_DISPLAY_TFT

ui::display& DISP = ui::display::get();
