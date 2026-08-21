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
    , _lit(true)
    , _hold(false)
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
    int used = _id_used;
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
            // half-written entry: _id_used is what makes the slot visible
            _ids[used] = id;
            _hits[used] = 1;
            _hits_prev[used] = 0;
            _rates[used] = 0;
            _id_used = used + 1;
        }
        else
        {
            _id_overflow++;
        }
    }

    uint32_t pos = _ring_pos;
    _ring[pos % ring_slots] = f;
    _ring_pos = pos + 1;
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

        uint32_t interval = _page == page_bus_id ? refresh_bus_ms : refresh_slow_ms;

        if (_lit && !_hold && (now - last_paint) >= interval)
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

            infoln("      Display page: %s%s%s", name,
                   _lit ? "" : " (dark)", _hold ? " (held)" : "");
            infoln("      Display draw: %lu us, worst %lu us, %lu chars",
                   (unsigned long)_draw_us, (unsigned long)_draw_us_max,
                   (unsigned long)_dirty);
            infoln("     Display stack: %lu bytes free",
                   (unsigned long)(_handle ? uxTaskGetStackHighWaterMark(_handle) : 0));
            infoln("        Census ids: %d, %lu over",
                   _id_used, (unsigned long)_id_overflow);
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
        // Backlight off also stops the repaint, which makes this the cheapest
        // experiment on the board: everything else keeps running, so any difference
        // in the frame counters afterwards is the display's cost and nothing else.
        _lit = !_lit;
        digitalWrite(TFT_BACKLITE, _lit ? HIGH : LOW);
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

    int used = _id_used;
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
    uint32_t status = CANCTLR.status();
    bool running = CANCTLR.running();
    const char* state = running ? bus_state(status) : "DOWN";
    uint16_t state_color = !running || (status & TWAI_LL_STATUS_BS) ? ST77XX_RED
                         : (status & TWAI_LL_STATUS_ES)            ? ST77XX_YELLOW
                                                                   : ST77XX_GREEN;

    row(0, state_color, "CAN 500k LISTEN-ONLY %s", state);
    row(1, ST77XX_WHITE, "rx  %9lu %5lu/s", (unsigned long)c.frames, (unsigned long)_rx_rate);
    row(2, ST77XX_WHITE, "fwd %9lu %5lu/s", (unsigned long)c.forwarded, (unsigned long)_fwd_rate);
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
        (unsigned long)CANCTLR.tec(), (unsigned long)CANCTLR.rec());

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
    int used = _id_used;

    row(0, ST77XX_CYAN, "ids seen %d%s", used, _id_overflow ? " (+more)" : "");

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
    uint32_t pos = _ring_pos;

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
    , _lit(false)
    , _hold(false)
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
void display::field(int, uint8_t, uint16_t, int, const char*) noexcept {}
void display::row(int, uint16_t, const char*, ...) noexcept {}
void display::wipe() noexcept {}
void display::page_bus() noexcept {}
void display::page_ids() noexcept {}
void display::page_last() noexcept {}

} // namespace ui

#endif // CONFIG_DISPLAY_TFT

ui::display& DISP = ui::display::get();
