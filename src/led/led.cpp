
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
#include "led.hpp"

#include <driver/ledc.h>

led::led& LED = led::led::get();

namespace led
{

led& led::get() noexcept
{
    static led instance;
    return instance;
}

led::led() noexcept
    : _timer_config {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 5000,
        .clk_cfg          = LEDC_AUTO_CLK
    }
    , _channel_config {
        .gpio_num       = LED_BUILTIN,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    }
#if defined(CONFIG_STATUS_LED)
    , _status_last(0xFFFFFFFFU)
    , _status_powered(false)
#endif
{
    ledc_timer_config(&_timer_config);
    ledc_channel_config(&_channel_config);
}

void led::builtin_off() noexcept
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void led::builtin_on() noexcept
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 8191 /* 13-bit max */);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

#if defined(CONFIG_STATUS_LED)

namespace
{

/// scale a full-scale component down to the level the pixel is actually driven at.
/// Rounding up from any non-zero input on purpose: a colour asked for dimly and
/// delivered as black is a state the rider cannot see.
__always_inline uint8_t dim(uint8_t v) noexcept
{
    uint32_t scaled = (static_cast<uint32_t>(v) * CONFIG_STATUS_LED_LEVEL) / 255U;
    return static_cast<uint8_t>((scaled == 0U && v != 0U) ? 1U : scaled);
}

} // namespace

void led::status_begin() noexcept
{
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, NEOPIXEL_POWER_ON);
    _status_powered = true;

    // Unconditional: this is the write that allocates the RMT channel, and the
    // de-duplication below would skip it if the cached colour happened to match.
    _status_last = 0U;
    rgbLedWrite(PIN_NEOPIXEL, 0, 0, 0);
}

void led::status(uint8_t r, uint8_t g, uint8_t b) noexcept
{
    const uint8_t dr = dim(r);
    const uint8_t dg = dim(g);
    const uint8_t db = dim(b);

    const uint32_t packed = (static_cast<uint32_t>(dr) << 16)
                          | (static_cast<uint32_t>(dg) << 8)
                          | static_cast<uint32_t>(db);

    if (!_status_powered)
    {
        digitalWrite(NEOPIXEL_POWER, NEOPIXEL_POWER_ON);
        _status_powered = true;
        // The pixel's own latch does not survive its power rail, so whatever was
        // cached describes a chip that has since forgotten it.
        _status_last = 0xFFFFFFFFU;
    }

    if (packed == _status_last)
    {
        return;
    }

    _status_last = packed;
    rgbLedWrite(PIN_NEOPIXEL, dr, dg, db);
}

void led::status_end() noexcept
{
    if (!_status_powered)
    {
        return;
    }

    // Dark first, then the rail: cutting power to a lit pixel leaves it lit for as
    // long as its capacitor holds, which looks like a fault rather than an off.
    rgbLedWrite(PIN_NEOPIXEL, 0, 0, 0);
    digitalWrite(NEOPIXEL_POWER, LOW);
    _status_powered = false;
    _status_last = 0xFFFFFFFFU;
}

#endif // CONFIG_STATUS_LED

} // namespace led
