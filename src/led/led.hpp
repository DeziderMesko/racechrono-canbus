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

#include <driver/ledc.h>

namespace led
{

#if defined(CONFIG_STATUS_LED)
/// Full brightness on a WS2812 is a torch in a mirror, and this one sits in the
/// rider's field of view by definition. Daylight sets the floor rather than night --
/// the display's third light position turns the pixel off entirely -- so this is ~9%
/// of full scale, and every colour below is scaled by it.
#ifndef CONFIG_STATUS_LED_LEVEL
#define CONFIG_STATUS_LED_LEVEL 24
#endif
#endif

/**
 * Basic LED controller for the various ESP32 boards.
 */
class led final
{
    CPP_NOCOPY(led);
    CPP_NOMOVE(led);

public:
    /**
     * Get LED instance (singleton).
     */
    static led& get() noexcept;

    ~led() noexcept = default;

    /**
     * Turn on built-in LED
     */
    void builtin_on() noexcept;

    /**
     * Turn off built-in LED
     */
    void builtin_off() noexcept;

#if defined(CONFIG_STATUS_LED)
    /**
     * Power the status pixel and leave it dark.
     *
     * Called once, from the display task, before the first status() -- the RMT
     * channel behind it allocates on first use, and doing that at start-up keeps the
     * free-heap baseline honest for anything measured later.
     */
    void status_begin() noexcept;

    /**
     * Write the status pixel, at status_level scale.
     *
     * Takes full-scale 0-255 components so callers can think in colours and let this
     * do the dimming. A write whose result matches what is already on the pixel is
     * dropped, so an animation that is between steps costs a comparison rather than
     * an RMT transaction.
     *
     * Display task only: one WS2812 is 24 bits at 800 kHz, and rmtWrite() blocks for
     * the ~30 us it takes. That is nothing next to a 280 us idle repaint and it would
     * be the whole budget inside the CAN ISR.
     */
    void status(uint8_t r, uint8_t g, uint8_t b) noexcept;

    /**
     * Dark, and the power rail off with it -- no quiescent draw, which is what makes
     * the display's third light position a real off rather than a black pixel.
     */
    void status_end() noexcept;
#endif

private:
    explicit led() noexcept;

private:
    ledc_timer_config_t _timer_config;
    ledc_channel_config_t _channel_config;

#if defined(CONFIG_STATUS_LED)
    /// last colour written, packed 0x00RRGGBB, or 0xFFFFFFFF for "unknown"
    uint32_t _status_last;
    /// whether NEOPIXEL_POWER is currently on
    bool _status_powered;
#endif
};

} // namespace led

extern led::led& LED;
