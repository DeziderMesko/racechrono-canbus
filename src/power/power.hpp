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

namespace power
{

/**
 * The board's MAX17048 fuel gauge, on the shared I2C bus at 0x36.
 *
 * What it actually measures is the **BAT rail** -- the 2-pin JST jack and the
 * charger's output, which are the same node. That is the battery's voltage when one
 * is fitted, and the charger's own output when one is not. It is never the 5 V the
 * bike's USB outlet supplies: this board brings VBUS out to a pad and to nothing
 * else, so no ADC on it can see that rail without a divider soldered on.
 *
 * The charge rate stands in for it instead, and answers the question the rider
 * actually has. Fed by the outlet, a cell charges; the moment the ignition takes the
 * outlet with it, the same cell starts discharging, and the sign flips within a
 * sample or two. So "+" means the bike is still feeding the device and "-" means it
 * is running on its own battery, whatever the voltage next to it says.
 *
 * **All of which needs a cell to be true**, and the gauge cannot tell you whether one
 * is fitted -- there is no presence bit, and with the jack empty it happily models the
 * charger's rail as if it were a battery. Measured on the bench with no cell fitted:
 * 4.10 V, 100.0% charged, discharging at 22%/hr, and the voltage not moving while it
 * said so. The rate is the dangerous one: at face value it reads "running on battery"
 * while the board sits on USB, which is backwards, and it is the configuration the
 * bike uses today.
 *
 * Two runtime tests for a missing cell were tried against the board and both failed,
 * which is why this is a compile-time `CONFIG_CELL_FITTED` and not a guess:
 *
 *   - **charge over 100%.** It reads 101.9% in the first seconds after power-up and
 *     then settles to 99.8-100.0%, so the window it detects anything in is the one
 *     window nobody is reading the panel in.
 *   - **a jittering voltage.** The first run moved 30 mV a sample and looked like a
 *     tell; the second sat at 4.097-4.098 V for its whole length. It was the rail
 *     settling, not a property of an empty jack.
 *
 * millivolts() is trustworthy in every configuration, which is why it is the one
 * thing the panel shows unconditionally.
 *
 * Display task only, like every other peripheral on core 0 -- see ui::display. One
 * sample is three 2-byte reads, ~150 us at 400 kHz, taken on the same 1 Hz tick as
 * the rates.
 */
class gauge final
{
    CPP_NOCOPY(gauge);
    CPP_NOMOVE(gauge);

public:
    static gauge& get() noexcept;

    ~gauge() noexcept = default;

    /**
     * Bring up the I2C bus and look for the gauge.
     *
     * Must be called after TFT_I2C_POWER is driven high: that pin gates the same
     * 3.3 V rail the STEMMA QT bus and this chip sit on, and a gauge with no power
     * does not answer. The display task drives it high before it calls this, which
     * is the only reason this is safe to call at all.
     *
     * Returns whether the gauge answered. A board with no battery still has the
     * chip, so a false here means the bus, not the battery.
     */
    bool begin() noexcept;

    /**
     * Re-read voltage, charge and rate. Cheap, blocking, and I2C: display task only.
     *
     * A read that fails leaves the last good values in place and only gives up on
     * the gauge after several consecutive failures -- one NAK on a bus shared with
     * whatever is hanging off the QT connector should not blank the readout.
     */
    void sample() noexcept;

    /// whether the last few samples reached the gauge
    bool present() const noexcept { return _present; }

    /// BAT rail, in millivolts
    uint16_t millivolts() const noexcept { return _mv; }

    /// state of charge, in tenths of a percent
    uint16_t soc_tenths() const noexcept { return _soc; }

    /// charge (positive) or discharge (negative) rate, in tenths of a percent per
    /// hour. Zero until the gauge has watched the cell for a few minutes.
    int16_t rate_tenths() const noexcept { return _rate; }

    /**
     * Whether the charge and the rate are about a cell rather than about a charger's
     * rail -- which is to say, whether CONFIG_CELL_FITTED is set and the gauge is
     * answering. Nothing is inferred here; see the note above the class for the two
     * inferences that were tried and what the board did to them.
     */
    bool cell() const noexcept
    {
#if defined(CONFIG_CELL_FITTED)
        return _present;
#else
        return false;
#endif
    }

    /// '+' charging, '-' discharging, ' ' neither, no cell fitted, or no gauge. The
    /// one character on the panel that says whether the bike is still feeding the
    /// device -- and blank rather than wrong when there is no cell to ask about.
    char flow() const noexcept;

private:
    explicit gauge() noexcept;

    bool read16(uint8_t reg, uint16_t& out) noexcept;
    bool write16(uint8_t reg, uint16_t val) noexcept;

private:
    bool _present;
    uint16_t _mv;
    uint16_t _soc;
    int16_t _rate;
    /// consecutive failed samples, so one NAK does not blank a good reading
    uint8_t _misses;
};

} // namespace power

extern power::gauge& PWR;
