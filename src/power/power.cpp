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

#include "power.hpp"

#if defined(CONFIG_FUEL_GAUGE)
#include <Wire.h>
#endif

power::gauge& PWR = power::gauge::get();

namespace power
{

gauge& gauge::get() noexcept
{
    static gauge instance;
    return instance;
}

gauge::gauge() noexcept
    : _present(false)
    , _mv(0U)
    , _soc(0U)
    , _rate(0)
    , _misses(0U)
{
}

char gauge::flow() const noexcept
{
    // cell(), not present(): a gauge with an empty jack reports a confident
    // discharge rate for a rail that is not going anywhere, and on this board that
    // is the normal configuration rather than an odd one.
    if (!cell())
    {
        return ' ';
    }

    // A tenth of a percent an hour either way is the gauge idling, not a direction.
    // The real thing is nowhere near this small: a 500 mAh cell charging at even a
    // tenth of a C moves several thousand tenths per hour.
    if (_rate > 5)
    {
        return '+';
    }
    if (_rate < -5)
    {
        return '-';
    }
    return ' ';
}

#if defined(CONFIG_FUEL_GAUGE)

namespace
{

/// MAX17048, at the address Adafruit wired it to on this board
constexpr uint8_t gauge_addr = 0x36;

/// cell voltage, 78.125 uV per LSB
constexpr uint8_t reg_vcell = 0x02;
/// state of charge, 1/256 % per LSB
constexpr uint8_t reg_soc = 0x04;
/// silicon version -- read only to see whether anything answers
constexpr uint8_t reg_version = 0x08;
/// hibernate thresholds
constexpr uint8_t reg_hibrt = 0x0A;
/// charge/discharge rate, 0.208 %/hr per LSB, signed
constexpr uint8_t reg_crate = 0x16;

/// how many samples in a row have to fail before the readout admits it has nothing
constexpr uint8_t miss_limit = 3;

/// how long a transaction may hold the display task before the driver gives up. The
/// core's default is 50 ms, which is a quarter of the BUS page's repaint interval
/// spent on core 0 waiting for a chip that is not going to answer.
constexpr uint16_t bus_timeout_ms = 10;

} // namespace

bool gauge::read16(uint8_t reg, uint16_t& out) noexcept
{
    Wire.beginTransmission(gauge_addr);
    Wire.write(reg);
    // Repeated start rather than a stop: the datasheet's own read sequence, and it
    // keeps the two halves of one register read from being split by anything else
    // that ever shares this bus.
    if (Wire.endTransmission(false) != 0)
    {
        return false;
    }

    if (Wire.requestFrom(gauge_addr, static_cast<uint8_t>(2)) != 2)
    {
        return false;
    }

    // Two statements, not one expression: the order of two Wire.read() calls either
    // side of a shift is unspecified, and getting it backwards would read as a
    // plausible-looking voltage rather than as an obvious fault.
    const uint8_t hi = static_cast<uint8_t>(Wire.read());
    const uint8_t lo = static_cast<uint8_t>(Wire.read());

    out = static_cast<uint16_t>((static_cast<uint16_t>(hi) << 8) | lo);
    return true;
}

bool gauge::write16(uint8_t reg, uint16_t val) noexcept
{
    Wire.beginTransmission(gauge_addr);
    Wire.write(reg);
    Wire.write(static_cast<uint8_t>(val >> 8));
    Wire.write(static_cast<uint8_t>(val & 0xFFU));
    return Wire.endTransmission() == 0;
}

bool gauge::begin() noexcept
{
    Wire.begin();
    Wire.setClock(400000);
    Wire.setTimeOut(bus_timeout_ms);

    uint16_t version = 0U;
    if (!read16(reg_version, version))
    {
        _present = false;
        _misses = miss_limit;
        errorln("ERROR: no fuel gauge at I2C 0x%02X", gauge_addr);
        return false;
    }

    _present = true;
    _misses = 0U;

    // Hibernate off. Left on, the gauge drops to one ADC conversion every 45 s
    // whenever the cell is quiet, which is exactly the condition under which someone
    // is standing over the board wondering why the voltage on the glass is frozen.
    // It costs about 20 uA against a board drawing tens of milliamps.
    write16(reg_hibrt, 0x0000U);

    infoln("       Fuel gauge: MAX17048 rev 0x%04X at 0x%02X",
           static_cast<unsigned>(version), gauge_addr);

    sample();
    return true;
}

void gauge::sample() noexcept
{
    uint16_t vcell = 0U;
    uint16_t soc = 0U;
    uint16_t crate = 0U;

    if (!read16(reg_vcell, vcell) || !read16(reg_soc, soc) || !read16(reg_crate, crate))
    {
        if (_misses < miss_limit)
        {
            _misses++;
        }
        if (_misses >= miss_limit)
        {
            _present = false;
        }
        return;
    }

    _misses = 0U;
    _present = true;

    // 78.125 uV is exactly 5/64 mV, so this is integer arithmetic with nothing
    // rounded away and no float pulled into a build that has none anywhere else.
    _mv = static_cast<uint16_t>((static_cast<uint32_t>(vcell) * 5U) >> 6);
    // 1/256 % per LSB, kept in tenths
    _soc = static_cast<uint16_t>((static_cast<uint32_t>(soc) * 10U) >> 8);
    // 0.208 %/hr per LSB, signed, kept in tenths
    // Clamped before it narrows: 0.208 %/hr per LSB times a full-scale register is
    // ~6800 %/hr, which does not fit an int16_t of tenths, and the wrap turns a
    // maximum discharge into a confident-looking charge. Real rates are three orders
    // of magnitude below this, so the clamp only ever fires on a garbage read -- which
    // is exactly when a sign that lies is worst.
    int32_t rate = (static_cast<int32_t>(static_cast<int16_t>(crate)) * 208) / 100;
    if (rate > INT16_MAX) { rate = INT16_MAX; }
    if (rate < INT16_MIN) { rate = INT16_MIN; }
    _rate = static_cast<int16_t>(rate);
}

#else // !CONFIG_FUEL_GAUGE

// Boards without the gauge compile and run; they simply have nothing to report, and
// present() stays false, which is what the panel draws as "n/a".

bool gauge::begin() noexcept { return false; }
void gauge::sample() noexcept {}
bool gauge::read16(uint8_t, uint16_t&) noexcept { return false; }
bool gauge::write16(uint8_t, uint16_t) noexcept { return false; }

#endif // CONFIG_FUEL_GAUGE

} // namespace power
