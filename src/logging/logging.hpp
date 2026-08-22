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

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <cstdio>
#include <utility>

namespace logging
{

/**
 * logging levels
 */
enum class log_level
{
    off,
    boot,
    error,
    warn,
    info,
    verbose,
    debug,
};

/**
 * logger class
 */
class logger final
{
    CPP_NOCOPY(logger);
    CPP_NOMOVE(logger);

public:
    static logger& get() noexcept;

    ~logger() noexcept;

    /**
     * get current log level
     */
    log_level level() const noexcept { return _log_level; }

    /**
     * set current log level
     */
    void set_level(log_level level) noexcept { _log_level = level; }

    /**
     * how long a log call will wait for another task to finish its line before
     * giving up and dropping this one. A logger that blocks a task for longer
     * than this is worse than a logger that loses a line.
     */
    static constexpr TickType_t lock_wait = pdMS_TO_TICKS(50);

    /**
     * log message using printf-style formatting
     */
    template <typename ...TArgs>
    void log(log_level level, const char* fmt, TArgs&& ...args) noexcept
    {
        if (level <= _log_level)
        {
            emit(false, fmt, std::forward<TArgs>(args)...);
        }
    }

    /**
     * log message using printf-style formatting and append newline
     */
    template <typename ...TArgs>
    void logln(log_level level, const char* fmt, TArgs&& ...args) noexcept
    {
        if (level <= _log_level)
        {
            emit(true, fmt, std::forward<TArgs>(args)...);
        }
    }

private:
    explicit logger() noexcept;

    /**
     * Format one message and hand it to Serial as a single write.
     *
     * Three rules hold this together, and each one replaces something that used to
     * be here:
     *
     * 1. **No critical section.** This used to wrap the write in
     *    portENTER_CRITICAL/portEXIT_CRITICAL. On this board Serial is USB CDC:
     *    USBCDC::write takes a FreeRTOS semaphore, and then spins on
     *    tud_cdc_n_write_available() until the *host* reads, for up to
     *    tx_timeout_ms. Doing that with interrupts disabled stops the USB
     *    interrupt that would drain the buffer, so the wait can only end by
     *    timing out -- 250 ms by default, against a 300 ms interrupt watchdog.
     *    It also freezes every other task on that core for the duration, and it
     *    calls into TinyUSB from a context TinyUSB does not expect.
     *
     * 2. **One write() per line**, newline included, instead of write() +
     *    println(). That is what the removed flush() was really buying: a line
     *    from one core could not be cut in half by a line from the other. Both
     *    USBCDC and HardwareSerial hold their own lock for the whole of a single
     *    write(), so a whole line goes out whole. flush() itself only waited for
     *    the host and is not needed -- write() already hands the bytes to the
     *    USB stack.
     *
     * 3. **A mutex, not a spinlock**, and one that gives up. It bounds the wait
     *    at lock_wait and lets the holder be preempted, which a spinlock with
     *    interrupts off cannot do.
     *
     * Not called from an interrupt handler: a log call inside the CAN ISR would
     * be a bug, and dropping the message is a better way to say so than the
     * assertion a blocking take would fire.
     */
    template <typename ...TArgs>
    void emit(bool newline, const char* fmt, TArgs&& ...args) noexcept
    {
        if (xPortInIsrContext())
        {
            return;
        }

        // Two bytes held back for the newline, one for the terminator snprintf
        // always writes.
        char buf[buf_size];
        constexpr size_t cap = buf_size - 2;

        int n = snprintf(buf, cap, fmt, std::forward<TArgs>(args)...);
        if (n < 0)
        {
            return;
        }

        // snprintf returns what it *would* have written. Taking that as a length
        // reads off the end of buf for any message longer than the buffer, which
        // is a stack overread on a board with no MMU to catch it.
        size_t len = static_cast<size_t>(n) < cap - 1 ? static_cast<size_t>(n) : cap - 1;

        if (newline)
        {
            buf[len++] = '\r';
            buf[len++] = '\n';
        }

        if (_lock == nullptr || xSemaphoreTake(_lock, lock_wait) != pdTRUE)
        {
            return;
        }
        Serial.write(reinterpret_cast<const uint8_t*>(buf), len);
        xSemaphoreGive(_lock);
    }

private:
    static constexpr size_t buf_size = 128;

    log_level _log_level;
    SemaphoreHandle_t _lock;
    StaticSemaphore_t _lock_buffer;
};

} // namespace logging

template <typename ...TArgs>
void boot(const char* fmt, TArgs&& ...args) noexcept
{
    logging::logger::get().log(logging::log_level::boot, fmt, std::forward<TArgs>(args)...);
}

template <typename ...TArgs>
void bootln(const char* fmt, TArgs&& ...args) noexcept
{
    logging::logger::get().logln(logging::log_level::boot, fmt, std::forward<TArgs>(args)...);
}

template <typename ...TArgs>
void error(const char* fmt, TArgs&& ...args) noexcept
{
    logging::logger::get().log(logging::log_level::error, fmt, std::forward<TArgs>(args)...);
}

template <typename ...TArgs>
void errorln(const char* fmt, TArgs&& ...args) noexcept
{
    logging::logger::get().logln(logging::log_level::error, fmt, std::forward<TArgs>(args)...);
}

template <typename ...TArgs>
void warn(const char* fmt, TArgs&& ...args) noexcept
{
    logging::logger::get().log(logging::log_level::warn, fmt, std::forward<TArgs>(args)...);
}

template <typename ...TArgs>
void warnln(const char* fmt, TArgs&& ...args) noexcept
{
    logging::logger::get().logln(logging::log_level::warn, fmt, std::forward<TArgs>(args)...);
}

#if defined(DEBUG)

template <typename ...TArgs>
void info(const char* fmt, TArgs&& ...args) noexcept
{
    logging::logger::get().log(logging::log_level::info, fmt, std::forward<TArgs>(args)...);
}

template <typename ...TArgs>
void infoln(const char* fmt, TArgs&& ...args) noexcept
{
    logging::logger::get().logln(logging::log_level::info, fmt, std::forward<TArgs>(args)...);
}

template <typename ...TArgs>
void verbose(const char* fmt, TArgs&& ...args) noexcept
{
    logging::logger::get().log(logging::log_level::verbose, fmt, std::forward<TArgs>(args)...);
}

template <typename ...TArgs>
void verboseln(const char* fmt, TArgs&& ...args) noexcept
{
    logging::logger::get().logln(logging::log_level::verbose, fmt, std::forward<TArgs>(args)...);
}

template <typename ...TArgs>
void debug(const char* fmt, TArgs&& ...args) noexcept
{
    logging::logger::get().log(logging::log_level::debug, fmt, std::forward<TArgs>(args)...);
}

template <typename ...TArgs>
void debugln(const char* fmt, TArgs&& ...args) noexcept
{
    logging::logger::get().logln(logging::log_level::debug, fmt, std::forward<TArgs>(args)...);
}

#else

#define info(_fmt, ...)
#define infoln(_fmt, ...)
#define verbose(_fmt, ...)
#define verboseln(_fmt, ...)
#define debug(_fmt, ...)
#define debugln(_fmt, ...)

#endif
