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
#include "../logging/logging.hpp"
#include "../utils/timer.hpp"

#include "frame.hpp"

#include <atomic>

namespace canbus
{

/**
 * CAN-bus controller. this is a singleton class, as there is only
 * one controller on a standard ESP32 board.
 */
class controller final
{
    CPP_NOCOPY(controller);
    CPP_NOMOVE(controller);

public:
    ~controller() noexcept = default;

    /**
     * get controller instance
     */
    static controller& get() noexcept;

    /**
     * return true is controller is initialized and running (interrupt handler installed and controller active)
     */
    __always_inline bool running() const noexcept { return _running; }

    /**
     * print any controller stats
     */
#if defined(DEBUG)
    void stats() noexcept;
#else
    void stats() noexcept {}
#endif

    /**
     * A snapshot of the ISR's counters, for anything that wants to report them
     * without owning them -- the display, primarily.
     *
     * Every count here is cumulative and monotonic, so a caller derives a rate by
     * subtracting its own previous sample. That is the reason stats() no longer
     * zeroes them: two readers cannot share a counter that either of them resets,
     * and a DEBUG build has two.
     */
    struct counters_t
    {
        uint32_t interrupts;  //!< TWAI interrupts serviced
        uint32_t errors;      //!< error/arbitration/bus-error interrupts
        uint32_t frames;      //!< frames the ISR read out of the hardware
        uint32_t forwarded;   //!< frames queued for core 0
        uint32_t dropped;     //!< frames lost to a full queue
        uint32_t peak;        //!< deepest the queue has ever been
        uint32_t extended;    //!< 29-bit frames discarded
        uint32_t remote;      //!< remote-transmission requests discarded
        uint32_t queued;      //!< queue depth right now
        uint32_t capacity;    //!< queue depth it would take to drop a frame
    };

    /**
     * read the counters. Not const: the queue depth comes from the queue itself.
     */
    counters_t counters() noexcept;

    /**
     * controller status register (TWAI_LL_STATUS_* bits)
     */
    uint32_t status() const noexcept;

    /**
     * transmit and receive error counters. Both are frozen in listen-only mode,
     * where the controller neither transmits nor acknowledges, so they are reported
     * as context rather than read as a health signal -- status() is the health signal.
     */
    uint32_t tec() const noexcept;
    uint32_t rec() const noexcept;

    /**
     * install controller driver
     */
    bool install() noexcept;

    /**
     * uninstall controller driver
     */
    bool uninstall() noexcept;

    /**
     * start controller
     */
    bool start() noexcept;

    /**
     * stop controller
     */
    bool stop() noexcept;

    /**
     * receive a frame from the internal buffer
     *
     * @param wait ticks to block for when the buffer is empty. 0 polls, which is
     *        what a task at tskIDLE_PRIORITY can afford because it time-shares the
     *        core with the idle task anyway. Anything above idle priority must pass
     *        a wait, or the idle task never runs and
     *        CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0 turns that into a panic reset.
     */
    bool recv(frame& f, TickType_t wait = 0) noexcept;

private:
    explicit controller() noexcept;

    /**
     * interrupt service handler
     */
    static void IRAM_ATTR isr(void* arg);

    /**
     * interrupt service handler
     */
    void isr() noexcept;

    __always_inline void ENTER_CRITICAL() noexcept { portENTER_CRITICAL(&_lock); }
    __always_inline void EXIT_CRITICAL() noexcept { portEXIT_CRITICAL(&_lock); }
    __always_inline void ENTER_CRITICAL_ISR() noexcept { portENTER_CRITICAL_ISR(&_lock); }
    __always_inline void EXIT_CRITICAL_ISR() noexcept { portEXIT_CRITICAL_ISR(&_lock); }

private:
    portMUX_TYPE _lock;
    bool _running;
    QueueHandle_t _queue;
    utils::timer _stats_timer;
    std::atomic<uint32_t> _ir_count;
    std::atomic<uint32_t> _er_count;
    std::atomic<uint32_t> _cb_count;
    std::atomic<uint32_t> _rc_count;
    std::atomic<uint32_t> _dr_count;
    std::atomic<uint32_t> _hw_count;
    /// frame classes the ISR discards without queueing. Cumulative like the drop
    /// counter, because the useful reading is "none seen at all" and an average
    /// would bury a handful of them.
    std::atomic<uint32_t> _ef_count;
    std::atomic<uint32_t> _rt_count;
    /// what stats() saw last time, so it can print a delta without zeroing a
    /// counter the display is also reading
    uint32_t _ir_last;
    uint32_t _er_last;
    uint32_t _cb_last;
    uint32_t _rc_last;
    intr_handle_t _isr_handle;
    /// frames the queue holds while core 0 is not draining it. At the R9's ~1050
    /// msg/s that is ~1.9 s of bus, and ~0.5 s of a saturated 500 kbps one. Costs
    /// 2000 * 13 bytes of DRAM. Upstream's 8 was ~8 ms of R9 traffic: any BLE stall
    /// longer than that dropped frames, silently.
    static constexpr uint32_t _queue_length = 2000;
    static constexpr uint32_t _queue_item_size = sizeof(canbus::frame);
    uint8_t _queue_storage[_queue_length * _queue_item_size];
    StaticQueue_t _static_queue;
};

} // namespace canbus

extern canbus::controller& CANCTLR;
