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
#include "../racechrono-canbus.hpp"

#include "decoder.hpp"
#include "frame.hpp"

#include <esp_intr_alloc.h>
#include <esp_rom_gpio.h>
#include <hal/twai_ll.h>
#include <driver/periph_ctrl.h>

#include "controller.hpp"

/*
 * ESP32 CAN controller:
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/twai.html
 * https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf#twai
 *
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/twai.html
 * https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf#twai
 */

namespace
{

twai_dev_t* dev = &TWAI;

}

namespace canbus
{

controller::controller() noexcept
    : _lock(portMUX_INITIALIZER_UNLOCKED)
    , _running(false)
    , _queue(nullptr)
    , _stats_timer{}
    , _ir_count(0U)
    , _er_count(0U)
    , _cb_count(0U)
    , _rc_count(0U)
    , _dr_count(0U)
    , _hw_count(0U)
    , _isr_handle(nullptr)
    , _queue_storage{}
    , _static_queue{}
{
}

controller& controller::get() noexcept
{
    static controller instance;
    return instance;
}

#if defined(DEBUG)
void controller::stats() noexcept
{
    if (logging::logger::get().level() >= logging::log_level::info)
    {
        unsigned long delta = _stats_timer.elapsed(CONFIG_RC_STATS_TIMEOUT);

        if (delta > 0UL)
        {
            uint32_t ir_count = _ir_count.exchange(0UL, std::memory_order_relaxed);
            uint32_t er_count = _er_count.exchange(0UL, std::memory_order_relaxed);
            uint32_t cb_count = _cb_count.exchange(0UL, std::memory_order_relaxed);
            uint32_t rc_count = _rc_count.exchange(0UL, std::memory_order_relaxed);
            uint32_t waiting = uxQueueMessagesWaiting(_queue);
            uint32_t available = uxQueueSpacesAvailable(_queue);
            // these two are cumulative on purpose: a single dropped frame matters,
            // and the peak is what says whether the queue is big enough
            uint32_t dr_count = _dr_count.load(std::memory_order_relaxed);
            uint32_t hw_count = _hw_count.load(std::memory_order_relaxed);

            infoln("       Interrupts/s: %.2f", (static_cast<float>(ir_count) / static_cast<float>(delta)) * 1e6f);
            infoln("           Errors/s: %.2f", (static_cast<float>(er_count) / static_cast<float>(delta)) * 1e6f);
            infoln("      CAN bus msg/s: %.2f", (static_cast<float>(cb_count) / static_cast<float>(delta)) * 1e6f);
            infoln("   RaceChrono msg/s: %.2f", (static_cast<float>(rc_count) / static_cast<float>(delta)) * 1e6f);
            infoln("              Queue: %4u / %4u", waiting, available);
            infoln("         Queue peak: %4u of %4u", hw_count, _queue_length);
            infoln("     Dropped frames: %4u", dr_count);
        }
    }
}
#endif

bool controller::install() noexcept
{
    bootln("CAN bus starting...");

    ENTER_CRITICAL();

    bootln("CAN bus creating frame queue...");
    _queue = xQueueCreateStatic(_queue_length, _queue_item_size, _queue_storage, &_static_queue);
    bootln("CAN bus frame queue created...");

    // enable APB CLK to TWAI peripheral
    periph_module_reset(PERIPH_TWAI_MODULE);
    periph_module_enable(PERIPH_TWAI_MODULE);
    bootln("CAN bus peripheral enabled...");

    twai_ll_enter_reset_mode(dev);
    if (!twai_ll_is_in_reset_mode(dev))
    {
        EXIT_CRITICAL();
        return false;
    }
#if SOC_TWAI_SUPPORT_MULTI_ADDRESS_LAYOUT
    twai_ll_enable_extended_reg_layout(dev);
#endif
    // Listen-only. The controller is electrically passive: it never drives the bus,
    // never ACKs a frame and never sends an error frame, so a wrong bit rate or a
    // wiring mistake cannot disturb the bike's network. It also freezes REC.
    // ESP32 Arduino core 3.x takes the mode as three flags instead of a twai_mode_t.
    twai_ll_set_mode(dev, true /* listen_only */, false /* no_ack */, false /* loopback */);
    // reset RX and TX error counters
    twai_ll_set_rec(dev, 0);
    twai_ll_set_tec(dev, 0);
    twai_ll_set_err_warn_lim(dev, 96);

    bootln("CAN bus mode reset...");

    // configure bus timing, acceptance filter, CLKOUT, and interrupts
    // get timing and filter from car specific decoder
    twai_timing_config_t t_config = CANDEC.timing();
    twai_filter_config_t f_config = CANDEC.filter();

    // Core 3.x's TWAI_TIMING_CONFIG_*() macros no longer carry a prescaler. They
    // state the bit rate as quanta_resolution_hz and leave brp at 0, expecting the
    // twai_* driver to divide the peripheral clock down to it. Nothing does that on
    // this path, and a brp of 0 reaches the register as a nonsense divider: the
    // controller then samples at the wrong bit rate, receives nothing, and raises a
    // single bus error. So derive the prescaler here.
    uint32_t brp = t_config.brp;
    if (brp == 0 && t_config.quanta_resolution_hz != 0)
    {
        brp = getApbFrequency() / t_config.quanta_resolution_hz;
    }

    twai_ll_set_bus_timing(dev, brp, t_config.sjw, t_config.tseg_1, t_config.tseg_2, t_config.triple_sampling);
    twai_ll_set_acc_filter(dev, f_config.acceptance_code, f_config.acceptance_mask, f_config.single_filter);
    twai_ll_set_clkout(dev, 0);
    // enable interrupts
    // disable tx interrupts, as we are listen-only
    // disable data overrun and wakeup interrupts (both have issues on ESP32)
    twai_ll_set_enabled_intrs(dev, 0xA7); //0xE7);
    (void) twai_ll_get_and_clear_intrs(dev);    // clear any latched interrupts

    EXIT_CRITICAL();

    bootln("CAN bus timings reset...");
    bootln("     APB clock: %3u MHz", getApbFrequency() / 1000000);
    bootln("        Quanta: %3u MHz", t_config.quanta_resolution_hz / 1000000);
    bootln("          BRP: %3u", brp);
    bootln("          SJW: %3u", t_config.sjw);
    bootln("        TSEG1: %3u", t_config.tseg_1);
    bootln("        TSEG2: %3u", t_config.tseg_2);
    bootln("  3x Sampling: %3s", t_config.triple_sampling == 0 ? "No" : "Yes");

    // Only the RX pin is set up. TX is left unconnected to the peripheral, so the
    // pin cannot be driven at all - a second, physical layer of listen-only.
    gpio_set_pull_mode(CAN_RX_PIN, GPIO_FLOATING);
    esp_rom_gpio_connect_in_signal(CAN_RX_PIN, TWAI_RX_IDX, false);
    esp_rom_gpio_pad_select_gpio(CAN_RX_PIN);
    gpio_set_direction(CAN_RX_PIN, GPIO_MODE_INPUT);
    bootln("CAN bus GPIO pins reset...");

    // setup interrupt service routine
    esp_intr_alloc(ETS_TWAI_INTR_SOURCE, ESP_INTR_FLAG_LEVEL1, isr, this, &_isr_handle);
    bootln("CAN bus interrupt handler installed...");

    return true;
}

bool controller::uninstall() noexcept
{
    ENTER_CRITICAL();
    vQueueDelete(_queue);
    EXIT_CRITICAL();
    return true;
}

bool controller::start() noexcept
{
    ENTER_CRITICAL();

    xQueueReset(_queue);

    (void) twai_ll_get_and_clear_intrs(dev);    // clear any latched interrupts
    _running = true;
    twai_ll_exit_reset_mode(dev);

    EXIT_CRITICAL();

    bootln("CAN bus started!");

    return true;
}

bool controller::stop() noexcept
{
    return true;
}

bool controller::recv(frame& f) noexcept
{
    return _queue ? xQueueReceive(_queue, &f, 0) == pdTRUE : false;
}

void IRAM_ATTR controller::isr(void* arg)
{
    static_cast<controller*>(arg)->isr();
}

void controller::isr() noexcept
{
    BaseType_t task_woken = pdFALSE;

    ENTER_CRITICAL_ISR();

    _ir_count.fetch_add(1, std::memory_order_relaxed);

    uint32_t interrupts = twai_ll_get_and_clear_intrs(dev);
    // uint32_t status = twai_ll_get_status(dev);
    // uint32_t tec = twai_ll_get_tec(dev);
    // uint32_t rec = twai_ll_get_rec(dev);

    if (interrupts & TWAI_LL_INTR_RI)
    {
        _cb_count.fetch_add(1, std::memory_order_relaxed);

        // TODO: SOC_TWAI_SUPPORTS_RX_STATUS
        uint32_t msg_count = twai_ll_get_rx_msg_count(dev);
        for (uint32_t i = 0; i < msg_count; i++)
        {
            frame f;
            f.info.u8 = dev->tx_rx_buffer[0].val;

            if (f.info.rtr == frame_rtr::remote)
            {
                twai_ll_set_cmd_release_rx_buffer(dev);
                continue;
            }

            if (f.info.frame_format == frame_format::extended)
            {
                twai_ll_set_cmd_release_rx_buffer(dev);
                continue;
            }

            f.id = (dev->tx_rx_buffer[1].val << 3) | (dev->tx_rx_buffer[2].val >> 5);

            if (!CANDEC.should_decode(f.id))
            {
                twai_ll_set_cmd_release_rx_buffer(dev);
                continue;
            }

            // copy data bytes
            for (uint8_t i = 0; i < f.info.dlc; i++)
            {
                f.data.u8[i] = dev->tx_rx_buffer[i+3].val;
            }

            // A full queue drops the frame, and upstream ignored that: the counter
            // was incremented either way, so loss looked like it never happened.
            if (xQueueSendToBackFromISR(_queue, &f, &task_woken) == pdTRUE)
            {
                _rc_count.fetch_add(1, std::memory_order_relaxed);

                uint32_t waiting = uxQueueMessagesWaitingFromISR(_queue);
                if (waiting > _hw_count.load(std::memory_order_relaxed))
                {
                    _hw_count.store(waiting, std::memory_order_relaxed);
                }
            }
            else
            {
                _dr_count.fetch_add(1, std::memory_order_relaxed);
            }

            twai_ll_set_cmd_release_rx_buffer(dev);
        }
    }
    else if (interrupts & (TWAI_LL_INTR_EI | TWAI_LL_INTR_EPI | TWAI_LL_INTR_ALI | TWAI_LL_INTR_BEI))
    {
        _er_count.fetch_add(1, std::memory_order_relaxed);
    }

    EXIT_CRITICAL_ISR();

    if (task_woken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

} // namespace canbus

canbus::controller& CANCTLR = canbus::controller::get();
