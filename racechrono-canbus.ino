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
#include "src/racechrono-canbus.hpp"
#include "src/canbus/controller.hpp"
#include "src/canbus/decoder.hpp"
#include "src/canbus/frame.hpp"
#include "src/display/display.hpp"
#include "src/led/led.hpp"
#include "src/racechrono/device.hpp"

#include <cstdint>

using namespace logging;

namespace
{

constexpr uint32_t core0_stack_size = 6 * 1024;
StaticTask_t core0_buffer;
StackType_t core0_stack[core0_stack_size];
TaskHandle_t core0_handle;
volatile bool core0_started = false;

// How long the drain task blocks on an empty frame queue before going round again to
// let stats() print, and how many frames it forwards before handing core 0 back to the
// idle task. Both exist because the task no longer runs at tskIDLE_PRIORITY: see the
// comment on the priority below.
constexpr TickType_t core0_wait = pdMS_TO_TICKS(20);
constexpr uint32_t core0_batch = 128U;

void core0(void*);

}

void setup()
{
    LED.builtin_off();

    Serial.begin(115200);

#if ARDUINO_USB_CDC_ON_BOOT
    // The core defaults this to 250 ms, and a USB CDC write blocks until the host
    // reads or the timeout expires. 250 ms of a blocked drain task is 250 ms of
    // frames piling up in a queue that holds 2000, for the sake of a debug line.
    // 20 ms is far more than a console that is actually reading ever needs, and a
    // console that is not reading should cost a truncated line, not a backlog.
    Serial.setTxTimeoutMs(20);
#endif

    delay(1500);
    Serial.print("Starting up on core ");
    Serial.println(xPortGetCoreID());
    Serial.flush();

#if defined(DEBUG)
    delay(5000);
#endif

    // set logging level
    logger::get().set_level(log_level::info);

    //
    // ATTENTION:
    //   All Bluetooth LE related activity must be pinned to core 0
    //   All CAN-bus related activity must be pinned to core 1
    //

    assert(xPortGetCoreID() == 1);

    // The panel, on its own task on core 0. Started before anything else has a
    // status worth showing, so a board that fails to bring the CAN controller up
    // says so on the glass instead of only over a serial console a release build
    // does not have.
    DISP.start();

    core0_handle = xTaskCreateStaticPinnedToCore(
        core0,
        "racechrono",
        core0_stack_size,
        nullptr,
        // Above the idle task, level with the display task rather than under it.
        //
        // At tskIDLE_PRIORITY this task round-robins with core 0's idle task, so it
        // gets about half of whatever the BLE host leaves, and the display task
        // preempts it on top of that. Level with the display costs nothing -- the
        // panel sleeps between repaints -- and stops the idle task taking every
        // second slice while frames are waiting.
        //
        // It is level with, not above, the display on purpose. Under saturation this
        // task's drain loop stops exiting, and the panel is the only instrument that
        // still reports while that lasts.
        tskIDLE_PRIORITY + 1,
        core0_stack,
        &core0_buffer,
        0
    );
    RCASSERT(core0_handle);

    // wait for core 0 to start (not strictly necessary)
    while (!core0_started) { delay(100); }

    // setup can-bus on core 1 (default), interrupt handler will be serviced on core 1
    if (CANCTLR.install())
    {
        if (CANCTLR.start())
        {
            LED.builtin_on();
        }
        else
        {
            errorln("ERROR: CAN bus controller startup failed!");
            esp_restart();
        }
    }
    else
    {
        errorln("ERROR: CAN bus driver install failed!");
        esp_restart();
    }
}

namespace
{

// core 0 - receive can frames from queue, send over bluetooth le
void core0(void*)
{
    // xQueue copies all data, so we can use a static buffer here
    static canbus::frame f;

    // start up bluetooth le connection
    if (RCDEV.start(&CANDEC))
    {
        core0_started = true;

        uint32_t drained = 0;

        while (true)
        {
            // Blocking, not polling. Above tskIDLE_PRIORITY a spin on an empty queue
            // would keep core 0's idle task off the CPU forever, and the task watchdog
            // panics after five seconds of that. The wait only applies when there is
            // nothing to take, so a busy queue still drains at full speed.
            while (CANCTLR.recv(f, core0_wait))
            {
#if defined(DEBUG)
                uint32_t id = f.id;
                uint8_t len = f.info.dlc;
                verboseln("PID 0x%03x LEN %u", id, len);
#endif
                RCDEV.send(reinterpret_cast<uint8_t*>(&f.id), sizeof(uint32_t) + f.info.dlc);
                // After the send, not before: the phone is the product, the census is
                // the instrument, and the instrument never delays the product.
                DISP.note(f);

                // A queue that never empties never blocks either, which is the same
                // watchdog problem by a different route. One tick every 128 frames is
                // the cheapest way to guarantee the idle task runs -- 1 ms per 128
                // frames is 0.2% of the core at the rate this link gives up at, and
                // the queue holds 2000.
                //
                // Breaking out afterwards is what lets stats() print at all. It sits
                // after this loop, and the loop used to stop exiting under load, so
                // the board went silent exactly when it was worth reading. stats() is
                // gated on its own 5 s timer and compiled out of a release build, so
                // reaching it every 128 frames costs a comparison.
                if (++drained >= core0_batch)
                {
                    drained = 0;
                    vTaskDelay(1);
                    break;
                }
            }
            RCDEV.stats();
        }
    }
    else
    {
        errorln("ERROR: RaceChrono bluetooth device startup failed!");
        esp_restart();
    }

    // should never reach here...
    RCASSERT(false);
}

} // namespace

// core 1 - can-bus interrupt handler is running here, but also print some stats
void loop()
{
    // print out stats
    CANCTLR.stats();
}
