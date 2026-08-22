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

#include <Arduino.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

/// define to build the decoder for the BMW g8x
// #define CONFIG_CANBUS_DECODER_BMWG8X 1

/// define to build the raw pass-through decoder for the Yamaha R9
#define CONFIG_CANBUS_DECODER_YAMAHAR9 1

/// statistics timeout in microseconds
#define CONFIG_RC_STATS_TIMEOUT 5000000

// Connection parameters to request from the phone once it connects, in the units the
// link layer uses: intervals in 1.25 ms, timeout in 10 ms.
//
// The interval is the ceiling on throughput -- the radio only speaks at connection
// events -- so the R9's ~1050 msg/s wants this as short as it will go, and 7.5 ms is
// as short as BLE goes.
//
// This is the Android window. **Change it before pairing an iPhone**: iOS rejects a
// connection-parameter request outright unless min >= 15 ms, max >= min + 15 ms,
// latency <= 30 and timeout <= 6 s, and a rejected request leaves whatever the phone
// picked on its own -- worse than asking for less. The iOS-safe pair is 12/24
// (15-30 ms), legal on both platforms.
//
// Either way the phone has the final say, so the negotiated interval is read back in
// onConnParamsUpdate() and shown in the display header rather than assumed.
#define CONFIG_BLE_CONN_ITVL_MIN 6    // 7.5 ms  (iOS: 12, 15.0 ms)
#define CONFIG_BLE_CONN_ITVL_MAX 10   // 12.5 ms (iOS: 24, 30.0 ms)
#define CONFIG_BLE_CONN_LATENCY  0
#define CONFIG_BLE_CONN_TIMEOUT  400  // 4 s

/// Send frames through BLECharacteristic::notify() instead of straight at the NimBLE
/// host. Slower by three heap allocations and a mutex per frame, and kept only so the
/// two paths can be measured against each other on one afternoon's connection -- the
/// phone renegotiates the connection interval on every reconnect, so an A/B that needs
/// two firmware builds needs them within the same negotiated interval to mean anything.
// #define RC_WRAPPER_NOTIFY

/// if DEBUG is defined, logger will be enabled and print to serial console
// #define DEBUG

/// _x branch is likely to be true
#define RCLIKELY(_x)    __builtin_expect(!!(_x), 1)

/// _x branch is unlikely to be true
#define RCUNLIKELY(_x)  __builtin_expect(!!(_x), 0)

/// assert _expr is true (always not just DEBUG mode)
#define RCASSERT(_expr) do {                  \
    if (RCUNLIKELY(!(_expr))) { abort(); }    \
} while (0)

// Adafruit ESP32 Feather (Huzzah32)
#if defined(ARDUINO_FEATHER_ESP32)
#define CAN_RX_PIN GPIO_NUM_26
#define CAN_TX_PIN GPIO_NUM_25
#define BLE_PWR_LVL ESP_PWR_LVL_P9
#endif

// Adafruit Feather S3
#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3)
#define CAN_RX_PIN GPIO_NUM_18
#define CAN_TX_PIN GPIO_NUM_17
#define BLE_PWR_LVL ESP_PWR_LVL_P12
#endif

// Adafruit Feather ESP32-S3 Reverse TFT
#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_REVTFT)
#define CAN_RX_PIN GPIO_NUM_18   // A0
#define CAN_TX_PIN GPIO_NUM_17   // A1
#define BLE_PWR_LVL ESP_PWR_LVL_P12
/// this board has the 240x135 ST7789 and the three buttons; src/display drives them
#define CONFIG_DISPLAY_TFT 1
/// ...and one WS2812 on PIN_NEOPIXEL, gated by NEOPIXEL_POWER. It is the only
/// instrument on this board readable from a moving bike, so src/display drives it as
/// a status light for the chain -- bus, queue, radio, phone. See
/// motocan/bluecan/firmware.md, patch 16.
#define CONFIG_STATUS_LED 1
#endif

// ESP32 Dev Module
#if defined(ARDUINO_ESP32_DEV)
#define LED_BUILTIN 13
#define CAN_RX_PIN GPIO_NUM_26
#define CAN_TX_PIN GPIO_NUM_25
#define BLE_PWR_LVL ESP_PWR_LVL_P9
#endif

/// disable copy
#define CPP_NOCOPY(_name)                     \
    _name(_name const&) = delete;             \
    _name& operator=(_name const&) = delete

/// disable move
#define CPP_NOMOVE(_name)                     \
    _name(_name&&) = delete;                  \
    _name& operator=(_name&&) = delete

// C++11 ugh
template <typename T, typename U = T>
inline T exchange(T& obj, U&& new_val)
{
    T old_val = std::move(obj);
    obj = std::forward<U>(new_val);
    return old_val;
}
