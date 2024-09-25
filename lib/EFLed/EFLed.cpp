// MIT License
//
// Copyright 2024 Eurofurence e.V. 
// 
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the “Software”),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

/**
 * @author Honigeintopf
 */

#include <Arduino.h>
#include <FastLED.h>

#include <EFLogging.h>

#include "EFLed.h"

static const EFLedClass::LEDPosition led_positions[] = {
    {17, 126},  // 0
    {21, 106},
    {23, 91},
    {41, 86},
    {35, 48},
    {37, 43},  // 5
    {61, 15},
    {61, 27},
    {61, 41},
    {61, 54},
    {61, 67},  // 10
    {61, 79},
    {61, 93},
    {61, 105},
    {61, 118},
    {61, 131},  // 15
    {61, 144}
};

EFLedClass::EFLedClass()
: max_brightness(0)
, led_data({0})
{
}

void EFLedClass::init() {
    this->init(EFLED_MAX_BRIGHTNESS_DEFAULT);
}

void EFLedClass::init(const uint8_t absolute_max_brightness) {
    for (uint8_t i = 0; i < EFLED_TOTAL_NUM; i++) {
        this->led_data[i] = CRGB::Black;
    }

    FastLED.clearData();
    FastLED.addLeds<WS2812B, EFLED_PIN_LED_DATA, GRB>(this->led_data, EFLED_TOTAL_NUM);

    this->max_brightness = absolute_max_brightness;
    FastLED.setBrightness(this->max_brightness);

    enablePower();
}

void EFLedClass::enablePower() {
    pinMode(EFLED_PIN_5VBOOST_ENABLE, OUTPUT);
    digitalWrite(EFLED_PIN_5VBOOST_ENABLE, HIGH);
    delay(1);
}

void EFLedClass::disablePower() {
    digitalWrite(EFLED_PIN_5VBOOST_ENABLE, LOW);
    delay(10);
}

void EFLedClass::clear() {
    for (uint8_t i = 0; i < EFLED_TOTAL_NUM; i++) {
        this->led_data[i] = CRGB::Black;
    }
    this->ledsShow();
}

void EFLedClass::setBrightnessPercent(uint8_t brightness) {
    FastLED.setBrightness(round((min(brightness, (uint8_t) 100) / (float) 100) * this->max_brightness));
    this->ledsShow();
}

uint8_t EFLedClass::getBrightnessPercent() const {
    return (uint8_t) round(FastLED.getBrightness() / (float) this->max_brightness * 100);
}

void EFLedClass::setAll(const CRGB color[EFLED_TOTAL_NUM]) {
    for (uint8_t i = 0; i < EFLED_TOTAL_NUM; i++) {
        this->led_data[i] = color[i];
    }
    this->ledsShow();
}

void EFLedClass::setAllSolid(const CRGB color) {
    for (uint8_t i = 0; i < EFLED_TOTAL_NUM; i++) {
        this->led_data[i] = color;
    }
    this->ledsShow();
}

void EFLedClass::setDragonNose(const CRGB color) {
    this->led_data[EFLED_DRAGON_NOSE_IDX] = color;
    this->ledsShow();
}

void EFLedClass::setDragonMuzzle(const CRGB color) {
    this->led_data[EFLED_DRAGON_MUZZLE_IDX] = color;
    this->ledsShow();
}

void EFLedClass::setDragonEye(const CRGB color) {
    this->led_data[EFLED_DRAGON_EYE_IDX] = color;
    this->ledsShow();
}

void EFLedClass::setDragonCheek(const CRGB color) {
    this->led_data[EFLED_DRAGON_CHEEK_IDX] = color;
    this->ledsShow();
}

void EFLedClass::setDragonEarBottom(const CRGB color) {
    this->led_data[EFLED_DRAGON_EAR_BOTTOM_IDX] = color;
    this->ledsShow();
}

void EFLedClass::setDragonEarTop(const CRGB color) {
    this->led_data[EFLED_DRAGON_EAR_TOP_IDX] = color;
    this->ledsShow();
}

void EFLedClass::setDragon(const CRGB color[EFLED_DRAGON_NUM]) {
    for (uint8_t i = 0; i < EFLED_DRAGON_NUM; i++) {
        this->led_data[EFLED_DARGON_OFFSET + i] = color[i];
    }
    this->ledsShow();
}

void EFLedClass::setEFBar(const CRGB color[EFLED_EFBAR_NUM]) {
    for (uint8_t i = 0; i < EFLED_EFBAR_NUM; i++) {
        this->led_data[EFLED_EFBAR_OFFSET + i] = color[i];
    }
    this->ledsShow();
}

void EFLedClass::setEFBar(uint8_t idx, const CRGB color) {
    if (idx >= EFLED_EFBAR_NUM) {
        return;
    }

    this->led_data[EFLED_EFBAR_OFFSET + idx] = color;
    this->ledsShow();
}

void EFLedClass::setOverrides(volatile uint8_t* overrides, volatile uint32_t* override_values) {
    this->led_override_values = override_values;
    this->led_overrides = overrides;
    this->ledsShow();
}

void EFLedClass::ledsShow() {
    if(this->led_overrides != nullptr && this->led_override_values != nullptr) {
        for(uint8_t i = 0; i < EFLED_TOTAL_NUM; i++) {
            if(!this->led_overrides[i]) continue;
            this->led_data[i] = this->led_override_values[i];
        }
    }
    FastLED.show();
}

void EFLedClass::setEFBarCursor(
    uint8_t idx,
    const CRGB color_on,
    const CRGB color_off
) {
    for (int8_t i = 0; i < EFLED_EFBAR_NUM; i++) {
        int8_t distance = abs((int16_t)idx - i);
        uint8_t fade = static_cast<uint8_t>(std::clamp(distance * 64.0f, 0.0f, 255.0f));
        this->led_data[EFLED_EFBAR_OFFSET + i] = (i == idx) ? color_on : color_off.scale8(fade);
    }
    this->ledsShow();
}

EFLedClass::LEDPosition EFLedClass::getLEDPosition(const uint8_t idx) {
    if (idx < std::size(led_positions)) {
        return led_positions[idx];
    }
    return {0, 0};  // Returning default position (0, 0) for out-of-bounds
}

void EFLedClass::fillEFBarProportionally(
    uint8_t percent,
    const CRGB color_on,
    const CRGB color_off
) {
    uint8_t num_leds_on = min(int(round(percent / (100.0f / EFLED_EFBAR_NUM))), EFLED_EFBAR_NUM);
    for (uint8_t i = 0; i < num_leds_on; i++) {
        this->led_data[EFLED_EFBAR_OFFSET + i] = color_on;
    }
    for (uint8_t i = num_leds_on; i < EFLED_EFBAR_NUM; i++) {
        this->led_data[EFLED_EFBAR_OFFSET + i] = color_off;
    }
    this->ledsShow();
}

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_EFLED)
EFLedClass EFLed;
#endif
