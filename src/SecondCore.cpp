#include <Arduino.h>
#include <EFLogging.h>

#include "SecondCore.h"

TaskHandle_t Task2;
volatile uint8_t led_overrides[EFLED_TOTAL_NUM];
volatile uint32_t led_override_values[EFLED_TOTAL_NUM];

void Task2main(void* pvParameters) {
    sleep(10);
    LOG_INFO("(2nd Core) I’m alive!");
    led_override_values[EFLED_DRAGON_EAR_BOTTOM_IDX] = CRGB::BlueViolet;
    led_overrides[EFLED_DRAGON_EAR_BOTTOM_IDX] = 1;
    while(true) {
        sleep(1);
        led_override_values[EFLED_DRAGON_EAR_BOTTOM_IDX] = CRGB::IndianRed;
        sleep(1);
        led_override_values[EFLED_DRAGON_EAR_BOTTOM_IDX] = CRGB::BlueViolet;
    }
}

void StartSecondCore() {
    for(uint8_t i = 0; i < EFLED_TOTAL_NUM; i++) led_overrides[i] = 0;
    xTaskCreatePinnedToCore(Task2main, "Task2", 10240, NULL, 1, &Task2, 1);
    delay(500);
}