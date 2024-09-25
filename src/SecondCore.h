#ifndef SECOND_CORE_H_
#define SECOND_CORE_H_

#include <EFLed.h>

extern volatile uint8_t led_overrides[EFLED_TOTAL_NUM];
extern volatile uint32_t led_override_values[EFLED_TOTAL_NUM];

void StartSecondCore();

#endif