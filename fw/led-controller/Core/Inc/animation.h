#pragma once
#include <stdint.h>
#include "led.h"

void animation_render(led_t *buf, uint8_t step);
void animation_set_pattern(uint8_t pat);
