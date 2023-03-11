#pragma once
#include <stdint.h>

#define N_LEDS_PER_SIDE (112)
typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
} led_t;

void led_set_lr(led_t *buf, uint16_t i, led_t v);
void led_set_l(led_t *buf, uint16_t i, led_t v);
void led_set_r(led_t *buf, uint16_t i, led_t v);
