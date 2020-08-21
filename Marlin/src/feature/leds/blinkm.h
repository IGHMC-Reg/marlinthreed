/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once
#include "Wire.h"
/**
 * blinkm.h - Control a BlinkM over i2c
 */

extern bool cancel_heatup;

struct LEDColor;
typedef LEDColor LEDColor;

void blinkm_set_led_color(const LEDColor &color);

#define BLINK_LED(MS) blink_time = MS
#define BLINKFEED_LED(MS) blinkfeed_time = MS
#define BLINKRETRACT_LED(MS) blinkretract_time = MS

extern uint8_t key_flag;
extern uint8_t cancel_print;
extern uint16_t blink_time;
extern uint8_t print_key_flag;
extern float z_height_stop;
extern uint8_t print_pause;
extern uint32_t sys_time;
extern volatile uint32_t protect_time;

enum LED_STATUD {
  LED_ON = 4000,
  LED_BLINK_0 = 2500,
  LED_BLINK_1 = 1500,
  LED_BLINK_2 = 1000,

  LED_BLINK_3 = 800,
  LED_BLINK_4 = 500,
  LED_BLINK_5 = 300,
  LED_BLINK_6 = 150,
  LED_BLINK_7 = 50,
  LED_OFF = 0,
};

// extern uint16_t blink_time;
