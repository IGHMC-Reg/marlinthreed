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

/**
 * About Marlin
 *
 * This firmware is a mashup between Sprinter and grbl.
 *  - https://github.com/kliment/Sprinter
 *  - https://github.com/grbl/grbl
 */

#include "MarlinCore.h"

#include "HAL/shared/Delay.h"
#include "HAL/shared/esp_wifi.h"

#ifdef ARDUINO
  #include <pins_arduino.h>
#endif
#include <math.h>

#include "core/utility.h"
#include "lcd/ultralcd.h"
#include "module/motion.h"
#include "module/planner.h"
#include "module/stepper.h"
#include "module/endstops.h"
#include "module/probe.h"
#include "module/temperature.h"
#include "sd/cardreader.h"
#include "module/configuration_store.h"
#include "module/printcounter.h" // PrintCounter or Stopwatch
#include "feature/closedloop.h"

#include "module/stepper/indirection.h"

#include "libs/nozzle.h"

#include "gcode/gcode.h"
#include "gcode/parser.h"
#include "gcode/queue.h"

#if HAS_TFT_LVGL_UI
  #include "lcd/extui/lib/mks_ui/tft_lvgl_configuration.h"
  #include "lcd/extui/lib/mks_ui/draw_ui.h"
  #include "lcd/extui/lib/mks_ui/mks_hardware_test.h"
  #include <lvgl.h>
#endif

#if ENABLED(DWIN_CREALITY_LCD)
  #include "lcd/dwin/dwin.h"
  #include "lcd/dwin/dwin_lcd.h"
  #include "lcd/dwin/rotary_encoder.h"
#endif

#if ENABLED(IIC_BL24CXX_EEPROM)
  #include "libs/BL24CXX.h"
#endif

#if ENABLED(DIRECT_STEPPING)
  #include "feature/direct_stepping.h"
#endif

#if ENABLED(TOUCH_BUTTONS)
  #include "feature/touch/xpt2046.h"
#endif

#if ENABLED(HOST_ACTION_COMMANDS)
  #include "feature/host_actions.h"
#endif

#if USE_BEEPER
  #include "libs/buzzer.h"
#endif

#if HAS_I2C_DIGIPOT
  #include "feature/digipot/digipot.h"
#endif

#if ENABLED(MIXING_EXTRUDER)
  #include "feature/mixing.h"
#endif

#if ENABLED(MAX7219_DEBUG)
  #include "feature/max7219.h"
#endif

#if HAS_COLOR_LEDS
  #include "feature/leds/leds.h"
#endif

#if ENABLED(BLTOUCH)
  #include "feature/bltouch.h"
#endif

#if ENABLED(POLL_JOG)
  #include "feature/joystick.h"
#endif

#if HAS_SERVOS
  #include "module/servo.h"
#endif

#if ENABLED(DAC_STEPPER_CURRENT)
  #include "feature/dac/stepper_dac.h"
#endif

#if ENABLED(EXPERIMENTAL_I2CBUS)
  #include "feature/twibus.h"
  TWIBus i2c;
#endif

#if ENABLED(I2C_POSITION_ENCODERS)
  #include "feature/encoder_i2c.h"
#endif

#if HAS_TRINAMIC_CONFIG && DISABLED(PSU_DEFAULT_OFF)
  #include "feature/tmc_util.h"
#endif

#if HAS_CUTTER
  #include "feature/spindle_laser.h"
#endif

#if ENABLED(SDSUPPORT)
  CardReader card;
#endif

#if ENABLED(G38_PROBE_TARGET)
  uint8_t G38_move; // = 0
  bool G38_did_trigger; // = false
#endif

#if ENABLED(DELTA)
  #include "module/delta.h"
#elif IS_SCARA
  #include "module/scara.h"
#endif

#if HAS_LEVELING
  #include "feature/bedlevel/bedlevel.h"
#endif

#if BOTH(ADVANCED_PAUSE_FEATURE, PAUSE_PARK_NO_STEPPER_TIMEOUT)
  #include "feature/pause.h"
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "feature/powerloss.h"
#endif

#if ENABLED(CANCEL_OBJECTS)
  #include "feature/cancel_object.h"
#endif

#if HAS_FILAMENT_SENSOR
  #include "feature/runout.h"
#endif

#if ENABLED(HOTEND_IDLE_TIMEOUT)
  #include "feature/hotend_idle.h"
#endif

#if ENABLED(TEMP_STAT_LEDS)
  #include "feature/leds/tempstat.h"
#endif

#if HAS_CASE_LIGHT
  #include "feature/caselight.h"
#endif

#if HAS_FANMUX
  #include "feature/fanmux.h"
#endif

#if DO_SWITCH_EXTRUDER || ANY(SWITCHING_NOZZLE, PARKING_EXTRUDER, MAGNETIC_PARKING_EXTRUDER, ELECTROMAGNETIC_SWITCHING_TOOLHEAD, SWITCHING_TOOLHEAD)
  #include "module/tool_change.h"
#endif

#if ENABLED(USE_CONTROLLER_FAN)
  #include "feature/controllerfan.h"
#endif

#if ENABLED(PRUSA_MMU2)
  #include "feature/mmu2/mmu2.h"
#endif

#if HAS_L64XX
  #include "libs/L64XX/L64XX_Marlin.h"
#endif

PGMSTR(NUL_STR, "");
PGMSTR(M112_KILL_STR, "M112 Shutdown");
PGMSTR(G28_STR, "G28");
PGMSTR(M21_STR, "M21");
PGMSTR(M23_STR, "M23 %s");
PGMSTR(M24_STR, "M24");
PGMSTR(SP_P_STR, " P");  PGMSTR(SP_T_STR, " T");
PGMSTR(X_STR,     "X");  PGMSTR(Y_STR,     "Y");  PGMSTR(Z_STR,     "Z");  PGMSTR(E_STR,     "E");
PGMSTR(X_LBL,     "X:"); PGMSTR(Y_LBL,     "Y:"); PGMSTR(Z_LBL,     "Z:"); PGMSTR(E_LBL,     "E:");
PGMSTR(SP_A_STR, " A");  PGMSTR(SP_B_STR, " B");  PGMSTR(SP_C_STR, " C");
PGMSTR(SP_X_STR, " X");  PGMSTR(SP_Y_STR, " Y");  PGMSTR(SP_Z_STR, " Z");  PGMSTR(SP_E_STR, " E");
PGMSTR(SP_X_LBL, " X:"); PGMSTR(SP_Y_LBL, " Y:"); PGMSTR(SP_Z_LBL, " Z:"); PGMSTR(SP_E_LBL, " E:");

MarlinState marlin_state = MF_INITIALIZING;

// For M109 and M190, this flag may be cleared (by M108) to exit the wait loop
bool wait_for_heatup = true;

// For M0/M1, this flag may be cleared (by M108) to exit the wait-for-user loop
#if HAS_RESUME_CONTINUE
  bool wait_for_user; // = false;

  void wait_for_user_response(millis_t ms/*=0*/, const bool no_sleep/*=false*/) {
    TERN(ADVANCED_PAUSE_FEATURE,,UNUSED(no_sleep));
    KEEPALIVE_STATE(PAUSED_FOR_USER);
    wait_for_user = true;
    if (ms) ms += millis(); // expire time
    while (wait_for_user && !(ms && ELAPSED(millis(), ms)))
      idle(TERN_(ADVANCED_PAUSE_FEATURE, no_sleep));
    wait_for_user = false;
  }

#endif

#if PIN_EXISTS(CHDK)
  extern millis_t chdk_timeout;
#endif

#if ENABLED(I2C_POSITION_ENCODERS)
  I2CPositionEncodersMgr I2CPEM;
#endif

/**
 * ***************************************************************************
 * ******************************** FUNCTIONS ********************************
 * ***************************************************************************
 */

void setup_killpin() {
  #if HAS_KILL
    #if KILL_PIN_STATE
      SET_INPUT_PULLDOWN(KILL_PIN);
    #else
      SET_INPUT_PULLUP(KILL_PIN);
    #endif
  #endif
}

void setup_powerhold() {
  #if HAS_SUICIDE
    OUT_WRITE(SUICIDE_PIN, !SUICIDE_PIN_INVERTING);
  #endif
  #if ENABLED(PSU_CONTROL)
    powersupply_on = ENABLED(PSU_DEFAULT_OFF);
    if (ENABLED(PSU_DEFAULT_OFF)) PSU_OFF(); else PSU_ON();
  #endif
}

/**
 * Stepper Reset (RigidBoard, et.al.)
 */
#if HAS_STEPPER_RESET
  void disableStepperDrivers() { OUT_WRITE(STEPPER_RESET_PIN, LOW); } // Drive down to keep motor driver chips in reset
  void enableStepperDrivers()  { SET_INPUT(STEPPER_RESET_PIN); }      // Set to input, allowing pullups to pull the pin high
#endif

#if ENABLED(EXPERIMENTAL_I2CBUS) && I2C_SLAVE_ADDRESS > 0

  void i2c_on_receive(int bytes) { // just echo all bytes received to serial
    i2c.receive(bytes);
  }

  void i2c_on_request() {          // just send dummy data for now
    i2c.reply("Hello World!\n");
  }

#endif

/**
 * Sensitive pin test for M42, M226
 */

#include "pins/sensitive_pins.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnarrowing"

bool pin_is_protected(const pin_t pin) {
  static const pin_t sensitive_pins[] PROGMEM = SENSITIVE_PINS;
  LOOP_L_N(i, COUNT(sensitive_pins)) {
    pin_t sensitive_pin;
    memcpy_P(&sensitive_pin, &sensitive_pins[i], sizeof(pin_t));
    if (pin == sensitive_pin) return true;
  }
  return false;
}

#pragma GCC diagnostic pop

void protected_pin_err() {
  SERIAL_ERROR_MSG(STR_ERR_PROTECTED_PIN);
}

void quickstop_stepper() {
  planner.quick_stop();
  planner.synchronize();
  set_current_from_steppers_for_axis(ALL_AXES);
  sync_plan_position();
}

void enable_e_steppers() {
  #define _ENA_E(N) ENABLE_AXIS_E##N();
  REPEAT(E_STEPPERS, _ENA_E)
}

void enable_all_steppers() {
  TERN_(AUTO_POWER_CONTROL, powerManager.power_on());
  ENABLE_AXIS_X();
  ENABLE_AXIS_Y();
  ENABLE_AXIS_Z();
  enable_e_steppers();
}

void disable_e_steppers() {
  #define _DIS_E(N) DISABLE_AXIS_E##N();
  REPEAT(E_STEPPERS, _DIS_E)
}

void disable_e_stepper(const uint8_t e) {
  #define _CASE_DIS_E(N) case N: DISABLE_AXIS_E##N(); break;
  switch (e) {
    REPEAT(EXTRUDERS, _CASE_DIS_E)
  }
}

void disable_all_steppers() {
  DISABLE_AXIS_X();
  DISABLE_AXIS_Y();
  DISABLE_AXIS_Z();
  disable_e_steppers();
}

#if ENABLED(G29_RETRY_AND_RECOVER)

  void event_probe_failure() {
    #ifdef ACTION_ON_G29_FAILURE
      host_action(PSTR(ACTION_ON_G29_FAILURE));
    #endif
    #ifdef G29_FAILURE_COMMANDS
      gcode.process_subcommands_now_P(PSTR(G29_FAILURE_COMMANDS));
    #endif
    #if ENABLED(G29_HALT_ON_FAILURE)
      #ifdef ACTION_ON_CANCEL
        host_action_cancel();
      #endif
      kill(GET_TEXT(MSG_LCD_PROBING_FAILED));
    #endif
  }

  void event_probe_recover() {
    TERN_(HOST_PROMPT_SUPPORT, host_prompt_do(PROMPT_INFO, PSTR("G29 Retrying"), DISMISS_STR));
    #ifdef ACTION_ON_G29_RECOVER
      host_action(PSTR(ACTION_ON_G29_RECOVER));
    #endif
    #ifdef G29_RECOVER_COMMANDS
      gcode.process_subcommands_now_P(PSTR(G29_RECOVER_COMMANDS));
    #endif
  }

#endif

#if ENABLED(ADVANCED_PAUSE_FEATURE)
  #include "feature/pause.h"
#else
  constexpr bool did_pause_print = false;
#endif

#if MB(MELZI_THREED)
  #include "src/feature/leds/blinkm.h"

  bool cancel_heatup = false;
  uint32_t cancel_gohome_ms = 0;
  uint8_t cancel_print = 0;
  uint16_t blink_time = 0;
  uint16_t blinkfeed_time = 0;
  uint16_t blinkretract_time = 0;

  uint8_t print_key_flag = 0;
  float z_height_stop = 0;
  float temperature_protect_last = 0;

  volatile uint32_t protect_time = 0;

  uint8_t print_pause = 0;

  uint32_t sys_time = 0;

  uint8_t key_flag = 0;


  void setup_homepin(void) {
      SET_INPUT(HOME_PIN);
      WRITE(HOME_PIN, HIGH);
  }

  void BlinkLed(void) {
      static uint32_t blink_time_previous = 0;
      static uint32_t blink_time_start = 0;
      if (blink_time == 0) {
          WRITE(MT_PRINT_LED, 1);
          return;
      }
      if (blink_time > 3000) {
          WRITE(MT_PRINT_LED, 0);
          return;
      }
      if (blink_time_previous != blink_time) {
          blink_time_previous = blink_time;
          blink_time_start = millis();
      }
      if (millis() < blink_time_start + blink_time) {
          WRITE(MT_PRINT_LED, 0);
      } else if (millis() < blink_time_start + 2 * blink_time) {
          WRITE(MT_PRINT_LED, 1);
      } else {
          blink_time_start = millis();
      }
  }

  void BlinkFeedLed(void) {
      static uint32_t blink_time_previous = 0;
      static uint32_t blink_time_start = 0;
      if (blinkfeed_time == 0) {
          WRITE(FEED_LED, 1);
          return;
      }
      if (blinkfeed_time > 3000) {
          WRITE(FEED_LED, 0);
          return;
      }
      if (blink_time_previous != blinkfeed_time) {
          blink_time_previous = blinkfeed_time;
          blink_time_start = millis();
      }
      if (millis() < blink_time_start + blinkfeed_time) {
          WRITE(FEED_LED, 0);
      } else if (millis() < blink_time_start + 2 * blinkfeed_time) {
          WRITE(FEED_LED, 1);
      } else {
          blink_time_start = millis();
      }
  }
  void BlinkRetractLed(void) {
      static uint32_t blink_time_previous = 0;
      static uint32_t blink_time_start = 0;
      if (blinkretract_time == 0) {
          WRITE(RETRACT_LED, 1);
          return;
      }
      if (blinkretract_time > 3000) {
          WRITE(RETRACT_LED, 0);
          return;
      }
      if (blink_time_previous != blinkretract_time) {
          blink_time_previous = blinkretract_time;
          blink_time_start = millis();
      }
      if (millis() < blink_time_start + blinkretract_time) {
          WRITE(RETRACT_LED, 0);
      } else if (millis() < blink_time_start + 2 * blinkretract_time) {
          WRITE(RETRACT_LED, 1);
      } else {
          blink_time_start = millis();
      }
  }

  void SendColors(byte red, byte grn, byte blu) {
      Wire.begin();
      Wire.beginTransmission(0x09);
      Wire.write('o'); // to disable ongoing script, only needs to be used once
      Wire.write('n');
      Wire.write(red);
      Wire.write(grn);
      Wire.write(blu);
      Wire.endTransmission();
  }

  void PrintOneKey(void) {
      // #pragma printone
      SERIAL_ECHOLNPAIR("PrintOneKey : ", key_flag);
      static uint8_t key_status = 0;
      static uint32_t key_time = 0;
      static uint8_t pause_flag = 0;
      feedRate_t old_feedrate;
      if (key_flag == 1 || key_flag == 2 || key_flag == 3)
          return;

      if (key_status == 0) {
          if (!READ(PRINT_START_PIN)) {
              key_time = millis();
              key_status = 1;
          }
      } else if (key_status == 1) {
          if (key_time + 30 < millis()) {
              if (!READ(PRINT_START_PIN)) {
                  key_time = millis();
                  key_status = 2;
              } else {
                  key_status = 0;
              }
          }
      } else if (key_status == 2) {
          if (READ(PRINT_START_PIN)) {
              if (key_time + 1000 > millis()) {
                  if (print_key_flag == 0) {
                      // card.initsd();
                      CardReader::mount();
                      if (card.flag.mounted == false) {
                          BLINK_LED(LED_OFF);
                          key_status = 0;
                          key_time = 0;
                          return;
                      }
                      uint16_t filecnt = card.get_num_Files();
                      card.getfilename_sorted(filecnt);
                      while (card.flag.filenameIsDir) {
                          if (filecnt > 1) {
                              filecnt--;
                              card.getfilename_sorted(filecnt);
                          }
                      }
                      CardReader::openAndPrintFile(card.filename);
                  } else if (print_key_flag == 1) {
                      SERIAL_ECHOLN("pause");
                      BLINK_LED(LED_ON);
                      card.pauseSDPrint();
                      print_pause = 1;
                      print_key_flag = 2;
                  } else if (print_key_flag == 2) {
                      SERIAL_ECHOLN("print");
                      if (temperature_protect_last > 60) {
                          thermalManager.setTargetHotend(temperature_protect_last, 0);
                          temperature_protect_last = 0;
                      }
                      BLINK_LED(LED_BLINK_0);
                      card.startFileprint();
                      print_pause = 0;
                      print_key_flag = 1;
                  } else {
                      print_key_flag = 0;
                  }

              } else {
                  if (print_key_flag == 0) {
                      if (IsRunning()) {
                          destination[Z_AXIS] += 10;
                          old_feedrate = feedrate_mm_s;
                          feedrate_mm_s = 10 * 60;
                          prepare_internal_move_to_destination(feedrate_mm_s);
                          feedrate_mm_s = old_feedrate;
                      }
                  } else {
                      z_height_stop = planner.get_axis_position_mm(Z_AXIS);
                      SERIAL_ECHOLN("cancel");
                      card.flag.sdprinting = false;
                      cancel_heatup = true;
                      cancel_gohome_ms = millis() + 500;

                      quickstop_stepper();
                      thermalManager.disable_all_heaters();

                      card.closefile();
                      ; // switch off all heaters.

                      cancel_print = 1;
                      key_flag = 0;
                      BLINK_LED(LED_OFF);
                      print_key_flag = 0;
                      print_pause = 0;
                  }
              }
              key_status = 0;
              key_time = 0;
          }
      } else {
          key_status = 0;
          key_time = 0;
          print_key_flag = 0;
      }
  }

      void feed_filament(void) {
          SERIAL_ECHOLNPAIR("feed_filament : ", key_flag);
          if (IsRunning()) {
              feedRate_t old_feedrate;
              SERIAL_ECHOLN("feed");
              destination[E_AXIS] += 100;
              old_feedrate = feedrate_mm_s;
              feedrate_mm_s = 1.5 * 60;
              prepare_internal_move_to_destination(feedrate_mm_s);
              feedrate_mm_s = old_feedrate;
          }
      }

      void retract_filament(void) {
          SERIAL_ECHOLNPAIR("retract_filament : ", key_flag);
          feedRate_t old_feedrate;
          if (IsRunning()) {
              SERIAL_ECHOLN("retract");
              destination[E_AXIS] += 2;
              old_feedrate = feedrate_mm_s;
              feedrate_mm_s = 3 * 60;
              prepare_internal_move_to_destination(feedrate_mm_s);

              destination[E_AXIS] -= 300;
              feedrate_mm_s = 15.0 * 60;
              prepare_internal_move_to_destination(feedrate_mm_s);
              feedrate_mm_s = old_feedrate;
          }
      }

      void LoadFilament(void) {
          SERIAL_ECHOLNPAIR("load_filament : ", key_flag);
          static uint8_t filament_status = 0;
          static uint8_t filament_flag = 0;
          static uint8_t key = 0;
          static uint32_t key_time = 0;

          if (key_flag == 1 || key_flag == 4)
              return;

          if (filament_status == 0) {
              if (!READ(RETRACT_PIN)) {
                  key = 0x01;
              };
              if (!READ(FEED_PIN)) {
                  key = 0x02;
              };
              if (key) {
                  filament_status++;
                  key_time = millis() + 20;
              }
          } else if (filament_status == 1) {
              if (key_time <= millis()) {
                  if (READ(RETRACT_PIN)) {
                      key &= ~0x01;
                  }
                  if (READ(FEED_PIN)) {
                      key &= ~0x02;
                  }
                  if (key) {
                      protect_time = millis();
                      // setTargetHotend0(220);
                      thermalManager.setTargetHotend(220, 0);
                      if (key & 0x01) {
                          BLINKRETRACT_LED(LED_BLINK_7);
                          BLINKFEED_LED(LED_OFF);
                          key_flag = 2;
                      }
                      if (key & 0x02) {
                          BLINKFEED_LED(LED_BLINK_7);
                          BLINKRETRACT_LED(LED_OFF);
                          key_flag = 3;
                      }
                      filament_status++;
                  } else {
                      key_flag = 0;
                      filament_status = 0;
                  }
              }
          } else if (filament_status == 2) {
              if (READ(RETRACT_PIN) && READ(FEED_PIN)) {
                  filament_status = 3;
              }
          } else if (filament_status == 3) {
              if (Temperature::degHotend(0) >= 210) {
                  filament_status++;
              } else {
                  if (!READ(RETRACT_PIN)) {
                      if (key == 0x01) {
                          key = 0;
                          key_flag = 0;
                          filament_status = 6;
                          // setTargetHotend0(0);
                          Temperature::setTargetHotend(0, 0);
                          SERIAL_ECHOLN("retract_hot0_cancel");
                      } else {
                          key = 0x01;
                          BLINKRETRACT_LED(LED_BLINK_7);
                          BLINKFEED_LED(LED_OFF);
                          key_flag = 2;
                          SERIAL_ECHOLN("retract_hot0");
                          filament_status = 2;
                      }
                  };
                  if (!READ(FEED_PIN)) {
                      if (key == 0x02) {
                          key = 0;
                          key_flag = 0;
                          filament_status = 6;
                          Temperature::setTargetHotend(0,0);
                          SERIAL_ECHOLN("feed_hot0_cancel");
                      } else {
                          key = 0x02;
                          BLINKFEED_LED(LED_BLINK_7);
                          BLINKRETRACT_LED(LED_OFF);
                          key_flag = 3;
                          SERIAL_ECHOLN("feed_hot0");
                          filament_status = 2;
                      }
                  };
              }
          } else if (filament_status == 4) {
              if (key & 0x01) {
                  retract_filament();
                  BLINKRETRACT_LED(LED_BLINK_5);
                  BLINKFEED_LED(LED_OFF);
              };
              if (key & 0x02) {
                  feed_filament();
                  BLINKFEED_LED(LED_BLINK_5);
                  BLINKRETRACT_LED(LED_OFF);
              };
              filament_status++;
          } else if (filament_status == 5) {
              if (!planner.has_blocks_queued()) {
                  BLINKRETRACT_LED(LED_OFF);
                  BLINKFEED_LED(LED_OFF);
                  filament_status = 0;
                  key_flag = 0;
                  key = 0;
              }

              if (!READ(RETRACT_PIN)) {
                  if (key == 0x01)
                      planner.cleaning_buffer_counter = 2;
                  BLINKRETRACT_LED(LED_OFF);
                  filament_status = 6;
                  key_flag = 0;
              } else {
                  planner.cleaning_buffer_counter = 2;
                  key = 0x01;
                  key_flag = 2;
                  filament_status = 7;
              }
          }
          if (!READ(FEED_PIN)) {
              if (key == 0x02) {
                  planner.cleaning_buffer_counter = 2;
                  BLINKFEED_LED(LED_OFF);
                  key_flag = 0;
                  filament_status = 6;
              } else {
                  planner.cleaning_buffer_counter = 2;
                  key = 0x02;
                  key_flag = 3;
                  filament_status = 7;
              }
          }
            else if (filament_status == 8) {
          if (READ(RETRACT_PIN) && READ(FEED_PIN)) {
              filament_status = 3;
          }
      }
      else if (filament_status == 7) {
          if (READ(RETRACT_PIN) && READ(FEED_PIN)) {
              filament_status = 4;
          }
      }
      else if (filament_status == 6) {
          if (READ(RETRACT_PIN) && READ(FEED_PIN)) {
              BLINKRETRACT_LED(LED_OFF);
              BLINKFEED_LED(LED_OFF);
              filament_status = 0;
          }
      }
      else {
          filament_status = 0;
      }
  }

  void home_key() {
      static uint8_t key_status = 0;
      static uint32_t key_time;
      SERIAL_ECHOLNPAIR("home_key pressed : ", key_flag);
      if (key_flag == 4 || key_flag == 2 || key_flag == 3)
          return;

      if (key_status == 0) {
          if (!READ(HOME_PIN)) {
              key_time = millis() + 50;
              key_status = 1;
              SERIAL_ECHOLNPAIR("home_key status 0 : ", key_status);
          }
      } else if (key_status == 1) {
          if (key_time <= millis()) {
              if (!READ(HOME_PIN)) {
                  key_status = 2;
              } else {
                  key_status = 0;
              }
              SERIAL_ECHOLNPAIR("home_key status 1 : ", key_status);
          }
      } else if (key_status == 2) {
          WRITE(HOME_LED, 0);
          DISABLE_AXIS_X();
          DISABLE_AXIS_Y();
          // enqueuecommand("G28 Z0");
          queue.enqueue_now_P(PSTR("G28 Z0"));
          queue.advance();
          key_flag = 1;
          key_status = 4;
          SERIAL_ECHOLNPAIR("home_key status 2 : ", key_status);
      } else if (key_status == 4) {
          if (READ(HOME_PIN)) {
              key_status = 3;
          }
          SERIAL_ECHOLNPAIR("home_key status 4 : ", key_status);
      } else if (key_status == 3) {
          if (!planner.has_blocks_queued()) {
              key_status = 0;
              WRITE(HOME_LED, 1);
              key_flag = 0;
              SERIAL_ECHOLNPAIR("home_key status 3 : ", key_status);
          }
      }
      SERIAL_ECHOLNPAIR("end of home_key status: ", key_status);
    }

  void heat_protect(void) {
      SERIAL_ECHOLNPAIR("heat_protect : ", Temperature::degHotend(0));
      if (Temperature::degHotend(0) < 60)
          return;
      if (IS_SD_PRINTING() == true)
          return;
      if (planner.has_blocks_queued())
          return;
      if (queue.has_commands_queued())
          return;
      if (key_flag != 0)
          return;

      if (IS_SD_PRINTING() == false) {
          if (protect_time + 600000l < millis()) {
              protect_time = millis();
              temperature_protect_last = Temperature::degHotend(0);
              thermalManager.disable_all_heaters();
              SERIAL_ECHOLN("extruder heating off!");
          }
      }
  }

  void cancel_gohome(void) {
    feedRate_t old_feedrate;
    SERIAL_ECHOLN("extruder heating off!");
    if (cancel_print == 0)
      return;
    if (cancel_gohome_ms > millis())
      return;

    cancel_print = 0;
    if (IsRunning()) {
      destination[Z_AXIS] += 10;
      old_feedrate = feedrate_mm_s;
      feedrate_mm_s = 10 * 60;
      prepare_internal_move_to_destination(feedrate_mm_s);
      feedrate_mm_s = old_feedrate;
    }
    return;
  }



#endif //IGHMC

  /**
   * A Print Job exists when the timer is running or SD printing
   */
  bool printJobOngoing() {
    return print_job_timer.isRunning() || IS_SD_PRINTING();
}

/**
 * Printing is active when the print job timer is running
 */
bool printingIsActive() {
  return !did_pause_print && (print_job_timer.isRunning() || IS_SD_PRINTING());
}

/**
 * Printing is paused according to SD or host indicators
 */
bool printingIsPaused() {
  return did_pause_print || print_job_timer.isPaused() || IS_SD_PAUSED();
}

void startOrResumeJob() {
  if (!printingIsPaused()) {
    TERN_(CANCEL_OBJECTS, cancelable.reset());
    TERN_(LCD_SHOW_E_TOTAL, e_move_accumulator = 0);
    #if BOTH(LCD_SET_PROGRESS_MANUALLY, USE_M73_REMAINING_TIME)
      ui.reset_remaining_time();
    #endif
  }
  print_job_timer.start();
}

#if ENABLED(SDSUPPORT)

  inline void abortSDPrinting() {
    card.endFilePrint(TERN_(SD_RESORT, true));
    queue.clear();
    quickstop_stepper();
    print_job_timer.stop();
    #if DISABLED(SD_ABORT_NO_COOLDOWN)
      thermalManager.disable_all_heaters();
    #endif
    #if !HAS_CUTTER
      thermalManager.zero_fan_speeds();
    #else
      cutter.kill();              // Full cutter shutdown including ISR control
    #endif
    wait_for_heatup = false;
    TERN_(POWER_LOSS_RECOVERY, recovery.purge());
    #ifdef EVENT_GCODE_SD_STOP
      queue.inject_P(PSTR(EVENT_GCODE_SD_STOP));
    #endif
  }

  inline void finishSDPrinting() {
    if (queue.enqueue_one_P(PSTR("M1001")))
      marlin_state = MF_RUNNING;
  }

#endif // SDSUPPORT

/**
 * Minimal management of Marlin's core activities:
 *  - Keep the command buffer full
 *  - Check for maximum inactive time between commands
 *  - Check for maximum inactive time between stepper commands
 *  - Check if CHDK_PIN needs to go LOW
 *  - Check for KILL button held down
 *  - Check for HOME button held down
 *  - Check if cooling fan needs to be switched on
 *  - Check if an idle but hot extruder needs filament extruded (EXTRUDER_RUNOUT_PREVENT)
 *  - Pulse FET_SAFETY_PIN if it exists
 */
inline void manage_inactivity(const bool ignore_stepper_queue=false) {

  if (queue.length < BUFSIZE) queue.get_available_commands();

  const millis_t ms = millis();

  // Prevent steppers timing-out in the middle of M600
  // unless PAUSE_PARK_NO_STEPPER_TIMEOUT is disabled
  const bool parked_or_ignoring = ignore_stepper_queue ||
     (BOTH(ADVANCED_PAUSE_FEATURE, PAUSE_PARK_NO_STEPPER_TIMEOUT) && did_pause_print);

  // Reset both the M18/M84 activity timeout and the M85 max 'kill' timeout
  if (parked_or_ignoring) gcode.reset_stepper_timeout(ms);

  if (gcode.stepper_max_timed_out(ms)) {
    SERIAL_ERROR_START();
    SERIAL_ECHOLNPAIR(STR_KILL_INACTIVE_TIME, parser.command_ptr);
    kill();
  }

  // M18 / M94 : Handle steppers inactive time timeout
  if (gcode.stepper_inactive_time) {

    static bool already_shutdown_steppers; // = false

    // Any moves in the planner? Resets both the M18/M84
    // activity timeout and the M85 max 'kill' timeout
    if (planner.has_blocks_queued())
      gcode.reset_stepper_timeout(ms);
    else if (!parked_or_ignoring && gcode.stepper_inactive_timeout()) {
      if (!already_shutdown_steppers) {
        already_shutdown_steppers = true;  // L6470 SPI will consume 99% of free time without this

        // Individual axes will be disabled if configured
        if (ENABLED(DISABLE_INACTIVE_X)) DISABLE_AXIS_X();
        if (ENABLED(DISABLE_INACTIVE_Y)) DISABLE_AXIS_Y();
        if (ENABLED(DISABLE_INACTIVE_Z)) DISABLE_AXIS_Z();
        if (ENABLED(DISABLE_INACTIVE_E)) disable_e_steppers();

        TERN_(AUTO_BED_LEVELING_UBL, ubl.steppers_were_disabled());
      }
    }
    else
      already_shutdown_steppers = false;
  }

  #if PIN_EXISTS(CHDK) // Check if pin should be set to LOW (after M240 set it HIGH)
    if (chdk_timeout && ELAPSED(ms, chdk_timeout)) {
      chdk_timeout = 0;
      WRITE(CHDK_PIN, LOW);
    }
  #endif

  #if HAS_KILL

    // Check if the kill button was pressed and wait just in case it was an accidental
    // key kill key press
    // -------------------------------------------------------------------------------
    static int killCount = 0;   // make the inactivity button a bit less responsive
    const int KILL_DELAY = 750;
    if (kill_state())
      killCount++;
    else if (killCount > 0)
      killCount--;

    // Exceeded threshold and we can confirm that it was not accidental
    // KILL the machine
    // ----------------------------------------------------------------
    if (killCount >= KILL_DELAY) {
      SERIAL_ERROR_MSG(STR_KILL_BUTTON);
      kill();
    }
  #endif

  #if HAS_HOME
    // Handle a standalone HOME button
    constexpr millis_t HOME_DEBOUNCE_DELAY = 1000UL;
    static millis_t next_home_key_ms; // = 0
    SERIAL_ECHOLNPAIR("ighmc standalone home key: ", HOME_PIN);
    if (!IS_SD_PRINTING() && !READ(HOME_PIN)) { // HOME_PIN goes LOW when pressed
        const millis_t ms = millis();
        if (ELAPSED(ms, next_home_key_ms)) {
            next_home_key_ms = ms + HOME_DEBOUNCE_DELAY;
            LCD_MESSAGEPGM(MSG_AUTO_HOME);
            queue.enqueue_now_P(G28_STR);
            queue.advance();
        }
    }
#endif

  TERN_(USE_CONTROLLER_FAN, controllerFan.update()); // Check if fan should be turned on to cool stepper drivers down

  TERN_(AUTO_POWER_CONTROL, powerManager.check());

  TERN_(HOTEND_IDLE_TIMEOUT, hotend_idle.check());

  #if ENABLED(EXTRUDER_RUNOUT_PREVENT)
    if (thermalManager.degHotend(active_extruder) > EXTRUDER_RUNOUT_MINTEMP
      && ELAPSED(ms, gcode.previous_move_ms + SEC_TO_MS(EXTRUDER_RUNOUT_SECONDS))
      && !planner.has_blocks_queued()
    ) {
      #if ENABLED(SWITCHING_EXTRUDER)
        bool oldstatus;
        switch (active_extruder) {
          default: oldstatus = E0_ENABLE_READ(); ENABLE_AXIS_E0(); break;
          #if E_STEPPERS > 1
            case 2: case 3: oldstatus = E1_ENABLE_READ(); ENABLE_AXIS_E1(); break;
            #if E_STEPPERS > 2
              case 4: case 5: oldstatus = E2_ENABLE_READ(); ENABLE_AXIS_E2(); break;
              #if E_STEPPERS > 3
                case 6: case 7: oldstatus = E3_ENABLE_READ(); ENABLE_AXIS_E3(); break;
              #endif // E_STEPPERS > 3
            #endif // E_STEPPERS > 2
          #endif // E_STEPPERS > 1
        }
      #else // !SWITCHING_EXTRUDER
        bool oldstatus;
        switch (active_extruder) {
          default:
          #define _CASE_EN(N) case N: oldstatus = E##N##_ENABLE_READ(); ENABLE_AXIS_E##N(); break;
          REPEAT(E_STEPPERS, _CASE_EN);
        }
      #endif

      const float olde = current_position.e;
      current_position.e += EXTRUDER_RUNOUT_EXTRUDE;
      line_to_current_position(MMM_TO_MMS(EXTRUDER_RUNOUT_SPEED));
      current_position.e = olde;
      planner.set_e_position_mm(olde);
      planner.synchronize();

      #if ENABLED(SWITCHING_EXTRUDER)
        switch (active_extruder) {
          default: oldstatus = E0_ENABLE_WRITE(oldstatus); break;
          #if E_STEPPERS > 1
            case 2: case 3: oldstatus = E1_ENABLE_WRITE(oldstatus); break;
            #if E_STEPPERS > 2
              case 4: case 5: oldstatus = E2_ENABLE_WRITE(oldstatus); break;
            #endif // E_STEPPERS > 2
          #endif // E_STEPPERS > 1
        }
      #else // !SWITCHING_EXTRUDER
        switch (active_extruder) {
          #define _CASE_RESTORE(N) case N: E##N##_ENABLE_WRITE(oldstatus); break;
          REPEAT(E_STEPPERS, _CASE_RESTORE);
        }
      #endif // !SWITCHING_EXTRUDER

      gcode.reset_stepper_timeout(ms);
    }
  #endif // EXTRUDER_RUNOUT_PREVENT

  #if ENABLED(DUAL_X_CARRIAGE)
    // handle delayed move timeout
    if (delayed_move_time && ELAPSED(ms, delayed_move_time + 1000UL) && IsRunning()) {
      // travel moves have been received so enact them
      delayed_move_time = 0xFFFFFFFFUL; // force moves to be done
      destination = current_position;
      prepare_line_to_destination();
    }
  #endif

  TERN_(TEMP_STAT_LEDS, handle_status_leds());

  TERN_(MONITOR_DRIVER_STATUS, monitor_tmc_drivers());

  TERN_(MONITOR_L6470_DRIVER_STATUS, L64xxManager.monitor_driver());

  // Limit check_axes_activity frequency to 10Hz
  static millis_t next_check_axes_ms = 0;
  if (ELAPSED(ms, next_check_axes_ms)) {
    planner.check_axes_activity();
    next_check_axes_ms = ms + 100UL;
  }

  #if PIN_EXISTS(FET_SAFETY)
    static millis_t FET_next;
    if (ELAPSED(ms, FET_next)) {
      FET_next = ms + FET_SAFETY_DELAY;  // 2µs pulse every FET_SAFETY_DELAY mS
      OUT_WRITE(FET_SAFETY_PIN, !FET_SAFETY_INVERTED);
      DELAY_US(2);
      WRITE(FET_SAFETY_PIN, FET_SAFETY_INVERTED);
    }
  #endif
}

/**
 * Standard idle routine keeps the machine alive:
 *  - Core Marlin activities
 *  - Manage heaters (and Watchdog)
 *  - Max7219 heartbeat, animation, etc.
 *
 *  Only after setup() is complete:
 *  - Handle filament runout sensors
 *  - Run HAL idle tasks
 *  - Handle Power-Loss Recovery
 *  - Run StallGuard endstop checks
 *  - Handle SD Card insert / remove
 *  - Handle USB Flash Drive insert / remove
 *  - Announce Host Keepalive state (if any)
 *  - Update the Print Job Timer state
 *  - Update the Beeper queue
 *  - Read Buttons and Update the LCD
 *  - Run i2c Position Encoders
 *  - Auto-report Temperatures / SD Status
 *  - Update the Prusa MMU2
 *  - Handle Joystick jogging
 */
void idle(TERN_(ADVANCED_PAUSE_FEATURE, bool no_stepper_sleep/*=false*/)) {

  // Core Marlin activities
  manage_inactivity(TERN_(ADVANCED_PAUSE_FEATURE, no_stepper_sleep));

  // Manage Heaters (and Watchdog)
  thermalManager.manage_heater();

  cancel_gohome();
  PrintOneKey();
  BlinkLed();
  BlinkFeedLed();
  BlinkRetractLed();
  home_key();
  LoadFilament();
  heat_protect();
  // Max7219 heartbeat, animation, etc
  TERN_(MAX7219_DEBUG, max7219.idle_tasks());

  // Return if setup() isn't completed
  if (marlin_state == MF_INITIALIZING) return;

  // Handle filament runout sensors
  TERN_(HAS_FILAMENT_SENSOR, runout.run());

  // Run HAL idle tasks
  #ifdef HAL_IDLETASK
    HAL_idletask();
  #endif

  // Handle Power-Loss Recovery
  #if ENABLED(POWER_LOSS_RECOVERY) && PIN_EXISTS(POWER_LOSS)
    if (printJobOngoing()) recovery.outage();
  #endif

  // Run StallGuard endstop checks
  #if ENABLED(SPI_ENDSTOPS)
    if (endstops.tmc_spi_homing.any
      && TERN1(IMPROVE_HOMING_RELIABILITY, ELAPSED(millis(), sg_guard_period))
    ) LOOP_L_N(i, 4) // Read SGT 4 times per idle loop
        if (endstops.tmc_spi_homing_check()) break;
  #endif

  // Handle SD Card insert / remove
  TERN_(SDSUPPORT, card.manage_media());

  // Handle USB Flash Drive insert / remove
  TERN_(USB_FLASH_DRIVE_SUPPORT, Sd2Card::idle());

  // Announce Host Keepalive state (if any)
  TERN_(HOST_KEEPALIVE_FEATURE, gcode.host_keepalive());

  // Update the Print Job Timer state
  TERN_(PRINTCOUNTER, print_job_timer.tick());

  // Update the Beeper queue
  TERN_(USE_BEEPER, buzzer.tick());

  // Handle UI input / draw events
  TERN(DWIN_CREALITY_LCD, DWIN_Update(), ui.update());

  // Run i2c Position Encoders
  #if ENABLED(I2C_POSITION_ENCODERS)
    static millis_t i2cpem_next_update_ms;
    if (planner.has_blocks_queued()) {
      const millis_t ms = millis();
      if (ELAPSED(ms, i2cpem_next_update_ms)) {
        I2CPEM.update();
        i2cpem_next_update_ms = ms + I2CPE_MIN_UPD_TIME_MS;
      }
    }
  #endif

  // Auto-report Temperatures / SD Status
  #if HAS_AUTO_REPORTING
    if (!gcode.autoreport_paused) {
      TERN_(AUTO_REPORT_TEMPERATURES, thermalManager.auto_report_temperatures());
      TERN_(AUTO_REPORT_SD_STATUS, card.auto_report_sd_status());
    }
  #endif

  // Update the Prusa MMU2
  TERN_(PRUSA_MMU2, mmu2.mmu_loop());

  // Handle Joystick jogging
  TERN_(POLL_JOG, joystick.inject_jog_moves());

  // Direct Stepping
  TERN_(DIRECT_STEPPING, page_manager.write_responses());

  #if HAS_TFT_LVGL_UI
    LV_TASK_HANDLER();
  #endif
}

/**
 * Kill all activity and lock the machine.
 * After this the machine will need to be reset.
 */
void kill(PGM_P const lcd_error/*=nullptr*/, PGM_P const lcd_component/*=nullptr*/, const bool steppers_off/*=false*/) {
  thermalManager.disable_all_heaters();

  TERN_(HAS_CUTTER, cutter.kill()); // Full cutter shutdown including ISR control

  SERIAL_ERROR_MSG(STR_ERR_KILLED);

  #if HAS_DISPLAY
    ui.kill_screen(lcd_error ?: GET_TEXT(MSG_KILLED), lcd_component ?: NUL_STR);
  #else
    UNUSED(lcd_error);
    UNUSED(lcd_component);
  #endif

  #ifdef ACTION_ON_KILL
    host_action_kill();
  #endif

  minkill(steppers_off);
}

void minkill(const bool steppers_off/*=false*/) {

  // Wait a short time (allows messages to get out before shutting down.
  for (int i = 1000; i--;) DELAY_US(600);

  cli(); // Stop interrupts

  // Wait to ensure all interrupts stopped
  for (int i = 1000; i--;) DELAY_US(250);

  // Reiterate heaters off
  thermalManager.disable_all_heaters();

  TERN_(HAS_CUTTER, cutter.kill());  // Reiterate cutter shutdown

  // Power off all steppers (for M112) or just the E steppers
  steppers_off ? disable_all_steppers() : disable_e_steppers();

  TERN_(PSU_CONTROL, PSU_OFF());

  TERN_(HAS_SUICIDE, suicide());

  #if HAS_KILL

    // Wait for kill to be released
    while (kill_state()) watchdog_refresh();

    // Wait for kill to be pressed
    while (!kill_state()) watchdog_refresh();

    void (*resetFunc)() = 0;      // Declare resetFunc() at address 0
    resetFunc();                  // Jump to address 0

  #else

    for (;;) watchdog_refresh();  // Wait for reset

  #endif
}

/**
 * Turn off heaters and stop the print in progress
 * After a stop the machine may be resumed with M999
 */
void stop() {
  thermalManager.disable_all_heaters(); // 'unpause' taken care of in here
  print_job_timer.stop();

  #if ENABLED(PROBING_FANS_OFF)
    if (thermalManager.fans_paused) thermalManager.set_fans_paused(false); // put things back the way they were
  #endif

  if (IsRunning()) {
    SERIAL_ERROR_MSG(STR_ERR_STOPPED);
    LCD_MESSAGEPGM(MSG_STOPPED);
    safe_delay(350);       // allow enough time for messages to get out before stopping
    marlin_state = MF_STOPPED;
  }
}

/**
 * Marlin entry-point: Set up before the program loop
 *  - Set up the kill pin, filament runout, power hold
 *  - Start the serial port
 *  - Print startup messages and diagnostics
 *  - Get EEPROM or default settings
 *  - Initialize managers for:
 *    • temperature
 *    • planner
 *    • watchdog
 *    • stepper
 *    • photo pin
 *    • servos
 *    • LCD controller
 *    • Digipot I2C
 *    • Z probe sled
 *    • status LEDs
 *    • Max7219
 */
void setup() {

  #if ENABLED(MARLIN_DEV_MODE)
    auto log_current_ms = [&](PGM_P const msg) {
      SERIAL_ECHO_START();
      SERIAL_CHAR('['); SERIAL_ECHO(millis()); SERIAL_ECHOPGM("] ");
      serialprintPGM(msg);
      SERIAL_EOL();
    };
    #define SETUP_LOG(M) log_current_ms(PSTR(M))
  #else
    #define SETUP_LOG(...) NOOP
  #endif
  #define SETUP_RUN(C) do{ SETUP_LOG(STRINGIFY(C)); C; }while(0)

  #if EITHER(DISABLE_DEBUG, DISABLE_JTAG)
    // Disable any hardware debug to free up pins for IO
    #if ENABLED(DISABLE_DEBUG) && defined(JTAGSWD_DISABLE)
      JTAGSWD_DISABLE();
    #elif defined(JTAG_DISABLE)
      JTAG_DISABLE();
    #else
      #error "DISABLE_(DEBUG|JTAG) is not supported for the selected MCU/Board."
    #endif
  #endif

  #if NUM_SERIAL > 0
    MYSERIAL0.begin(BAUDRATE);
    uint32_t serial_connect_timeout = millis() + 1000UL;
    while (!MYSERIAL0 && PENDING(millis(), serial_connect_timeout)) { /*nada*/ }
    #if HAS_MULTI_SERIAL
      MYSERIAL1.begin(BAUDRATE);
      serial_connect_timeout = millis() + 1000UL;
      while (!MYSERIAL1 && PENDING(millis(), serial_connect_timeout)) { /*nada*/ }
    #endif
    SERIAL_ECHO_MSG("start");
  #endif

  SETUP_RUN(HAL_init());

  #if HAS_L64XX
    SETUP_RUN(L64xxManager.init());  // Set up SPI, init drivers
  #endif

  #if ENABLED(SMART_EFFECTOR) && PIN_EXISTS(SMART_EFFECTOR_MOD)
    OUT_WRITE(SMART_EFFECTOR_MOD_PIN, LOW);   // Put Smart Effector into NORMAL mode
  #endif

  #if HAS_FILAMENT_SENSOR
    SETUP_RUN(runout.setup());
  #endif

  #if ENABLED(POWER_LOSS_RECOVERY)
    SETUP_RUN(recovery.setup());
  #endif

  SETUP_RUN(setup_killpin());

  #if HAS_TMC220x
    SETUP_RUN(tmc_serial_begin());
  #endif

  SETUP_RUN(setup_powerhold());

  #if HAS_STEPPER_RESET
    SETUP_RUN(disableStepperDrivers());
  #endif

  #if HAS_TMC_SPI
    #if DISABLED(TMC_USE_SW_SPI)
      SETUP_RUN(SPI.begin());
    #endif
    SETUP_RUN(tmc_init_cs_pins());
  #endif

  #ifdef BOARD_INIT
    SETUP_LOG("BOARD_INIT");
    BOARD_INIT();
  #endif
    //Threed buttons
    SET_OUTPUT(MT_PRINT_LED);
    SET_INPUT_PULLUP(FEED_PIN);
    SET_INPUT_PULLUP(RETRACT_PIN);

    pinMode(FEED_PIN, INPUT_PULLUP);
    pinMode(FEED_LED, OUTPUT);
    WRITE(FEED_LED, 1);

    pinMode(RETRACT_PIN, INPUT_PULLUP);
    pinMode(RETRACT_LED, OUTPUT);
    WRITE(RETRACT_LED, 1);

    pinMode(HOME_PIN, INPUT_PULLUP);
    pinMode(HOME_LED, OUTPUT);
    WRITE(HOME_LED, 1);

    BLINK_LED(4000);
    SETUP_RUN(esp_wifi_init());

    // Check startup - does nothing if bootloader sets MCUSR to 0
    const byte mcu = HAL_get_reset_source();
    if (mcu & RST_POWER_ON) SERIAL_ECHOLNPGM(STR_POWERUP);
    if (mcu & RST_EXTERNAL) SERIAL_ECHOLNPGM(STR_EXTERNAL_RESET);
    if (mcu & RST_BROWN_OUT) SERIAL_ECHOLNPGM(STR_BROWNOUT_RESET);
    if (mcu & RST_WATCHDOG) SERIAL_ECHOLNPGM(STR_WATCHDOG_RESET);
    if (mcu & RST_SOFTWARE) SERIAL_ECHOLNPGM(STR_SOFTWARE_RESET);
    HAL_clear_reset_source();

    serialprintPGM(GET_TEXT(MSG_MARLIN));
    SERIAL_CHAR(' ');
    SERIAL_ECHOLNPGM(SHORT_BUILD_VERSION);
    SERIAL_EOL();

  #if defined(STRING_DISTRIBUTION_DATE) && defined(STRING_CONFIG_H_AUTHOR)
    SERIAL_ECHO_MSG(
      STR_CONFIGURATION_VER
      STRING_DISTRIBUTION_DATE
      STR_AUTHOR STRING_CONFIG_H_AUTHOR
    );
    SERIAL_ECHO_MSG("Compiled: " __DATE__);
  #endif

  SERIAL_ECHO_START();
  SERIAL_ECHOLNPAIR(STR_FREE_MEMORY, freeMemory(), STR_PLANNER_BUFFER_BYTES, (int)sizeof(block_t) * (BLOCK_BUFFER_SIZE));

  // Set up LEDs early
  #if HAS_COLOR_LEDS
    SETUP_RUN(leds.setup());
  #endif

  #if ENABLED(USE_CONTROLLER_FAN)     // Set up fan controller to initialize also the default configurations.
    SETUP_RUN(controllerFan.setup());
  #endif

  // UI must be initialized before EEPROM
  // (because EEPROM code calls the UI).

  #if ENABLED(DWIN_CREALITY_LCD)
    delay(800);   // Required delay (since boot?)
    SERIAL_ECHOPGM("\nDWIN handshake ");
    if (DWIN_Handshake()) SERIAL_ECHOLNPGM("ok."); else SERIAL_ECHOLNPGM("error.");
    DWIN_Frame_SetDir(1); // Orientation 90°
    DWIN_UpdateLCD();     // Show bootscreen (first image)
  #else
    SETUP_RUN(ui.init());
    #if HAS_SPI_LCD && ENABLED(SHOW_BOOTSCREEN)
      SETUP_RUN(ui.show_bootscreen());
    #endif
    SETUP_RUN(ui.reset_status());     // Load welcome message early. (Retained if no errors exist.)
  #endif

  #if BOTH(SDSUPPORT, SDCARD_EEPROM_EMULATION)
    SETUP_RUN(card.mount());          // Mount media with settings before first_load
  #endif

  SETUP_RUN(settings.first_load());   // Load data from EEPROM if available (or use defaults)
                                      // This also updates variables in the planner, elsewhere

  #if ENABLED(TOUCH_BUTTONS)
    SETUP_RUN(touch.init());
  #endif

  TERN_(HAS_M206_COMMAND, current_position += home_offset); // Init current position based on home_offset

  sync_plan_position();               // Vital to init stepper/planner equivalent for current_position

  SETUP_RUN(thermalManager.init());   // Initialize temperature loop

  SETUP_RUN(print_job_timer.init());  // Initial setup of print job timer

  SETUP_RUN(endstops.init());         // Init endstops and pullups

  SETUP_RUN(stepper.init());          // Init stepper. This enables interrupts!

  #if HAS_SERVOS
    SETUP_RUN(servo_init());
  #endif

  #if HAS_Z_SERVO_PROBE
    SETUP_RUN(probe.servo_probe_init());
  #endif

  #if HAS_PHOTOGRAPH
    OUT_WRITE(PHOTOGRAPH_PIN, LOW);
  #endif

  #if HAS_CUTTER
    SETUP_RUN(cutter.init());
  #endif

  #if ENABLED(COOLANT_MIST)
    OUT_WRITE(COOLANT_MIST_PIN, COOLANT_MIST_INVERT);   // Init Mist Coolant OFF
  #endif
  #if ENABLED(COOLANT_FLOOD)
    OUT_WRITE(COOLANT_FLOOD_PIN, COOLANT_FLOOD_INVERT); // Init Flood Coolant OFF
  #endif

  #if HAS_BED_PROBE
    SETUP_RUN(endstops.enable_z_probe(false));
  #endif

  #if HAS_STEPPER_RESET
    SETUP_RUN(enableStepperDrivers());
  #endif

  #if HAS_I2C_DIGIPOT
    SETUP_RUN(digipot_i2c_init());
  #endif

  #if ENABLED(DAC_STEPPER_CURRENT)
    SETUP_RUN(dac_init());
  #endif

  #if EITHER(Z_PROBE_SLED, SOLENOID_PROBE) && HAS_SOLENOID_1
    OUT_WRITE(SOL1_PIN, LOW); // OFF
  #endif

  #if HAS_HOME
    SET_INPUT_PULLUP(HOME_PIN);
  #endif

  #if PIN_EXISTS(STAT_LED_RED)
    OUT_WRITE(STAT_LED_RED_PIN, LOW); // OFF
  #endif

  #if PIN_EXISTS(STAT_LED_BLUE)
    OUT_WRITE(STAT_LED_BLUE_PIN, LOW); // OFF
  #endif

  #if HAS_CASE_LIGHT
    #if DISABLED(CASE_LIGHT_USE_NEOPIXEL)
      if (PWM_PIN(CASE_LIGHT_PIN)) SET_PWM(CASE_LIGHT_PIN); else SET_OUTPUT(CASE_LIGHT_PIN);
    #endif
    SETUP_RUN(update_case_light());
  #endif

  #if ENABLED(MK2_MULTIPLEXER)
    SETUP_LOG("MK2_MULTIPLEXER");
    SET_OUTPUT(E_MUX0_PIN);
    SET_OUTPUT(E_MUX1_PIN);
    SET_OUTPUT(E_MUX2_PIN);
  #endif

  #if HAS_FANMUX
    SETUP_RUN(fanmux_init());
  #endif

  #if ENABLED(MIXING_EXTRUDER)
    SETUP_RUN(mixer.init());
  #endif

  #if ENABLED(BLTOUCH)
    SETUP_RUN(bltouch.init(/*set_voltage=*/true));
  #endif

  #if ENABLED(I2C_POSITION_ENCODERS)
    SETUP_RUN(I2CPEM.init());
  #endif

  #if ENABLED(EXPERIMENTAL_I2CBUS) && I2C_SLAVE_ADDRESS > 0
    SETUP_LOG("i2c...");
    i2c.onReceive(i2c_on_receive);
    i2c.onRequest(i2c_on_request);
  #endif

  #if DO_SWITCH_EXTRUDER
    SETUP_RUN(move_extruder_servo(0));  // Initialize extruder servo
  #endif

  #if ENABLED(SWITCHING_NOZZLE)
    SETUP_LOG("SWITCHING_NOZZLE");
    // Initialize nozzle servo(s)
    #if SWITCHING_NOZZLE_TWO_SERVOS
      lower_nozzle(0);
      raise_nozzle(1);
    #else
      move_nozzle_servo(0);
    #endif
  #endif

  #if ENABLED(MAGNETIC_PARKING_EXTRUDER)
    SETUP_RUN(mpe_settings_init());
  #endif

  #if ENABLED(PARKING_EXTRUDER)
    SETUP_RUN(pe_solenoid_init());
  #endif

  #if ENABLED(SWITCHING_TOOLHEAD)
    SETUP_RUN(swt_init());
  #endif

  #if ENABLED(ELECTROMAGNETIC_SWITCHING_TOOLHEAD)
    SETUP_RUN(est_init());
  #endif

  #if ENABLED(USE_WATCHDOG)
    SETUP_RUN(watchdog_init());       // Reinit watchdog after HAL_get_reset_source call
  #endif

  #if ENABLED(EXTERNAL_CLOSED_LOOP_CONTROLLER)
    SETUP_RUN(closedloop.init());
  #endif

  #ifdef STARTUP_COMMANDS
    SETUP_LOG("STARTUP_COMMANDS");
    queue.inject_P(PSTR(STARTUP_COMMANDS));
  #endif

  #if ENABLED(HOST_PROMPT_SUPPORT)
    SETUP_RUN(host_action_prompt_end());
  #endif

  #if HAS_TRINAMIC_CONFIG && DISABLED(PSU_DEFAULT_OFF)
    SETUP_RUN(test_tmc_connection(true, true, true, true));
  #endif

  #if ENABLED(PRUSA_MMU2)
    SETUP_RUN(mmu2.init());
  #endif

  #if ENABLED(IIC_BL24CXX_EEPROM)
    BL24CXX::init();
    const uint8_t err = BL24CXX::check();
    SERIAL_ECHO_TERNARY(err, "BL24CXX Check ", "failed", "succeeded", "!\n");
  #endif

  #if ENABLED(DWIN_CREALITY_LCD)
    Encoder_Configuration();
    HMI_Init();
    HMI_StartFrame(true);
  #endif

  #if HAS_SERVICE_INTERVALS && DISABLED(DWIN_CREALITY_LCD)
    ui.reset_status(true);  // Show service messages or keep current status
  #endif

  #if ENABLED(MAX7219_DEBUG)
    SETUP_RUN(max7219.init());
  #endif

  #if ENABLED(DIRECT_STEPPING)
    SETUP_RUN(page_manager.init());
  #endif

  #if HAS_TFT_LVGL_UI
    if (!card.isMounted()) SETUP_RUN(card.mount()); // Mount SD to load graphics and fonts
    SETUP_RUN(tft_lvgl_init());
  #endif

  marlin_state = MF_RUNNING;

  SETUP_LOG("setup() completed.");
}

/**
 * The main Marlin program loop
 *
 *  - Call idle() to handle all tasks between G-code commands
 *      Note that no G-codes from the queue can be executed during idle()
 *      but many G-codes can be called directly anytime like macros.
 *  - Check whether SD card auto-start is needed now.
 *  - Check whether SD print finishing is needed now.
 *  - Run one G-code command from the immediate or main command queue
 *    and open up one space. Commands in the main queue may come from sd
 *    card, host, or by direct injection. The queue will continue to fill
 *    as long as idle() or manage_inactivity() are being called.
 */
void loop() {
  do {
    idle();

    #if ENABLED(SDSUPPORT)
      card.checkautostart();
      if (card.flag.abort_sd_printing) abortSDPrinting();
      if (marlin_state == MF_SD_COMPLETE) finishSDPrinting();
    #endif

    queue.advance();

    endstops.event_handler();

    TERN_(HAS_TFT_LVGL_UI, printer_state_polling());

  } while (ENABLED(__AVR__)); // Loop forever on slower (AVR) boards
}
