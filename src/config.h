#ifndef CONFIG_H
#define CONFIG_H

/* BEGIN CONFIGURATION */

// #define USE_MPU
#define USE_SERIAL // currently set up for debug
// #define USE_BT // currently used for data
#define USE_SD 
#define USE_STEPPER
// #define USE_SHIFTERS
#define SERIAL_BAUDRATE 576000
#define BT_NAME "ESP32_CVT"

// PIN DEFINITIONS
#define ENGINE_TACH_PIN     36
#define SECONDARY_TACH_PIN  37
#define STEP_PIN 25
#define DIR_PIN 26
#define READY_LED 10
#define ERROR_LED 9
#define UPSHIFT_PIN 11
#define DOWNSHIFT_PIN 12

/* SPI (SD card) pins for ESP32 are as follows:
CS: 5
MOSI: 23
CLK: 18
MISO: 19
*/

// PID DEFAULT PARAMETERS // using ziegler nichols
#define KP_DEFAULT -0.18
#define KI_DEFAULT -0.25
#define KD_DEFAULT -0.03
#define TOLERANCE 0.05 // error tolerance in ratio
#define MAX_ERROR_DERIV 20
#define E_RPM_TARGET_DEFAULT 3000

// TACH CONFIGURATION
#define ENGINE_NUM_MAGS 8
#define SECONDARY_NUM_MAGS 4
#define ENGINE_AVG 64
#define SECONDARY_AVG 32
#define FAKE_DEF_RPM 1
#define ENGINE_IDLE_MAX 2000

// Task Priorities
#define PID_TASK_PRIORITY 2
#define LOG_TASK_PRIORITY 1
#define SD_TASK_PRIORITY 1
#define COMMAND_TASK_PRIORITY 1
#define STEP_TASK_PRIORITY 5 // only applies to SAMD

// Task Delays in ms
#define PID_TASK_DELAY 1
#define LOG_TASK_DELAY 30
#define SD_TASK_DELAY 50
#define SERIAL_COMMAND_DELAY 250
#define STEP_DELAY_US 50

// Mechanical Configuration
#define STEPPER_MAX_ACCEL 3200
#define STEPPER_MAX_SPEED 1600
#define STEPS_PER_INCH 1600 // 200 steps/rev * 4 rev/in * 2 in/in leverage
#define MAX_POS_INCH 1.205
#define IDLE_POS 0.335
#define ENGAGE_POS 0.535
#define MANUAL_POS_STEP 0.100

/* END CONFIGURATION */

#ifdef USE_BT
#include <BluetoothSerial.h>
extern BluetoothSerial BT;
#endif // USE_BT

#if !defined(USE_BT) && defined(USE_SERIAL)
#define data_print(x) Serial.print(x)
#define data_println(x) Serial.println(x)
#define data_read(x) Serial.read(x)
#define data_available(x) Serial.available(x)
#endif

#if defined(USE_BT)
#define data_print(x) BT.print(x)
#define data_println(x) BT.println(x)
#define data_read(x) BT.read(x)
#define data_available(x) BT.available(x)
#endif

#if !defined(USE_BT) && !defined(USE_SERIAL)
#define data_print(x) 
#define data_println(x) 
#define data_read(x) 
#define data_available(x) 
#endif

#ifdef USE_SD
#include <SD.h>
#endif

#endif // CONFIG_H