#ifndef CONFIG_H
#define CONFIG_H

/* BEGIN CONFIGURATION */

//#define USE_MPU
#define USE_SERIAL // currently set up for debug
#define USE_BT // currently used for data
//#define USE_SD 
#define SERIAL_BAUDRATE 115200
#define BT_NAME "ESP32"

// PIN DEFINITIONS
#define ENGINE_TACH_PIN     36
#define SECONDARY_TACH_PIN  37
#define STEP_PIN 25
#define DIR_PIN 26
#define READY_LED 10
#define ERROR_LED 9
/* SPI (SD card) pins for ESP32 are as follows:
CS: 5
MOSI: 23
CLK: 18
MISO: 19

*/
// PID DEFAULT PARAMETERS
#define KP_DEFAULT -1
#define KI_DEFAULT -0.25
#define KD_DEFAULT 0.00
#define E_RPM_TARGET_DEFAULT 3000

// TACH CONFIGURATION
#define ENGINE_NUM_MAGS 1
#define SECONDARY_NUM_MAGS 1
#define ENGINE_AVG 4
#define SECONDARY_AVG 4
#define FAKE_DEF_RPM 3000

// Task Priorities
#define PID_TASK_PRIORITY 2
#define LOG_TASK_PRIORITY 1
#define SD_TASK_PRIORITY 1
#define COMMAND_TASK_PRIORITY 1
#define STEP_TASK_PRIORITY 5 // only applies to SAMD

// Task Delays in ms
#define PID_TASK_DELAY 2
#define LOG_TASK_DELAY 50
#define SD_TASK_DELAY 10
#define SERIAL_COMMAND_DELAY 250
#define STEP_DELAY_US 50

// Mechanical Configuration
#define STEPPER_MAX_ACCEL 6400
#define STEPPER_MAX_SPEED 6400
#define STEPS_PER_LINCH 800
#define MAX_POS_INCH 0.925

/* END CONFIGURATION */

#ifdef USE_BT
#include <BluetoothSerial.h>
extern BluetoothSerial BT;
#endif // USE_BT

#ifdef USE_SD
#include <SD.h>
#endif

#endif // CONFIG_H