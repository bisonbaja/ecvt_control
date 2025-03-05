#ifndef CONFIG_H
#define CONFIG_H

#define USE_MPU
//#define USE_SERIAL
#define USE_BT

// PIN DEFINITIONS
#define ENGINE_TACH_PIN     4
#define SECONDARY_TACH_PIN  5
#define STEP_PIN 6
#define DIR_PIN 7
#define READY_LED 8
#define ERROR_LED 9

// PID DEFAULT PARAMETERS
#define KP_DEFAULT -0.25
#define KI_DEFAULT -0.5
#define KD_DEFAULT -0.02
#define E_RPM_TARGET_DEFAULT 3000

// TACH CONFIGURATION
#define ENGINE_NUM_MAGS 0.25
#define SECONDARY_NUM_MAGS 0.25
#define ENGINE_AVG 4
#define SECONDARY_AVG 4
#define FAKE_DEF_RPM 2000

// Task Priorities
#define PID_TASK_PRIORITY 2
#define SERIAL_TASK_PRIORITY 1
#define COMMAND_TASK_PRIORITY 1

// Task Delays in ms
#define PID_TASK_DELAY 5
#define LOG_TASK_DELAY 200
#define SERIAL_COMMAND_DELAY 500

// Mechanical Configuration
#define STEPPER_MAX_ACCEL 800
#define STEPPER_MAX_SPEED 3200
#define STEPS_PER_LINCH 800
#define MAX_POS_INCH 0.925

#if defined (USE_SERIAL) && !defined (USE_BT)
#define Serial_begin(x) Serial.begin(x)
#define Serial_println(x) Serial.println(x)
#define Serial_print(x) Serial.print(x)
#define Serial_read(x) Serial.read(x)
#define Serial_available(x) Serial.available(x)
#endif // USE_SERIAL && !USE_BT

#ifndef USE_SERIAL
#define Serial_begin(x)
#define Serial_println(x)
#define Serial_print(x)
#define Serial_read(x)
#define Serial_available(x)
#endif // USE_SERIAL

#if defined (USE_BT) && defined (USE_SERIAL)
#include <BluetoothSerial.h>
extern BluetoothSerial SerialBT;
#define Serial_begin(x) SerialBT.begin(x)
#define Serial_println(x) SerialBT.println(x)
#define Serial_print(x) SerialBT.print(x)
#define Serial_read(x) SerialBT.read(x)
#define Serial_available(x) SerialBT.available(x)
#endif // USE_BT

#endif // CONFIG_H