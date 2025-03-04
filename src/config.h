#ifndef CONFIG_H
#define CONFIG_H

#include "FastAccelStepper.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "BluetoothSerial.h"

// PIN DEFINITIONS
#define ENGINE_TACH_PIN     4
#define SECONDARY_TACH_PIN  5
#define STEP_PIN 6
#define DIR_PIN 7
#define READY_LED 8
#define ERROR_LED 9

// TACH CONFIGURATION
#define ENGINE_NUM_MAGS 0.25
#define SECONDARY_NUM_MAGS 0.25
#define ENGINE_AVG 4
#define SECONDARY_AVG 4
#define FAKE_DEF_RPM 2000

// Tuning Parameters:
extern double e_rpm_t;
extern const double e_rpm_const;
extern const double s_rpm_const;
extern const unsigned int steps_per_linch;
extern const unsigned int stepper_max_accel;
extern const unsigned int stepper_max_speed;
extern double Kp;
extern double Ki;
extern double Kd;
extern const double max_pos_inch;
extern const unsigned long delta_t;
extern const double dt;
extern const unsigned long log_delay;
extern unsigned long log_last_time;

// Exported from Octave, currently unused
extern const double model_r[];
extern const double model_P[];

// Stepper Drive
extern FastAccelStepperEngine engine;
extern FastAccelStepper *stepper;

// Bluetooth Logging
extern BluetoothSerial SerialBT;

// Accelerometer Gyroscope
extern MPU6050 mpu;
extern int const INTERRUPT_PIN;

/*---MPU6050 Control/Status Variables---*/
extern bool DMPReady;
extern uint8_t MPUIntStatus;
extern uint8_t devStatus;
extern uint16_t packetSize;
extern uint8_t FIFOBuffer[64];

/*---Orientation/Motion Variables---*/ 
extern Quaternion q;
extern VectorInt16 aa;
extern VectorInt16 gy;
extern VectorInt16 aaReal;
extern VectorInt16 aaWorld;
extern VectorFloat gravity;
extern float euler[3];
extern float ypr[3];

void fail();

/*------Interrupt detection routine------*/
extern volatile bool MPUInterrupt;
void DMPDataReady();

void e_isr();
double e_avg_delta();
void s_isr();
double s_avg_delta();

struct Param {
    const char* name;
    double* address;
};

extern Param params[];

bool set(char* token);
bool zero(char* rest);
bool max(char* rest);

struct Command {
    const char* name;
    bool (*func)(char* rest);
};

extern Command commands[];

extern double error;
extern double last_error;
extern double error_deriv;
extern double error_integ;
extern double max_error_integ;
extern double min_error_integ;
extern double e_rpm_m;
extern double s_rpm_m;
extern double r_t;
extern double r_m;
extern double target_pos_inch;

extern volatile unsigned long e_new_pulse;
extern volatile unsigned long e_last_pulse;
extern volatile unsigned long e_last_delta;
extern volatile unsigned long e_deltas[ENGINE_AVG];
extern volatile unsigned short e_delta_i;

extern volatile unsigned long s_new_pulse;
extern volatile unsigned long s_last_pulse;
extern volatile unsigned long s_last_delta;
extern volatile unsigned long s_deltas[SECONDARY_AVG];
extern volatile unsigned short s_delta_i;

#endif // CONFIG_H