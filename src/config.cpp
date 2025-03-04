#include "config.h"

// Tuning Parameters
double e_rpm_t = 0.0;
const double e_rpm_const = 1.0; // Example value, replace with actual
const double s_rpm_const = 1.0; // Example value, replace with actual
const unsigned int steps_per_linch = 200; // Example value, replace with actual
const unsigned int stepper_max_accel = 1000; // Example value, replace with actual
const unsigned int stepper_max_speed = 1000; // Example value, replace with actual
double Kp = 1.0; // Example value, replace with actual
double Ki = 0.0; // Example value, replace with actual
double Kd = 0.0; // Example value, replace with actual
const double max_pos_inch = 10.0; // Example value, replace with actual
const unsigned long delta_t = 100; // Example value, replace with actual
const double dt = 0.1; // Example value, replace with actual
const unsigned long log_delay = 1000; // Example value, replace with actual
unsigned long log_last_time = 0;

// Exported from Octave, currently unused
const double model_r[] = {0.0}; // Example values, replace with actual
const double model_P[] = {0.0}; // Example values, replace with actual

// Stepper Drive
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// Bluetooth Logging
BluetoothSerial SerialBT;

// Accelerometer Gyroscope
MPU6050 mpu;
int const INTERRUPT_PIN = 2; // Example value, replace with actual

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;
uint8_t MPUIntStatus = 0;
uint8_t devStatus = 0;
uint16_t packetSize = 0;
uint8_t FIFOBuffer[64];

/*---Orientation/Motion Variables---*/ 
Quaternion q;
VectorInt16 aa;
VectorInt16 gy;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];

volatile bool MPUInterrupt = false;

double error = 0.0;
double last_error = 0.0;
double error_deriv = 0.0;
double error_integ = 0.0;
double max_error_integ = 100.0; // Example value, replace with actual
double min_error_integ = -100.0; // Example value, replace with actual
double e_rpm_m = 0.0;
double s_rpm_m = 0.0;
double r_t = 0.0;
double r_m = 0.0;
double target_pos_inch = 0.0;

volatile unsigned long e_new_pulse = 0;
volatile unsigned long e_last_pulse = 0;
volatile unsigned long e_last_delta = 0;
volatile unsigned long e_deltas[ENGINE_AVG] = {0};
volatile unsigned short e_delta_i = 0;

volatile unsigned long s_new_pulse = 0;
volatile unsigned long s_last_pulse = 0;
volatile unsigned long s_last_delta = 0;
volatile unsigned long s_deltas[SECONDARY_AVG] = {0};
volatile unsigned short s_delta_i = 0;

Param params[] = {
    {"Kp", &Kp},
    {"Ki", &Ki},
    {"Kd", &Kd},
    {"e_rpm_t", &e_rpm_t},
    {"max_error_integ", &max_error_integ},
    {"min_error_integ", &min_error_integ}
};

Command commands[] = {
    {"set", set},
    {"zero", zero},
    {"max", max}
};