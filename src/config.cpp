#include "config.h"

// Tuning Parameters
double e_rpm_t = 3000;
const double e_rpm_const = 60000000.0 / ENGINE_NUM_MAGS;
const double s_rpm_const = 60000000.0 / SECONDARY_NUM_MAGS;
const unsigned int steps_per_linch = 800; // 4 rev/in * 200 step/rev 
const unsigned int stepper_max_accel = steps_per_linch * 2;
const unsigned int stepper_max_speed = steps_per_linch * 4;
double Kp = -0.25;
double Ki = -0.5;
double Kd = -0.02;
const double max_pos_inch = 0.925 * 2;
const unsigned long delta_t = 20; // millis -> 100Hz 
const double dt = delta_t * 0.001; // in seconds for PID
const unsigned long log_delay = 200;
unsigned long log_last_time = 0;

// Exported from Octave, currently unused
const double model_r[] = {0.9, 1, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8, 3.9};
const double model_P[] = {1.0007, 0.9295, 0.86404, 0.80379, 0.74825, 0.697, 0.6496, 0.6057, 0.56497, 0.5271, 0.49182, 0.4589, 0.42813, 0.39931, 0.37227, 0.34686, 0.32295, 0.30041, 0.27913, 0.25901, 0.23996, 0.22191, 0.20477, 0.18849, 0.17299, 0.15824, 0.14417, 0.13073, 0.1179, 0.10563, 0.093882};

// Stepper Drive
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// Bluetooth Logging
BluetoothSerial SerialBT;

// Accelerometer Gyroscope
MPU6050 mpu;
int const INTERRUPT_PIN = 2;

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
double max_error_integ = 100.0;
double min_error_integ = -100.0;
double e_rpm_m = 0.0;
double s_rpm_m = 0.0;
double r_t = 3.9;
double r_m = 3.9;
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