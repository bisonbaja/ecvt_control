#ifndef PID_H
#define PID_H

#ifdef ARDUINO_ARCH_ESP32
#include "FastAccelStepper.h"
#elif defined(ARDUINO_ARCH_SAMD)
#include <AccelStepper.h>
#endif

void updatePID();
void e_isr();
void s_isr();
void stepper_setup();
extern float Kp;
extern float Ki;
extern float Kd;
extern float e_rpm_m;
extern float s_rpm_m;
extern float r_m;
extern float r_t;
extern float error;
extern float error_integ;
extern float error_deriv;
extern float e_rpm_t;
extern float target_pos_inch;

#ifdef ARDUINO_ARCH_ESP32
extern FastAccelStepper *stepper;
#elif defined(ARDUINO_ARCH_SAMD)
extern AccelStepper stepper;
#endif

#endif // PID_H