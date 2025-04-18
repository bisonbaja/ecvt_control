#ifndef PID_H
#define PID_H

#include "FastAccelStepper.h"

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
extern float max_error_deriv;
extern float engage_pos;
extern float max_pos_inch;
extern float ff_pos;

enum runMode {
    DEBUG,
    RPM,
    MANUAL
};

extern runMode current_mode;

extern FastAccelStepper *stepper;

#endif // PID_H