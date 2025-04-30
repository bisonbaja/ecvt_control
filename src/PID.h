#ifndef PID_H
#define PID_H

#include "FastAccelStepper.h"

void updatePID();
void e_isr();
void s_isr();
void stepper_setup();

extern float Kp, Ki, Kd, e_rpm_m, s_rpm_m, r_m, r_t, error, error_integ, 
             error_deriv, e_rpm_t, target_pos_inch, max_error_deriv, 
             engage_pos, max_pos_inch, ff_pos;

extern float manual_ratios[];
extern const unsigned int num_ratios;
extern unsigned int current_ratio;

enum runMode {
    POSITION,
    RATIO,
    RPM,
};

extern runMode current_mode;

extern FastAccelStepper *stepper;

#endif // PID_H