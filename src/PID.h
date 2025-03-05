#ifndef PID_H
#define PID_H
#include "FastAccelStepper.h"

void updatePID();
void e_isr();
void s_isr();
void stepper_setup();
extern double Kp;
extern double Ki;
extern double Kd;
extern double e_rpm_m;
extern double s_rpm_m;
extern double r_m;
extern double r_t;
extern double error;
extern double error_integ;
extern double error_deriv;
extern double e_rpm_t;
extern double target_pos_inch;
extern FastAccelStepper *stepper;

#endif // PID_H