#include "PID.h"
#include "config.h"
#include <Arduino.h>

const float e_rpm_const = 60000000.0 / ENGINE_NUM_MAGS; // 60s/min * 1000ms/s / num_mags
const float s_rpm_const = 60000000.0 / SECONDARY_NUM_MAGS;


// Exported from Octave, currently unused
const float model_r[] = {0.9, 1, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8, 3.9};
const float model_P[] = {1.0007, 0.9295, 0.86404, 0.80379, 0.74825, 0.697, 0.6496, 0.6057, 0.56497, 0.5271, 0.49182, 0.4589, 0.42813, 0.39931, 0.37227, 0.34686, 0.32295, 0.30041, 0.27913, 0.25901, 0.23996, 0.22191, 0.20477, 0.18849, 0.17299, 0.15824, 0.14417, 0.13073, 0.1179, 0.10563, 0.093882};
float error = 0.0;
float last_error = 0.0;
float error_deriv = 0.0;
float error_integ = 0.0;
const float max_pos_inch = 0.925 * 2; // *2 from yoke leverage
float max_error_integ = max_pos_inch / KI_DEFAULT;
float min_error_integ = -max_error_integ;
float e_rpm_t = E_RPM_TARGET_DEFAULT;
float e_rpm_m = 0.0;
float s_rpm_m = 0.0;
float r_t = 3.9;
float r_m = 3.9;
float target_pos_inch = 0.0;
float Ki = KI_DEFAULT;
float Kp = KP_DEFAULT;
float Kd = KD_DEFAULT;

// Stepper Drive
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void stepper_setup() {
    // Initialize stepper for accel control
    engine.init();
    stepper = engine.stepperConnectToPin(STEP_PIN);
    stepper->setDirectionPin(DIR_PIN);
    stepper->setSpeedInHz(STEPPER_MAX_SPEED);
    stepper->setAcceleration(STEPPER_MAX_ACCEL);
}

const unsigned int delta_t = PID_TASK_DELAY; // in ms
const float dt = delta_t * 0.001; // in seconds for PID

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

void e_isr() { // 4 micros
    e_new_pulse = micros();
    e_last_delta = e_new_pulse - e_last_pulse;
    e_last_pulse = e_new_pulse;
    e_deltas[e_delta_i] = e_last_delta;

    if (e_delta_i == ENGINE_AVG-1) e_delta_i = 0;
    else e_delta_i++;
}

float e_avg_delta() {
    float ret_val=0;
    for (byte i = 0; i < ENGINE_AVG; i++) {
        ret_val += e_deltas[i];
    }
    return ret_val/ENGINE_AVG;
}

void s_isr() {
    s_new_pulse = micros();
    s_last_delta = s_new_pulse - s_last_pulse;
    s_last_pulse = s_new_pulse;
    s_deltas[s_delta_i] = s_last_delta;

    if (s_delta_i == SECONDARY_AVG-1) s_delta_i = 0;
    else s_delta_i++;
}

float s_avg_delta() {
    float ret_val=0;
    for (byte i = 0; i < SECONDARY_AVG; i++) {
        ret_val += s_deltas[i];
    }
    return ret_val/SECONDARY_AVG;
}

void updatePID() {
    e_rpm_m = e_rpm_const / e_avg_delta();
    s_rpm_m = s_rpm_const / s_avg_delta();
    if (e_rpm_m <= 5 || e_rpm_m > 5000) e_rpm_m = FAKE_DEF_RPM;
    if (s_rpm_m <= 5 || s_rpm_m > 5000) s_rpm_m = FAKE_DEF_RPM;
    r_m = e_rpm_m / s_rpm_m;
    r_t = e_rpm_t / s_rpm_m;

    last_error = error;
    error = r_t - r_m;
    error_deriv = (error - last_error) / dt;
    error_integ += error * dt;

    if (error_integ > max_error_integ) error_integ = max_error_integ;
    if (error_integ < min_error_integ) error_integ = min_error_integ;

    // Output position = bias + P + I + D
    target_pos_inch = 0.5 * max_pos_inch + Kp * error + Ki * error_integ + Kd * error_deriv;

    if (target_pos_inch < 0) target_pos_inch = 0;
    if (target_pos_inch > max_pos_inch) target_pos_inch = max_pos_inch;

    stepper->moveTo(target_pos_inch * STEPS_PER_LINCH);

    delay(delta_t);
}