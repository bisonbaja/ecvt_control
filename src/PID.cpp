#include "PID.h"
#include "config.h"
#include "utils.h"
#include <Arduino.h>

const float e_rpm_const = 60000000.0 / ENGINE_NUM_MAGS; // 60s/min * 1000ms/s / num_mags
const float s_rpm_const = 60000000.0 / SECONDARY_NUM_MAGS;

const float model_r[] = {0.900000,1.150000,1.400000,1.650000,1.900000,2.150000,2.400000,2.650000,2.900000,3.150000,3.400000,3.650000,3.900000};
const float model_P[] = {0.906800,0.739412,0.603112,0.491077,0.397937,0.319600,0.252978,0.195733,0.146079,0.102643,0.064352,0.030362,0.000000};
const unsigned int model_size = sizeof(model_r) / sizeof(model_r[0]);

float error = 0.0;
float last_error = 0.0;
float error_deriv = 0.0;
float error_integ = 0.0;
float max_error_integ = abs(MAX_POS_INCH / KI_DEFAULT)/2;
float max_error_deriv = MAX_ERROR_DERIV;
float e_rpm_t = E_RPM_TARGET_DEFAULT;
float e_rpm_m = 0.0;
float s_rpm_m = 0.0;
float r_t = 3.9;
float r_m = 3.9;
float ff_pos = 0;
float target_pos_inch = 0.0;
float last_target_pos_inch = 0.0;
float max_pos_inch = MAX_POS_INCH;
float engage_pos = ENGAGE_POS;


float Ki = KI_DEFAULT;
float Kp = KP_DEFAULT;
float Kd = KD_DEFAULT;

runMode current_mode = DEBUG;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
void stepper_setup() {
    // Initialize stepper for accel control
    engine.init(0);
    stepper = engine.stepperConnectToPin(STEP_PIN);
    stepper->setDirectionPin(DIR_PIN);
    stepper->setSpeedInHz(STEPPER_MAX_SPEED);
    stepper->setAcceleration(STEPPER_MAX_ACCEL);
}

const unsigned int delta_t = PID_TASK_DELAY; // in ms
const float dt = delta_t * 0.001; // in seconds for PID

volatile unsigned long e_new_pulse = 0;
volatile unsigned long e_last_pulse = 0;
volatile unsigned int e_delta = 0;
volatile unsigned int e_last_delta = 0;
volatile unsigned long e_deltas[ENGINE_AVG] = {0};
volatile unsigned short e_delta_i = 0;

volatile unsigned long s_new_pulse = 0;
volatile unsigned long s_last_pulse = 0;
volatile unsigned int s_delta = 0;
volatile unsigned int s_last_delta = 0;
volatile unsigned long s_deltas[SECONDARY_AVG] = {0};
volatile unsigned short s_delta_i = 0;

IRAM_ATTR void e_isr() { // 4 micros
    e_new_pulse = micros();
    e_delta = e_new_pulse - e_last_pulse;
    e_last_pulse = e_new_pulse;
    e_deltas[e_delta_i] = e_delta;
    e_last_delta = e_delta;

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

IRAM_ATTR void s_isr() {
    s_new_pulse = micros();
    s_delta = s_new_pulse - s_last_pulse;
    s_last_pulse = s_new_pulse;
    s_deltas[s_delta_i] = s_delta;
    s_last_delta = s_delta;

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
    // Update RPM measurements and reject odd readings (<5 or >10000)
    e_rpm_m = e_rpm_const / e_avg_delta();
    s_rpm_m = s_rpm_const / s_avg_delta();
    if (e_rpm_m > 10000) e_rpm_m = FAKE_DEF_RPM;
    if (s_rpm_m > 10000) s_rpm_m = FAKE_DEF_RPM;
    r_m = e_rpm_m / s_rpm_m;

    if (current_mode == RPM) r_t = e_rpm_t / s_rpm_m;

    last_error = error;
    error = r_t - r_m;
    error_deriv = (error - last_error) / dt;
    error_deriv = symminmax(error_deriv, max_error_deriv);
    error_integ += error * dt;
    max_error_integ = abs(max_pos_inch/Ki)/8;
    error_integ = symminmax(error_integ, max_error_integ);
    ff_pos = interpolate(r_t, model_r, model_P, model_size);
    
    if (current_mode == RPM || current_mode == MANUAL) {
        if (e_rpm_m > 1000) target_pos_inch = engage_pos - 0.050; // Sets a floor at engagement position to prevent rattling of yoke
        target_pos_inch = engage_pos + ff_pos + Kp * error + Ki * error_integ + Kd * error_deriv;

    }

    // Keep position within bounds
    if (target_pos_inch < 0) target_pos_inch = 0;
    if (target_pos_inch > max_pos_inch) target_pos_inch = max_pos_inch; 
    if (e_rpm_m < 1000 && target_pos_inch > engage_pos - 0.05) target_pos_inch = engage_pos - 0.05; // Don't allow engagement under 1000 RPM

    if (abs(error) < TOLERANCE) {
        target_pos_inch = last_target_pos_inch;
    }

    stepper->moveTo(target_pos_inch * STEPS_PER_INCH);
    last_target_pos_inch = target_pos_inch;

    delay(delta_t);
}