// Main file for Baja CVT Controller
// Program name: eCVT Control
// Christian Wilson 11/8/24
// Last major update 1/28/25

/* DONE:
 - stepper control
 - PID feedback
 - data logging
 - ready/error lights
*/

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <AccelStepper.h>
#define ENGINE_TACH_PIN     4
#define SECONDARY_TACH_PIN  5
#define ENGINE_NUM_MAGS 4
#define SECONDARY_NUM_MAGS 4

#define STEP_PIN 6
#define DIR_PIN 7
#define READY_LED 8
#define ERROR_LED 9
//#define STEPPER_ENABLE

// Tuning Parameters:
const double e_rpm_t = 3000;
const double e_rpm_const = 60000000.0/ENGINE_NUM_MAGS;
const double s_rpm_const = 60000000.0/SECONDARY_NUM_MAGS;
const unsigned int steps_per_linch = 800; // 4 rev/in * 200 step/rev 
const double Kp = 0.25;
const double Ki = 0.01;
const double Kd = 0.25;
const double max_pos_inch = 0.925;
unsigned long delta_t = 100; // millis -> 100Hz 

#ifdef STEPPER_ENABLE
// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
#endif

const double model_r[] = {0.9,    1,     1.1,    1.2,    1.3,    1.4,    1.5,    1.6,   1.7,    1.8,    1.9,    2,      2.1,    2.2,    2.3,    2.4,    2.5,    2.6,    2.7,    2.8,    2.9,    3,      3.1,    3.2,    3.3,    3.4,    3.5,    3.6,    3.7,    3.8,    3.9};
const double model_P[] = {2.6793, 2.5465,2.4243, 2.3119, 2.2083, 2.1126, 2.0242, 1.9423,1.8662,	1.7956,	1.7298,	1.6683,	1.6109,	1.5571,	1.5067,	1.4593,	1.4146,	1.3726,	1.3329,	1.2953,	1.2598,	1.2261,	1.1941,	1.1637,	1.1348,	1.1073,	1.081,	1.056,	1.032,	1.0091,	0.98719};

double error;
double last_error;
double error_deriv;
double error_integ;
double e_rpm_m = 0;
double s_rpm_m = 0;
double r_t = 3.9;
double r_m = 3.9;
double target_pos_inch = 0;

unsigned long start_time;
unsigned long last_time;

File log_file;
unsigned long lines_written = 0;
unsigned long last_flush = 0;
char new_line[256];

volatile unsigned long e_new_pulse = 0;
volatile unsigned long e_last_pulse = 0;
volatile unsigned long e_last_delta = 0;
volatile unsigned long e_deltas[ENGINE_NUM_MAGS];
volatile unsigned short e_delta_i = 0;

volatile unsigned long s_new_pulse = 0;
volatile unsigned long s_last_pulse = 0;
volatile unsigned long s_last_delta = 0;
volatile unsigned long s_deltas[SECONDARY_NUM_MAGS];
volatile unsigned short s_delta_i = 0;

void e_isr() { // 4 micros
    e_new_pulse = micros();
    e_last_delta = e_new_pulse - e_last_pulse;
    e_last_pulse = e_new_pulse;
    e_deltas[e_delta_i] = e_last_delta;

    if (e_delta_i == ENGINE_NUM_MAGS-1) e_delta_i = 0;
    else e_delta_i++;
}

double e_avg_delta() {
    double ret_val=0;
    for (byte i = 0; i < ENGINE_NUM_MAGS; i++) {
        ret_val += e_deltas[i];
    }
    return ret_val/ENGINE_NUM_MAGS;
}

void s_isr() {
    s_new_pulse = micros();
    s_last_delta = s_new_pulse - s_last_pulse;
    s_last_pulse = s_new_pulse;
    s_deltas[s_delta_i] = s_last_delta;

    if (s_delta_i == SECONDARY_NUM_MAGS-1) s_delta_i = 0;
    else s_delta_i++;
}

double s_avg_delta() {
    double ret_val=0;
    for (byte i = 0; i < SECONDARY_NUM_MAGS; i++) {
        ret_val += s_deltas[i];
    }
    return ret_val/SECONDARY_NUM_MAGS;
}

// Compute RPM values, calculate ratios, error, and new target position
void updatePID() { // ~200 micros
    e_rpm_m = e_rpm_const/e_avg_delta();
    s_rpm_m = s_rpm_const/s_avg_delta();
    if (e_rpm_m < 0 or e_rpm_m > 5000) e_rpm_m = 100;
    if (s_rpm_m < 0 or s_rpm_m > 5000) s_rpm_m = 100;
    r_m = e_rpm_m / s_rpm_m;
    r_t = e_rpm_t / s_rpm_m;

    last_error = error;
    error = r_t - r_m;
    error_deriv = (error-last_error)/delta_t;
    error_integ += error*delta_t;
    target_pos_inch = Kp*error + Ki*error_integ + Kd*error_deriv;
    if (target_pos_inch < 0) target_pos_inch = 0;
    if (target_pos_inch > max_pos_inch) target_pos_inch = max_pos_inch;
}

void fail() {
    digitalWrite(READY_LED, LOW);
    while (true) {
        digitalWrite(ERROR_LED, HIGH);
        delay(1000);
        digitalWrite(ERROR_LED, LOW);
        delay(1000);
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    // Initialize Tach Inputs
    attachInterrupt(digitalPinToInterrupt(ENGINE_TACH_PIN), e_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(SECONDARY_TACH_PIN), s_isr, RISING);

    // Init lights
    pinMode(READY_LED, OUTPUT);
    pinMode(ERROR_LED, OUTPUT);
    digitalWrite(ERROR_LED, HIGH);

    // Initialize SD comms
    while (!SD.begin(SDCARD_SS_PIN)) {
        if (millis() > 5000) fail();
    }

    // Generate filename
    char filename[32] = "cvt_data.csv";
    unsigned int file_index = 0;
    while (SD.exists(filename)) {
        file_index++;
        sprintf(filename, "cvt_data_%u.csv", file_index);
        if (file_index > 1000) fail();
    }
    Serial.println(filename);
    log_file = SD.open(filename, FILE_WRITE);

    // Mark start time
    start_time = millis();
    last_time = start_time;

    #ifdef STEPPER_ENABLE
    // Initialize stepper for accel control
    stepper.setMaxSpeed(1*steps_per_linch);
    stepper.setAcceleration(2000);
    #endif
    delay(1000);
    log_file.println("time (ms), eRPM, sRPM, r_t, r_m, target_pos, curr_pos");

    digitalWrite(READY_LED, HIGH);
    digitalWrite(ERROR_LED, LOW);
}

void loop() {
    // Check if enough time has passed to recompute target position
    if (millis() - last_time >= delta_t) { // this routine takes around 3000 micros
        last_time += delta_t;
        updatePID();

        #ifdef STEPPER_ENABLE
        stepper.moveTo(target_pos_inch*steps_per_linch);
        #endif

        // LOG FORMAT
        // time (ms), eRPM, sRPM, r_t, r_m, target_pos, curr_pos
        
        sprintf(new_line, "%u, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", 
            last_time, 
            e_rpm_m,
            s_rpm_m,
            r_t,
            r_m,
            target_pos_inch,
            #ifdef STEPPER_ENABLE
            (stepper.currentPosition()/double(steps_per_linch) )
            #else
            0
            #endif
            );
        char serial_line[256];
        sprintf(serial_line, ">Engine RPM:%.3f\n>Secondary RPM:%.3f\n>Ratio:%.3f", e_rpm_m, s_rpm_m, r_m);
        Serial.println(serial_line);
        log_file.println(new_line);
        lines_written++;
        if (lines_written - last_flush > 500) {
            log_file.flush();
            last_flush = lines_written;
        }
    }
    #ifdef STEPPER_ENABLE
    stepper.run();
    #endif
}