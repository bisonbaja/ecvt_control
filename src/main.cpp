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
#define TACH_SCALE          2475/128
#define STEP_PIN 6
#define DIR_PIN 7
#define READY_LED 8
#define ERROR_LED 9

volatile unsigned long e_new_pulse = 0;
volatile unsigned long e_last_pulse = 0;
volatile unsigned long e_last_delta = 0;
volatile unsigned long s_new_pulse = 0;
volatile unsigned long s_last_pulse = 0;
volatile unsigned long s_last_delta = 0;

void e_isr() {
    e_new_pulse = micros();
    e_last_delta = e_new_pulse - e_last_pulse;
    e_last_pulse = e_new_pulse;
}

void s_isr() {
    s_new_pulse = micros();
    e_last_delta = e_new_pulse - e_last_pulse;
    e_last_pulse = e_new_pulse;
}

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

// Control EQN:
// P = 0.75 - (Ka / (Rt+1)) + Kp E + Kd dE/dt + Ki integ(E)
// Exlicit term, proportional, derivative, integral

// Tuning Parameters:
const double e_rpm_t = 3000;
const double Ka = 1;
const double Kp = 0.25;
const double Ki = 0.01;
const double Kd = 0.25;

const unsigned int steps_per_linch = 800; // 4 rev/in * 200 step/rev 

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
unsigned long delta_t = 10; // millis -> 100Hz 

File log_file;
unsigned long lines_written = 0;
unsigned long last_flush = 0;

void updatePID() {
    e_rpm_m = (60.0/1000000.0)/e_last_delta;
    s_rpm_m = (60.0/1000000.0)/s_last_delta;
    r_m = e_rpm_m / s_rpm_m;
    r_t = e_rpm_t / s_rpm_m;

    last_error = error;
    error = r_t - r_m;
    error_deriv = (error-last_error)/delta_t;
    error_integ += error*delta_t;

    target_pos_inch = 0.75 - (Ka / (r_t +1)) + Kp*error + Ki*error_integ + Kd*error_deriv;
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
    // Initialize Tach Inputs
    attachInterrupt(digitalPinToInterrupt(ENGINE_TACH_PIN), e_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(SECONDARY_TACH_PIN), s_isr, RISING);

    // Init lights
    pinMode(READY_LED, OUTPUT);
    pinMode(ERROR_LED, OUTPUT);

    char filename[32] = "cvt_data.csv";
    unsigned int file_index = 0;
    while (SD.exists(filename)) {
        file_index++;
        sprintf(filename, "cvt_data_%u.csv", file_index);
        if (file_index > 100) fail();
    }
    log_file = SD.open(filename);

    // Mark start time
    start_time = millis();
    last_time = start_time;

    // Initialize stepper for accel control
    stepper.setMaxSpeed(1*steps_per_linch);
    stepper.setAcceleration(2000);
}

void loop() {
    // Check if enough time has passed to recompute target position
    if (millis() - last_time >= delta_t) {
        last_time += delta_t;
        updatePID();
        stepper.moveTo(target_pos_inch*steps_per_linch);

        // LOG FORMAT
        // time (ms), eRPM, sRPM, r_t, r_m, target_pos, curr_pos
        char new_line[256];
        sprintf(new_line, "%u, %.0f, %.0f, %.4f, %.4f, %.4f, %.4f", 
            last_time, e_rpm_m, s_rpm_m, r_t, r_m, target_pos_inch, (stepper.currentPosition()/double(steps_per_linch) ));
        log_file.println(new_line);
        lines_written++;
        if (lines_written - last_flush > 50) {
            log_file.flush();
            last_flush = lines_written;
        }
    }
    stepper.run();
}