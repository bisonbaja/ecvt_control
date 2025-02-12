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
#define ENGINE_NUM_MAGS 0.2
#define SECONDARY_NUM_MAGS 2
#define ENGINE_AVG 8
#define SECONDARY_AVG 8
#define FAKE_DEF_RPM 2000

#define STEP_PIN 6
#define DIR_PIN 7
#define READY_LED 8
#define ERROR_LED 9
#define STEPPER_ENABLE

// Tuning Parameters:
const double e_rpm_t = 3000;
const double e_rpm_const = 60000000.0/ENGINE_NUM_MAGS;
const double s_rpm_const = 60000000.0/SECONDARY_NUM_MAGS;
const unsigned int steps_per_linch = 800; // 4 rev/in * 200 step/rev 
const double Kp = -0.01;
const double Ki = -0.001;
const double Kd = -0.25;
const double max_pos_inch = (0.925*2);
unsigned long delta_t = 50; // millis -> 100Hz 

#ifdef STEPPER_ENABLE
// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
#endif

const double model_r[] = {0.9,    1,     1.1,    1.2,    1.3,    1.4,    1.5,    1.6,   1.7,    1.8,    1.9,    2,      2.1,    2.2,    2.3,    2.4,    2.5,    2.6,    2.7,    2.8,    2.9,    3,      3.1,    3.2,    3.3,    3.4,    3.5,    3.6,    3.7,    3.8,    3.9};
//const double model_P[] = {2.6793, 2.5465,2.4243, 2.3119, 2.2083, 2.1126, 2.0242, 1.9423,1.8662,	1.7956,	1.7298,	1.6683,	1.6109,	1.5571,	1.5067,	1.4593,	1.4146,	1.3726,	1.3329,	1.2953,	1.2598,	1.2261,	1.1941,	1.1637,	1.1348,	1.1073,	1.081,	1.056,	1.032,	1.0091,	0.98719};
const double model_P[] = {1.0007, 0.9295,0.86404,0.80379,0.74825,0.697,  0.6496, 0.6057,0.56497,0.5271,	0.49182,0.4589,	0.42813,0.39931,0.37227,0.34686,0.32295,0.30041,0.27913,0.25901,0.23996,0.22191,0.20477,0.18849,0.17299,0.15824,0.14417,0.13073,0.1179, 0.10563,0.093882};


double error;
double last_error;
double error_deriv;
double error_integ;
double max_error_integ = abs(max_pos_inch/(Ki))/4;
double min_error_integ = -max_error_integ;
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
volatile unsigned long e_deltas[ENGINE_AVG];
volatile unsigned short e_delta_i = 0;

volatile unsigned long s_new_pulse = 0;
volatile unsigned long s_last_pulse = 0;
volatile unsigned long s_last_delta = 0;
volatile unsigned long s_deltas[SECONDARY_AVG];
volatile unsigned short s_delta_i = 0;

void e_isr() { // 4 micros
    e_new_pulse = micros();
    e_last_delta = e_new_pulse - e_last_pulse;
    e_last_pulse = e_new_pulse;
    e_deltas[e_delta_i] = e_last_delta;

    if (e_delta_i == ENGINE_AVG-1) e_delta_i = 0;
    else e_delta_i++;
}

double e_avg_delta() {
    double ret_val=0;
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

double s_avg_delta() {
    double ret_val=0;
    for (byte i = 0; i < SECONDARY_AVG; i++) {
        ret_val += s_deltas[i];
    }
    return ret_val/SECONDARY_AVG;
}

// Function to interpolate between calibration points
double interpolate(double x, const double *xValues, const double *yValues) {
    unsigned int i = 0;
    unsigned int size = sizeof(xValues) / sizeof(xValues[0]);
    // Find the two closest calibration points
    while (x > xValues[i] && i < size - 1) {
      i++;
    }
    // Interpolate between the closest calibration points
    float x0 = xValues[i - 1], x1 = xValues[i];
    float y0 = yValues[i - 1], y1 = yValues[i];
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}

// Compute RPM values, calculate ratios, error, and new target position
void updatePID() { // ~200 micros
    e_rpm_m = e_rpm_const/e_avg_delta();
    s_rpm_m = s_rpm_const/s_avg_delta();
    if (e_rpm_m <= 5 or e_rpm_m > 5000) e_rpm_m = FAKE_DEF_RPM;
    if (s_rpm_m <= 5 or s_rpm_m > 5000) s_rpm_m = FAKE_DEF_RPM;
    r_m = e_rpm_m / s_rpm_m;
    r_t = e_rpm_t / s_rpm_m;

    last_error = error;
    error = r_t - r_m;
    error_deriv = (error-last_error)/delta_t;
    error_integ += error*delta_t;
    if (error_integ > max_error_integ) error_integ = max_error_integ;
    if (error_integ < min_error_integ) error_integ = min_error_integ;
    target_pos_inch = interpolate(r_t, model_r, model_P) + Kp*error + Ki*error_integ + Kd*error_deriv;
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
    while (!Serial) ;
    //delay(10000);
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
    stepper.setMaxSpeed(4*steps_per_linch);
    stepper.setAcceleration(8*steps_per_linch);
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
        double pos_actual = (stepper.currentPosition()/double(steps_per_linch) );
        sprintf(serial_line, ">Engine RPM:%.2f\n>Secondary RPM:%.2f\n>Ratio:%.2f\n>Target Ratio:%.2f\n>Stepper Position:%.2f\n>Stepper Target:%.2f\n>Error:%.2f\n>Error Integ:%.2f\n>Dist to go:%.2f", 
            e_rpm_m, s_rpm_m, r_m,r_t,pos_actual, target_pos_inch, error, error_integ, (target_pos_inch - pos_actual));
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