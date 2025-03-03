// Main file for Baja CVT Controller
// Program name: eCVT Control
// Christian Wilson 11/8/24
// Last major update 1/28/25
// Ported to esp32 2/26

/* DONE:
 - stepper control
 - PID feedback
 - data logging
 - ready/error lights
 - accelerometer
 - bluetooth
 - on-the-fly parameter updates
*/
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "sdkconfig.h"
#include <Arduino.h>
#include "FastAccelStepper.h"
#include "BluetoothSerial.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"

// PIN DEFINITIONS
#define ENGINE_TACH_PIN     4
#define SECONDARY_TACH_PIN  5
#define STEP_PIN 6
#define DIR_PIN 7
#define READY_LED 8
#define ERROR_LED 9

// TACH CONFIGURATION
#define ENGINE_NUM_MAGS 0.25
#define SECONDARY_NUM_MAGS 0.25
#define ENGINE_AVG 4
#define SECONDARY_AVG 4
#define FAKE_DEF_RPM 2000

// Tuning Parameters:
double e_rpm_t = 3000;
const double e_rpm_const = 60000000.0/ENGINE_NUM_MAGS;
const double s_rpm_const = 60000000.0/SECONDARY_NUM_MAGS;
const unsigned int steps_per_linch = 800; // 4 rev/in * 200 step/rev 
const unsigned int stepper_max_accel = steps_per_linch*2;
const unsigned int stepper_max_speed = steps_per_linch*4;
double Kp = -0.25;
double Ki = -0.5;
double Kd = -0.02;
const double max_pos_inch = (0.925*2);
const unsigned long delta_t = 20; // millis -> 100Hz 
const double dt = delta_t*0.001; // in seconds for PID
const unsigned long log_delay = 200;
unsigned long log_last_time;

// Exported from Octave, currently unused
const double model_r[] = {0.9,    1,     1.1,    1.2,    1.3,    1.4,    1.5,    1.6,   1.7,    1.8,    1.9,    2,      2.1,    2.2,    2.3,    2.4,    2.5,    2.6,    2.7,    2.8,    2.9,    3,      3.1,    3.2,    3.3,    3.4,    3.5,    3.6,    3.7,    3.8,    3.9};
//const double model_P[] = {2.6793, 2.5465,2.4243, 2.3119, 2.2083, 2.1126, 2.0242, 1.9423,1.8662,	1.7956,	1.7298,	1.6683,	1.6109,	1.5571,	1.5067,	1.4593,	1.4146,	1.3726,	1.3329,	1.2953,	1.2598,	1.2261,	1.1941,	1.1637,	1.1348,	1.1073,	1.081,	1.056,	1.032,	1.0091,	0.98719};
const double model_P[] = {1.0007, 0.9295,0.86404,0.80379,0.74825,0.697,  0.6496, 0.6057,0.56497,0.5271,	0.49182,0.4589,	0.42813,0.39931,0.37227,0.34686,0.32295,0.30041,0.27913,0.25901,0.23996,0.22191,0.20477,0.18849,0.17299,0.15824,0.14417,0.13073,0.1179, 0.10563,0.093882};

// Stepper Drive
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// Bluetooth Logging
BluetoothSerial SerialBT;

// Display

// Accelerometer Gyroscope
MPU6050 mpu;
int const INTERRUPT_PIN = 2;

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---Orientation/Motion Variables---*/ 
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
VectorInt16 gy;         // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            Gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

void fail() {
    digitalWrite(READY_LED, LOW);
    while (true) {
        digitalWrite(ERROR_LED, HIGH);
        delay(1000);
        digitalWrite(ERROR_LED, LOW);
        delay(1000);
    }
}

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}

void MPUsetup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    if (!mpu.testConnection()) {
        SerialBT.println("MPU6050 Connection Failed!");
        fail();
    }
    else SerialBT.println("MPU6050 Connection Successful!");

    SerialBT.println("Initialzing DMP...");
    devStatus = mpu.dmpInitialize();

    /* Supply your gyro offsets here, scaled for min sensitivity */
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);

    /* Making sure it worked (returns 0 if so) */ 
    if (devStatus == 0) {
        mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateGyro(6);
        SerialBT.println("These are the Active offsets: ");
        mpu.PrintActiveOffsets();
        SerialBT.println(F("Enabling DMP..."));   //Turning ON DMP
        mpu.setDMPEnabled(true);

        /*Enable Arduino interrupt detection*/
        SerialBT.print(F("Enabling interrupt detection (Arduino external interrupt "));
        SerialBT.print(digitalPinToInterrupt(INTERRUPT_PIN));
        SerialBT.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
        MPUIntStatus = mpu.getIntStatus();

        /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
        SerialBT.println(F("DMP ready! Waiting for first interrupt..."));
        DMPReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
    } 
    else {
        Serial.print(F("DMP Initialization failed (code ")); //Print the error code
        Serial.print(devStatus);
        Serial.println(F(")"));
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
    }

}

void updateMPUWorld() {
    /* Display initial world-frame acceleration, adjusted to remove gravity
      and rotated based on known orientation from Quaternion */
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetAccel(&aa, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
}
// Initialize PID loop global variables
double error;
double last_error;
double error_deriv;
double error_integ;
double max_error_integ = abs(max_pos_inch/(Ki))/2;
double min_error_integ = -max_error_integ;
double e_rpm_m = 0;
double s_rpm_m = 0;
double r_t = 3.9;
double r_m = 3.9;
double target_pos_inch = 0;

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
void updatePID_task(void * parameter) { // ~200 micros
    while (true) {
        e_rpm_m = e_rpm_const/e_avg_delta();
        s_rpm_m = s_rpm_const/s_avg_delta();
        if (e_rpm_m <= 5 or e_rpm_m > 5000) e_rpm_m = FAKE_DEF_RPM;
        if (s_rpm_m <= 5 or s_rpm_m > 5000) s_rpm_m = FAKE_DEF_RPM;
        r_m = e_rpm_m / s_rpm_m;
        r_t = e_rpm_t / s_rpm_m;

        last_error = error;
        error = r_t - r_m;
        error_deriv = (error-last_error)/dt;
        error_integ += error*dt;

        if (error_integ > max_error_integ) error_integ = max_error_integ;
        if (error_integ < min_error_integ) error_integ = min_error_integ;
        // BIAS + P + I + D
        target_pos_inch = 0.5*max_pos_inch + Kp*error + Ki*error_integ + Kd*error_deriv;

        if (target_pos_inch < 0) target_pos_inch = 0;
        if (target_pos_inch > max_pos_inch) target_pos_inch = max_pos_inch;
        delay(delta_t);
    }
}

void logSerial_task(void * parameter) {
    while (true) {
        char serial_line[256];
        double pos_actual = (stepper->getCurrentPosition() /double(steps_per_linch) );
        sprintf(serial_line, 
            ">Engine RPM:%.2f\n"
            ">Secondary RPM:%.2f\n"
            ">Ratio:%.2f\n"
            ">Target Ratio:%.2f\n"
            ">Stepper Position:%.2f\n"
            ">Stepper Target:%.2f\n"
            ">Accel:%.3f\n"
            ">Error:%.2f\n"
            ">Error Integ:%.2f\n"
            ">Error Deriv:%.2f\n"
            ">Dist to go:%.2f\n"
            ">P Corr:%.2f\n"
            ">I Corr:%.2f\n"
            ">D Corr:%.2f,", 
            e_rpm_m, 
            s_rpm_m, 
            r_m,
            r_t,
            pos_actual, 
            target_pos_inch, 
            aaWorld.getMagnitude(),
            error, 
            error_integ, 
            error_deriv, 
            (target_pos_inch - pos_actual), 
            Kp*error, 
            Ki*error_integ, 
            Kd*error_deriv);
        SerialBT.println(serial_line);
        delay(log_delay);
    }
}

struct Param{
    const char* name;
    double* address;
} ;

Param params[] = {
    {"kp", &Kp},
    {"ki", &Ki},
    {"kd", &Kd},
    {"et", &e_rpm_t}
};

char input[32];
char* last_input_char;

// "set kp -0.20"

bool split_string(char** left, char** right, char delim) {
    if (**right == 0) return false;
    
    *left = *right;
    while (**right && **right != delim) {
        *right += 1;
    }
    if (**right != 0) {
        **right = 0;
        *right += 1;
    }
    return true;
}

bool check_serial() {
	while (last_input_char < &input[sizeof(input)]) {
		int new_char = SerialBT.read();
		if (new_char < 0) break;
		*last_input_char = (char)new_char;
		if ((char)new_char == '\n') break;
		last_input_char += 1;
	}

	// check if input was too long for input buffer
	if (last_input_char == &input[sizeof(input)]) {
		if (*last_input_char != '\n') {
			last_input_char = input;
			return false;
		}
	}

	// if we haven't yet gotten a newline, just return
	if (*last_input_char != '\n') return true;

	// if we got the newline, replace the newline with null terminator and do the command processing
	*last_input_char = 0;
	last_input_char = input;    // reset the last input character to start of input buffer

	// now we process a command

	char *next, *token;
	next = input;

	if (!split_string(&token, &next, ' ')) return false;

	if (!strcmp(token, "set")) {
		if (!split_string(&token, &next, ' ')) return false;

		const int count = sizeof(params) / sizeof(params[0]);
		for (int i = 0; i < count; i += 1) {
			Param* param = &params[i];
			if (!strcmp(token, param->name)) {
				if (!split_string(&token, &next, ' ')) return false;
				*param->address = atof(token);
				return true;
			}
		}
	}

	if (!strcmp(token, "zero")) {
		stepper->setCurrentPosition(0);
		return true;
	}

	if (!strcmp(token, "max")) {
		stepper->setCurrentPosition(max_pos_inch*steps_per_linch);
		return true;
	}
	
	return true;
}

void serial_command_task(void * parameter) {
    for (;;) {
        check_serial();
        delay(500);
    }
}

void setup() {
    SerialBT.begin(115200);

    // Initialize Tach Inputs
    pinMode(ENGINE_TACH_PIN, INPUT);
    pinMode(SECONDARY_TACH_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENGINE_TACH_PIN), e_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(SECONDARY_TACH_PIN), s_isr, RISING);

    // Init lights
    pinMode(READY_LED, OUTPUT);
    pinMode(ERROR_LED, OUTPUT);
    digitalWrite(ERROR_LED, HIGH);

    // Initialize stepper for accel control
    engine.init();
    stepper = engine.stepperConnectToPin(STEP_PIN);
    stepper->setDirectionPin(DIR_PIN);
    stepper->setSpeedInHz(stepper_max_speed);
    stepper->setAcceleration(stepper_max_accel);

    digitalWrite(READY_LED, HIGH);
    digitalWrite(ERROR_LED, LOW);

    xTaskCreate(updatePID_task, "PID Update Loop", 8000, NULL, 2, NULL);
    xTaskCreate(logSerial_task, "Serial Logging", 8000, NULL, 1, NULL);
    xTaskCreate(serial_command_task, "Read Serial and execute commands", 2000, NULL, 1, NULL);
}

// per arduino-esp32 implementation, this is run at priority 1 on core 1. 
void loop() {
    vTaskDelay(portMAX_DELAY);
}