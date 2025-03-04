#include "config.h" // Tuning Parameters
#include "tasks.h" // Task definitions 
#include "mpu.h" // MPU6050 setup
#include "utils.h" // Utility functions

#include <Arduino.h>
#include "FastAccelStepper.h"
#include "BluetoothSerial.h"
#include "MPU6050_6Axis_MotionApps612.h"

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