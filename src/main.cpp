#include "config.h" // Tuning Parameters
#include "tasks.h" // Task definitions 
#include "mpu.h" // MPU6050 setup
#include "PID.h" // PID Loop
#include "log.h"
#include <Arduino.h>

void setup() {
    Serial_begin(115200);

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
    stepper_setup();

    digitalWrite(READY_LED, HIGH);
    digitalWrite(ERROR_LED, LOW);

    xTaskCreate(updatePID_task, "PID Update Loop", 8000, NULL, PID_TASK_PRIORITY, NULL);

    #ifdef USE_SD
    SD_init();
    #endif // USE_SD
    
    xTaskCreate(log_task, "Serial Logging", 8000, NULL, LOG_TASK_PRIORITY, NULL);
    #ifdef USE_SERIAL
    xTaskCreate(serial_command_task, "Read Serial and execute commands", 2000, NULL, COMMAND_TASK_PRIORITY, NULL);
    #endif // USE_SERIAL

    vTaskDelete(NULL); // delete setup task
}

// Never run
void loop() {
    vTaskDelay(portMAX_DELAY);
}