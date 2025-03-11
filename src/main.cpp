#include "config.h" // Tuning Parameters
#include "tasks.h" // Task definitions 
#include "PID.h" // PID Loop
#include "log.h"
#include <Arduino.h>

void setup() {
    // Init lights and signal error until ready
    pinMode(READY_LED, OUTPUT);
    pinMode(ERROR_LED, OUTPUT);
    digitalWrite(ERROR_LED, HIGH);

    #ifdef USE_SERIAL
    Serial.begin(SERIAL_BAUDRATE);
    #endif

    #ifdef USE_BT
    BT.enableSSP();
    BT.begin(BT_NAME);
    #endif

    // Initialize Tach Inputs
    pinMode(ENGINE_TACH_PIN, INPUT);
    pinMode(SECONDARY_TACH_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENGINE_TACH_PIN), e_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(SECONDARY_TACH_PIN), s_isr, RISING);

    // Initialize stepper for accel control
    stepper_setup();
    xTaskCreatePinnedToCore(updatePID_task, "PID Update Loop", 8000, NULL, PID_TASK_PRIORITY, NULL, 1);

    #ifdef USE_SD
    SD_init();
    xTaskCreatePinnedToCore(SD_task, "SD Card Log", 8000, NULL, SD_TASK_PRIORITY, NULL, 1);
    #endif // USE_SD
    
    #ifdef USE_BT
    xTaskCreatePinnedToCore(log_task, "Serial Logging", 8000, NULL, LOG_TASK_PRIORITY, NULL, 1);
    xTaskCreatePinnedToCore(serial_command_task, "Read Serial and execute commands", 2000, NULL, COMMAND_TASK_PRIORITY, NULL, 1);
    #endif

    // SIGNAL READY
    digitalWrite(READY_LED, HIGH);
    digitalWrite(ERROR_LED, LOW);
    vTaskDelete(NULL);
}

// Never run
void loop() {
    vTaskDelay(portMAX_DELAY);
}