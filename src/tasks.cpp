#include "config.h"
#include "tasks.h"
#include "utils.h"
#include "PID.h"
#include "mpu.h"
#include "log.h"
#include <Arduino.h>

void updatePID_task(void * parameter) {
    for (;;) updatePID();
}

void log_task(void * parameter) {
    while (true) {
        #ifdef USE_SD
        log_CSV();
        #endif // USE_SD
    
        #ifdef USE_SERIAL
        log_teleplot();
        #endif // USE_SERIAL
        delay(LOG_TASK_DELAY);
    }
}

void serial_command_task(void * parameter) {
    for (;;) {
        check_serial();
        delay(500);
    }
}

#ifdef ARDUINO_ARCH_SAMD
#include <FreeRTOS_SAMD21.h>
void stepper_task(void* parameter) {
    for (;;) {
        stepper.run();
        vTaskDelay(100 / portTICK_PERIOD_US);
    }
}
#endif // ARDUINO_ARCH_SAMD