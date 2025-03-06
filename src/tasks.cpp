#include "config.h"
#include "tasks.h"
#include "utils.h"
#include "PID.h"
#include "mpu.h"
#include "log.h"
#include <Arduino.h>
#include <FastAccelStepper.h>

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