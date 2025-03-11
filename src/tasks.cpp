#include "config.h"
#include "tasks.h"
#include "utils.h"
#include "PID.h"
#include "mpu.h"
#include "log.h"
#include <Arduino.h>

float last_lt = 0;

/*
void step_task(void * parameter) {
    for (;;) {
        stepper.run();
        delayMicroseconds(STEP_DELAY_US);
    }
}
    */

void updatePID_task(void * parameter) {
    for (;;) updatePID();
}

void log_task(void * parameter) {
    while (true) {
        log_teleplot();
        delay(LOG_TASK_DELAY);
    }
}

void SD_task(void * parameter) {
    for (;;){
        log_CSV();
        delay(SD_TASK_DELAY);
    }
}

void serial_command_task(void * parameter) {
    for (;;) {
        check_serial();
        delay(SERIAL_COMMAND_DELAY);
    }
}
