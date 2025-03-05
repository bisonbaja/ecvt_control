#include "config.h"
#include "tasks.h"
#include "utils.h"
#include "PID.h"
#include "mpu.h"
#include <Arduino.h>
#include <FastAccelStepper.h>

void updatePID_task(void * parameter) {
    for (;;) updatePID();
}

void logSerial_task(void * parameter) {
    while (true) {
        char serial_line[256];
        double pos_actual = (stepper->getCurrentPosition() / double(STEPS_PER_LINCH));
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
            Kp * error,
            Ki * error_integ,
            Kd * error_deriv);
        Serial_println(serial_line);
        delay(LOG_TASK_DELAY);
    }
}

void serial_command_task(void * parameter) {
    for (;;) {
        check_serial();
        delay(500);
    }
}