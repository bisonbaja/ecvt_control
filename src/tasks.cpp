#include "tasks.h"
#include "config.h"
#include "utils.h"
#include <Arduino.h>
#include "BluetoothSerial.h"
#include "FastAccelStepper.h"

extern FastAccelStepper *stepper;
extern BluetoothSerial SerialBT;

void updatePID_task(void * parameter) {
    while (true) {
        e_rpm_m = e_rpm_const / e_avg_delta();
        s_rpm_m = s_rpm_const / s_avg_delta();
        if (e_rpm_m <= 5 || e_rpm_m > 5000) e_rpm_m = FAKE_DEF_RPM;
        if (s_rpm_m <= 5 || s_rpm_m > 5000) s_rpm_m = FAKE_DEF_RPM;
        r_m = e_rpm_m / s_rpm_m;
        r_t = e_rpm_t / s_rpm_m;

        last_error = error;
        error = r_t - r_m;
        error_deriv = (error - last_error) / dt;
        error_integ += error * dt;

        if (error_integ > max_error_integ) error_integ = max_error_integ;
        if (error_integ < min_error_integ) error_integ = min_error_integ;

        target_pos_inch = 0.5 * max_pos_inch + Kp * error + Ki * error_integ + Kd * error_deriv;

        if (target_pos_inch < 0) target_pos_inch = 0;
        if (target_pos_inch > max_pos_inch) target_pos_inch = max_pos_inch;

        delay(delta_t);
    }
}

void logSerial_task(void * parameter) {
    while (true) {
        char serial_line[256];
        double pos_actual = (stepper->getCurrentPosition() / double(steps_per_linch));
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
        SerialBT.println(serial_line);
        delay(log_delay);
    }
}

void serial_command_task(void * parameter) {
    for (;;) {
        check_serial();
        delay(500);
    }
}