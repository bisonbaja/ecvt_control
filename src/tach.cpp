#include "tach.h"
#include "config.h"

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