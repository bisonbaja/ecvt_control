#include "utils.h"
#include "config.h"
#include "PID.h"
#include "Arduino.h"

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

float interpolate(float x, const float *xValues, const float *yValues) {
    unsigned int i = 0;
    unsigned int size = sizeof(xValues) / sizeof(xValues[0]);
    while (x > xValues[i] && i < size - 1) {
        i++;
    }
    float x0 = xValues[i - 1], x1 = xValues[i];
    float y0 = yValues[i - 1], y1 = yValues[i];
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}

void fail() {
    digitalWrite(READY_LED, LOW);
    while (true) {
        digitalWrite(ERROR_LED, HIGH);
        delay(1000);
        digitalWrite(ERROR_LED, LOW);
        delay(1000);
    }
}
