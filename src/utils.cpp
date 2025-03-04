#include "utils.h"
#include "config.h"
#include <Arduino.h>
#include "BluetoothSerial.h"

extern BluetoothSerial SerialBT;

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

double interpolate(double x, const double *xValues, const double *yValues) {
    unsigned int i = 0;
    unsigned int size = sizeof(xValues) / sizeof(xValues[0]);
    while (x > xValues[i] && i < size - 1) {
        i++;
    }
    float x0 = xValues[i - 1], x1 = xValues[i];
    float y0 = yValues[i - 1], y1 = yValues[i];
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}