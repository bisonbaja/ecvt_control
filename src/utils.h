#ifndef UTILS_H
#define UTILS_H

// Splits a string on delim, leaving left pointing at beginning of newly split string, and right pointing at position after delim
bool split_string(char** left, char** right, char delim);

// Interpolates without extrapolation
float interpolate(float x, const float *xValues, const float *yValues, int size);

// Returns x if between -cap and cap, otherwise returns -cap or cap
float symminmax(float x, float cap);

// Fail mode with blinking error led
void fail();

#endif // UTILS_H