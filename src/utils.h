#ifndef UTILS_H
#define UTILS_H

bool split_string(char** left, char** right, char delim);
float interpolate(float x, const float *xValues, const float *yValues);
void fail();

#endif // UTILS_H