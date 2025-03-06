#ifndef UTILS_H
#define UTILS_H

bool split_string(char** left, char** right, char delim);
float interpolate(float x, const float *xValues, const float *yValues);
bool check_serial();
bool set(char* token);
bool zero(char* rest);
bool max(char* rest);
void fail();

#endif // UTILS_H