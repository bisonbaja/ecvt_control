#ifndef UTILS_H
#define UTILS_H

bool split_string(char** left, char** right, char delim);
double interpolate(double x, const double *xValues, const double *yValues);

#endif // UTILS_H