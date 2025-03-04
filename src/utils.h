#ifndef UTILS_H
#define UTILS_H

bool split_string(char** left, char** right, char delim);
double interpolate(double x, const double *xValues, const double *yValues);
bool check_serial();

struct Param {
    const char* name;
    double* address;
};

extern Param params[];

struct Command {
    const char* name;
    bool (*func)(char* rest);
};

bool set(char* token);
bool zero(char* rest);
bool max(char* rest);

extern Command commands[];

char input[128];
char* last_input_char;
#endif // UTILS_H