#ifndef LOG_H
#define LOG_H
#include "config.h"

void log_CSV();

void log_teleplot();

void SD_init();

bool check_serial();

bool max(char* rest);
bool set(char* token);
bool zero(char* rest);

#ifndef USE_SD
#define SD_init()
#define log_CSV()
#endif // USE_SD

#endif // LOG_H