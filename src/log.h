#ifndef LOG_H
#define LOG_H
#include "config.h"

void log_CSV();

void log_teleplot();

void SD_init();

#ifndef USE_SD
#define SD_init()
#define log_CSV()
#endif // USE_SD

#ifndef USE_SERIAL
#define log_teleplot()
#endif // USE_SERIAL

#endif // LOG_H