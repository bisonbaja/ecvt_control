#include "log.h"
#include "PID.h"
#include "config.h"

#ifdef USE_SD
#include <SD.h>
#include <SPI.h>
File logFile;
#endif // USE_SD

char csv_buffer[256];
char teleplot_buffer[256];

struct Datapoint {
    char name[32];
    float* value;
    char format[8];
    bool displayInTeleplot;
    bool displayInCSV;
};

Datapoint datapoints[] = {
    {"Error", &error, "%.2f", true, true},
    {"Error Integral", &error_integ, "%.2f", true, true},
    {"Error Derivative", &error_deriv, "%.2f", true, true},
    {"Engine RPM Target", &e_rpm_t, "%.0f", true, true},
    {"Engine RPM", &e_rpm_m, "%.0f", true, true},
    {"Secondary RPM", &s_rpm_m, "%.0f", true, true},
    {"Target Ratio", &r_t, "%.2f", true, true},
    {"Measured Ratio", &r_m, "%.2f", true, true},
    {"Stepper Position", &target_pos_inch, "%.2f", true, true},
    {"Error", &error, "%.2f", true, true},
    {"Error Integral", &error_integ, "%.2f", true, true},
    {"Error Derivative", &error_deriv, "%.2f", true, true},
};

// Builds one line of CSV and returns the next character to write to
char* build_csv(char* buffer) { 
    char* next = buffer;
    const int count = sizeof(datapoints) / sizeof(datapoints[0]);
    for (int i = 0; i < count; i += 1) {
        Datapoint* datapoint = &datapoints[i];
        if (datapoint->displayInCSV) {
            next += sprintf(next, datapoint->format, *datapoint->value);
            *next = ',';
            next += 1;
        }
    }
    *(next - 1) = '\n';
    return next;
}

// Builds one line of teleplot and returns the next character to write to
char* build_teleplot(char* buffer) { 
    char* next = buffer;
    char formbuf[128];
    const int count = sizeof(datapoints) / sizeof(datapoints[0]);
    for (int i = 0; i < count; i += 1) {
        Datapoint* datapoint = &datapoints[i];
        if (datapoint->displayInTeleplot) {
            snprintf(formbuf, sizeof(formbuf), ">%s:%lu:%s\n", datapoint->name, millis(), datapoint->format);
            next += sprintf(next, formbuf, *datapoint->value);
        }
    }
    return next;
}

#ifdef USE_SD
void SD_init() {
    if (!SD.begin(SD_CS)) {
        Serial_println("SD Card failed to initialize, or not present");
        return;
    }
    char filename[16] = "D0.csv";
    unsigned short file_index = 0;
    while (SD.exists(filename)) {
        file_index++;
        sprintf(filename, "D%d.csv", file_index);
    }
    logFile = SD.open(filename, FILE_WRITE);
    if (!logFile) {
        Serial_println("Failed to open log.csv");
        return;
    }
}

void log_CSV() {
    if (!logFile) return;
    build_csv(csv_buffer);
    logFile.print(csv_buffer);
}
#endif // USE_SD

#ifdef USE_SERIAL
void log_teleplot() {
    if (!Serial) return;
    build_teleplot(teleplot_buffer);
    Serial_print(teleplot_buffer);
}
#endif // USE_SERIAL