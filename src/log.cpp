#include "log.h"
#include "PID.h"
#include "config.h"
#include "tasks.h"
#include "utils.h"

struct Param {
    const char* name;
    float* address;
};

Param params[] = {
    {"kp", &Kp},
    {"ki", &Ki},
    {"kd", &Kd},
    {"rpmt", &e_rpm_t},
    {"rt", &r_t},
    {"pt", &target_pos_inch},
    {"engage", &engage_pos}, 
    {"maxderiv", &max_error_deriv},
    {"r1", &manual_ratios[0]},
    {"r2", &manual_ratios[1]},
    {"r3", &manual_ratios[2]},
    {"r4", &manual_ratios[3]},
    {"r5", &manual_ratios[4]},
    {"r6", &manual_ratios[5]},
};

bool set(char* token) {
	char *next;
	next = token;
	if (!split_string(&token, &next, ' ')) return false;
	const int count = sizeof(params) / sizeof(params[0]);
	for (int i = 0; i < count; i += 1) {
		Param* param = &params[i];
		if (!strcasecmp(token, param->name)) {
			if (!split_string(&token, &next, ' ')) return false;
			*param->address = atof(token);
            data_print("SET PARAMETER: ");
            data_println(*param->address);
			return true;
		}
	}
    return false;
}

bool zero(char* rest) {
	stepper->setCurrentPosition(0);
    stepper->moveTo(0);
    data_println("ZEROED");
	return true;
}

bool max(char* rest) {
	stepper->setCurrentPosition(MAX_POS_INCH*STEPS_PER_INCH);
    stepper->moveTo(MAX_POS_INCH*STEPS_PER_INCH);
    data_println("MAXXED");
	return true;
}

bool set_engage(char* rest) {
    stepper->setCurrentPosition(ENGAGE_POS*STEPS_PER_INCH);
    stepper->moveTo(ENGAGE_POS*STEPS_PER_INCH);
    data_println("Datum set to engagement position");
    return true;
}

bool print_pos(char* rest) {
    data_print("Current Position: ");
    data_println(stepper->getCurrentPosition() / STEPS_PER_INCH);
    return true;
}

bool go(char* token) {
    char *next = token;
	if (!split_string(&token, &next, ' ')) return false; // token now contains value
    target_pos_inch = atof(token);
    return true;
}

bool manual_mode(char* rest) {
    current_mode = RATIO;
    data_println("Manual Mode");
    return true;
}

bool normal_mode(char* rest) {
    current_mode = RPM;
    data_println("RPM (Normal) Mode");
    return true;
}

bool debug_mode(char* rest) {
    current_mode = POSITION;
    data_println("Debug Mode");
    return true;
}

struct Command {
    const char* name;
    bool (*func)(char* rest);
};

Command commands[] = {
    {"set", set},
    {"zero", zero},
    {"max", max},
    {"seteng", set_engage},
    {"getpos", print_pos},
    {"go", go},
    {"rpm", normal_mode},
    {"position", debug_mode},
    {"manual", manual_mode}
};

char input[1024];
char* last_input_char = input;

#if defined(USE_BT) || defined(USE_SERIAL)
bool check_serial() {
	while (last_input_char < &input[sizeof(input)] && data_available()) { // if still in buffer limits 
		int new_char = data_read(); // read a single character
		if (new_char < 0) break; // if timeout (for some reason? even though serial.available was true)
		*last_input_char = (char)new_char; // write new char to next place in input buffer
		if ((char)new_char == '\n') break; // break when at end of line 
		last_input_char += 1; // advance input place pointer
	}

	// check if input was too long for input buffer
	if (last_input_char == &input[sizeof(input)]) { // if the input pointer is at the end of the buffer
		if (*last_input_char != '\n') { // if it wasn't a newline
			last_input_char = input; // reset the pointer
			return false; // we failed ;(
		}
	}

	// if we haven't yet gotten a newline, just return
	if (*last_input_char != '\n') return true;

	// if we got the newline, replace the newline with null terminator and do the command processing
	*last_input_char = 0;
	last_input_char = input;    // reset the last input character to start of input buffer

	// now we process a command

	char *next, *token;
	next = input;

	if (!split_string(&token, &next, ' ')) return false;

	const int count = sizeof(commands) / sizeof(commands[0]);
		for (int i = 0; i < count; i += 1) {
			Command* command = &commands[i];
			
			if (!strcasecmp(token, command->name)) {
				return command->func(next);
			}
		}

	return true;
}
#endif 

#ifdef USE_SD
#include <SD.h>
#include <SPI.h>
File logFile;
#endif // USE_SD

char csv_buffer[1024];
char teleplot_buffer[1024];

struct Datapoint {
    char name[32];
    float* value;
    char format[8];
    bool displayInTeleplot;
    bool displayInCSV;
};

Datapoint datapoints[] = {
    {"Error", &error, "%.2f", false, false},
    {"Error Integral", &error_integ, "%.2f", false, false},
    {"Error Derivative", &error_deriv, "%.2f", false, false},
    {"Feed Forward Position", &ff_pos, "%.2f", false, false},
    {"Engine RPM Target", &e_rpm_t, "%.0f", false, false},
    {"Engine RPM", &e_rpm_m, "%.0f", true, true},
    {"Secondary RPM", &s_rpm_m, "%.0f", true, true},
    {"Target Ratio", &r_t, "%.2f", false, false},
    {"Measured Ratio", &r_m, "%.2f", true, true},
    {"Stepper Position", &target_pos_inch, "%.2f", false, false},
    {"Kp", &Kp, "%.2f", false, false},
    {"Ki", &Ki, "%.2f", false, false},
    {"Kd", &Kd, "%.2f", false, false},
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
    char formbuf[512];
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
    if (!SD.begin()) {
        data_println("SD Card failed to initialize, or not present");
        return;
    }
    char filename[16] = "/D0.csv";
    unsigned short file_index = 0;
    while (SD.exists(filename)) {
        file_index++;
        sprintf(filename, "/D%d.csv", file_index);
    }
    logFile = SD.open(filename, FILE_WRITE);
    if (!logFile) {
        data_println("Failed to open log.csv");
        return;
    }

    const int count = sizeof(datapoints) / sizeof(datapoints[0]);
    for (int i = 0; i < count; i += 1) {
        Datapoint* datapoint = &datapoints[i];
        if (datapoint->displayInCSV) {
            logFile.print(datapoint->name);
            logFile.print(',');
        }
    }
    logFile.print('\n');
}

unsigned long lines_written  = 0;
unsigned long last_flush = 0;

void log_CSV() {
    if (!logFile) return;
    build_csv(csv_buffer);
    logFile.print(csv_buffer);
    if (lines_written-last_flush > (5000 / SD_TASK_DELAY)) {
        logFile.flush();
        last_flush = lines_written;
    }
    lines_written++;
}
#endif // USE_SD

#if defined(USE_BT) || defined(USE_SERIAL)
void log_teleplot() {
    build_teleplot(teleplot_buffer);
    data_println(teleplot_buffer);
}
#endif // USE_SERIAL