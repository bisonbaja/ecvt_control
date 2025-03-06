#include "utils.h"
#include "config.h"
#include "PID.h"
#include "string.h"

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

struct Param {
    const char* name;
    float* address;
};

struct Command {
    const char* name;
    bool (*func)(char* rest);
};

Param params[] = {
    {"Kp", &Kp},
    {"Ki", &Ki},
    {"Kd", &Kd},
    {"e_rpm_t", &e_rpm_t},
};

Command commands[] = {
    {"set", set},
    {"zero", zero},
    {"max", max}
};

bool set(char* token) {
	char *next;
	next = token;
	if (!split_string(&token, &next, ' ')) return false;
	const int count = sizeof(params) / sizeof(params[0]);
	for (int i = 0; i < count; i += 1) {
		Param* param = &params[i];
		if (!strcmp(token, param->name)) {
			if (!split_string(&token, &next, ' ')) return false;
			*param->address = atof(token);
			return true;
		}
	}
    return false;
}

bool zero(char* rest) {
	stepper->setCurrentPosition(0);
	return true;
}

bool max(char* rest) {
	stepper->setCurrentPosition(MAX_POS_INCH*STEPS_PER_LINCH);
	return true;
}

char input[256];
char* last_input_char = input;

#ifdef USE_SERIAL
bool check_serial() {
	while (last_input_char < &input[sizeof(input)] && Serial_available()) { // if still in buffer limits 
		int new_char = Serial_read(); // read a single character
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
			if (!strcmp(token, command->name)) {
				return command->func(next);
			}
		}

	return true;
}
#endif // USE_SERIAL

float interpolate(float x, const float *xValues, const float *yValues) {
    unsigned int i = 0;
    unsigned int size = sizeof(xValues) / sizeof(xValues[0]);
    while (x > xValues[i] && i < size - 1) {
        i++;
    }
    float x0 = xValues[i - 1], x1 = xValues[i];
    float y0 = yValues[i - 1], y1 = yValues[i];
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}

void fail() {
    digitalWrite(READY_LED, LOW);
    while (true) {
        digitalWrite(ERROR_LED, HIGH);
        delay(1000);
        digitalWrite(ERROR_LED, LOW);
        delay(1000);
    }
}