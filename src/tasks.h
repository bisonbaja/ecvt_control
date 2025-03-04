#ifndef TASKS_H
#define TASKS_H

void updatePID_task(void * parameter);
void logSerial_task(void * parameter);
bool check_serial();
void serial_command_task(void * parameter);

#endif // TASKS_H