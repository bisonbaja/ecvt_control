#ifndef TASKS_H
#define TASKS_H

void updatePID_task(void * parameter);
void log_task(void * parameter);
void serial_command_task(void * parameter);

#endif // TASKS_H