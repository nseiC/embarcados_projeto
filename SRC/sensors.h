#ifndef SENSORS_H
#define SENSORS_H

#include "threads.h"

#define HCSR04_TRIG_GPIO 23
#define HCSR04_ECHO_GPIO 24
#define MCP3008_LM35_CHANNEL 0
#define SENSORS_PERIOD_MS 500

int sensors_hw_init(void);
void sensors_hw_close(void);
void *thread_sensors(void *arg);

#endif
