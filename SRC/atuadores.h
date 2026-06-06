#ifndef ATUADORES_H
#define ATUADORES_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "threads.h"

#if defined(__has_include)
#if __has_include(<gpiod.h>)
#include <gpiod.h>
#define ATUADORES_HAS_GPIOD 1
#else
#define ATUADORES_HAS_GPIOD 0
#endif
#else
#include <gpiod.h>
#define ATUADORES_HAS_GPIOD 1
#endif

#define GPIO_CHIP_NAME "gpiochip0"
#define LED_GPIO 17
#define RELAY_GPIO 27
#define SERVO_GPIO 18

int atuadores_hw_init(void);
void atuadores_hw_close(void);

atuador_status_t aplicar_led(int state);
atuador_status_t aplicar_relay(int state);
atuador_status_t aplicar_servo(int angle);

void *thread_atuadores(void *arg);

#endif
