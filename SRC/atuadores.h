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

#if ATUADORES_HAS_GPIOD
#if defined(GPIOD_API)
#define ATUADORES_GPIOD_V1 1
#define ATUADORES_GPIOD_V2 0
#else
#define ATUADORES_GPIOD_V1 0
#define ATUADORES_GPIOD_V2 1
#endif
#endif

#define GPIO_CHIP_NAME "gpiochip0"
#define GPIO_CHIP_PATH "/dev/" GPIO_CHIP_NAME
#define LED_GPIO 26
#define RELAY_GPIO 19
#define SERVO_GPIO 18

int atuadores_hw_init(void);
void atuadores_hw_close(void);

atuador_status_t aplicar_led(int state);
atuador_status_t aplicar_relay(int state);
atuador_status_t aplicar_servo(int angle);

void *thread_atuadores(void *arg);
void *thread_fsm_update(void *arg);
void *thread_watchdog(void *arg);

#endif
