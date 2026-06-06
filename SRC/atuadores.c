#include "atuadores.h"

#include <errno.h>
#include <string.h>
#include <time.h>

#define SERVO_PERIOD_US 20000
#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2500
#define SERVO_PULSE_CYCLES 25

#if ATUADORES_HAS_GPIOD
static struct gpiod_chip *gpio_chip;
static struct gpiod_line *led_line;
static struct gpiod_line *relay_line;
static struct gpiod_line *servo_line;
static int led_requested;
static int relay_requested;
static int servo_requested;
#endif

static void now_monotonic(struct timespec *ts)
{
    clock_gettime(CLOCK_MONOTONIC, ts);
}

static long elapsed_ms_since(const struct timespec *start)
{
    struct timespec end;
    now_monotonic(&end);

    return (long)((end.tv_sec - start->tv_sec) * 1000L +
                  (end.tv_nsec - start->tv_nsec) / 1000000L);
}

static void metrics_inc(pthread_mutex_t *mutex, unsigned long *counter)
{
    pthread_mutex_lock(mutex);
    (*counter)++;
    pthread_mutex_unlock(mutex);
}

static void set_fsm_state(atuadores_context_t *ctx, fsm_state_t state)
{
    pthread_mutex_lock(&ctx->state_mutex);
    ctx->state.fsm_state = state;
    pthread_mutex_unlock(&ctx->state_mutex);
}

static void update_heartbeat(atuadores_context_t *ctx)
{
    pthread_mutex_lock(&ctx->heartbeat_mutex);
    now_monotonic(&ctx->actuator_heartbeat);
    pthread_mutex_unlock(&ctx->heartbeat_mutex);
}

static int queue_pop(atuadores_context_t *ctx, atuador_cmd_t *cmd)
{
    atuadores_queue_t *queue = &ctx->queue;

    pthread_mutex_lock(&queue->mutex);
    while (queue->count == 0 && ctx->running) {
        pthread_cond_wait(&queue->not_empty, &queue->mutex);
    }

    if (queue->count == 0 && !ctx->running) {
        pthread_mutex_unlock(&queue->mutex);
        return 0;
    }

    *cmd = queue->items[queue->head];
    queue->head = (queue->head + 1) % ATUADORES_QUEUE_CAPACITY;
    queue->count--;
    pthread_mutex_unlock(&queue->mutex);

    return 1;
}

const char *fsm_state_to_string(fsm_state_t state)
{
    switch (state) {
    case FSM_INIT:
        return "INIT";
    case FSM_IDLE:
        return "IDLE";
    case FSM_MONITORING:
        return "MONITORING";
    case FSM_PROCESSING_COMMAND:
        return "PROCESSING_COMMAND";
    case FSM_ACTUATING:
        return "ACTUATING";
    case FSM_TIMEOUT:
        return "TIMEOUT";
    case FSM_ERROR:
        return "ERROR";
    case FSM_RECOVERY:
        return "RECOVERY";
    default:
        return "ERROR";
    }
}

void atuadores_context_init(atuadores_context_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));

    pthread_mutex_init(&ctx->queue.mutex, NULL);
    pthread_cond_init(&ctx->queue.not_empty, NULL);
    pthread_mutex_init(&ctx->state_mutex, NULL);
    pthread_mutex_init(&ctx->metrics_mutex, NULL);
    pthread_mutex_init(&ctx->heartbeat_mutex, NULL);

    ctx->state.led = 0;
    ctx->state.relay = 0;
    ctx->state.servo_deg = 90;
    ctx->state.fsm_state = FSM_INIT;
    ctx->running = 1;
    now_monotonic(&ctx->actuator_heartbeat);
}

void atuadores_context_destroy(atuadores_context_t *ctx)
{
    pthread_cond_destroy(&ctx->queue.not_empty);
    pthread_mutex_destroy(&ctx->queue.mutex);
    pthread_mutex_destroy(&ctx->state_mutex);
    pthread_mutex_destroy(&ctx->metrics_mutex);
    pthread_mutex_destroy(&ctx->heartbeat_mutex);
}

atuador_status_t atuadores_enqueue(atuadores_context_t *ctx, atuador_cmd_t cmd)
{
    atuadores_queue_t *queue = &ctx->queue;

    now_monotonic(&cmd.received_at);

    pthread_mutex_lock(&queue->mutex);
    if (!ctx->running) {
        pthread_mutex_unlock(&queue->mutex);
        return ATUADOR_STATUS_ERROR;
    }

    if (queue->count == ATUADORES_QUEUE_CAPACITY) {
        pthread_mutex_unlock(&queue->mutex);
        metrics_inc(&ctx->metrics_mutex, &ctx->metrics.queue_full_count);
        return ATUADOR_STATUS_QUEUE_FULL;
    }

    queue->items[queue->tail] = cmd;
    queue->tail = (queue->tail + 1) % ATUADORES_QUEUE_CAPACITY;
    queue->count++;
    pthread_cond_signal(&queue->not_empty);
    pthread_mutex_unlock(&queue->mutex);

    return ATUADOR_STATUS_OK;
}

atuador_status_t atuadores_request_stop(atuadores_context_t *ctx)
{
    atuador_cmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.type = ATUADOR_CMD_STOP;
    cmd.deadline_ms = 1000;

    if (atuadores_enqueue(ctx, cmd) == ATUADOR_STATUS_OK) {
        return ATUADOR_STATUS_OK;
    }

    pthread_mutex_lock(&ctx->queue.mutex);
    ctx->running = 0;
    pthread_cond_broadcast(&ctx->queue.not_empty);
    pthread_mutex_unlock(&ctx->queue.mutex);

    return ATUADOR_STATUS_QUEUE_FULL;
}

size_t atuadores_queue_size(atuadores_context_t *ctx)
{
    size_t count;

    pthread_mutex_lock(&ctx->queue.mutex);
    count = ctx->queue.count;
    pthread_mutex_unlock(&ctx->queue.mutex);

    return count;
}

int atuadores_hw_init(void)
{
#if ATUADORES_HAS_GPIOD
    gpio_chip = gpiod_chip_open_by_name(GPIO_CHIP_NAME);
    if (gpio_chip == NULL) {
        fprintf(stderr, "[ATUADORES] erro abrindo %s: %s\n",
                GPIO_CHIP_NAME, strerror(errno));
        return -1;
    }

    led_line = gpiod_chip_get_line(gpio_chip, LED_GPIO);
    relay_line = gpiod_chip_get_line(gpio_chip, RELAY_GPIO);
    servo_line = gpiod_chip_get_line(gpio_chip, SERVO_GPIO);

    if (led_line == NULL || relay_line == NULL || servo_line == NULL) {
        fprintf(stderr, "[ATUADORES] erro obtendo linhas GPIO\n");
        atuadores_hw_close();
        return -1;
    }

    if (gpiod_line_request_output(led_line, "embarcados-led", 0) < 0) {
        fprintf(stderr, "[ATUADORES] erro requisitando LED como saida: %s\n",
                strerror(errno));
        atuadores_hw_close();
        return -1;
    }
    led_requested = 1;

    if (gpiod_line_request_output(relay_line, "embarcados-relay", 0) < 0) {
        fprintf(stderr, "[ATUADORES] erro requisitando rele como saida: %s\n",
                strerror(errno));
        atuadores_hw_close();
        return -1;
    }
    relay_requested = 1;

    if (gpiod_line_request_output(servo_line, "embarcados-servo", 0) < 0) {
        fprintf(stderr, "[ATUADORES] erro requisitando servo como saida: %s\n",
                strerror(errno));
        atuadores_hw_close();
        return -1;
    }
    servo_requested = 1;
#else
    printf("[ATUADORES] libgpiod indisponivel; modo simulacao ativo\n");
#endif

    return 0;
}

void atuadores_hw_close(void)
{
#if ATUADORES_HAS_GPIOD
    if (led_line != NULL) {
        if (led_requested) {
            gpiod_line_set_value(led_line, 0);
            gpiod_line_release(led_line);
        }
        led_line = NULL;
        led_requested = 0;
    }

    if (relay_line != NULL) {
        if (relay_requested) {
            gpiod_line_set_value(relay_line, 0);
            gpiod_line_release(relay_line);
        }
        relay_line = NULL;
        relay_requested = 0;
    }

    if (servo_line != NULL) {
        if (servo_requested) {
            gpiod_line_set_value(servo_line, 0);
            gpiod_line_release(servo_line);
        }
        servo_line = NULL;
        servo_requested = 0;
    }

    if (gpio_chip != NULL) {
        gpiod_chip_close(gpio_chip);
        gpio_chip = NULL;
    }
#endif
}

atuador_status_t aplicar_led(int state)
{
    if (state != 0 && state != 1) {
        return ATUADOR_STATUS_INVALID_VALUE;
    }

#if ATUADORES_HAS_GPIOD
    if (led_line == NULL || gpiod_line_set_value(led_line, state) < 0) {
        return ATUADOR_STATUS_ERROR;
    }
#else
    printf("[ATUADORES] LED=%d\n", state);
#endif

    return ATUADOR_STATUS_OK;
}

atuador_status_t aplicar_relay(int state)
{
    if (state != 0 && state != 1) {
        return ATUADOR_STATUS_INVALID_VALUE;
    }

#if ATUADORES_HAS_GPIOD
    if (relay_line == NULL || gpiod_line_set_value(relay_line, state) < 0) {
        return ATUADOR_STATUS_ERROR;
    }
#else
    printf("[ATUADORES] RELAY=%d\n", state);
#endif

    return ATUADOR_STATUS_OK;
}

atuador_status_t aplicar_servo(int angle)
{
    int pulse_us;

    if (angle < 0 || angle > 180) {
        return ATUADOR_STATUS_INVALID_VALUE;
    }

    pulse_us = SERVO_MIN_PULSE_US +
               ((SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * angle) / 180;

#if ATUADORES_HAS_GPIOD
    if (servo_line == NULL) {
        return ATUADOR_STATUS_ERROR;
    }

    for (int i = 0; i < SERVO_PULSE_CYCLES; i++) {
        if (gpiod_line_set_value(servo_line, 1) < 0) {
            return ATUADOR_STATUS_ERROR;
        }
        usleep((unsigned int)pulse_us);

        if (gpiod_line_set_value(servo_line, 0) < 0) {
            return ATUADOR_STATUS_ERROR;
        }
        usleep((unsigned int)(SERVO_PERIOD_US - pulse_us));
    }
#else
    printf("[ATUADORES] SERVO=%d graus, pulso=%dus\n", angle, pulse_us);
#endif

    return ATUADOR_STATUS_OK;
}

void *thread_atuadores(void *arg)
{
    atuadores_context_t *ctx = (atuadores_context_t *)arg;
    atuador_cmd_t cmd;

    if (ctx == NULL) {
        return NULL;
    }

    set_fsm_state(ctx, FSM_IDLE);

    if (atuadores_hw_init() != 0) {
        metrics_inc(&ctx->metrics_mutex, &ctx->metrics.actuator_errors);
        set_fsm_state(ctx, FSM_ERROR);
        pthread_mutex_lock(&ctx->queue.mutex);
        ctx->running = 0;
        pthread_cond_broadcast(&ctx->queue.not_empty);
        pthread_mutex_unlock(&ctx->queue.mutex);
        return NULL;
    }

    set_fsm_state(ctx, FSM_MONITORING);

    while (queue_pop(ctx, &cmd)) {
        atuador_status_t status = ATUADOR_STATUS_OK;

        update_heartbeat(ctx);

        if (cmd.type == ATUADOR_CMD_STOP) {
            pthread_mutex_lock(&ctx->queue.mutex);
            ctx->running = 0;
            pthread_cond_broadcast(&ctx->queue.not_empty);
            pthread_mutex_unlock(&ctx->queue.mutex);
            break;
        }

        metrics_inc(&ctx->metrics_mutex, &ctx->metrics.total_cmds);
        set_fsm_state(ctx, FSM_PROCESSING_COMMAND);

        if (cmd.deadline_ms > 0 &&
            elapsed_ms_since(&cmd.received_at) > cmd.deadline_ms) {
            metrics_inc(&ctx->metrics_mutex, &ctx->metrics.deadlines_missed);
            set_fsm_state(ctx, FSM_TIMEOUT);
            continue;
        }

        set_fsm_state(ctx, FSM_ACTUATING);

        switch (cmd.type) {
        case ATUADOR_CMD_SET_LED:
            status = aplicar_led(cmd.value);
            if (status == ATUADOR_STATUS_OK) {
                pthread_mutex_lock(&ctx->state_mutex);
                ctx->state.led = cmd.value;
                pthread_mutex_unlock(&ctx->state_mutex);
            }
            break;

        case ATUADOR_CMD_SET_RELAY:
            status = aplicar_relay(cmd.value);
            if (status == ATUADOR_STATUS_OK) {
                pthread_mutex_lock(&ctx->state_mutex);
                ctx->state.relay = cmd.value;
                pthread_mutex_unlock(&ctx->state_mutex);
            }
            break;

        case ATUADOR_CMD_SET_SERVO:
            status = aplicar_servo(cmd.value);
            if (status == ATUADOR_STATUS_OK) {
                pthread_mutex_lock(&ctx->state_mutex);
                ctx->state.servo_deg = cmd.value;
                pthread_mutex_unlock(&ctx->state_mutex);
            }
            break;

        default:
            status = ATUADOR_STATUS_ERROR;
            break;
        }

        if (status == ATUADOR_STATUS_INVALID_VALUE) {
            metrics_inc(&ctx->metrics_mutex, &ctx->metrics.invalid_value_count);
            set_fsm_state(ctx, FSM_ERROR);
        } else if (status != ATUADOR_STATUS_OK) {
            metrics_inc(&ctx->metrics_mutex, &ctx->metrics.actuator_errors);
            set_fsm_state(ctx, FSM_ERROR);
        } else if (cmd.deadline_ms > 0 &&
                   elapsed_ms_since(&cmd.received_at) > cmd.deadline_ms) {
            metrics_inc(&ctx->metrics_mutex, &ctx->metrics.deadlines_missed);
            set_fsm_state(ctx, FSM_TIMEOUT);
        } else {
            set_fsm_state(ctx, FSM_MONITORING);
        }
    }

    atuadores_hw_close();
    set_fsm_state(ctx, FSM_IDLE);

    return NULL;
}
