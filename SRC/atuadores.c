#include "atuadores.h"

#include <errno.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>

#define SERVO_PERIOD_US 20000
#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2500
#define SERVO_PULSE_CYCLES 25
#define FSM_TICK_MS 100

#if ATUADORES_HAS_GPIOD
#if defined(GPIOD_API)
#define ATUADORES_GPIOD_V1 1
#define ATUADORES_GPIOD_V2 0
#else
#define ATUADORES_GPIOD_V1 0
#define ATUADORES_GPIOD_V2 1
#endif

static struct gpiod_chip *gpio_chip;
#if ATUADORES_GPIOD_V1
static struct gpiod_line *led_line;
static struct gpiod_line *relay_line;
static struct gpiod_line *servo_line;
static int led_requested;
static int relay_requested;
static int servo_requested;
#else
static struct gpiod_line_request *gpio_request;
#endif
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
    pthread_mutex_init(&ctx->sensors_mutex, NULL);
    pthread_mutex_init(&ctx->fsm_events.mutex, NULL);
    pthread_mutex_init(&ctx->log_queue.mutex, NULL);
    pthread_cond_init(&ctx->log_queue.not_empty, NULL);

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
    pthread_mutex_destroy(&ctx->sensors_mutex);
    pthread_mutex_destroy(&ctx->fsm_events.mutex);
    pthread_cond_destroy(&ctx->log_queue.not_empty);
    pthread_mutex_destroy(&ctx->log_queue.mutex);
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
        pthread_mutex_lock(&ctx->log_queue.mutex);
        pthread_cond_broadcast(&ctx->log_queue.not_empty);
        pthread_mutex_unlock(&ctx->log_queue.mutex);
        return ATUADOR_STATUS_OK;
    }

    pthread_mutex_lock(&ctx->queue.mutex);
    ctx->running = 0;
    pthread_cond_broadcast(&ctx->queue.not_empty);
    pthread_mutex_unlock(&ctx->queue.mutex);

    pthread_mutex_lock(&ctx->log_queue.mutex);
    pthread_cond_broadcast(&ctx->log_queue.not_empty);
    pthread_mutex_unlock(&ctx->log_queue.mutex);

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

#if ATUADORES_HAS_GPIOD && ATUADORES_GPIOD_V2
static enum gpiod_line_value gpio_value_from_int(int value)
{
    return value ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE;
}

static int request_gpio_outputs_v2(void)
{
    static const unsigned int offsets[] = { LED_GPIO, RELAY_GPIO, SERVO_GPIO };
    struct gpiod_line_settings *settings = NULL;
    struct gpiod_line_config *line_config = NULL;
    struct gpiod_request_config *request_config = NULL;
    int status = -1;

    settings = gpiod_line_settings_new();
    line_config = gpiod_line_config_new();
    request_config = gpiod_request_config_new();
    if (settings == NULL || line_config == NULL || request_config == NULL) {
        goto cleanup;
    }

    if (gpiod_line_settings_set_direction(settings,
                                          GPIOD_LINE_DIRECTION_OUTPUT) < 0) {
        goto cleanup;
    }

    if (gpiod_line_settings_set_output_value(
            settings, GPIOD_LINE_VALUE_INACTIVE) < 0) {
        goto cleanup;
    }

    if (gpiod_line_config_add_line_settings(
            line_config, offsets, sizeof(offsets) / sizeof(offsets[0]),
            settings) < 0) {
        goto cleanup;
    }

    gpiod_request_config_set_consumer(request_config, "embarcados-atuadores");
    gpio_request = gpiod_chip_request_lines(gpio_chip, request_config,
                                            line_config);
    if (gpio_request == NULL) {
        goto cleanup;
    }

    status = 0;

cleanup:
    if (request_config != NULL) {
        gpiod_request_config_free(request_config);
    }
    if (line_config != NULL) {
        gpiod_line_config_free(line_config);
    }
    if (settings != NULL) {
        gpiod_line_settings_free(settings);
    }

    return status;
}
#endif

int atuadores_hw_init(void)
{
#if ATUADORES_HAS_GPIOD
#if ATUADORES_GPIOD_V1
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
    gpio_chip = gpiod_chip_open(GPIO_CHIP_PATH);
    if (gpio_chip == NULL) {
        fprintf(stderr, "[ATUADORES] erro abrindo %s: %s\n",
                GPIO_CHIP_PATH, strerror(errno));
        return -1;
    }

    if (request_gpio_outputs_v2() < 0) {
        fprintf(stderr, "[ATUADORES] erro requisitando GPIOs como saida: %s\n",
                strerror(errno));
        atuadores_hw_close();
        return -1;
    }
#endif
#else
    printf("[ATUADORES] libgpiod indisponivel; modo simulacao ativo\n");
#endif

    return 0;
}

void atuadores_hw_close(void)
{
#if ATUADORES_HAS_GPIOD
#if ATUADORES_GPIOD_V1
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
#else
    if (gpio_request != NULL) {
        gpiod_line_request_set_value(gpio_request, LED_GPIO,
                                     GPIOD_LINE_VALUE_INACTIVE);
        gpiod_line_request_set_value(gpio_request, RELAY_GPIO,
                                     GPIOD_LINE_VALUE_INACTIVE);
        gpiod_line_request_set_value(gpio_request, SERVO_GPIO,
                                     GPIOD_LINE_VALUE_INACTIVE);
        gpiod_line_request_release(gpio_request);
        gpio_request = NULL;
    }

    if (gpio_chip != NULL) {
        gpiod_chip_close(gpio_chip);
        gpio_chip = NULL;
    }
#endif
#endif
}

static atuador_status_t validate_command(const atuador_cmd_t *cmd)
{
    switch (cmd->type) {
    case ATUADOR_CMD_SET_LED:
    case ATUADOR_CMD_SET_RELAY:
        if (cmd->value != 0 && cmd->value != 1) {
            return ATUADOR_STATUS_INVALID_VALUE;
        }
        return ATUADOR_STATUS_OK;

    case ATUADOR_CMD_SET_SERVO:
        if (cmd->value < 0 || cmd->value > 180) {
            return ATUADOR_STATUS_INVALID_VALUE;
        }
        return ATUADOR_STATUS_OK;

    case ATUADOR_CMD_STOP:
        return ATUADOR_STATUS_OK;

    default:
        return ATUADOR_STATUS_ERROR;
    }
}

atuador_status_t aplicar_led(int state)
{
    if (state != 0 && state != 1) {
        return ATUADOR_STATUS_INVALID_VALUE;
    }

#if ATUADORES_HAS_GPIOD
#if ATUADORES_GPIOD_V1
    if (led_line == NULL || gpiod_line_set_value(led_line, state) < 0) {
        return ATUADOR_STATUS_ERROR;
    }
#else
    if (gpio_request == NULL ||
        gpiod_line_request_set_value(gpio_request, LED_GPIO,
                                     gpio_value_from_int(state)) < 0) {
        return ATUADOR_STATUS_ERROR;
    }
#endif
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
#if ATUADORES_GPIOD_V1
    if (relay_line == NULL || gpiod_line_set_value(relay_line, state) < 0) {
        return ATUADOR_STATUS_ERROR;
    }
#else
    if (gpio_request == NULL ||
        gpiod_line_request_set_value(gpio_request, RELAY_GPIO,
                                     gpio_value_from_int(state)) < 0) {
        return ATUADOR_STATUS_ERROR;
    }
#endif
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
#if ATUADORES_GPIOD_V1
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
    if (gpio_request == NULL) {
        return ATUADOR_STATUS_ERROR;
    }

    for (int i = 0; i < SERVO_PULSE_CYCLES; i++) {
        if (gpiod_line_request_set_value(gpio_request, SERVO_GPIO,
                                         GPIOD_LINE_VALUE_ACTIVE) < 0) {
            return ATUADOR_STATUS_ERROR;
        }
        usleep((unsigned int)pulse_us);

        if (gpiod_line_request_set_value(gpio_request, SERVO_GPIO,
                                         GPIOD_LINE_VALUE_INACTIVE) < 0) {
            return ATUADOR_STATUS_ERROR;
        }
        usleep((unsigned int)(SERVO_PERIOD_US - pulse_us));
    }
#endif
#else
    printf("[ATUADORES] SERVO=%d graus, pulso=%dus\n", angle, pulse_us);
#endif

    return ATUADOR_STATUS_OK;
}

static int fsm_init(atuadores_context_t *ctx)
{
    printf("[FSM] INIT: inicializando estruturas compartilhadas e GPIOs\n");

    update_heartbeat(ctx);

    if (atuadores_hw_init() != 0) {
        metrics_inc(&ctx->metrics_mutex, &ctx->metrics.actuator_errors);
        atuadores_post_event(ctx, FSM_EVT_INIT_FAILED);
        printf("[FSM] INIT: falha no hardware (postado INIT_FAILED)\n");
        return -1;
    }

    pthread_mutex_lock(&ctx->state_mutex);
    ctx->state.led = 0;
    ctx->state.relay = 0;
    ctx->state.servo_deg = 90;
    pthread_mutex_unlock(&ctx->state_mutex);

    atuadores_post_event(ctx, FSM_EVT_INIT_DONE);
    printf("[FSM] INIT: inicializacao completa (postado INIT_DONE)\n");
    return 0;
}

void atuadores_log(atuadores_context_t *ctx, const char *tag, const char *fmt, ...)
{
    log_entry_t entry;
    log_queue_t *q;
    va_list args;

    if (ctx == NULL) {
        return;
    }

    snprintf(entry.tag, sizeof(entry.tag), "%s", tag != NULL ? tag : "LOG");

    va_start(args, fmt);
    vsnprintf(entry.message, sizeof(entry.message), fmt != NULL ? fmt : "", args);
    va_end(args);

    q = &ctx->log_queue;

    pthread_mutex_lock(&q->mutex);
    if (q->count < LOG_QUEUE_CAPACITY) {
        q->items[q->tail] = entry;
        q->tail = (q->tail + 1) % LOG_QUEUE_CAPACITY;
        q->count++;
        pthread_cond_signal(&q->not_empty);
    }
    pthread_mutex_unlock(&q->mutex);
}

void atuadores_post_event(atuadores_context_t *ctx, fsm_event_t evt)
{
    fsm_event_queue_t *q;

    if (ctx == NULL) {
        return;
    }

    q = &ctx->fsm_events;

    pthread_mutex_lock(&q->mutex);
    if (q->count < FSM_EVENT_QUEUE_CAPACITY) {
        q->items[q->tail] = evt;
        q->tail = (q->tail + 1) % FSM_EVENT_QUEUE_CAPACITY;
        q->count++;
    }
    pthread_mutex_unlock(&q->mutex);
}

static int pop_fsm_event(atuadores_context_t *ctx, fsm_event_t *evt)
{
    fsm_event_queue_t *q = &ctx->fsm_events;
    int ok = 0;

    pthread_mutex_lock(&q->mutex);
    if (q->count > 0) {
        *evt = q->items[q->head];
        q->head = (q->head + 1) % FSM_EVENT_QUEUE_CAPACITY;
        q->count--;
        ok = 1;
    }
    pthread_mutex_unlock(&q->mutex);

    return ok;
}

static fsm_state_t read_fsm_state(atuadores_context_t *ctx)
{
    fsm_state_t state;

    pthread_mutex_lock(&ctx->state_mutex);
    state = ctx->state.fsm_state;
    pthread_mutex_unlock(&ctx->state_mutex);

    return state;
}

const char *fsm_event_to_string(fsm_event_t evt)
{
    switch (evt) {
    case FSM_EVT_INIT_DONE:        return "INIT_DONE";
    case FSM_EVT_INIT_FAILED:      return "INIT_FAILED";
    case FSM_EVT_MQTT_RX:          return "MQTT_RX";
    case FSM_EVT_CMD_DEQUEUED:     return "CMD_DEQUEUED";
    case FSM_EVT_CMD_VALIDATED:    return "CMD_VALIDATED";
    case FSM_EVT_CMD_DONE:         return "CMD_DONE";
    case FSM_EVT_CMD_INVALID:      return "CMD_INVALID";
    case FSM_EVT_CMD_ERROR:        return "CMD_ERROR";
    case FSM_EVT_DEADLINE_MISSED:  return "DEADLINE_MISSED";
    case FSM_EVT_RESET_REQUESTED:  return "RESET_REQUESTED";
    default:                       return "UNKNOWN";
    }
}

static fsm_state_t fsm_next_state(fsm_state_t current, fsm_event_t evt)
{
    switch (current) {
    case FSM_INIT:
        if (evt == FSM_EVT_INIT_DONE)   return FSM_IDLE;
        if (evt == FSM_EVT_INIT_FAILED) return FSM_ERROR;
        break;

    case FSM_IDLE:
        if (evt == FSM_EVT_MQTT_RX) return FSM_MONITORING;
        break;

    case FSM_MONITORING:
        if (evt == FSM_EVT_CMD_DEQUEUED)    return FSM_PROCESSING_COMMAND;
        if (evt == FSM_EVT_RESET_REQUESTED) return FSM_RECOVERY;
        break;

    case FSM_PROCESSING_COMMAND:
        if (evt == FSM_EVT_CMD_VALIDATED)   return FSM_ACTUATING;
        if (evt == FSM_EVT_DEADLINE_MISSED) return FSM_TIMEOUT;
        if (evt == FSM_EVT_CMD_INVALID ||
            evt == FSM_EVT_CMD_ERROR)       return FSM_ERROR;
        break;

    case FSM_ACTUATING:
        if (evt == FSM_EVT_CMD_DONE)        return FSM_MONITORING;
        if (evt == FSM_EVT_DEADLINE_MISSED) return FSM_TIMEOUT;
        if (evt == FSM_EVT_CMD_ERROR ||
            evt == FSM_EVT_CMD_INVALID)     return FSM_ERROR;
        break;

    case FSM_TIMEOUT:
    case FSM_ERROR:
        if (evt == FSM_EVT_RESET_REQUESTED) return FSM_RECOVERY;
        break;

    case FSM_RECOVERY:
        break;

    default:
        break;
    }

    return current;
}

void *thread_fsm_update(void *arg)
{
    atuadores_context_t *ctx = (atuadores_context_t *)arg;
    struct timespec tick = {
        .tv_sec = FSM_TICK_MS / 1000,
        .tv_nsec = (long)(FSM_TICK_MS % 1000) * 1000000L,
    };

    if (ctx == NULL) {
        return NULL;
    }

    printf("[FSM_UPDATE] orquestrador iniciado (tick %dms)\n", FSM_TICK_MS);

    while (ctx->running) {
        fsm_event_t evt;
        fsm_state_t current;

        nanosleep(&tick, NULL);

        if (!ctx->running) {
            break;
        }

        while (pop_fsm_event(ctx, &evt)) {
            fsm_state_t next;

            current = read_fsm_state(ctx);
            next = fsm_next_state(current, evt);

            if (next != current) {
                set_fsm_state(ctx, next);
                printf("[FSM] %s --(%s)--> %s\n",
                       fsm_state_to_string(current),
                       fsm_event_to_string(evt),
                       fsm_state_to_string(next));
            }
        }

        current = read_fsm_state(ctx);
        if (current == FSM_TIMEOUT) {
            set_fsm_state(ctx, FSM_RECOVERY);
            printf("[FSM] TIMEOUT --(tick)--> RECOVERY\n");
        } else if (current == FSM_RECOVERY) {
            set_fsm_state(ctx, FSM_IDLE);
            printf("[FSM] RECOVERY --(tick)--> IDLE\n");
        }
    }

    printf("[FSM_UPDATE] orquestrador encerrado\n");
    return NULL;
}

void *thread_atuadores(void *arg)
{
    atuadores_context_t *ctx = (atuadores_context_t *)arg;
    atuador_cmd_t cmd;

    if (ctx == NULL) {
        return NULL;
    }

    if (fsm_init(ctx) != 0) {
        pthread_mutex_lock(&ctx->queue.mutex);
        ctx->running = 0;
        pthread_cond_broadcast(&ctx->queue.not_empty);
        pthread_mutex_unlock(&ctx->queue.mutex);
        return NULL;
    }

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
        atuadores_post_event(ctx, FSM_EVT_CMD_DEQUEUED);

        printf("[FSM] PROCESSING_COMMAND: cmd_id=%d type=%d value=%d deadline=%dms\n",
               cmd.cmd_id, (int)cmd.type, cmd.value, cmd.deadline_ms);

        if (cmd.deadline_ms > 0 &&
            elapsed_ms_since(&cmd.received_at) > cmd.deadline_ms) {
            metrics_inc(&ctx->metrics_mutex, &ctx->metrics.deadlines_missed);
            printf("[FSM] PROCESSING_COMMAND: deadline excedido (%ldms > %dms)\n",
                   elapsed_ms_since(&cmd.received_at), cmd.deadline_ms);
            atuadores_post_event(ctx, FSM_EVT_DEADLINE_MISSED);
            continue;
        }

        {
            atuador_status_t validation = validate_command(&cmd);
            if (validation == ATUADOR_STATUS_INVALID_VALUE) {
                metrics_inc(&ctx->metrics_mutex, &ctx->metrics.invalid_value_count);
                printf("[FSM] PROCESSING_COMMAND: valor invalido (%d)\n", cmd.value);
                atuadores_post_event(ctx, FSM_EVT_CMD_INVALID);
                continue;
            }
            if (validation != ATUADOR_STATUS_OK) {
                metrics_inc(&ctx->metrics_mutex, &ctx->metrics.actuator_errors);
                printf("[FSM] PROCESSING_COMMAND: tipo de comando desconhecido\n");
                atuadores_post_event(ctx, FSM_EVT_CMD_ERROR);
                continue;
            }
        }

        atuadores_post_event(ctx, FSM_EVT_CMD_VALIDATED);

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
            atuadores_post_event(ctx, FSM_EVT_CMD_INVALID);
        } else if (status != ATUADOR_STATUS_OK) {
            metrics_inc(&ctx->metrics_mutex, &ctx->metrics.actuator_errors);
            atuadores_post_event(ctx, FSM_EVT_CMD_ERROR);
        } else if (cmd.deadline_ms > 0 &&
                   elapsed_ms_since(&cmd.received_at) > cmd.deadline_ms) {
            metrics_inc(&ctx->metrics_mutex, &ctx->metrics.deadlines_missed);
            atuadores_post_event(ctx, FSM_EVT_DEADLINE_MISSED);
        } else {
            atuadores_post_event(ctx, FSM_EVT_CMD_DONE);
        }
    }

    atuadores_hw_close();

    return NULL;
}
