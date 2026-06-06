#ifndef THREADS_H
#define THREADS_H

#include <pthread.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

#define ATUADORES_QUEUE_CAPACITY 16

typedef enum {
    FSM_INIT = 0,
    FSM_IDLE,
    FSM_MONITORING,
    FSM_PROCESSING_COMMAND,
    FSM_ACTUATING,
    FSM_TIMEOUT,
    FSM_ERROR,
    FSM_RECOVERY
} fsm_state_t;

typedef enum {
    ATUADOR_CMD_SET_LED = 0,
    ATUADOR_CMD_SET_RELAY,
    ATUADOR_CMD_SET_SERVO,
    ATUADOR_CMD_STOP
} atuador_cmd_type_t;

typedef enum {
    ATUADOR_STATUS_OK = 0,
    ATUADOR_STATUS_INVALID_VALUE,
    ATUADOR_STATUS_QUEUE_FULL,
    ATUADOR_STATUS_TIMEOUT,
    ATUADOR_STATUS_ERROR
} atuador_status_t;

typedef struct {
    int cmd_id;
    atuador_cmd_type_t type;
    int value;
    int deadline_ms;
    struct timespec received_at;
} atuador_cmd_t;

typedef struct {
    atuador_cmd_t items[ATUADORES_QUEUE_CAPACITY];
    size_t head;
    size_t tail;
    size_t count;
    pthread_mutex_t mutex;
    pthread_cond_t not_empty;
} atuadores_queue_t;

typedef struct {
    int led;
    int relay;
    int servo_deg;
    fsm_state_t fsm_state;
} shared_state_t;

typedef struct {
    unsigned long total_cmds;
    unsigned long deadlines_missed;
    unsigned long queue_full_count;
    unsigned long invalid_value_count;
    unsigned long actuator_errors;
} actuator_metrics_t;

typedef struct {
    atuadores_queue_t queue;
    shared_state_t state;
    actuator_metrics_t metrics;
    pthread_mutex_t state_mutex;
    pthread_mutex_t metrics_mutex;
    pthread_mutex_t heartbeat_mutex;
    struct timespec actuator_heartbeat;
    int running;
} atuadores_context_t;

const char *fsm_state_to_string(fsm_state_t state);

void atuadores_context_init(atuadores_context_t *ctx);
void atuadores_context_destroy(atuadores_context_t *ctx);
atuador_status_t atuadores_enqueue(atuadores_context_t *ctx, atuador_cmd_t cmd);
atuador_status_t atuadores_request_stop(atuadores_context_t *ctx);
size_t atuadores_queue_size(atuadores_context_t *ctx);

#endif
