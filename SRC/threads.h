// UPDATE 
#ifndef THREADS_H
#define THREADS_H

#include <pthread.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

#define ATUADORES_QUEUE_CAPACITY 16
#define FSM_EVENT_QUEUE_CAPACITY 64

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
    FSM_EVT_INIT_DONE = 0,
    FSM_EVT_INIT_FAILED,
    FSM_EVT_MQTT_RX,
    FSM_EVT_CMD_DEQUEUED,
    FSM_EVT_CMD_VALIDATED,
    FSM_EVT_CMD_DONE,
    FSM_EVT_CMD_INVALID,
    FSM_EVT_CMD_ERROR,
    FSM_EVT_DEADLINE_MISSED,
    FSM_EVT_RESET_REQUESTED
} fsm_event_t;

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
    fsm_event_t items[FSM_EVENT_QUEUE_CAPACITY];
    size_t head;
    size_t tail;
    size_t count;
    pthread_mutex_t mutex;
} fsm_event_queue_t;

typedef struct {
    int led;
    int relay;
    int servo_deg;
    fsm_state_t fsm_state;
} shared_state_t;

typedef struct {
    float temperature_c;
    float distance_cm;
    int temperature_valid;
    int distance_valid;
} sensor_readings_t;

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
    sensor_readings_t sensors;
    pthread_mutex_t state_mutex;
    pthread_mutex_t metrics_mutex;
    pthread_mutex_t heartbeat_mutex;
    pthread_mutex_t sensors_mutex;
    struct timespec actuator_heartbeat;
    int running;
    fsm_event_queue_t fsm_events;
} atuadores_context_t;

const char *fsm_state_to_string(fsm_state_t state);
const char *fsm_event_to_string(fsm_event_t evt);

void atuadores_context_init(atuadores_context_t *ctx);
void atuadores_context_destroy(atuadores_context_t *ctx);
atuador_status_t atuadores_enqueue(atuadores_context_t *ctx, atuador_cmd_t cmd);
atuador_status_t atuadores_request_stop(atuadores_context_t *ctx);
size_t atuadores_queue_size(atuadores_context_t *ctx);
void atuadores_post_event(atuadores_context_t *ctx, fsm_event_t evt);

#endif
