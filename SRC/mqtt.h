// UPDATE 
#ifndef MQTT_H
#define MQTT_H

#include <mosquitto.h>
#include <pthread.h>
#include <stddef.h>
#include <time.h>

#include "threads.h"

#define MQTT_COMMAND_QUEUE_CAPACITY 16
#define MQTT_DEFAULT_HOST "127.0.0.1"
#define MQTT_DEFAULT_PORT 1883
#define MQTT_KEEPALIVE_SECONDS 60
#define MQTT_CLIENT_ID "grupo1-embarcados"
#define MQTT_GROUP_NAME "grupo1"

typedef enum {
    MQTT_CMD_PING = 0,
    MQTT_CMD_GET_STATE,
    MQTT_CMD_GET_METRICS,
    MQTT_CMD_SET_LED,
    MQTT_CMD_SET_RELAY,
    MQTT_CMD_SET_SERVO,
    MQTT_CMD_READ_SENSOR,
    MQTT_CMD_RESET_FSM,
    MQTT_CMD_INVALID
} mqtt_cmd_type_t;

typedef struct {
    int cmd_id;
    mqtt_cmd_type_t type;
    int value;
    int deadline_ms;
    struct timespec received_at;
} mqtt_command_t;

typedef struct {
    mqtt_command_t items[MQTT_COMMAND_QUEUE_CAPACITY];
    size_t head;
    size_t tail;
    size_t count;
    pthread_mutex_t mutex;
    pthread_cond_t not_empty;
} mqtt_command_queue_t;

typedef struct {
    char host[128];
    int port;
    char group[32];
    char topic_cmd[96];
    char topic_ack[96];
    char topic_state[96];
    char topic_metrics[96];
    char topic_log[96];
    struct mosquitto *client;
    atuadores_context_t *atuadores;
    mqtt_command_queue_t command_queue;
    pthread_mutex_t metrics_mutex;
    int connected;
    int running;
    unsigned long mqtt_rx_count;
    unsigned long mqtt_tx_count;
    unsigned long acks_sent;
    unsigned long invalid_cmd_count;
    unsigned long invalid_json_count;
    unsigned long command_queue_full_count;
} mqtt_context_t;

void mqtt_context_init(mqtt_context_t *ctx,
                       atuadores_context_t *atuadores,
                       const char *host,
                       int port,
                       const char *group);
void mqtt_context_destroy(mqtt_context_t *ctx);

int mqtt_start(mqtt_context_t *ctx);
void mqtt_stop(mqtt_context_t *ctx);
void *thread_mqtt(void *arg);
void *thread_logger(void *arg);

int mqtt_publish_ack(mqtt_context_t *ctx,
                     int cmd_id,
                     const char *status,
                     long latency_ms);
int mqtt_publish_state(mqtt_context_t *ctx);
int mqtt_publish_metrics(mqtt_context_t *ctx);
int mqtt_publish_log(mqtt_context_t *ctx, const char *tag, const char *message);

#endif
