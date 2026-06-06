#include "mqtt.h"

#include <cjson/cJSON.h>

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define MQTT_QOS 0

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

static void mqtt_metric_inc(mqtt_context_t *ctx, unsigned long *counter)
{
    pthread_mutex_lock(&ctx->metrics_mutex);
    (*counter)++;
    pthread_mutex_unlock(&ctx->metrics_mutex);
}

static int mqtt_command_enqueue(mqtt_context_t *ctx, const mqtt_command_t *cmd)
{
    mqtt_command_queue_t *queue = &ctx->command_queue;

    pthread_mutex_lock(&queue->mutex);

    if (!ctx->running) {
        pthread_mutex_unlock(&queue->mutex);
        return -1;
    }

    if (queue->count == MQTT_COMMAND_QUEUE_CAPACITY) {
        mqtt_metric_inc(ctx, &ctx->command_queue_full_count);
        pthread_mutex_unlock(&queue->mutex);
        return -1;
    }

    queue->items[queue->tail] = *cmd;
    queue->tail = (queue->tail + 1) % MQTT_COMMAND_QUEUE_CAPACITY;
    queue->count++;
    pthread_cond_signal(&queue->not_empty);
    pthread_mutex_unlock(&queue->mutex);

    return 0;
}

static int mqtt_command_pop(mqtt_context_t *ctx, mqtt_command_t *cmd)
{
    mqtt_command_queue_t *queue = &ctx->command_queue;

    pthread_mutex_lock(&queue->mutex);
    while (queue->count == 0 && ctx->running) {
        pthread_cond_wait(&queue->not_empty, &queue->mutex);
    }

    if (queue->count == 0 && !ctx->running) {
        pthread_mutex_unlock(&queue->mutex);
        return 0;
    }

    *cmd = queue->items[queue->head];
    queue->head = (queue->head + 1) % MQTT_COMMAND_QUEUE_CAPACITY;
    queue->count--;
    pthread_mutex_unlock(&queue->mutex);

    return 1;
}

static size_t mqtt_command_queue_size(mqtt_context_t *ctx)
{
    size_t count;

    pthread_mutex_lock(&ctx->command_queue.mutex);
    count = ctx->command_queue.count;
    pthread_mutex_unlock(&ctx->command_queue.mutex);

    return count;
}

static mqtt_cmd_type_t parse_cmd_name(const char *cmd)
{
    if (cmd == NULL) {
        return MQTT_CMD_INVALID;
    }

    if (strcmp(cmd, "PING") == 0) {
        return MQTT_CMD_PING;
    }
    if (strcmp(cmd, "GET_STATE") == 0) {
        return MQTT_CMD_GET_STATE;
    }
    if (strcmp(cmd, "GET_METRICS") == 0) {
        return MQTT_CMD_GET_METRICS;
    }
    if (strcmp(cmd, "SET_LED") == 0) {
        return MQTT_CMD_SET_LED;
    }
    if (strcmp(cmd, "SET_RELAY") == 0) {
        return MQTT_CMD_SET_RELAY;
    }
    if (strcmp(cmd, "SET_SERVO") == 0) {
        return MQTT_CMD_SET_SERVO;
    }
    if (strcmp(cmd, "READ_SENSOR") == 0) {
        return MQTT_CMD_READ_SENSOR;
    }
    if (strcmp(cmd, "RESET_FSM") == 0) {
        return MQTT_CMD_RESET_FSM;
    }

    return MQTT_CMD_INVALID;
}

static int parse_command_json(const char *payload, mqtt_command_t *out)
{
    cJSON *root;
    cJSON *cmd_id;
    cJSON *cmd;
    cJSON *value;
    cJSON *deadline_ms;

    root = cJSON_Parse(payload);
    if (root == NULL) {
        return -1;
    }

    cmd_id = cJSON_GetObjectItemCaseSensitive(root, "cmd_id");
    cmd = cJSON_GetObjectItemCaseSensitive(root, "cmd");
    value = cJSON_GetObjectItemCaseSensitive(root, "value");
    deadline_ms = cJSON_GetObjectItemCaseSensitive(root, "deadline_ms");

    if (!cJSON_IsNumber(cmd_id) ||
        !cJSON_IsString(cmd) ||
        !cJSON_IsNumber(value) ||
        !cJSON_IsNumber(deadline_ms)) {
        cJSON_Delete(root);
        return -1;
    }

    memset(out, 0, sizeof(*out));
    out->cmd_id = cmd_id->valueint;
    out->type = parse_cmd_name(cmd->valuestring);
    out->value = value->valueint;
    out->deadline_ms = deadline_ms->valueint;
    now_monotonic(&out->received_at);

    cJSON_Delete(root);
    return 0;
}

static void metrics_snapshot(mqtt_context_t *ctx,
                             shared_state_t *state,
                             actuator_metrics_t *metrics,
                             size_t *queue_size)
{
    pthread_mutex_lock(&ctx->atuadores->state_mutex);
    *state = ctx->atuadores->state;
    pthread_mutex_unlock(&ctx->atuadores->state_mutex);

    pthread_mutex_lock(&ctx->atuadores->metrics_mutex);
    *metrics = ctx->atuadores->metrics;
    pthread_mutex_unlock(&ctx->atuadores->metrics_mutex);

    *queue_size = atuadores_queue_size(ctx->atuadores);
}

static int publish_text(mqtt_context_t *ctx, const char *topic, const char *payload)
{
    int rc;

    if (ctx == NULL || ctx->client == NULL || topic == NULL || payload == NULL) {
        return MOSQ_ERR_INVAL;
    }

    rc = mosquitto_publish(ctx->client,
                           NULL,
                           topic,
                           (int)strlen(payload),
                           payload,
                           MQTT_QOS,
                           false);
    if (rc == MOSQ_ERR_SUCCESS) {
        mqtt_metric_inc(ctx, &ctx->mqtt_tx_count);
    }

    return rc;
}

static const char *status_from_enqueue(atuador_status_t status)
{
    switch (status) {
    case ATUADOR_STATUS_OK:
        return "OK";
    case ATUADOR_STATUS_INVALID_VALUE:
        return "INVALID_VALUE";
    case ATUADOR_STATUS_QUEUE_FULL:
        return "QUEUE_FULL";
    case ATUADOR_STATUS_TIMEOUT:
        return "TIMEOUT";
    case ATUADOR_STATUS_ERROR:
    default:
        return "ERROR";
    }
}

static atuador_status_t enqueue_actuator_command(mqtt_context_t *ctx,
                                                 const mqtt_command_t *cmd)
{
    atuador_cmd_t actuator_cmd;

    memset(&actuator_cmd, 0, sizeof(actuator_cmd));
    actuator_cmd.cmd_id = cmd->cmd_id;
    actuator_cmd.value = cmd->value;
    actuator_cmd.deadline_ms = cmd->deadline_ms;
    actuator_cmd.received_at = cmd->received_at;

    switch (cmd->type) {
    case MQTT_CMD_SET_LED:
        actuator_cmd.type = ATUADOR_CMD_SET_LED;
        break;
    case MQTT_CMD_SET_RELAY:
        actuator_cmd.type = ATUADOR_CMD_SET_RELAY;
        break;
    case MQTT_CMD_SET_SERVO:
        actuator_cmd.type = ATUADOR_CMD_SET_SERVO;
        break;
    default:
        return ATUADOR_STATUS_ERROR;
    }

    return atuadores_enqueue(ctx->atuadores, actuator_cmd);
}

static int actuator_value_is_valid(const mqtt_command_t *cmd)
{
    switch (cmd->type) {
    case MQTT_CMD_SET_LED:
    case MQTT_CMD_SET_RELAY:
        return cmd->value == 0 || cmd->value == 1;
    case MQTT_CMD_SET_SERVO:
        return cmd->value >= 0 && cmd->value <= 180;
    default:
        return 1;
    }
}

static int deadline_missed(const mqtt_command_t *cmd)
{
    return cmd->deadline_ms > 0 &&
           elapsed_ms_since(&cmd->received_at) > cmd->deadline_ms;
}

static void handle_command(mqtt_context_t *ctx, const mqtt_command_t *cmd)
{
    atuador_status_t enqueue_status;
    long latency_ms;

    if (cmd->type == MQTT_CMD_INVALID) {
        mqtt_metric_inc(ctx, &ctx->invalid_cmd_count);
        mqtt_publish_ack(ctx, cmd->cmd_id, "INVALID_CMD",
                         elapsed_ms_since(&cmd->received_at));
        return;
    }

    if (deadline_missed(cmd)) {
        mqtt_publish_ack(ctx, cmd->cmd_id, "DEADLINE_MISSED",
                         elapsed_ms_since(&cmd->received_at));
        return;
    }

    switch (cmd->type) {
    case MQTT_CMD_PING:
        mqtt_publish_ack(ctx, cmd->cmd_id, "OK",
                         elapsed_ms_since(&cmd->received_at));
        break;

    case MQTT_CMD_GET_STATE:
        mqtt_publish_state(ctx);
        mqtt_publish_ack(ctx, cmd->cmd_id, "OK",
                         elapsed_ms_since(&cmd->received_at));
        break;

    case MQTT_CMD_GET_METRICS:
        mqtt_publish_metrics(ctx);
        mqtt_publish_ack(ctx, cmd->cmd_id, "OK",
                         elapsed_ms_since(&cmd->received_at));
        break;

    case MQTT_CMD_READ_SENSOR:
        mqtt_publish_state(ctx);
        mqtt_publish_ack(ctx, cmd->cmd_id, "OK",
                         elapsed_ms_since(&cmd->received_at));
        break;

    case MQTT_CMD_RESET_FSM:
        pthread_mutex_lock(&ctx->atuadores->state_mutex);
        ctx->atuadores->state.fsm_state = FSM_MONITORING;
        pthread_mutex_unlock(&ctx->atuadores->state_mutex);
        mqtt_publish_ack(ctx, cmd->cmd_id, "OK",
                         elapsed_ms_since(&cmd->received_at));
        break;

    case MQTT_CMD_SET_LED:
    case MQTT_CMD_SET_RELAY:
    case MQTT_CMD_SET_SERVO:
        if (!actuator_value_is_valid(cmd)) {
            mqtt_publish_ack(ctx, cmd->cmd_id, "INVALID_VALUE",
                             elapsed_ms_since(&cmd->received_at));
            return;
        }

        enqueue_status = enqueue_actuator_command(ctx, cmd);
        latency_ms = elapsed_ms_since(&cmd->received_at);
        mqtt_publish_ack(ctx, cmd->cmd_id,
                         status_from_enqueue(enqueue_status),
                         latency_ms);
        break;

    case MQTT_CMD_INVALID:
    default:
        break;
    }
}

static void on_connect(struct mosquitto *mosq, void *userdata, int rc)
{
    mqtt_context_t *ctx = (mqtt_context_t *)userdata;

    if (rc != 0) {
        fprintf(stderr, "[MQTT] falha na conexao: %s\n", mosquitto_connack_string(rc));
        return;
    }

    ctx->connected = 1;
    mosquitto_subscribe(mosq, NULL, ctx->topic_cmd, MQTT_QOS);
    mqtt_publish_log(ctx, "MQTT", "conectado e inscrito no topico de comandos");
}

static void on_disconnect(struct mosquitto *mosq, void *userdata, int rc)
{
    mqtt_context_t *ctx = (mqtt_context_t *)userdata;
    (void)mosq;
    (void)rc;

    ctx->connected = 0;
}

static void on_message(struct mosquitto *mosq,
                       void *userdata,
                       const struct mosquitto_message *message)
{
    mqtt_context_t *ctx = (mqtt_context_t *)userdata;
    mqtt_command_t cmd;
    char *payload;

    (void)mosq;

    if (message == NULL || message->payload == NULL || message->payloadlen <= 0) {
        mqtt_metric_inc(ctx, &ctx->invalid_json_count);
        return;
    }

    payload = calloc((size_t)message->payloadlen + 1, sizeof(char));
    if (payload == NULL) {
        return;
    }

    memcpy(payload, message->payload, (size_t)message->payloadlen);
    mqtt_metric_inc(ctx, &ctx->mqtt_rx_count);

    if (parse_command_json(payload, &cmd) != 0) {
        mqtt_metric_inc(ctx, &ctx->invalid_json_count);
        mqtt_publish_log(ctx, "MQTT_RX", "JSON invalido recebido");
        free(payload);
        return;
    }

    if (mqtt_command_enqueue(ctx, &cmd) != 0) {
        mqtt_publish_ack(ctx, cmd.cmd_id, "QUEUE_FULL",
                         elapsed_ms_since(&cmd.received_at));
    }

    free(payload);
}

void mqtt_context_init(mqtt_context_t *ctx,
                       atuadores_context_t *atuadores,
                       const char *host,
                       int port,
                       const char *group)
{
    memset(ctx, 0, sizeof(*ctx));

    snprintf(ctx->host, sizeof(ctx->host), "%s",
             host != NULL ? host : MQTT_DEFAULT_HOST);
    snprintf(ctx->group, sizeof(ctx->group), "%s",
             group != NULL ? group : MQTT_GROUP_NAME);

    ctx->port = port > 0 ? port : MQTT_DEFAULT_PORT;
    ctx->atuadores = atuadores;
    ctx->running = 1;

    pthread_mutex_init(&ctx->command_queue.mutex, NULL);
    pthread_cond_init(&ctx->command_queue.not_empty, NULL);
    pthread_mutex_init(&ctx->metrics_mutex, NULL);

    snprintf(ctx->topic_cmd, sizeof(ctx->topic_cmd), "/lab/%s/cmd", ctx->group);
    snprintf(ctx->topic_ack, sizeof(ctx->topic_ack), "/lab/%s/ack", ctx->group);
    snprintf(ctx->topic_state, sizeof(ctx->topic_state), "/lab/%s/state", ctx->group);
    snprintf(ctx->topic_metrics, sizeof(ctx->topic_metrics), "/lab/%s/metrics", ctx->group);
    snprintf(ctx->topic_log, sizeof(ctx->topic_log), "/lab/%s/log", ctx->group);
}

void mqtt_context_destroy(mqtt_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    pthread_cond_destroy(&ctx->command_queue.not_empty);
    pthread_mutex_destroy(&ctx->command_queue.mutex);
    pthread_mutex_destroy(&ctx->metrics_mutex);
}

int mqtt_start(mqtt_context_t *ctx)
{
    int rc;

    mosquitto_lib_init();

    ctx->client = mosquitto_new(MQTT_CLIENT_ID, true, ctx);
    if (ctx->client == NULL) {
        return MOSQ_ERR_NOMEM;
    }

    mosquitto_connect_callback_set(ctx->client, on_connect);
    mosquitto_disconnect_callback_set(ctx->client, on_disconnect);
    mosquitto_message_callback_set(ctx->client, on_message);

    rc = mosquitto_connect_async(ctx->client,
                                 ctx->host,
                                 ctx->port,
                                 MQTT_KEEPALIVE_SECONDS);
    if (rc != MOSQ_ERR_SUCCESS) {
        mosquitto_destroy(ctx->client);
        ctx->client = NULL;
        mosquitto_lib_cleanup();
        return rc;
    }

    rc = mosquitto_loop_start(ctx->client);
    if (rc != MOSQ_ERR_SUCCESS) {
        mosquitto_destroy(ctx->client);
        ctx->client = NULL;
        mosquitto_lib_cleanup();
        pthread_mutex_lock(&ctx->command_queue.mutex);
        ctx->running = 0;
        pthread_cond_broadcast(&ctx->command_queue.not_empty);
        pthread_mutex_unlock(&ctx->command_queue.mutex);
    }

    return rc;
}

void mqtt_stop(mqtt_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    pthread_mutex_lock(&ctx->command_queue.mutex);
    ctx->running = 0;
    pthread_cond_broadcast(&ctx->command_queue.not_empty);
    pthread_mutex_unlock(&ctx->command_queue.mutex);

    if (ctx->client == NULL) {
        return;
    }

    mosquitto_disconnect(ctx->client);
    mosquitto_loop_stop(ctx->client, true);
    mosquitto_destroy(ctx->client);
    ctx->client = NULL;
    mosquitto_lib_cleanup();
}

void *thread_mqtt(void *arg)
{
    mqtt_context_t *ctx = (mqtt_context_t *)arg;
    mqtt_command_t cmd;

    if (ctx == NULL) {
        return NULL;
    }

    if (mqtt_start(ctx) != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "[MQTT] nao foi possivel iniciar cliente\n");
        pthread_mutex_lock(&ctx->command_queue.mutex);
        ctx->running = 0;
        pthread_cond_broadcast(&ctx->command_queue.not_empty);
        pthread_mutex_unlock(&ctx->command_queue.mutex);
        return NULL;
    }

    while (mqtt_command_pop(ctx, &cmd)) {
        handle_command(ctx, &cmd);
    }

    mqtt_stop(ctx);
    return NULL;
}

int mqtt_publish_ack(mqtt_context_t *ctx,
                     int cmd_id,
                     const char *status,
                     long latency_ms)
{
    shared_state_t state;
    actuator_metrics_t metrics;
    size_t queue_size;
    char payload[256];
    int rc;

    metrics_snapshot(ctx, &state, &metrics, &queue_size);
    (void)metrics;
    (void)queue_size;

    snprintf(payload, sizeof(payload),
             "{\"cmd_id\":%d,\"status\":\"%s\",\"latency_ms\":%ld,"
             "\"fsm_state\":\"%s\"}",
             cmd_id,
             status,
             latency_ms,
             fsm_state_to_string(state.fsm_state));

    rc = publish_text(ctx, ctx->topic_ack, payload);
    if (rc == MOSQ_ERR_SUCCESS) {
        mqtt_metric_inc(ctx, &ctx->acks_sent);
    }

    return rc;
}

int mqtt_publish_state(mqtt_context_t *ctx)
{
    shared_state_t state;
    actuator_metrics_t metrics;
    size_t queue_size;
    char payload[512];

    metrics_snapshot(ctx, &state, &metrics, &queue_size);
    (void)metrics;
    (void)queue_size;

    snprintf(payload, sizeof(payload),
             "{\"fsm_state\":\"%s\","
             "\"sensors\":{},"
             "\"actuators\":{\"led\":%d,\"relay\":%d,\"servo_deg\":%d},"
             "\"system\":{\"cpu_usage_percent\":0.0,"
             "\"ram_usage_percent\":0.0,"
             "\"cpu_temp_c\":0.0}}",
             fsm_state_to_string(state.fsm_state),
             state.led,
             state.relay,
             state.servo_deg);

    return publish_text(ctx, ctx->topic_state, payload);
}

int mqtt_publish_metrics(mqtt_context_t *ctx)
{
    shared_state_t state;
    actuator_metrics_t metrics;
    size_t queue_size;
    size_t mqtt_queue_size;
    unsigned long acks_sent;
    unsigned long mqtt_rx_count;
    unsigned long mqtt_tx_count;
    unsigned long command_queue_full_count;
    unsigned long invalid_cmd_count;
    unsigned long invalid_json_count;
    char payload[512];

    metrics_snapshot(ctx, &state, &metrics, &queue_size);
    (void)state;
    mqtt_queue_size = mqtt_command_queue_size(ctx);

    pthread_mutex_lock(&ctx->metrics_mutex);
    acks_sent = ctx->acks_sent;
    mqtt_rx_count = ctx->mqtt_rx_count;
    mqtt_tx_count = ctx->mqtt_tx_count;
    command_queue_full_count = ctx->command_queue_full_count;
    invalid_cmd_count = ctx->invalid_cmd_count;
    invalid_json_count = ctx->invalid_json_count;
    pthread_mutex_unlock(&ctx->metrics_mutex);

    snprintf(payload, sizeof(payload),
             "{\"total_cmds\":%lu,"
             "\"acks_sent\":%lu,"
             "\"deadlines_missed\":%lu,"
             "\"watchdog_events\":0,"
             "\"cmd_queue_size\":%lu,"
             "\"mqtt_cmd_queue_size\":%lu,"
             "\"log_queue_size\":0,"
             "\"mqtt_rx_count\":%lu,"
             "\"mqtt_tx_count\":%lu,"
             "\"queue_full_count\":%lu,"
             "\"mqtt_queue_full_count\":%lu,"
             "\"invalid_cmd_count\":%lu,"
             "\"invalid_json_count\":%lu,"
             "\"actuator_errors\":%lu}",
             metrics.total_cmds,
             acks_sent,
             metrics.deadlines_missed,
             (unsigned long)queue_size,
             (unsigned long)mqtt_queue_size,
             mqtt_rx_count,
             mqtt_tx_count,
             metrics.queue_full_count,
             command_queue_full_count,
             invalid_cmd_count,
             invalid_json_count,
             metrics.actuator_errors);

    return publish_text(ctx, ctx->topic_metrics, payload);
}

int mqtt_publish_log(mqtt_context_t *ctx, const char *tag, const char *message)
{
    char payload[256];

    snprintf(payload, sizeof(payload),
             "[MQTT][%s] %s",
             tag != NULL ? tag : "LOG",
             message != NULL ? message : "");

    return publish_text(ctx, ctx->topic_log, payload);
}
