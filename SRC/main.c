// update
#include "atuadores.h"
#include "mqtt.h"
#include "sensors.h"

#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static volatile sig_atomic_t should_stop;

static void handle_signal(int signal_number)
{
    (void)signal_number;
    should_stop = 1;
}

int main(int argc, char **argv)
{
    const char *host = MQTT_DEFAULT_HOST;
    const char *group = MQTT_GROUP_NAME;
    int port = MQTT_DEFAULT_PORT;
    atuadores_context_t atuadores;
    mqtt_context_t mqtt;
    pthread_t atuadores_thread;
    pthread_t mqtt_thread;
    pthread_t fsm_update_thread;
    pthread_t sensors_thread;
    pthread_t logger_thread;
    pthread_t watchdog_thread;
    pthread_t publisher_thread;

    if (argc > 1) {
        host = argv[1];
    }
    if (argc > 2) {
        group = argv[2];
    }
    if (argc > 3) {
        port = atoi(argv[3]);
    }

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    atuadores_context_init(&atuadores);
    mqtt_context_init(&mqtt, &atuadores, host, port, group);

    if (pthread_create(&atuadores_thread, NULL, thread_atuadores, &atuadores) != 0) {
        fprintf(stderr, "erro criando thread de atuadores\n");
        mqtt_context_destroy(&mqtt);
        atuadores_context_destroy(&atuadores);
        return 1;
    }

    if (pthread_create(&fsm_update_thread, NULL, thread_fsm_update, &atuadores) != 0) {
        fprintf(stderr, "erro criando thread de atualizacao da FSM\n");
        atuadores_request_stop(&atuadores);
        pthread_join(atuadores_thread, NULL);
        mqtt_context_destroy(&mqtt);
        atuadores_context_destroy(&atuadores);
        return 1;
    }

    if (pthread_create(&sensors_thread, NULL, thread_sensors, &atuadores) != 0) {
        fprintf(stderr, "erro criando thread de sensores\n");
        atuadores_request_stop(&atuadores);
        pthread_join(atuadores_thread, NULL);
        pthread_join(fsm_update_thread, NULL);
        mqtt_context_destroy(&mqtt);
        atuadores_context_destroy(&atuadores);
        return 1;
    }

    if (pthread_create(&logger_thread, NULL, thread_logger, &mqtt) != 0) {
        fprintf(stderr, "erro criando thread de logger\n");
        atuadores_request_stop(&atuadores);
        pthread_join(atuadores_thread, NULL);
        pthread_join(fsm_update_thread, NULL);
        pthread_join(sensors_thread, NULL);
        mqtt_context_destroy(&mqtt);
        atuadores_context_destroy(&atuadores);
        return 1;
    }

    if (pthread_create(&watchdog_thread, NULL, thread_watchdog, &atuadores) != 0) {
        fprintf(stderr, "erro criando thread de watchdog\n");
        atuadores_request_stop(&atuadores);
        pthread_join(atuadores_thread, NULL);
        pthread_join(fsm_update_thread, NULL);
        pthread_join(sensors_thread, NULL);
        pthread_join(logger_thread, NULL);
        mqtt_context_destroy(&mqtt);
        atuadores_context_destroy(&atuadores);
        return 1;
    }

    if (pthread_create(&publisher_thread, NULL, thread_publisher, &mqtt) != 0) {
        fprintf(stderr, "erro criando thread publisher\n");
        atuadores_request_stop(&atuadores);
        pthread_join(atuadores_thread, NULL);
        pthread_join(fsm_update_thread, NULL);
        pthread_join(sensors_thread, NULL);
        pthread_join(logger_thread, NULL);
        pthread_join(watchdog_thread, NULL);
        mqtt_context_destroy(&mqtt);
        atuadores_context_destroy(&atuadores);
        return 1;
    }

    if (pthread_create(&mqtt_thread, NULL, thread_mqtt, &mqtt) != 0) {
        fprintf(stderr, "erro criando thread MQTT\n");
        atuadores_request_stop(&atuadores);
        pthread_join(atuadores_thread, NULL);
        pthread_join(fsm_update_thread, NULL);
        pthread_join(sensors_thread, NULL);
        pthread_join(logger_thread, NULL);
        pthread_join(watchdog_thread, NULL);
        pthread_join(publisher_thread, NULL);
        mqtt_context_destroy(&mqtt);
        atuadores_context_destroy(&atuadores);
        return 1;
    }

    printf("Sistema iniciado. Broker=%s:%d Grupo=%s\n", host, port, group);
    printf("Assinando comandos em /lab/%s/cmd\n", group);

    while (!should_stop) {
        sleep(1);
    }

    mqtt_stop(&mqtt);
    atuadores_request_stop(&atuadores);

    pthread_join(mqtt_thread, NULL);
    pthread_join(atuadores_thread, NULL);
    pthread_join(fsm_update_thread, NULL);
    pthread_join(sensors_thread, NULL);
    pthread_join(logger_thread, NULL);
    pthread_join(watchdog_thread, NULL);
    pthread_join(publisher_thread, NULL);

    mqtt_context_destroy(&mqtt);
    atuadores_context_destroy(&atuadores);

    return 0;
}
