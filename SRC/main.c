// update 
#include "atuadores.h"
#include "mqtt.h"

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

    if (pthread_create(&mqtt_thread, NULL, thread_mqtt, &mqtt) != 0) {
        fprintf(stderr, "erro criando thread MQTT\n");
        atuadores_request_stop(&atuadores);
        pthread_join(atuadores_thread, NULL);
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

    mqtt_context_destroy(&mqtt);
    atuadores_context_destroy(&atuadores);

    return 0;
}
