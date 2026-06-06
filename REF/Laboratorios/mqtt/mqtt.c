#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <mosquitto.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <ifaddrs.h>
#include <net/if.h>
#include <arpa/inet.h>

#define MQTT_HOST "localhost"
#define MQTT_PORT 1883
#define MQTT_TOPIC "casa/sala/temperatura"

/* -------------------------------------------------
   Função para obter IP local automaticamente
------------------------------------------------- */

void obter_ip_local(char *ip_str, size_t tamanho)
{
    struct ifaddrs *ifaddr, *ifa;

    snprintf(ip_str, tamanho, "0.0.0.0");

    if (getifaddrs(&ifaddr) == -1) {
        return;
    }

    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {

        if (ifa->ifa_addr == NULL)
            continue;

        if (ifa->ifa_addr->sa_family != AF_INET)
            continue;

        if (ifa->ifa_flags & IFF_LOOPBACK)
            continue;

        struct sockaddr_in *addr = (struct sockaddr_in *)ifa->ifa_addr;

        inet_ntop(AF_INET,
                  &(addr->sin_addr),
                  ip_str,
                  tamanho);

        break;
    }

    freeifaddrs(ifaddr);
}

int main(void)
{
    struct mosquitto *mosq;

    char payload[256];
    char timestamp[16];
    char sensor_ip[32];

    int tempo_envio_s = 2;   // intervalo entre envios

    float temperatura = 25.4;
    int umidade = 61;

    /* ----------------------------------------
       Obtém IP automaticamente
    ---------------------------------------- */
    //obter_ip_local(sensor_ip);
    obter_ip_local(sensor_ip, sizeof(sensor_ip));

    printf("IP local detectado: %s\n", sensor_ip);

    /* ----------------------------------------
       Inicializa Mosquitto
    ---------------------------------------- */
    mosquitto_lib_init();

    mosq = mosquitto_new("publisher_raspberry", true, NULL);

    if (mosq == NULL) {
        fprintf(stderr, "Erro ao criar cliente MQTT.\n");
        return 1;
    }

    /* ----------------------------------------
       Conecta ao broker
    ---------------------------------------- */
    if (mosquitto_connect(mosq,
                          MQTT_HOST,
                          MQTT_PORT,
                          60) != MOSQ_ERR_SUCCESS)
    {
        fprintf(stderr, "Erro ao conectar ao broker MQTT.\n");

        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();

        return 1;
    }

    printf("Conectado ao broker MQTT.\n");

    /* ----------------------------------------
       Loop principal
    ---------------------------------------- */
    while (1)
    {
        time_t agora = time(NULL);

        struct tm *tm_info = localtime(&agora);

        strftime(timestamp,
                 sizeof(timestamp),
                 "%H:%M:%S",
                 tm_info);

        /* ----------------------------------------
           Monta JSON
        ---------------------------------------- */
        snprintf(payload,
                 sizeof(payload),

                 "{"
                 "\"sensor IP\":\"%s\","
                 "\"temperatura\":%.1f,"
                 "\"umidade\":%d,"
                 "\"timestamp\":\"%s\""
                 "}",

                 sensor_ip,
                 temperatura,
                 umidade,
                 timestamp);

        printf("\nPublicando MQTT:\n%s\n",
               payload);

        /* ----------------------------------------
           Publica MQTT
        ---------------------------------------- */
        mosquitto_publish(mosq,
                          NULL,
                          MQTT_TOPIC,
                          strlen(payload),
                          payload,
                          0,
                          false);

        /* ----------------------------------------
           Simulação
        ---------------------------------------- */
        temperatura += 0.2;

        if (temperatura > 35.0) {
            temperatura = 25.0;
        }

        sleep(tempo_envio_s);
    }

    /* ----------------------------------------
       Finalização
    ---------------------------------------- */
    mosquitto_disconnect(mosq);

    mosquitto_destroy(mosq);

    mosquitto_lib_cleanup();

    return 0;
}
