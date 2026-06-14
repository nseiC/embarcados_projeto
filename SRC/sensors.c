#include "sensors.h"
#include "atuadores.h"

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include <linux/spi/spidev.h>

#define SPI_DEVICE "/dev/spidev0.0"
#define SPI_SPEED_HZ 1000000
#define MCP3008_VREF 3.3f
#define HCSR04_TIMEOUT_US 30000
#define HCSR04_PULSE_TO_CM 58.0f

#if ATUADORES_HAS_GPIOD
static struct gpiod_chip *sensors_chip;
#if ATUADORES_GPIOD_V1
static struct gpiod_line *trig_line;
static struct gpiod_line *echo_line;
static int trig_requested;
static int echo_requested;
#else
static struct gpiod_line_request *sensors_request;
#endif
#endif

static int spi_fd = -1;

static long elapsed_us_since(const struct timespec *start)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec - start->tv_sec) * 1000000L +
           (now.tv_nsec - start->tv_nsec) / 1000L;
}

static int trig_set(int v)
{
#if ATUADORES_HAS_GPIOD
#if ATUADORES_GPIOD_V1
    if (trig_line == NULL) {
        return -1;
    }
    return gpiod_line_set_value(trig_line, v);
#else
    if (sensors_request == NULL) {
        return -1;
    }
    return gpiod_line_request_set_value(
        sensors_request, HCSR04_TRIG_GPIO,
        v ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
#endif
#else
    (void)v;
    return 0;
#endif
}

static int echo_get(void)
{
#if ATUADORES_HAS_GPIOD
#if ATUADORES_GPIOD_V1
    if (echo_line == NULL) {
        return -1;
    }
    return gpiod_line_get_value(echo_line);
#else
    if (sensors_request == NULL) {
        return -1;
    }
    return gpiod_line_request_get_value(sensors_request, HCSR04_ECHO_GPIO) ==
                   GPIOD_LINE_VALUE_ACTIVE
               ? 1
               : 0;
#endif
#else
    return 0;
#endif
}

static int spi_init(void)
{
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = SPI_SPEED_HZ;

    spi_fd = open(SPI_DEVICE, O_RDWR);
    if (spi_fd < 0) {
        fprintf(stderr, "[SENSORS] erro abrindo %s: %s\n",
                SPI_DEVICE, strerror(errno));
        return -1;
    }

    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0 ||
        ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        fprintf(stderr, "[SENSORS] erro configurando SPI: %s\n",
                strerror(errno));
        close(spi_fd);
        spi_fd = -1;
        return -1;
    }

    return 0;
}

static int read_mcp3008(int channel)
{
    uint8_t tx[3] = { 0x01, (uint8_t)(0x80 | ((channel & 0x07) << 4)), 0x00 };
    uint8_t rx[3] = { 0, 0, 0 };
    struct spi_ioc_transfer tr;

    if (spi_fd < 0) {
        return -1;
    }

    memset(&tr, 0, sizeof(tr));
    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = 3;
    tr.speed_hz = SPI_SPEED_HZ;
    tr.bits_per_word = 8;

    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        return -1;
    }

    return ((rx[1] & 0x03) << 8) | rx[2];
}

static int read_temperature(float *out)
{
    int adc = read_mcp3008(MCP3008_LM35_CHANNEL);
    if (adc < 0) {
        return -1;
    }

    float voltage = (adc * MCP3008_VREF) / 1023.0f;
    *out = voltage * 100.0f;
    return 0;
}

static int read_distance(float *out)
{
    struct timespec t_rise;
    struct timespec t_fall;
    struct timespec t_wait;
    long pulse_us;

    if (trig_set(0) < 0) {
        return -1;
    }
    usleep(2);
    if (trig_set(1) < 0) {
        return -1;
    }
    usleep(10);
    if (trig_set(0) < 0) {
        return -1;
    }

    clock_gettime(CLOCK_MONOTONIC, &t_wait);
    while (echo_get() == 0) {
        if (elapsed_us_since(&t_wait) > HCSR04_TIMEOUT_US) {
            return -1;
        }
    }
    clock_gettime(CLOCK_MONOTONIC, &t_rise);

    while (echo_get() == 1) {
        if (elapsed_us_since(&t_rise) > HCSR04_TIMEOUT_US) {
            return -1;
        }
    }
    clock_gettime(CLOCK_MONOTONIC, &t_fall);

    pulse_us = (t_fall.tv_sec - t_rise.tv_sec) * 1000000L +
               (t_fall.tv_nsec - t_rise.tv_nsec) / 1000L;

    *out = (float)pulse_us / HCSR04_PULSE_TO_CM;
    return 0;
}

#if ATUADORES_HAS_GPIOD && ATUADORES_GPIOD_V2
static int request_sensor_lines_v2(void)
{
    struct gpiod_line_settings *trig_settings = NULL;
    struct gpiod_line_settings *echo_settings = NULL;
    struct gpiod_line_config *line_config = NULL;
    struct gpiod_request_config *request_config = NULL;
    const unsigned int trig_off[] = { HCSR04_TRIG_GPIO };
    const unsigned int echo_off[] = { HCSR04_ECHO_GPIO };
    int status = -1;

    trig_settings = gpiod_line_settings_new();
    echo_settings = gpiod_line_settings_new();
    line_config = gpiod_line_config_new();
    request_config = gpiod_request_config_new();
    if (!trig_settings || !echo_settings || !line_config || !request_config) {
        goto cleanup;
    }

    if (gpiod_line_settings_set_direction(trig_settings,
                                          GPIOD_LINE_DIRECTION_OUTPUT) < 0 ||
        gpiod_line_settings_set_output_value(trig_settings,
                                             GPIOD_LINE_VALUE_INACTIVE) < 0 ||
        gpiod_line_settings_set_direction(echo_settings,
                                          GPIOD_LINE_DIRECTION_INPUT) < 0) {
        goto cleanup;
    }

    if (gpiod_line_config_add_line_settings(line_config, trig_off, 1,
                                            trig_settings) < 0 ||
        gpiod_line_config_add_line_settings(line_config, echo_off, 1,
                                            echo_settings) < 0) {
        goto cleanup;
    }

    gpiod_request_config_set_consumer(request_config, "embarcados-sensors");
    sensors_request = gpiod_chip_request_lines(sensors_chip, request_config,
                                               line_config);
    if (sensors_request == NULL) {
        goto cleanup;
    }

    status = 0;

cleanup:
    if (request_config) gpiod_request_config_free(request_config);
    if (line_config) gpiod_line_config_free(line_config);
    if (echo_settings) gpiod_line_settings_free(echo_settings);
    if (trig_settings) gpiod_line_settings_free(trig_settings);
    return status;
}
#endif

int sensors_hw_init(void)
{
#if ATUADORES_HAS_GPIOD
#if ATUADORES_GPIOD_V1
    sensors_chip = gpiod_chip_open_by_name(GPIO_CHIP_NAME);
    if (sensors_chip == NULL) {
        fprintf(stderr, "[SENSORS] erro abrindo %s: %s\n",
                GPIO_CHIP_NAME, strerror(errno));
        goto fail_no_chip;
    }

    trig_line = gpiod_chip_get_line(sensors_chip, HCSR04_TRIG_GPIO);
    echo_line = gpiod_chip_get_line(sensors_chip, HCSR04_ECHO_GPIO);
    if (trig_line == NULL || echo_line == NULL) {
        fprintf(stderr, "[SENSORS] erro obtendo linhas trig/echo\n");
        goto fail;
    }

    if (gpiod_line_request_output(trig_line, "embarcados-trig", 0) < 0) {
        fprintf(stderr, "[SENSORS] erro requisitando TRIG: %s\n",
                strerror(errno));
        goto fail;
    }
    trig_requested = 1;

    if (gpiod_line_request_input(echo_line, "embarcados-echo") < 0) {
        fprintf(stderr, "[SENSORS] erro requisitando ECHO: %s\n",
                strerror(errno));
        goto fail;
    }
    echo_requested = 1;
#else
    sensors_chip = gpiod_chip_open(GPIO_CHIP_PATH);
    if (sensors_chip == NULL) {
        fprintf(stderr, "[SENSORS] erro abrindo %s: %s\n",
                GPIO_CHIP_PATH, strerror(errno));
        goto fail_no_chip;
    }
    if (request_sensor_lines_v2() < 0) {
        fprintf(stderr, "[SENSORS] erro requisitando trig/echo (v2): %s\n",
                strerror(errno));
        goto fail;
    }
#endif
#else
    printf("[SENSORS] libgpiod indisponivel; HC-SR04 em simulacao\n");
#endif

    if (spi_init() != 0) {
        fprintf(stderr, "[SENSORS] SPI indisponivel; LM35 em simulacao\n");
    }

    return 0;

#if ATUADORES_HAS_GPIOD
fail:
    sensors_hw_close();
fail_no_chip:
    return -1;
#endif
}

void sensors_hw_close(void)
{
    if (spi_fd >= 0) {
        close(spi_fd);
        spi_fd = -1;
    }

#if ATUADORES_HAS_GPIOD
#if ATUADORES_GPIOD_V1
    if (trig_line != NULL) {
        if (trig_requested) {
            gpiod_line_set_value(trig_line, 0);
            gpiod_line_release(trig_line);
        }
        trig_line = NULL;
        trig_requested = 0;
    }
    if (echo_line != NULL) {
        if (echo_requested) {
            gpiod_line_release(echo_line);
        }
        echo_line = NULL;
        echo_requested = 0;
    }
    if (sensors_chip != NULL) {
        gpiod_chip_close(sensors_chip);
        sensors_chip = NULL;
    }
#else
    if (sensors_request != NULL) {
        gpiod_line_request_release(sensors_request);
        sensors_request = NULL;
    }
    if (sensors_chip != NULL) {
        gpiod_chip_close(sensors_chip);
        sensors_chip = NULL;
    }
#endif
#endif
}

void *thread_sensors(void *arg)
{
    atuadores_context_t *ctx = (atuadores_context_t *)arg;
    struct timespec period = {
        .tv_sec = SENSORS_PERIOD_MS / 1000,
        .tv_nsec = (long)(SENSORS_PERIOD_MS % 1000) * 1000000L,
    };

    if (ctx == NULL) {
        return NULL;
    }

    if (sensors_hw_init() != 0) {
        fprintf(stderr, "[SENSORS] falha na inicializacao do hardware\n");
    }

    printf("[SENSORS] thread iniciada (periodo %dms)\n", SENSORS_PERIOD_MS);

    while (ctx->running) {
        float temperature_c = 0.0f;
        float distance_cm = 0.0f;
        int temperature_ok = (read_temperature(&temperature_c) == 0);
        int distance_ok = (read_distance(&distance_cm) == 0);

        pthread_mutex_lock(&ctx->sensors_mutex);
        if (temperature_ok) {
            ctx->sensors.temperature_c = temperature_c;
            ctx->sensors.temperature_valid = 1;
        }
        if (distance_ok) {
            ctx->sensors.distance_cm = distance_cm;
            ctx->sensors.distance_valid = 1;
        }
        pthread_mutex_unlock(&ctx->sensors_mutex);

        nanosleep(&period, NULL);
    }

    sensors_hw_close();
    printf("[SENSORS] thread encerrada\n");
    return NULL;
}
