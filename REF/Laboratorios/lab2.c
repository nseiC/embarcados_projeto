/*
*   Arquivo: fsm2.c 
*   Objetivo: criar a lógica especializada de um sistema de alarme usando libgpiod 
*   Compilacao gcc -Wall fsm2.c -lgpiod
*   Execução sudo ./fsm
*/

#include <gpiod.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <signal.h> // Para a lógica de parar o código e liberar o GPIO
#include <fcntl.h>  
#include <string.h>  

#define CHIP "/dev/gpiochip0"

// Mapeamento GPIO (Índices do vetor de saída: 0=R, 1=Y, 2=G)
#define LED_R 17
#define LED_Y 27
#define LED_G 22
#define BTN   5

typedef enum {
    DESARMADO,
    ARMANDO,
    ARMADO,
    DISPARADO
} state_t;

state_t state = DESARMADO;

struct gpiod_chip *chip;
struct gpiod_line_request *req_out;
struct gpiod_line_request *req_in;

// Função para traduzir estados para strings no log
const char* get_state_name(state_t s) {
    switch(s) {
        case DESARMADO:   return "DESARMADO";
        case ARMANDO:     return "ARMANDO";
        case ARMADO:      return "ARMADO";
        case DISPARADO:   return "DISPARADO";
        default:          return "DESCONHECIDO";
    }
}

// (Regra H) Gravação em arquivo usando open/write/close estritamente POSIX
void log_transition(int timestamp, state_t old_st, state_t new_st, const char* event) {
    // O_WRONLY = Escrita, O_CREAT = Cria se não existir, O_APPEND = Adiciona ao final
    // 0644 são as permissões de arquivo (Leitura/Escrita para o dono)
    int fd = open("log_fsm.txt", O_WRONLY | O_CREAT | O_APPEND, 0644);
    if (fd != -1) {
        char buffer[256];
        int len = snprintf(buffer, sizeof(buffer), "[%ds] %s -> %s | Condicao: %s\n", 
                           timestamp, get_state_name(old_st), get_state_name(new_st), event);
        write(fd, buffer, len);
        close(fd);
    } else {
        perror("Erro ao abrir log_fsm.txt");
    }
}

// Encerramento seguro em caso de interrupção
void handle_sigint(int sig) {
    printf("\nInterrupcao detectada! Desligando LEDs e saindo...\n");
    if (req_out) {
        int all_off[3] = {0, 0, 0};
        gpiod_line_request_set_values(req_out, all_off);
        gpiod_line_request_release(req_out);
    }
    if (req_in) gpiod_line_request_release(req_in);
    if (chip) gpiod_chip_close(chip);
    exit(0);
}

int main() {
    signal(SIGINT, handle_sigint);

    unsigned int out_offsets[] = {LED_R, LED_Y, LED_G};
    unsigned int in_offsets[] = {BTN};

    chip = gpiod_chip_open(CHIP);
    if(!chip){
        perror("Erro ao abrir chip");
        return 1;
    }

    // ---------- CONFIG SAÍDAS ----------
    // USADA A REFERÊNCIA DE IA 

    struct gpiod_line_settings *out_settings = gpiod_line_settings_new();
    gpiod_line_settings_set_direction(out_settings, GPIOD_LINE_DIRECTION_OUTPUT);
    struct gpiod_line_config *out_config = gpiod_line_config_new();
    gpiod_line_config_add_line_settings(out_config, out_offsets, 3, out_settings);

    req_out = gpiod_chip_request_lines(chip, NULL, out_config);
    if(!req_out){
        perror("Erro ao requisitar saidas");
        return 1;
    }

    // ---------- CONFIG ENTRADA ----------
    // USADA A REFERÊNCIA DE IA 
    struct gpiod_line_settings *in_settings = gpiod_line_settings_new();
    gpiod_line_settings_set_direction(in_settings, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_edge_detection(in_settings, GPIOD_LINE_EDGE_BOTH);
    struct gpiod_line_config *in_config = gpiod_line_config_new();
    gpiod_line_config_add_line_settings(in_config, in_offsets, 1, in_settings);

    req_in = gpiod_chip_request_lines(chip, NULL, in_config);
    if(!req_in){
        perror("Erro ao requisitar entrada");
        return 1;
    }

    // Base de Tempo: 10ms (10.000.000 nanosegundos)
    struct timespec ts = {0, 10000000L}; 

    // Variáveis de controle de Tempo
    int tick_count = 0;      // Contador principal do laço (incrementa a cada 10ms)
    int total_seconds = 0;   // Segundos reais desde o início
    int state_timer = 0;     // Tempo em que o sistema está no estado atual (em ticks de 10ms)

    // Variáveis de controle do Botão
    int btn_stable_state = 1; // Estado estável após debounce (1 = solto, lógica invertida)
    int debounce_timer = 0;
    int pressed_time = 0;     // Quanto tempo (em ticks) o botão está sendo mantido pressionado
    int short_press_event = 0;// Flag acionada no momento em que um toque curto é finalizado

    // Início do Log
    int fd_init = open("log_fsm.txt", O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if(fd_init != -1) {
        const char* msg = "--- INICIO DA EXECUCAO ---\n";
        write(fd_init, msg, strlen(msg));
        close(fd_init);
    }

    while(1) {
        // ====================================================================
        // 1. LEITURA E DEBOUNCE DO BOTÃO
        // ====================================================================
        int raw_btn_val;
        gpiod_line_request_get_values(req_in, &raw_btn_val);

        // Tratamento de Debounce (30ms = 3 ciclos de 10ms)
        if (raw_btn_val != btn_stable_state) {
            debounce_timer++;
            if (debounce_timer >= 3) { 
                btn_stable_state = raw_btn_val;
                debounce_timer = 0;
            }
        } else {
            debounce_timer = 0;
        }

        // Lógica de Duração do Pressionamento
        short_press_event = 0; // Limpa o evento de toque curto no início do ciclo

        if (btn_stable_state == 0) { 
            // Se fisicamente pressionado
            pressed_time++;
        } else { 
            // Se foi solto neste momento e o tempo acumulado for menor que 500ms (50 ticks)
            if (pressed_time > 0 && pressed_time < 50) {
                short_press_event = 1;
            }
            pressed_time = 0; // Zera a duração pois foi solto
        }

        // ====================================================================
        // 2. MÁQUINA DE ESTADOS: TRANSIÇÕES LÓGICAS
        // ====================================================================
        state_t next_state = state;
        const char* event_trigger = "";

        switch(state) {
            // Transição de desarmado para ARMANDO ao pressionar botão por 2s
            case DESARMADO:
                if (pressed_time >= 200) { 
                    next_state = ARMANDO;
                    event_trigger = "botao : >= 2s continuo";
                }
                break;
            // Aguarda 5s antes de migrar para o status de alarme armado
            case ARMANDO:
                if (state_timer >= 500) { 
                    next_state = ARMADO;
                    event_trigger = "tempo : 5s decorrido";
                }
                break;
            // Mantém o sistema em operação, e é ativado caso haja um toque curto do botão.
            case ARMADO:
                if (short_press_event) { // Regra D: Toque curto concluído (< 0.5s)
                    next_state = DISPARADO;
                    event_trigger = "botao : < 0.5s (toque curto)";
                }
                break;
            // Mantém o alarme disparado por 10s ou até o botão ser pressionado continuamente por 3.
            case DISPARADO:
                if (pressed_time >= 300) { 
                    pressed_time = 0;
                    next_state = DESARMADO;
                    event_trigger = "botao : >= 3s continuo";
                } else if (state_timer >= 1000) {
                    next_state = DESARMADO;
                    event_trigger = "tempo : 10s alarme finalizado";
                }
                break;
        }

        // Se houve mudança de estado, executa ações de transição
        if (state != next_state) {
            log_transition(total_seconds, state, next_state, event_trigger);
            state = next_state;
            state_timer = 0; // Zera o temporizador para o novo estado
        }

        // ====================================================================
        // 3. MÁQUINA DE ESTADOS: SAÍDAS E AÇÕES (Estilo Moore)
        // ====================================================================
        // USADA A REFERÊNCIA DE IA 
        int out[3] = {0, 0, 0}; // Vetor de Saída: [Vermelho, Amarelo, Verde]

        switch(state) {
            case DESARMADO:
                // Tudo apagado.
                break;

            case ARMANDO:
                // Pisca Amarelo. Exemplo: 500ms aceso, 500ms apagado (período de 1s = 100 ticks)
                if ((state_timer % 100) < 50) {
                    out[1] = 1;
                }
                break;

            case ARMADO:
                out[2] = 1; // LED Verde permanentemente aceso
                break;

            case DISPARADO:
                // Regra E: Vermelho pisca com período de 200ms (100ms aceso / 100ms apagado)
                // 200ms = 20 ticks. Os primeiros 10 acendem, os últimos 10 apagam.
                if ((state_timer % 20) < 10) {
                    out[0] = 1;
                }
                // Emite ALERTA! no terminal a cada 0.5s (50 ticks)
                if (state_timer % 50 == 0 && state_timer > 0) {
                    printf("[%ds] ALERTA!\n", total_seconds);
                }
                break;
        }

        // Atualiza fisicamente as saídas do hardware
        gpiod_line_request_set_values(req_out, out);

        // ====================================================================
        // 4. ATUALIZAÇÃO DOS CONTADORES E DELAY
        // ====================================================================
        tick_count++;
        state_timer++;
        
        // A cada 100 ciclos de 10ms, passou-se 1 segundo inteiro
        if (tick_count % 100 == 0) {
            total_seconds++;
        }

        nanosleep(&ts, NULL); // Dorme por 10ms
    }

    return 0;
}