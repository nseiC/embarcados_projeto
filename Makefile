# Makefile - Projeto Final Sistemas Embarcados
#
# Uso:
#   make           compila o binario 'atuadores'
#   make clean     remove objetos e binario
#   make run       compila e executa com localhost:1883 grupo1
#   make run HOST=192.168.0.42 GRUPO=grupo1 PORTA=1883

CC      = gcc
CFLAGS  = -Wall -Wextra -O2 -pthread
LDFLAGS = -pthread
LIBS    = -lmosquitto -lcjson -lgpiod

SRC_DIR = SRC
SOURCES = $(SRC_DIR)/main.c $(SRC_DIR)/atuadores.c $(SRC_DIR)/mqtt.c
OBJECTS = $(SOURCES:.c=.o)
TARGET  = atuadores

HOST  ?= localhost
GRUPO ?= grupo1
PORTA ?= 1883

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -o $(TARGET) $(LDFLAGS) $(LIBS)

$(SRC_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(TARGET)

run: $(TARGET)
	./$(TARGET) $(HOST) $(GRUPO) $(PORTA)
