CC ?= gcc
CFLAGS ?= -Wall -Wextra -std=c11 -D_DEFAULT_SOURCE
LDLIBS ?= -lmosquitto -lcjson -lgpiod -pthread

TARGET = embarcados
OBJS = main.o atuadores.o mqtt.o

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS) $(LDLIBS)

%.o: %.c
	$(CC) $(CFLAGS) -c $<

clean:
	rm -f $(TARGET) $(OBJS)
