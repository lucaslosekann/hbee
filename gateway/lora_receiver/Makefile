CC = gcc
CFLAGS = -I../libloragw/inc -Wall
LDFLAGS = -L../libloragw -lloragw -L../libtools -ltinymt32 -lmosquitto -lm -lrt
TARGET = lora_receiver

all: $(TARGET)

$(TARGET): main.o
	$(CC) main.o $(LDFLAGS) -o $(TARGET)

main.o: main.c
	$(CC) $(CFLAGS) -c main.c

clean:
	rm -f *.o $(TARGET)
