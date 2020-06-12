# dragino lora testing
# Single lora testing app

CC=g++
CFLAGS=-c -Wall
LIBS=-lwiringPi -lcurl

all: farms-mon-lora

farms-mon-lora: main.o
	$(CC) main.o $(LIBS) -o farms-mon-lora

main.o: main.c
	$(CC) $(CFLAGS) main.c

clean:
	rm *.o farms-mon-lora
