CXX=g++
CC=gcc
CFLAGS=-g -c
LDFLAGS=
AR=ar

all: icsscand 

icsscand: icsscand.o
	$(CC) $(LDFLAGS) icsscand.o -o icsscand -lpthread -lftdi -licsneoapi

icsscand.o: icsscand.c
	$(CC) $(CFLAGS) icsscand.c

clean:
	rm -rf *.o icsscand
