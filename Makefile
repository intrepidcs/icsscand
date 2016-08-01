CXX=g++
CC=gcc
CFLAGS=-g -c -O2
LDFLAGS=
AR=ar

all: icsscand

icsscand: icsscand.o
	$(CC) $(LDFLAGS) icsscand.o -o icsscand -lpthread -licsneoapi

icsscand.o: icsscand.c
	$(CC) $(CFLAGS) icsscand.c

clean:
	rm -rf *.o icsscand
