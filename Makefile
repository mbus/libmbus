CFLAGS = -Wall -Wextra -g

all:	libmbus.o

libmbus.o:	libmbus.c libmbus.h

clean:
	rm -f libmbus.o
