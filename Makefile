CFLAGS = -Wall -Wextra -g

all:	bitbang.o

bitbang.o:	bitbang.c bitbang.h

clean:
	rm -f bitbang.o
