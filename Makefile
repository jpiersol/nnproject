#Makefile for Flatworld II V1.0 tpc 17 March 2009
CC = gcc
FILES = FWmain.o
#for Linux
#IPATH= -I/usr/X11R6/include 
#LPATH= -L/usr/X11R6/lib

OPSYS = $(shell uname)

COPTS = -O2
LDLIBS = -lglut -lGL -lm

ifeq ($(OPSYS), Darwin)
	IPATH += -I/sw/include
	LPATH += -L/sw/lib
	LDLIBS = -framework GLUT -framework OpenGL -framework Cocoa
#COPTS += -flat_namespace -undefined suppress
endif


FWmain: $(FILES)
	 $(CC) -g -Wall $(COPTS) $(FILES) $(IPATH) $(LPATH) $(LDLIBS) -o FWmain

FWmain.o: FWmain.c 

	$(CC) -c -Wall $(COPTS) $(IPATH) FWmain.c
clean:
	-rm -f *.o FWmain
