CC  = gcc
CXX = g++

# Let’s leave a place holder for additional include directories

INCLUDES = 

# Compilation options:
# -g for debugging info and -Wall enables all warnings

CFLAGS   = -g -Wall $(INCLUDES)
CXXFLAGS = -g -Wall $(INCLUDES)

# Linking options:
# -g for debugging info

LDFLAGS = -g 

# List the libraries you need to link with in LDLIBS
# For example, use "-lm" for the math library

LDLIBS = 

main: main.o tinyspline.o crc.o

main.o: main.c tinyspline.h CPFrames.h crc.h

tinyspline.o: tinyspline.c

crc.o: crc.c crc.h

RMC_communication_daemon: RMC_communication_daemon.o crc.o

RMC_communication_daemon.o: RMC_communication_daemon.c CPFrames.h crc.h


.PHONY: clean
clean:
	rm -f *.o a.out core main RMC_communication_daemon

.PHONY: all
all: clean main