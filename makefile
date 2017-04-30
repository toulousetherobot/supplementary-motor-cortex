CC  = gcc
CXX = g++

# Let’s leave a place holder for additional include directories

INCLUDES = 

# Compilation options:
# -g for debugging info and -Wall enables all warnings

CFLAGS   = -g -Wall $(INCLUDES) $(shell pkg-config --cflags json-c) $(shell pkg-config --cflags librabbitmq)
CXXFLAGS = -g -Wall $(INCLUDES)

# Linking options:
# -g for debugging info

LDFLAGS = -g $(shell pkg-config --libs json-c) $(shell pkg-config --libs librabbitmq)

# List the libraries you need to link with in LDLIBS
# For example, use "-lm" for the math library

LDLIBS = -lm

.PHONY: principal
principal: main RMC_communication_daemon send_RMC

main: main.o tinyspline.o crc.o

main.o: main.c tinyspline.h CPFrames.h crc.h

send_RMC: send_RMC.o crc.o

send_RMC.o: send_RMC.c CPFrames.h crc.h

tinyspline.o: tinyspline.c

crc.o: crc.c crc.h

os_communication.o: os_communication.c CPFrames.h

RMC_communication_daemon: RMC_communication_daemon.o crc.o os_communication.o

RMC_communication_daemon.o: RMC_communication_daemon.c CPFrames.h crc.h os_communication.h

.PHONY: clean
clean:
	rm -f *.o a.out core main RMC_communication_daemon send_RMC

.PHONY: all
all: clean principal