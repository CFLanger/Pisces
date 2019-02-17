#
# Solid Makefile
#
CC = gcc
CFLAGS = -c -Wall  
LDFLAGS = -O3 -rdynamic -lrt -lm -ldl -lpthread -lgps -lwiringPi
INC = 
 
# List of sources:
SOURCES = pinav.c bmp180.c
OBJECTS = $(SOURCES:.c=.o)
 
# Name of executable target:
EXECUTABLE = pinav

CFLAGS += `pkg-config --cflags gtk+-3.0`
LDFLAGS += `pkg-config --libs gtk+-3.0`

#CFLAGS += `xml2-config --cflags`
#LDFLAGS += `xml2-config --libs`
  
all:	$(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LDFLAGS)
 
.c.o:
	$(CC) $(INC) $(CFLAGS) $< -o $@
 
clean:
	rm $(OBJECTS) $(EXECUTABLE)
