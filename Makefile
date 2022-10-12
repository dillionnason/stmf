CC = arm-none-eabi-gcc

CFLAGS = -g -O2 -Wall
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4
CFLAGS += -fno-common -fno-builtin

SRC = $(wildcard src/*.c)
OBJ = $(SRC:.c=.o)
BIN = include

-include ../Makefile.lib

all: $(OBJ) 
	ar rvs $(BIN)/stmf.a $(OBJ)

%.o: %.c
	$(CC) -o $@ -c $< $(CFLAGS)

clean:	
	rm -f $(BIN)/stmf.a $(OBJ)  
