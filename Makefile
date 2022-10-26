CC = arm-none-eabi-gcc

CFLAGS = -g -Og -Wall
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4
CFLAGS += -fno-common -fno-builtin
CFLAGS += -Iinclude/

SRC = $(wildcard src/*.c)
OBJ = $(SRC:.c=.o)
BIN = include

-include ../Makefile.lib

all: $(OBJ) 
	ar rvs $(BIN)/stmf.a $^

%.o: %.c
	$(CC) -o $@ -c $< $(CFLAGS)

clean:	
	rm -f $(BIN)/stmf.a $(OBJ)  
