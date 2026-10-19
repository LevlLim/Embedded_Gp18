# Host build of the Buddy 4 packing tests. No Pico SDK needed.
#   make        build and run
#   make clean

CC      ?= gcc
CFLAGS  := -std=c99 -Wall -Wextra -O1 -DHOST_TEST -I../include
SRC     := test_imu_pack.c ../src/imu_terrain.c ../src/fault_flags.c
BIN     := test_imu_pack

all: run

$(BIN): $(SRC)
	$(CC) $(CFLAGS) $(SRC) -o $(BIN)

run: $(BIN)
	./$(BIN)

clean:
	rm -f $(BIN)

.PHONY: all run clean
