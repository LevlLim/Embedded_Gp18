# Host-side unit tests for the pure logic (Group AAI18).
#
# Runs on a PC with the normal gcc, NOT the ARM cross compiler.  The pure
# helpers in msg_defs.c, motor_control.c, barcode_decoder.c, imu_terrain.c and
# obstacle_scan.c are linked against host_stubs.c and the stub kernel
# headers in ../stub; no task or hardware path is executed.
#
#   make                 build and run every buddy's tests
#   make test1 .. test5  build and run ONE buddy's tests (plus the shared
#                        single-word checks every buddy depends on)
#   make target-sizes    sizeof() of the unpacked messages on the ARM target
#   make clean

CC      := gcc
CFLAGS  := -std=c11 -Wall -Wextra -Wconversion -Wsign-conversion \
           -Wshadow -Wpointer-arith -Wcast-qual -Wstrict-prototypes \
           -O1 -g -I../stub -I../../app_program

APP     := ../../app_program
SRCS    := test_host.c host_stubs.c $(APP)/msg_defs.c \
           $(APP)/subsystem2/motor_control.c \
           $(APP)/subsystem3/barcode_decoder.c $(APP)/subsystem3/barcode_mapping.c \
           $(APP)/subsystem4/imu_terrain.c \
           $(APP)/subsystem5/obstacle_scan.c
TARGET  := test_host

.PHONY: all run clean target-sizes test1 test2 test3 test4 test5

all: run

$(TARGET): $(SRCS) $(APP)/msg_defs.h
	$(CC) $(CFLAGS) -o $@ $(SRCS)

run: $(TARGET)
	./$(TARGET)

test1 test2 test3 test4 test5: $(TARGET)
	./$(TARGET) $(subst test,,$@)

target-sizes:
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -std=c11 -fcommon \
	    -I$(APP) -c sizes_probe.c -o sizes_probe.o
	@echo "sizeof() on the RP2040 target (bytes):"
	@arm-none-eabi-nm -S sizes_probe.o | while read addr size type name; \
	    do echo "  $$name $$((0x$$size))"; done
	@rm -f sizes_probe.o

clean:
	rm -f $(TARGET) sizes_probe.o
