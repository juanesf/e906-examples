# E906 "MPU6050 + LCD" example - Allwinner A523/T527 (Avaota-A1)
CROSS   = riscv64-unknown-elf-
CC      = $(CROSS)gcc
OBJCOPY = $(CROSS)objcopy
OBJDUMP = $(CROSS)objdump
READELF = $(CROSS)readelf
SIZE    = $(CROSS)size

ARCH    = -march=rv32imac_zicsr -mabi=ilp32

CFLAGS  = $(ARCH) -Os -ffreestanding -fno-builtin -nostdlib -nostartfiles \
          -fno-pic -fno-stack-protector -ffunction-sections -fdata-sections \
          -Wall -Wextra -g -I.
LDFLAGS = $(ARCH) -T linker.ld -nostdlib -nostartfiles -static \
          -Wl,--no-dynamic-linker -Wl,--gc-sections

TARGET  = e906-mpu6050.elf
OBJS    = crt0.o main.o

all: $(TARGET) e906-mpu6050.bin e906-mpu6050.lst

%.o: %.S
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET): $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) -o $@
	$(READELF) -h $@ | grep -E "Class|Entry point|Machine"
	$(SIZE) $@

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

%.lst: %.elf
	$(OBJDUMP) -d $< > $@

clean:
	rm -f *.o $(TARGET) *.bin *.lst

.PHONY: all clean
