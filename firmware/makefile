###################################################

PROJ_NAME=stm32aero

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

LOADER=libs/cmsis/loader/stm32_flash.ld

CFLAGS  = -g -O2 -Wall -T$(LOADER)
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16

###################################################

vpath %.c src

ROOT=$(shell pwd)

SRCS = sources/src/main.c \
libs/cmsis/src/system_stm32f4xx.c \
libs/cmsis/src/startup_stm32f407xx.s

OBJS = $(SRCS:.c=.o)

###################################################

.PHONY: proj

all: proj

proj: 	$(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ -I"sources/inc" -I"libs/cmsis/inc"
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin
	
.PHONY: clean

clean:
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin

.PHONY: flash

flash:
	st-flash write $(PROJ_NAME).bin 0x8000000
