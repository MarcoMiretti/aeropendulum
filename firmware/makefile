#-/boilerplate/----------------------------------------------------------------/

PROJ_NAME 	 = stm32aero

BUILD 		?= debug
build_dir 	:= ${BUILD}
libs_dir 	:= libs
cmsis_dir 	:= $(libs_dir)/cmsis
sources_dir 	:= sources
src_dir 	:= $(sources_dir)/src
inc_dir 	:= $(sources_dir)/inc
out_dir 	:= $(build_dir)/out

TARGET 		 = $(build_dir)/$(PROJ_NAME).elf
TARGET_BIN	 = $(build_dir)/$(PROJ_NAME).bin

LIBS 		 = $(sort $(dir $(wildcard libs/*/)))
MIDDLEWARES	 = $(sort $(dir $(wildcard middlewares/*/))) 

ifeq ($(USE_DSP),y)
DSPLIBS		 = $(sort $(dir $(wildcard $(cmsis_dir)/DSP/Source/*/)))
endif

FREERTOS	 = middlewares/FreeRTOS/Source
HEAP		 = heap_4.c

#--Main source files--
SRC 		 = $(wildcard $(src_dir)/*.c)
CXXSRC 		 = $(wildcard $(src_dir)/*.cpp)
ASRC 		 = $(wildcard $(src_dir)/*.s)

#--Add libs source files--
SRC 		+= $(foreach l, $(LIBS), $(wildcard $(l)/src/*.c))
CXXSRC 		+= $(foreach l, $(LIBS), $(wildcard $(l)/src/*.cpp))
ASRC 		+= $(foreach l, $(LIBS), $(wildcard $(l)/src/*.s))

#--Add DSP libs source files
ifeq ($(USE_DSP),y)
SRC             += $(foreach l, $(DSPLIBS), $(wildcard $(l)/*.c))
endif

#--Add middlewares source files--
SRC 		+= $(foreach l, $(MIDDLEWARES), $(wildcard $(l)/src/*.c))
##--FreeRTOS--
SRC		+= $(wildcard $(FREERTOS)/*.c)
SRC		+= $(FREERTOS)/portable/MemMang/$(HEAP)
SRC		+= $(FREERTOS)/portable/GCC/ARM_CM4F/port.c

RTOSFLAGS	 = -I$(FREERTOS)/include
RTOSFLAGS	+= -I$(FREERTOS)/portable/GCC/ARM_CM4F

#--Specify objs and deps--
OBJECTS		 = $(CXXSRC:%.cpp=$(out_dir)/%.o) $(ASRC:%.s=$(out_dir)/%.o) $(SRC:%.c=$(out_dir)/%.o)
DEPS		 = $(OBJ:%.o=%.d)

#--Add compiled libs--
ifeq ($(USE_DSP),y)
LIBSDEPS	 = $(wildcard $(cmsis_dir)/lib/*.a)
endif

#-/options/--------------------------------------------------------------------/

USE_NANO 	?= n
USE_SEMIHOST 	?= y 
VERBOSE 	?= n

#-/end-options/----------------------------------------------------------------/


SHELL 		:= /bin/bash

FLASH_TOOL 	:= st-flash
TOOLCHAIN	:= arm-none-eabi

AS 		 = $(TOOLCHAIN)-as
CC 		 = $(TOOLCHAIN)-gcc
CXX 		 = $(TOOLCHAIN)-g++
OBJCOPY 	 = $(TOOLCHAIN)-objcopy
SIZE 		 = $(TOOLCHAIN)-size
RM 		 = rm -f
RMD 		 = rm -rf
ifeq ($(CXXSRC),)
LD=$(TOOLCHAIN)-gcc
else
LD=$(TOOLCHAIN)-g++
endif
OOCD 		 = openocd

LDSCRIPT	 = $(cmsis_dir)/loader/stm32_flash.ld
OOCD_SCRIPT 	 = scripts/openocd/stm32f4discovery.cfg

INCLUDE_FLAGS 	 = $(foreach l, $(LIBS), -I$(l)inc) -I$(inc_dir) $(RTOSFLAGS) $(foreach l, $(MIDDLEWARES), -I$(l)inc) 
ifeq ($(USE_DSP),y)
INCLUDE_FLAGS	+= -I$(cmsis_dir)/DSP/Include
endif
TARGET_FLAGS 	 = -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork \
		   -mfloat-abi=hard -mfpu=fpv4-sp-d16

COMMON_FLAGS 	 = $(TARGET_FLAGS) $(INCLUDE_FLAGS)

CFLAGS 		 = $(COMMON_FLAGS)
CXXFLAGS 	 = $(COMMON_FLAGS)

ifeq ($(BUILD),debug)
CFLAGS 		+= -O0 -g -Wall
else
CFLAGS 		+= -O2 -s -DNDEBUG
endif

ifeq ($(BUILD),debug)
CXXFLAGS	+= -O0 -g -Wall
else
CXXFLAGS 	+= -O2 -s -DNDEBUG
endif

LDFLAGS 	 = $(TARGET_FLAGS)
LDFLAGS 	+= -T$(LDSCRIPT)
LDFLAGS		+= -specs=nosys.specs

ifeq ($(USE_NANO),y)
LDFLAGS 	+= -specs=nano.specs
endif

LDFLAGS 	+= -L$(cmsis_dir)/lib

ifeq ($(SEMIHOST),y)
TARGET_FLAGS 	+= USE_SEMIHOST
LDFLAGS 	+= -specs=rdimon.specs -lc -lrdimon
endif

ifeq ($(USE_DSP),y)
LDFLAGS		+= -larm_cortexM4lf_math
endif

ifeq ($(VERBOSE),y)
Q=
else
Q=@
endif

#-/boilerplate-end/------------------------------------------------------------/

all: $(TARGET) $(TARGET_BIN) size
	@echo

-include $(DEPS)
#.PHONY: debug
#debug:
#	make "BUILD=debug"
#.PHONY: release
#release:
#	make "BUILD=release"

$(out_dir)/%.o: %.c
	@echo CC $(notdir $<)
	@mkdir -p $(dir $@)
	$(Q)$(CC) -MMD $(CFLAGS) -c -o $@ $<

$(out_dir)/%.o: %.cpp
	@echo CXX $(notdir $<)
	@mkdir -p $(dir $@)
	$(Q)$(CXX) -MMD $(CXXFLAGS) -c -o $@ $<

$(out_dir)/%.o: %.s
	@echo AS $(notdir $<)
	@mkdir -p $(dir $@)
	$(Q)$(CC) -MMD $(CFLAGS) -c -o $@ $<

$(out_dir)/linker-params: $(OBJECTS) $(LIBSDEPS) makefile
	@echo LD params
	@mkdir -p $(dir $@)
	$(Q)echo "-Wl,-( $(OBJECTS) -Wl,-) $(LDFLAGS)" > $@

$(TARGET): $(out_dir)/linker-params
	@echo LD $@...
	$(Q)$(LD) -o $@ @$(out_dir)/linker-params

$(TARGET_BIN): $(TARGET)
	@echo COPY $(notdir $<) TO $(notdir $@)
	@mkdir -p $(dir $@)
	$(Q)$(OBJCOPY) -O binary $< $@

size: $(TARGET)
	$(Q)$(SIZE) $<

.PHONY: clean

clean:
	$(RMD) $(build_dir)

.PHONY: flash
	
download: $(TARGET)
	@echo DOWNLOAD to TARGET
	$(Q)$(OOCD) -f $(OOCD_SCRIPT) \
		-c "init" \
		-c "halt 0" \
		-c "flash write_image erase unlock $<" \
		-c "reset run" \
		-c "shutdown" 2>&1
		
server_open:
	@echo DOWNLOAD to FLASH
	$(Q)$(OOCD) -f $(OOCD_SCRIPT) \
		-c "init" \
		-c "halt 0"

server_connect: $(TARGET)
	@echo CONNECTING to DEBUG SESSION and DOWNLOADING TARGET	
	$(Q)arm-none-eabi-gdb $< -x scripts/openocd/gdbinit

documentation:
	@echo CALL DOXYGEN
	$(Q) cd ../docs && \
		rm -rf latex &&\
		rm -rf html && \
		doxygen stm32aero.config && \
	      	cd latex && \
		make && \
		cp refman.pdf ../

