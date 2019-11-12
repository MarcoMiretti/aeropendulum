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

#--Main source files--
SRC 		 = $(wildcard $(src_dir)/*.c)
CXXSRC 		 = $(wildcard $(src_dir)/*.cpp)
ASRC 		 = $(wildcard $(src_dir)/*.s)

#--Add libs source files--
SRC 		+= $(foreach l, $(LIBS), $(wildcard $(l)/src/*.c))
CXXSRC 		+= $(foreach l, $(LIBS), $(wildcard $(l)/src/*.cpp))
ASRC 		+= $(foreach l, $(LIBS), $(wildcard $(l)/src/*.s))


#--Specify objs and deps--
OBJECTS		 = $(CXXSRC:%.cpp=$(out_dir)/%.o) $(ASRC:%.s=$(out_dir)/%.o) $(SRC:%.c=$(out_dir)/%.o)
DEPS		 = $(OBJ:%.o=%.d)

#-/options/--------------------------------------------------------------------/

USE_NANO 	 = n
USE_SEMIHOST 	 = y
VERBOSE 	 = n

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

LDSCRIPT	 = $(cmsis_dir)/loader/stm32_flash.ld

INCLUDE_FLAGS 	 = $(foreach l, $(LIBS), -I$(l)inc) -I$(inc_dir)
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

LDFLAGS 	 = $(TARGET_FLAGS)
LDFLAGS 	+= -T$(LDSCRIPT)

ifeq ($(USE_NANO),y)
LDFLAGS 	+= --specs=nano.specs
endif

ifeq ($(SEMIHOST),y)
TARGET_FLAGS 	+= USE_SEMIHOST
LDFLAGS 	+= --specs=rdimon.specs
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

$(out_dir)/linker-params: $(OBJECTS) $(DEPS) makefile
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

download:
	$(FLASH_TOOL) write $(TARGET_BIN) 0x8000000
