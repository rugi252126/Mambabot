
# Tools paths
TOOLCHAIN_ROOT := /usr
TOOLCHAIN_PATH := $(TOOLCHAIN_ROOT)/bin
TOOLCHAIN_PREFIX := arm-none-eabi

# Optimization level, can be [0, 1, 2, 3, s].
OPTLVL:=0
DBG:=-g

FREERTOS:=$(MODULE_DIR)/third-party/FreeRTOS
SEGGER:=$(MODULE_DIR)/third-party/SEGGER
STARTUP:=$(MODULE_DIR)/startup


include $(ENV_DIR)/build/project.mak

# include files location
INCLUDE=-I$(MODULE_DIR)/core
INCLUDE+=-I$(FREERTOS)/org/Source/include
INCLUDE+=-I$(FREERTOS)/org/Source/portable/GCC/ARM_CM4F
INCLUDE+=-I$(SEGGER)/Config
INCLUDE+=-I$(SEGGER)/OS
INCLUDE+=-I$(SEGGER)/SEGGER
INCLUDE+=-I$(MODULE_DIR)/libraries/CMSIS/device
INCLUDE+=-I$(MODULE_DIR)/libraries/CMSIS/core
INCLUDE+=-I$(MODULE_DIR)/libraries/StdPeriph_Driver/inc
INCLUDE+=-I$(MODULE_DIR)/config

# vpath is used so object files are written to the current directory instead
# of the same directory as their source files
vpath %.c $(MODULE_DIR)/appl/src \
          $(MODULE_DIR)/libraries/StdPeriph_Driver/src \
          $(MODULE_DIR)/core $(FREERTOS)/org/Source \
          $(FREERTOS)/org/Source/portable/MemMang \
          $(FREERTOS)/org/Source/portable/GCC/ARM_CM4F \
          $(SEGGER)/Config $(SEGGER)/OS $(SEGGER)/SEGGER

vpath %.s $(STARTUP)
#ASRC=startup_stm32f4xx.s
ASRC=startup_stm32.s

# Project Source Files
include $(MODULE_DIR)/appl/build/module.mak
include $(MODULE_DIR)/core/build/module.mak
include $(FREERTOS)/org/Source/build/module.mak
include $(MODULE_DIR)/libraries/StdPeriph_Driver/build/module.mak

ifeq ($(SYSTEM_VIEW_TOOLS), true)
	# Add the SEGGER source files in the compilation
	include $(SEGGER)/build/module.mak
endif

# Preprocessor definitions
CDEFS=-DUSE_STDPERIPH_DRIVER
CDEFS+=-DSTM32F429_439xx
CDEFS+=-DSTM32
CDEFS+=-DSTM32F4
CDEFS+=-DSTM32F429ZITx
CDEFS+=-DNUCLEO_F429ZI
CDEFS+=-DDEBUG
CDEFS+=-DHSE_VALUE=8000000
#CDEFS+=-D__FPU_PRESENT=1
#CDEFS+=-D__FPU_USED=1
#CDEFS+=-DARM_MATH_CM4

MCUFLAGS=-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -finline-functions -Wdouble-promotion -std=gnu99
COMMONFLAGS=-O$(OPTLVL) $(DBG) -Wall -ffunction-sections -fdata-sections
CFLAGS=$(COMMONFLAGS) $(MCUFLAGS) $(INCLUDE) $(CDEFS)

LDLIBS=-lm -lc -lgcc
LDFLAGS=$(MCUFLAGS) -u _scanf_float -u _printf_float -fno-exceptions -Wl,--gc-sections,-T$(LINKER_FILE),-Map,$(BIN_DIR)/$(TARGET_NAME).map

CC=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-gcc
CXX=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-g++
LD=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-gcc
OBJCOPY=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-objcopy
AS=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-as
AR=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-ar
GDB=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-gdb


BUILD_DIR = $(ENV_DIR)/$(OUTPUT_BIN)/obj
BIN_DIR = $(ENV_DIR)/$(OUTPUT_BIN)/out

OBJ = $(SRC:%.c=$(BUILD_DIR)/%.o)

$(BUILD_DIR)/%.o: %.c
	@echo [CC] $(notdir $<)
	@$(CC) $(CFLAGS) $< -c -o $@

all: $(OBJ)
	@echo [AS] $(ASRC)
	@$(AS) -o $(ASRC:%.s=$(BUILD_DIR)/%.o) $(STARTUP)/$(ASRC)
	@echo [LD] $(TARGET_NAME).elf
	@$(CC) -o $(BIN_DIR)/$(TARGET_NAME).elf $(LDFLAGS) $(OBJ) $(ASRC:%.s=$(BUILD_DIR)/%.o) $(LDLIBS)
	@echo [HEX] $(TARGET_NAME).hex
	@$(OBJCOPY) -O ihex $(BIN_DIR)/$(TARGET_NAME).elf $(BIN_DIR)/$(TARGET_NAME).hex
	@echo [BIN] $(TARGET_NAME).bin
	@$(OBJCOPY) -O binary $(BIN_DIR)/$(TARGET_NAME).elf $(BIN_DIR)/$(TARGET_NAME).bin

.PHONY: clean

clean:
	@echo [RM] OBJ
	@rm -f $(OBJ)
	@rm -f $(ASRC:%.s=$(BUILD_DIR)/%.o)
	@echo [RM] BIN
	@rm -f $(BIN_DIR)/$(TARGET_NAME).elf
	@rm -f $(BIN_DIR)/$(TARGET_NAME).hex
	@rm -f $(BIN_DIR)/$(TARGET_NAME).bin
	@rm -f $(BIN_DIR)/$(TARGET_NAME).map

flash:
	@st-flash write $(BIN_DIR)/$(TARGET_NAME).bin 0x8000000

