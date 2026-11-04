# Firmware build for the real mouse (STM32F411 Black Pill), plain arm-none-eabi-gcc.
#
#   make                                   build build/mouse.elf and mouse.bin
#   make CMSIS_CORE=... CMSIS_DEVICE_F4=...   if the two CMSIS folders are elsewhere
#   make flash                             flash with an ST-Link (needs `st-flash`)
#
# Needs: gcc-arm-none-eabi, and two public ST repositories (CMSIS headers):
#   git clone https://github.com/STMicroelectronics/cmsis_core
#   git clone https://github.com/STMicroelectronics/cmsis_device_f4
#
# The pin warning from hal_stm32/pins.h stays until PINS_CONFIRMED is defined
# (run: make PINS=confirmed) once the proposed pins have been checked.

CMSIS_CORE ?= $(HOME)/cmsis/cmsis_core
CMSIS_DEVICE_F4 ?= $(HOME)/cmsis/cmsis_device_f4

CC := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE := arm-none-eabi-size
BUILD := build

include solver_flags.mk

CPU := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
INCLUDES := -Iinclude -Ihal_stm32 -I../MMS_Code \
  -I$(CMSIS_CORE)/CMSIS/Core/Include -I$(CMSIS_DEVICE_F4)/Include
DEFINES := -DSTM32F411xE -DHSE_VALUE=25000000
ifeq ($(PINS),confirmed)
DEFINES += -DPINS_CONFIRMED
endif
CFLAGS := $(CPU) -std=gnu99 -Os -g -Wall -Wextra -ffunction-sections -fdata-sections \
  $(INCLUDES) $(DEFINES)
LDFLAGS := $(CPU) -T hal_stm32/stm32f411ce.ld -Wl,--gc-sections -Wl,-Map=$(BUILD)/mouse.map \
  --specs=nano.specs --specs=nosys.specs

SRC := src/main_hw.c src/api_hw.c src/motion.c src/ir_sensors.c \
  hal_stm32/hal_stm32.c hal_stm32/hal_logic.c hal_stm32/syscalls.c
SYSTEM := $(CMSIS_DEVICE_F4)/Source/Templates/system_stm32f4xx.c
STARTUP := $(CMSIS_DEVICE_F4)/Source/Templates/gcc/startup_stm32f411xe.s

OBJ := $(addprefix $(BUILD)/,$(notdir $(SRC:.c=.o))) $(BUILD)/solver.o $(BUILD)/system.o $(BUILD)/startup.o

all: $(BUILD)/mouse.elf $(BUILD)/mouse.bin
	@$(SIZE) $(BUILD)/mouse.elf

$(BUILD):
	@mkdir -p $(BUILD)

# The solver is the same file the simulator uses. LOG goes to hal_log (the serial port).
$(BUILD)/solver.o: ../MMS_Code/solver.c | $(BUILD)
	$(CC) $(CFLAGS) $(SOLVER_FLAGS) -include include/hal.h '-DLOG(...)=hal_log(__VA_ARGS__)' -c $< -o $@

$(BUILD)/system.o: $(SYSTEM) | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/startup.o: $(STARTUP) | $(BUILD)
	$(CC) $(CPU) -x assembler-with-cpp -c $< -o $@

$(BUILD)/%.o: src/%.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: hal_stm32/%.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/mouse.elf: $(OBJ)
	$(CC) $(LDFLAGS) $(OBJ) -lm -o $@

$(BUILD)/mouse.bin: $(BUILD)/mouse.elf
	$(OBJCOPY) -O binary $< $@

flash: $(BUILD)/mouse.bin
	st-flash write $< 0x08000000

clean:
	rm -rf $(BUILD)

.PHONY: all flash clean
