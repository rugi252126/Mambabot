
# Default buildset
BUILDSET := bs_4wd

VARIANT := $(BUILDSET)
# Default system view tools status
SYSTEM_VIEW_TOOLS := true

# Name of output binary(.elf, .hex, etc...)
OUTPUT_BIN := $(VARIANT)

ifeq ($(BUILDSET), bs_4wd)
	# it is using stm32f429_439xx
    TARGET_NAME:= mambabot_4wd
	LINKER_FILE := \
					$(ENV_DIR)/build/stm32f4xx_flash.ld
else ifeq ($(BUILDSET), bs_diff_drive)
	TARGET_NAME:= mambabot_diff_drive
	LINKER_FILE := \
					$(ENV_DIR)/build/stm32f4xx_flash.ld
endif