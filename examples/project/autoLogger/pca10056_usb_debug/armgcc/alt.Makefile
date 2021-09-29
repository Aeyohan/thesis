PROJECT_NAME := ble_app_template_s130_pca10028

export OUTPUT_FILENAME
#MAKEFILE_NAME := $(CURDIR)/$(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST))
MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) ) 

SDK_ROOT := C:/Users/Aeyohan/Downloads/DeviceDownload/nRF5SDK1702d674dde/nRF5_SDK_17.0.2_d674dde
PROJ_DIR := ../..

TEMPLATE_PATH = $(SDK_ROOT)/components/toolchain/gcc
ifeq ($(OS),Windows_NT)
include $(TEMPLATE_PATH)/Makefile.windows
else
include $(TEMPLATE_PATH)/Makefile.posix
endif

MK := mkdir
RM := del /q /s 

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO := 
else
NO_ECHO := @
endif

# Toolchain commands
CC              := '$(GNU_INSTALL_ROOT)/$(GNU_PREFIX)-gcc'
CXX             := "$(GNU_INSTALL_ROOT)/$(GNU_PREFIX)-c++"
AS              := '$(GNU_INSTALL_ROOT)/$(GNU_PREFIX)-as'
AR              := '$(GNU_INSTALL_ROOT)/$(GNU_PREFIX)-ar' -r
LD              := '$(GNU_INSTALL_ROOT)/$(GNU_PREFIX)-ld'
NM              := '$(GNU_INSTALL_ROOT)/$(GNU_PREFIX)-nm'
OBJDUMP         := '$(GNU_INSTALL_ROOT)/$(GNU_PREFIX)-objdump'
OBJCOPY         := '$(GNU_INSTALL_ROOT)/$(GNU_PREFIX)-objcopy'
SIZE            := '$(GNU_INSTALL_ROOT)/$(GNU_PREFIX)-size'

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

#C files common to all targets
C_SOURCE_FILES += \
$(SDK_ROOT)/components/libraries/button/app_button.c \
$(SDK_ROOT)/components/libraries/util/app_error.c \
$(SDK_ROOT)/components/libraries/util/app_error_weak.c \
$(SDK_ROOT)/components/libraries/timer/app_timer.c \
$(SDK_ROOT)/components/libraries/trace/app_trace.c \
$(SDK_ROOT)/components/libraries/util/app_util_platform.c \
$(SDK_ROOT)/components/libraries/fstorage/fstorage.c \
$(SDK_ROOT)/components/libraries/util/nrf_assert.c \
$(SDK_ROOT)/components/libraries/util/nrf_log.c \
$(SDK_ROOT)/components/libraries/uart/retarget.c \
$(SDK_ROOT)/components/libraries/sensorsim/sensorsim.c \
$(SDK_ROOT)/external/segger_rtt/RTT_Syscalls_GCC.c \
$(SDK_ROOT)/external/segger_rtt/SEGGER_RTT.c \
$(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_printf.c \
$(SDK_ROOT)/components/libraries/uart/app_uart.c \
$(SDK_ROOT)/components/drivers_nrf/delay/nrf_delay.c \
$(SDK_ROOT)/components/drivers_nrf/common/nrf_drv_common.c \
$(SDK_ROOT)/components/drivers_nrf/gpiote/nrf_drv_gpiote.c \
$(SDK_ROOT)/components/drivers_nrf/uart/nrf_drv_uart.c \
$(SDK_ROOT)/components/drivers_nrf/pstorage/pstorage.c \
$(SDK_ROOT)/components/libraries/bsp/bsp.c \
$(SDK_ROOT)/components/ble/common/ble_advdata.c \
$(SDK_ROOT)/components/ble/ble_advertising/ble_advertising.c \
$(SDK_ROOT)/components/ble/common/ble_conn_params.c \
$(SDK_ROOT)/components/ble/common/ble_srv_common.c \
$(SDK_ROOT)/components/ble/device_manager/device_manager_peripheral.c \
$(SDK_ROOT)/components/toolchain/system_nrf51.c \
$(SDK_ROOT)/components/softdevice/common/softdevice_handler/softdevice_handler.c \
$(PROJ_DIR)/main.c \

# $(abspath ../../../../../bsp/bsp_btn_ble.c \

# CPP_SOURCE_FILES += \
# $(PROJ_DIR)/main.cpp \

# $(abspath ../../../cpptest.cpp \


#assembly files common to all targets
ASM_SOURCE_FILES  = $(SDK_ROOT)/components/toolchain/gcc/gcc_startup_nrf51.s

#includes common to all targets
# INC_PATHS  = -I$(PROJ_DIR)/config/ble_app_template_s130_pca10028
INC_PATHS += -I$(PROJ_DIR)/pca10056_usb_debug/config
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/config
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/nrd_soc_nosd
INC_PATHS += -I$(SDK_ROOT)/components/libraries/timer
INC_PATHS += -I$(SDK_ROOT)/components/libraries/fstorage/config
INC_PATHS += -I$(SDK_ROOT)/components/softdevice/s130/headers
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/delay
INC_PATHS += -I$(SDK_ROOT)/components/libraries/util
INC_PATHS += -I$(SDK_ROOT)/components/ble/device_manager
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/uart
INC_PATHS += -I$(SDK_ROOT)/components/ble/common
INC_PATHS += -I$(SDK_ROOT)/components/libraries/sensorsim
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/pstorage
INC_PATHS += -I$(SDK_ROOT)/components/libraries/uart
INC_PATHS += -I$(SDK_ROOT)/components/device
INC_PATHS += -I$(SDK_ROOT)/components/libraries/button
INC_PATHS += -I$(SDK_ROOT)/components/libraries/fstorage
INC_PATHS += -I$(SDK_ROOT)/components/libraries/experimental_section_vars
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/gpiote
INC_PATHS += -I$(SDK_ROOT)/external/segger_rtt
INC_PATHS += -I$(SDK_ROOT)/components/libraries/bsp
INC_PATHS += -I$(SDK_ROOT)/components/toolchain/CMSIS/Include
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/hal
INC_PATHS += -I$(SDK_ROOT)/components/toolchain/gcc
INC_PATHS += -I$(SDK_ROOT)/components/toolchain
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/common
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_advertising
INC_PATHS += -I$(SDK_ROOT)/components/softdevice/s130/headers/nrf51
INC_PATHS += -I$(SDK_ROOT)/components/libraries/trace
INC_PATHS += -I$(SDK_ROOT)/components/softdevice/common/softdevice_handler
INC_PATHS += -I$(SDK_ROOT)/modules/nrfx/mdk/
INC_PATHS += -I$(SDK_ROOT)/modules/nrfx/drivers/include
INC_PATHS += -I$(SDK_ROOT)/external/fprintf


OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

# C flags to all targets
CFLAGS  = -DNRF_LOG_USES_UART=1
CFLAGS += -DBOARD_PCA10028
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DNRF51
CFLAGS += -DS130
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DSWI_DISABLE0
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs
CFLAGS += -Wall -O3 -g3
CFLAGS += -mfloat-abi=soft
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums 

# CXX Flags

# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys

# Assembler flags
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DNRF_LOG_USES_UART=1
ASMFLAGS += -DBOARD_PCA10028
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DNRF51
ASMFLAGS += -DS130
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DSWI_DISABLE0
#default target - first one defined
default: clean nrf51422_xxac_s130

#building all targets
all: clean
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e nrf51422_xxac_s130

#target for printing all targets
help:
	@echo following targets are available:
	@echo 	nrf51422_xxac_s130
	@echo 	flash_softdevice

C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
CPP_SOURCE_FILE_NAMES = $(notdir $(CPP_SOURCE_FILES))
ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))

C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
CPP_PATHS = $(call remduplicates, $(dir $(CPP_SOURCE_FILES) ) )
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))

C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )
CPP_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(CPP_SOURCE_FILE_NAMES:.cpp=.o) )
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )	

vpath %.c   $(C_PATHS)
vpath %.cpp $(CPP_PATHS)
vpath %.s   $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(CPP_OBJECTS) $(ASM_OBJECTS)

nrf51422_xxac_s130: OUTPUT_FILENAME := nrf51422_xxac_s130
nrf51422_xxac_s130: LINKER_SCRIPT=ble_app_template_gcc_nrf51.ld
nrf51422_xxac_s130: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out

	$(NO_ECHO)$(CC) -std=gnu99 $(LDFLAGS) $(OBJECTS) $(LIBS) -lm -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

## Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) -std=gnu99 $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Create objects from Cpp SRC files
$(OBJECT_DIRECTORY)/%.o: %.cpp
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CXX) $(CFLAGS) $(CXXFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) -std=c99 $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<


# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out


## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

finalize: genbin genhex echosize

genbin:
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
genhex: 
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

echosize:
	-@echo ''
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ''

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o

flash: $(MAKECMDGOALS)
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/$<.hex
	nrfjprog --program $(OUTPUT_BINARY_DIRECTORY)/$<.hex -f nrf51  --sectorerase
	nrfjprog --reset

## Flash softdevice
flash_softdevice:
	@echo Flashing: s130_nrf51_2.0.0_softdevice.hex
	nrfjprog --program ../../../../../../components/softdevice/s130/hex/s130_nrf51_2.0.0_softdevice.hex -f nrf51 --chiperase
	nrfjprog --reset