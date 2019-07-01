#
# Application target name

TARGET = ZigbeeNodeControlBridge
###############################################################################
#User definable make parameters that may be overwritten from the command line

JENNIC_SDK ?= JN-SW-4168
ZBPRO_DEVICE_TYPE = ZCR
JENNIC_MAC ?= MiniMacShim
PDM_BUILD_TYPE ?= _NONE
BAUD ?= 115200
DEBUG ?= NONE
VERSION ?= 8448

STACK_SIZE = 6500
MINIMUM_HEAP_SIZE = 2000

# Chip variant

JENNIC_CHIP ?= JN5168
JENNIC_CHIP_FAMILY ?= JN516x


###############################################################################
# Default DK2 development kit target hardware

JENNIC_PCB ?= DEVKIT2

###############################################################################
# Select the network stack (e.g. MAC, ZBPro)

JENNIC_STACK ?= ZLLHA
###############################################################################
# commands options
# enable optional commands as required per applicaion.
#
STACK_MEASURE 	?= 0
EXTENDED_DEBUG 	?= 0
ZCL_ON_HOST		?= 0
DISCOVERY_ONLY_MODE ?= 0
APS_ACK			?= 1
MAC_ACK			?= 0
#
###############################################################################
# Debug options - define DEBUG to enable
#DEBUG =HW
#
# Define which UART to use for HW debug
DEBUG_PORT ?= NONE

###############################################################################

CFLAGS  += -DUART_BAUD_RATE=$(BAUD)
CFLAGS  += -DVERSION=$(VERSION)

ifeq ($(DEBUG_PORT), UART1)
CFLAGS  += -DUART_DEBUGGING=1
CFLAGS  += -DDEBUG_UART=DBG_E_UART_1
#CFLAGS  += -DDEBUG_APP_OTA
CFLAGS  += -DDEBUG_ZCL
CFLAGS	+= -DDEBUG_ZB_CONTROLBRIDGE_TASK
CFLAGS  += -DTRACE_APP
CFLAGS  += -DDEBUG_AGE_CHILDREN
CFALGS  += -DDEBUG_ZBP_UTILS
endif

ifeq ($(STACK_MEASURE), 1)
CFLAGS	+= -DSTACK_MEASURE
endif

ifeq ($(EXTENDED_DEBUG), 1)
CFLAGS	+= -DAPP_EXTENDED_DEBUG
endif

ifeq ($(ZCL_ON_HOST), 1)
CFLAGS	+= -DAPP_ZCL_ON_HOST
endif

ifeq ($(DISCOVERY_ONLY_MODE), 1)
CFLAGS	+= -DAPP_DISCOVERY_ONLY_MODE_ENABLED
endif

ifeq ($(APS_ACK), 1)
CFLAGS += -DAPP_APS_ACK
endif

ifeq ($(MAC_ACK), 1)
CFLAGS += -DAPP_MAC_ACK
endif
###############################################################################
# Path definitions

# Use if application directory contains multiple targets
BSTUDIO_BASE_DIR   	?= $(abspath ../../../../)
SDK_BASE_DIR   	 	= $(BSTUDIO_BASE_DIR)/sdk/$(JENNIC_SDK)
APP_BASE           	?= $(abspath ../../)
APP_BLD_DIR			= $(APP_BASE)/Build/ZigbeeNodeControlBridge
APP_SRC_DIR 	   	= $(APP_BASE)/Source/ZigbeeNodeControlBridge
UTIL_SRC_DIR        = $(COMPONENTS_BASE_DIR)/Utilities/Source
APP_COMMON_SRC_DIR  = $(APP_BASE)/Source/Common

###############################################################################
# Application Source files

# Note: Path to source file is found using vpath below, so only .c filename is required
APPSRC  = os_gen.c
APPSRC += os_irq.S
APPSRC += os_irq_alignment.S
APPSRC += os_irq_buserror.S
APPSRC += os_irq_illegalinstruction.S
APPSRC += os_irq_stackoverflowexception.S
APPSRC += os_irq_unimplementedmodule.S
APPSRC += pdum_gen.c
APPSRC += zps_gen.c
APPSRC += app_Znc_cmds.c
APPSRC += app_Znc_zcltask.c
APPSRC += app_ZncParser_task.c
APPSRC += app_ZncCommission_task.c
APPSRC += app_start.c
APPSRC += SerialLink.c
APPSRC += uart.c
APPSRC += NumToString.c
APPSRC += app_timer_driver.c
APPSRC += ecb_decrypt.c
APPSRC += appZpsBeaconHandler.c
APPSRC += appZdpExtraction.c
APPSRC += pdum_apdu.S
APPSRC += app_scenes.c
APPSRC += app_ota_server.c
APPSRC += app_poll_control_commands_handler.c
APPSRC += app_zbp_utilities.c
APPSRC += AgeChildren.c

ifeq ($(STACK_MEASURE), 1)
APPSRC += StackMeasure.c
endif

ifeq ($(PDM_BUILD_TYPE), _NONE)
APPSRC += pdm_host.c
endif
LIBNAME = $(TARGET)_PDM$(PDM_BUILD_TYPE)_BAUD_$(BAUD)_DEBUG_$(DEBUG)
APP_ZPSCFG = ZigbeeNodeControlBridge.zpscfg
OPTIONAL_STACK_FEATURES = $(shell $(ZPSCONFIG) -n $(TARGET) -f $(APP_SRC_DIR)/$(APP_ZPSCFG) -y )

###############################################################################
# Standard Application header search paths

INCFLAGS += -I$(APP_SRC_DIR)
INCFLAGS += -I$(APP_SRC_DIR)/..
INCFLAGS += -I$(APP_COMMON_SRC_DIR)

# Application specific include files
INCFLAGS += -I$(COMPONENTS_BASE_DIR)/JennicLogo/Include
INCFLAGS += -I$(COMPONENTS_BASE_DIR)/Utilities/Include
INCFLAGS += -I$(COMPONENTS_BASE_DIR)/Random/Include
INCFLAGS += -I$(COMPONENTS_BASE_DIR)/MAC/Include
INCFLAGS += -I$(COMPONENTS_BASE_DIR)/NXPLogo/Include
INCFLAGS += -I$(COMPONENTS_BASE_DIR)/Utilities/Include
INCFLAGS += -I$(COMPONENTS_BASE_DIR)/Patch/Include
INCFLAGS += -I$(COMPONENTS_BASE_DIR)/OVLY/Include
INCFLAGS += -I$(COMPONENTS_BASE_DIR)/Aes/Include
INCFLAGS += -I$(COMPONENTS_BASE_DIR)/Utilities/Include
###############################################################################
# Application libraries
# Specify additional Component libraries

#APPLIBS += 

###############################################################################

# You should not need to edit below this line
APP_CLUSTER_ZLL_SRC ?= 1
APP_CLUSTERS_MEASUREMENT_AND_SENSING ?= 1
APP_CLUSTERS_HVAC_SRC ?= 1
APP_CLUSTERS_IAS_SRC ?=  1
APP_CLUSTERS_SMART_ENERGY_SRC ?= 1
APP_CLUSTERS_ENERGY_AT_HOME_SRC ?= 1
APP_CLUSTERS_OTA_SRC ?= 1

###############################################################################
###############################################################################
# Configure for the selected chip or chip family

include $(SDK_BASE_DIR)/Chip/Common/Build/config.mk
include $(SDK_BASE_DIR)/Platform/Common/Build/Config.mk
include $(SDK_BASE_DIR)/Stack/Common/Build/config.mk

###############################################################################

TEMP = $(APPSRC:.c=.o)
APPOBJS = $(TEMP:.S=.o)

###############################################################################
# Application dynamic dependencies

APPDEPS = $(APPOBJS:.o=.d)

###############################################################################
# Determine if overlays are being used

OVERLAY_BUILD=$(shell $(OSCONFIG) -f $(APP_SRC_DIR)/$(TARGET).oscfgdiag -y )

###############################################################################
# Linker

# Add application libraries before chip specific libraries to linker so
# symbols are resolved correctly (i.e. ordering is significant for GCC)

APPLDLIBS := $(foreach lib,$(APPLIBS),$(if $(wildcard $(addprefix $(COMPONENTS_BASE_DIR)/Library/lib,$(addsuffix _$(JENNIC_CHIP).a,$(lib)))),$(addsuffix _$(JENNIC_CHIP),$(lib)),$(addsuffix _$(JENNIC_CHIP_FAMILY),$(lib))))
LDLIBS := $(APPLDLIBS) $(LDLIBS)

###############################################################################
# Dependency rules

.PHONY: all clean
# Path to directories containing application source
vpath % $(APP_SRC_DIR):$(APP_COMMON_SRC_DIR):$(UTIL_SRC_DIR):$(HA_SRC_DIRS):$(ZLL_SRC_DIRS)

all: $(TARGET)_$(JENNIC_CHIP)$(BIN_SUFFIX).bin

-include $(APPDEPS)
%.d:

$(APP_SRC_DIR)/os_gen.c $(APP_SRC_DIR)/os_gen.h $(APP_SRC_DIR)/os_irq.S $(APP_SRC_DIR)/os_irq_alignment.S $(APP_SRC_DIR)/os_irq_buserror.S $(APP_SRC_DIR)/os_irq_illegalinstruction.S $(APP_SRC_DIR)/os_irq_stackoverflowexception.S $(APP_SRC_DIR)/os_irq_unimplementedmodule.S: $(APP_SRC_DIR)/$(TARGET).oscfgdiag $(OSCONFIG) 
	$(info Configuring the OS ...)
	$(OSCONFIG) -f $< -o $(APP_SRC_DIR) -v $(JENNIC_CHIP)

$(APP_SRC_DIR)/pdum_gen.c $(APP_SRC_DIR)/pdum_gen.h: $(APP_SRC_DIR)/$(APP_ZPSCFG) $(PDUMCONFIG)
	$(info Configuring the PDUM ...)
	$(PDUMCONFIG) -z $(TARGET) -f $< -o $(APP_SRC_DIR)

$(APP_SRC_DIR)/zps_gen.c $(APP_SRC_DIR)/zps_gen.h: $(APP_SRC_DIR)/$(APP_ZPSCFG) $(ZPSCONFIG)
	$(info Configuring the Zigbee Protocol Stack ...)
	$(ZPSCONFIG) -n $(TARGET) -t $(JENNIC_CHIP) -l $(ZPS_NWK_LIB) -a $(ZPS_APL_LIB) -c $(TOOL_COMMON_BASE_DIR)/$(TOOLCHAIN_PATH) -f $< -o $(APP_SRC_DIR)
	
%.o: %.S
	$(info Assembling $< ...)
	$(CC) -c -o $(subst Source,Build,$@) $(CFLAGS) $(INCFLAGS) $< -MD -MF $*.d -MP
	@echo

%.o: %.c
	$(info Compiling $< ...)
	$(CC) -c -o $(subst Source,Build,$@) $(CFLAGS) $(INCFLAGS) $< -MD -MF $*.d -MP
	@echo

$(TARGET)_$(JENNIC_CHIP)$(BIN_SUFFIX).elf: $(APPOBJS) $(addsuffix.a,$(addprefix $(COMPONENTS_BASE_DIR)/Library/lib,$(APPLDLIBS))) 
	$(info Linking $@ ...)
	$(CC) -Wl,--cref,--gc-sections -Wl,-u_AppColdStart -Wl,-u_AppWarmStart $(LDFLAGS) -TAppBuildZBPro.ld -o $@ -Wl,--start-group $(APPOBJS) $(addprefix -l,$(LDLIBS))  -lm -Wl,--end-group -Wl,-Map,$(TARGET)_$(JENNIC_CHIP).map 
	 
	$(SIZE) $@ 

$(TARGET)_$(JENNIC_CHIP)$(BIN_SUFFIX).bin: $(TARGET)_$(JENNIC_CHIP)$(BIN_SUFFIX).elf
	$(info Generating binary ...)
	$(OBJCOPY) -j .version -j .bir -j .flashheader -j .vsr_table -j .vsr_handlers -j .ro_mac_address -j .ro_ota_header -j .ro_se_customData -j .rodata -j .text -j .data -j .bss -j .heap -j .stack -S -O binary $< $@
	cp $(TARGET)_$(JENNIC_CHIP)$(BIN_SUFFIX).bin	../bin


################################################################################

clean:
	rm -f $(APPOBJS) $(APPDEPS) $(TARGET)_$(JENNIC_CHIP)*.bin $(TARGET)_$(JENNIC_CHIP)*.elf $(TARGET)_$(JENNIC_CHIP)*.map *.o *.d
	rm -f $(APP_SRC_DIR)/os_gen.c $(APP_SRC_DIR)/os_gen.h $(APP_SRC_DIR)/os_irq*.S $(APP_SRC_DIR)/pdum_gen.* $(APP_SRC_DIR)/zps_gen*.* $(APP_SRC_DIR)/pdum_apdu.*

###############################################################################
