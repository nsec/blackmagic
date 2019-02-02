CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy
AR = $(CROSS_COMPILE)ar

CFLAGS += -Istm32/include -mcpu=cortex-m0 -mthumb \
	-DSTM32F0 -I../../libopencm3/include \
	-Iplatforms/stm32 -Os

LDFLAGS_BOOT := $(LDFLAGS) -lopencm3_stm32f0 \
	-Wl,-T,platforms/nsec18/nsec18.ld -nostartfiles -lc \
	-Wl,-Map=mapfile -mthumb -mcpu=cortex-m0 -Wl,-gc-sections \
	-L../../libopencm3/lib -lnosys
LDFLAGS = $(LDFLAGS_BOOT) -Wl,-Ttext=0x8000000

ifeq ($(ENABLE_DEBUG), 1)
LDFLAGS += --specs=rdimon.specs
else
LDFLAGS += --specs=nano.specs
endif

CFLAGS += -DENABLE_SELECTED_FEATURES_ONLY
CFLAGS += -DENABLE_TARGET_PROTO_SWDIO
# CFLAGS += -DENABLE_TARGET_STM32F1
CFLAGS += -DENABLE_TARGET_NRF51

ifeq ($(FLAVOR), crossdebug)
	CFLAGS += -DNSEC_BADGE_USE_EXTERNAL_PINS
endif

VPATH += platforms/stm32

SRC += 	cdcacm.c	\
	usbuart.c	\
	traceswo.c	\
	serialno.c	\
	timing.c	\
	timing_stm32.c	\

all:	blackmagic.bin libblackmagic.a

libblackmagic.a: $(OBJ)
	$(Q)$(AR) rcs "$@" $(filter-out main.o, $(OBJ))

blackmagic_dfu: usbdfu.o dfucore.o dfu_f0.o
	@echo "  LD      $@"
	$(Q)$(CC) $^ -o $@ $(LDFLAGS_BOOT)

host_clean:
	-$(Q)$(RM) -f blackmagic.bin blackmagic_dfu blackmagic_dfu.bin blackmagic_dfu.hex libblackmagic.a

