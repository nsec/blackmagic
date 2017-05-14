CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

CFLAGS += -Istm32/include -mcpu=cortex-m0 -mthumb \
	-DSTM32F0 -DBLACKMAGIC -I../../libopencm3/include \
	-Iplatforms/stm32

LDFLAGS_BOOT := $(LDFLAGS) -lopencm3_stm32f0 -Wl,--defsym,_stack=0x20004000 \
	-Wl,-T,platforms/nsec17/nsec17.ld -nostartfiles -lc \
	-Wl,-Map=mapfile -mthumb -mcpu=cortex-m0 -Wl,-gc-sections \
	-L../../libopencm3/lib
LDFLAGS = $(LDFLAGS_BOOT) -Wl,-Ttext=0x8000000

ifeq ($(ENABLE_DEBUG), 1)
LDFLAGS += --specs=rdimon.specs
else
LDFLAGS += --specs=nosys.specs
endif

VPATH += platforms/stm32

SRC += 	cdcacm.c	\
	traceswo.c	\
	usbuart.c	\
	serialno.c	\
	timing.c	\
	timing_stm32.c	\

all:	blackmagic.bin

blackmagic_dfu: usbdfu.o dfucore.o dfu_f0.o
	@echo "  LD      $@"
	$(Q)$(CC) $^ -o $@ $(LDFLAGS_BOOT)

host_clean:
	-$(Q)$(RM) -f blackmagic.bin blackmagic_dfu blackmagic_dfu.bin blackmagic_dfu.hex

