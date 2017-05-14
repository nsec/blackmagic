ifneq ($(V), 1)
MFLAGS += --no-print-dir
Q := @
endif

all:
	$(Q)$(MAKE) $(MFLAGS) -C ../libopencm3 lib TARGETS=stm32/f0
	$(Q)$(MAKE) $(MFLAGS) -C src

clean:
	$(Q)$(MAKE) $(MFLAGS) -C ../libopencm3 $@
	$(Q)$(MAKE) $(MFLAGS) -C src $@

