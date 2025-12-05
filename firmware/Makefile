#BOARD ?= lisa_m2.0
BOARD ?= supercan_v2.0
BMP_PORT ?= $(shell \
    for dev in /dev/ttyACM*; do \
        udevadm info -a -n $$dev | grep -q 'ATTRS{product}=="Black Magic Probe"' && echo $$dev; \
    done | head -n1 \
)

# Default GDB
GDB ?= gdb

# Provide a convenience target to select gdb-multiarch.
# Usage: make multiarch upload
ifneq ($(filter multiarch,$(MAKECMDGOALS)),)
GDB := gdb-multiarch
.PHONY: multiarch
multiarch:
	@:
endif

all: bootloader node

clean:
	rm -rf build

bootloader:
	make -C bootloader BOARD=$(BOARD)

node:
	make -C node BOARD=$(BOARD)

python_req:
	pip install empy==3.3.4 pexpect
	
protocol:
	cd ./lib/ && python3 dronecan_dsdlc/dronecan_dsdlc.py -O dsdlc_generated DSDL/dronecan DSDL/uavcan DSDL/com DSDL/ardupilot

%.upload:
	@echo "Using Black Magic Probe with SWD on BMP_PORT $(BMP_PORT)"
	@echo "Using GDB = $(GDB)"
	@echo "  BMP\t$<"
	$(Q)$(GDB) --batch \
		   -ex 'target extended-remote $(BMP_PORT)' \
		   -x swd_flash.scr \
		   build/$(BOARD)/$*/tuc_$*.elf

upload: bootloader node bl.upload node.upload

.PHONY: clean bootloader node upload