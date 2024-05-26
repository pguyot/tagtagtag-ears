# SPDX-License-Identifier: GPL-2.0
KERNELRELEASE ?= $(shell uname -r)

obj-m += tagtagtag-ears.o
dtbo-y += tagtagtag-ears.dtbo

targets += $(dtbo-y)

# Gracefully supporting the new always-y without cutting off older target with kernel 4.x
ifeq ($(firstword $(subst ., ,$(KERNELRELEASE))),4)
	always := $(dtbo-y)
else
	always-y := $(dtbo-y)
endif

ifeq ($(wildcard /boot/firmware/config.txt),)
    BOOT_CONFIG := /boot/config.txt
else
    BOOT_CONFIG := /boot/firmware/config.txt
endif

all:
	make -C /lib/modules/$(KERNELRELEASE)/build M=$(PWD) modules

# dtbo rule is no longer available
ifeq ($(firstword $(subst ., ,$(KERNELRELEASE))),6)
all: tagtagtag-ears.dtbo

tagtagtag-ears.dtbo: tagtagtag-ears-overlay.dts
	dtc -I dts -O dtb -o $@ $<
endif

clean:
	make -C /lib/modules/$(KERNELRELEASE)/build M=$(PWD) clean

install: tagtagtag-ears.ko tagtagtag-ears.dtbo
	install -o root -m 755 -d /lib/modules/$(KERNELRELEASE)/kernel/input/misc/
	install -o root -m 644 tagtagtag-ears.ko /lib/modules/$(KERNELRELEASE)/kernel/input/misc/
	depmod -a $(KERNELRELEASE)
	install -o root -m 644 tagtagtag-ears.dtbo /boot/overlays/
	sed /boot/config.txt -i -e "s/^#dtoverlay=tagtagtag-ears/dtoverlay=tagtagtag-ears/"
	grep -q -E "^dtoverlay=tagtagtag-ears" $(BOOT_CONFIG) || printf "dtoverlay=tagtagtag-ears\n" >> $(BOOT_CONFIG)

.PHONY: all clean install
