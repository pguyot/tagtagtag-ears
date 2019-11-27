# SPDX-License-Identifier: GPL-2.0
obj-m += tagtagtag-ears.o
dtbo-y += tagtagtag-ears.dtbo

targets += $(dtbo-y)
always  := $(dtbo-y)
kernel_img_gzip_offset := $(shell grep -m 1 -abo 'uncompression error' /boot/kernel.img | cut -d ':' -f 1)
kernel_img_gzip_offset := $(shell expr $(kernel_img_gzip_offset) + 20)
kernel_version := $(shell dd if=/boot/kernel.img skip=$(kernel_img_gzip_offset) iflag=skip_bytes of=/dev/stdout | zgrep -aPom1 'Linux version \K\S+')

all:
	make -C /lib/modules/$(kernel_version)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(kernel_version)/build M=$(PWD) clean

install: tagtagtag-ears.ko tagtagtag-ears.dtbo
	install -o root -m 755 -d /lib/modules/$(kernel_version)/kernel/input/misc/
	install -o root -m 644 tagtagtag-ears.ko /lib/modules/$(kernel_version)/kernel/input/misc/
	depmod -a $(kernel_version)
	install -o root -m 644 tagtagtag-ears.dtbo /boot/overlays/
	sed /boot/config.txt -i -e "s/^#dtoverlay=tagtagtag-ears/dtoverlay=tagtagtag-ears/"
	grep -q -E "^dtoverlay=tagtagtag-ears" /boot/config.txt || printf "dtoverlay=tagtagtag-ears\n" >> /boot/config.txt

.PHONY: all clean install
