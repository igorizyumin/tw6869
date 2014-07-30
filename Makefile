# call from kernel build system

tw68v-objs :=	 TW68-core.o  TW68-video.o TW68-ALSA.o 

# TW6864-i2c.o   

obj-m += tw68v.o

PWD := $(shell pwd)
KDIR ?= /lib/modules/$(shell uname -r)/build
TWMODS := /lib/modules/$(shell uname -r)
TWDEST := $(TWMODS)/kernel/drivers/media/video

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	rm -rf Module.markers  Module.symvers modules.order
	rm -rf *.o .*.cmd *ko* *mod* 
