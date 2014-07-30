	
	cp tw68v.ko /lib/modules/`uname -r`/kernel/drivers/media/video
	chmod 644 /lib/modules/`uname -r`/kernel/drivers/media/video/tw68v.ko
	depmod -ae
