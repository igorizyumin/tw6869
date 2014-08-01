modprobe videobuf_vmalloc

modprobe v4l2_common
modprobe videobuf_dma_sg
rmmod tw68v
insmod tw68v.ko

