# $LAAS$

obj-m	:= jr3.o
jr3-objs := jr3base.o download.o ../common/dspfunc.o
obj-m	+= ../common/

KDIR    := /lib/modules/$(shell uname -r)/build
IDIR    := /lib/modules/$(shell uname -r)/kernel/drivers/char
PWD     :=  $(shell pwd)

JR3_INC = $(src)/../include

EXTRA_CFLAGS = -I$(JR3_INC)

default:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules

install:
	install -c -o root -g root -m 744 jr3.ko $(IDIR)

clean:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) clean

.PHONY:
	install
