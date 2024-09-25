obj-m := ar0521.o

KDIR = ~/linux-imx-v5.15.71_2.2.2-phy/
PWD = $(shell pwd)

all:
	make -C $(KDIR) SUBDIRS=$(PWD) M=$(PWD) modules

clean:
	make -C $(KDIR) SUBDIRS=$(PWD) M=$(PWD) clean
