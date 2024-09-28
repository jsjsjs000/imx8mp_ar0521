# obj-m := ar0521.o

# ar5-objs := ar0521.o
ar5-objs := ar0521_small.o
obj-m += ar5.o

KDIR = ~/linux-imx-v5.15.71_2.2.2-phy/
PWD = $(shell pwd)

all:
	make -C $(KDIR) SUBDIRS=$(PWD) M=$(PWD) modules

clean:
	make -C $(KDIR) SUBDIRS=$(PWD) M=$(PWD) clean
