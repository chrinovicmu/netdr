MODULE_NAME := netdr
obj-m := src/$(MODULE_NAME).o

# Use absolute path for the include directory
# This ensures the compiler finds it regardless of which directory Kbuild is in
ccflags-y := -I$(PWD)/include

KDIR := /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

.PHONY: all clean
