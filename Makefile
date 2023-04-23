obj-m += virt_net_driver.o

all:
	make -w -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) modules

clean:
	make -w -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) clean



