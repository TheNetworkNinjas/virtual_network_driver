/*
* virt_net_driver - Virtual network adapter for Linux
*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Craig Opie");
MODULE_AUTHOR("Jake Imanaka");
MODULE_AUTHOR("Lydia Sollis");
MODULE_DESCRIPTION("Virtual network driver for Linux");
MODULE_VERSION("0.01");

static int __init virt_net_driver_init(void)
{
	printk(KERN_ALERT "Hello, world \n");
	return 0;
}

static void __exit virt_net_driver_exit(void)
{
	printk(KERN_ALERT "bye bye");
}

module_init(virt_net_driver_init);
module_exit(virt_net_driver_exit);
