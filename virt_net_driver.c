#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/kfifo.h>
#include "virt_net_driver.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Craig Opie");
MODULE_AUTHOR("Jake Imanaka");
MODULE_AUTHOR("Lydia Sollis");
MODULE_DESCRIPTION("Virtual network driver for Linux");
MODULE_VERSION("0.01");

static struct net_device *virt_net_dev;

static int __init virt_net_driver_init(void)
{
    int ret;

    /* Allocate and initialize net_device */
    virt_net_dev = alloc_etherdev(sizeof(struct virt_net_dev_priv));
    if (!virt_net_dev) {
        printk(KERN_ERR "%s: Failed to allocate net_device\n", VIRT_NET_DRIVER_NAME);
        return -ENOMEM;
    }

    /* Initialize net_device fields and operations */
	/* Set the device's name */
	strlcpy(virt_net_dev->name, VIRT_NET_DRIVER_NAME, sizeof(virt_net_dev->name));

	/* Assign the net_device operations */
	static const struct net_device_ops virt_net_dev_ops = {
		.ndo_open = virt_net_driver_open,
		.ndo_stop = virt_net_driver_stop,
		.ndo_start_xmit = virt_net_driver_start_xmit,
		.ndo_set_mac_address = virt_net_driver_set_mac_address,
		.ndo_do_ioctl = virt_net_driver_do_ioctl,
	};

	virt_net_dev->netdev_ops = &virt_net_dev_ops;

	/* Assign the ethtool operations */
    // TODO: Fill in necessary fields and function pointers in virt_net_dev
	static const struct ethtool_ops virt_net_ethtool_ops = {
		// .get_drvinfo = virt_net_driver_get_drvinfo,
		// .get_link = virt_net_driver_get_link,
		// .get_link_ksettings = virt_net_driver_get_link_ksettings,
		// .set_link_ksettings = virt_net_driver_set_link_ksettings,
		// ... other ethtool operations
	};

	virt_net_dev->ethtool_ops = &virt_net_ethtool_ops;

    /* Register the network device with the kernel */
    ret = register_netdev(virt_net_dev);
    if (ret) {
        printk(KERN_ERR "%s: Failed to register net_device: %d\n", VIRT_NET_DRIVER_NAME, ret);
        free_netdev(virt_net_dev);
        return ret;
    }

    printk(KERN_INFO "%s: Virtual network driver loaded\n", VIRT_NET_DRIVER_NAME);
    return 0;
}

static void __exit virt_net_driver_exit(void)
{
    /* Unregister the network device */
    unregister_netdev(virt_net_dev);

    /* Free the net_device memory */
    free_netdev(virt_net_dev);

    printk(KERN_INFO "%s: Virtual network driver unloaded\n", VIRT_NET_DRIVER_NAME);
}

static int init_virt_hw_resource(struct net_device *dev)
{
    struct virt_net_dev_priv *priv = netdev_priv(dev);
    int ret;

    /* Initialize the virtual FIFO buffer */
    spin_lock_init(&priv->tx_fifo.lock);
    ret = kfifo_alloc(&priv->tx_fifo.fifo, VIRT_FIFO_SIZE, GFP_KERNEL);
    if (ret) {
        printk(KERN_ERR "%s: Failed to allocate virtual FIFO buffer\n", dev->name);
        return ret;
    }

    /* Initialize the timer */
    timer_setup(&priv->timer, virt_net_timer_callback, 0);
    priv->timer.expires = jiffies + msecs_to_jiffies(1000); // Set the timer to expire in 1 second
    add_timer(&priv->timer);

    return 0;
}

static void release_virt_hw_resource(struct net_device *dev)
{
    struct virt_net_dev_priv *priv = netdev_priv(dev);

    /* Release the virtual FIFO buffer */
    kfifo_free(&priv->tx_fifo.fifo);

    /* Delete the timer */
    del_timer_sync(&priv->timer);
}

static void virt_net_timer_callback(struct timer_list *t)
{
    struct virt_net_dev_priv *priv = from_timer(priv, t, timer);
    struct net_device *dev = priv->netdev;

    /* Increment the counter */
    priv->counter++;

    /* Print a message */
    printk(KERN_INFO "%s: Timer tick, counter = %lu\n", dev->name, priv->counter);

    /* Reschedule the timer */
    mod_timer(&priv->timer, jiffies + msecs_to_jiffies(1000));
}

static int virt_net_driver_open(struct net_device *dev)
{
    struct virt_net_dev_priv *priv = netdev_priv(dev);

    /* Initialize any resources required for the virtual network driver */
    int ret = init_virt_hw_resource(dev);
    if (ret) {
        printk(KERN_ERR "%s: Failed to initialize hardware resource\n", dev->name);
        return ret;
    }

    /* Initialize a kernel timer */
    timer_setup(&priv->timer, virt_net_timer_callback, 0);
    mod_timer(&priv->timer, jiffies + msecs_to_jiffies(1000));

    /* Start the network device's transmit queue */
    netif_start_queue(dev);

    printk(KERN_INFO "%s: Virtual network device opened\n", dev->name);

    return 0;
}

static int virt_net_driver_stop(struct net_device *dev)
{
    struct virt_net_dev_priv *priv = netdev_priv(dev);

    /* Stop the network device's transmit queue */
    netif_stop_queue(dev);

    /* Cleanup any resources allocated for the virtual network driver */
    release_virt_hw_resource(dev);

    /* Delete the kernel timer */
    del_timer_sync(&priv->timer);

    printk(KERN_INFO "%s: Virtual network device closed\n", dev->name);

    return 0;
}

static void virt_net_tx_complete(struct net_device *dev, struct sk_buff *skb)
{
    dev->stats.tx_packets++;
    dev->stats.tx_bytes += skb->len;

    // netif_tx_complete(dev, skb);
    dev_kfree_skb(skb);
}

static netdev_tx_t virt_net_driver_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
    struct virt_net_dev_priv *priv = netdev_priv(dev);
    int ret;

    /* Lock the virtual FIFO buffer */
    spin_lock_bh(&priv->tx_fifo.lock);

    /* Add the skb to the virtual FIFO buffer */
    ret = kfifo_in(&priv->tx_fifo.fifo, &skb, sizeof(skb));
    if (ret != sizeof(skb)) {
        printk(KERN_ERR "%s: Failed to enqueue packet to the virtual FIFO buffer\n", dev->name);

        /* Unlock the virtual FIFO buffer */
        spin_unlock_bh(&priv->tx_fifo.lock);

        /* Update network device statistics */
        dev->stats.tx_dropped++;

        /* Free the skb */
        dev_kfree_skb(skb);

        return NETDEV_TX_BUSY;
    }

    /* Unlock the virtual FIFO buffer */
    spin_unlock_bh(&priv->tx_fifo.lock);

    /* Simulate a successful transmission by calling virt_net_tx_complete */
    virt_net_tx_complete(dev, skb);

    return NETDEV_TX_OK;
}

static int virt_net_driver_set_mac_address(struct net_device *dev, void *addr)
{
    struct sockaddr *sa = addr;

    /* Validate the new MAC address */
    if (!is_valid_ether_addr(sa->sa_data)) {
        printk(KERN_ERR "%s: Invalid MAC address\n", dev->name);
        return -EADDRNOTAVAIL;
    }

    /* Update the device's MAC address */
    memcpy(dev->dev_addr, sa->sa_data, dev->addr_len);

    printk(KERN_INFO "%s: MAC address set to %pM\n", dev->name, dev->dev_addr);

    return 0;
}

static int virt_net_driver_do_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
    int ret = -EOPNOTSUPP;

    /* Handle custom ioctl commands for the virtual network driver */
    switch (cmd) {
        // case VIRT_NET_DRIVER_CUSTOM_CMD1:
        //     ret = handle_custom_cmd1(dev, ifr);
        //     break;
        // case VIRT_NET_DRIVER_CUSTOM_CMD2:
        //     ret = handle_custom_cmd2(dev, ifr);
        //     break;
        default:
            printk(KERN_ERR "%s: Unsupported ioctl command: 0x%x\n", dev->name, cmd);
            break;
    }

    return ret;
}

static int virt_net_driver_cfg80211_scan(struct wiphy *wiphy, struct cfg80211_scan_request *request)
{
    // TODO: Perform a Wi-Fi scan using the virtual network driver and report the scan results
    // using cfg80211_scan_done()

    printk(KERN_INFO "Virtual Wi-Fi scan initiated\n");

    return 0;
}

static int virt_net_driver_cfg80211_connect(struct wiphy *wiphy, struct net_device *dev, struct cfg80211_connect_params *params)
{
    // TODO: Perform a Wi-Fi connection using the virtual network driver and report the connection
    // status using cfg80211_connect_result()

    printk(KERN_INFO "Virtual Wi-Fi connect initiated\n");

    return 0;
}

static int virt_net_driver_cfg80211_disconnect(struct wiphy *wiphy, struct net_device *dev, u16 reason_code)
{
    // TODO: Perform a Wi-Fi disconnection using the virtual network driver and report the disconnection
    // status using cfg80211_disconnected()

    printk(KERN_INFO "Virtual Wi-Fi disconnect initiated\n");

    return 0;
}

module_init(virt_net_driver_init);
module_exit(virt_net_driver_exit);
