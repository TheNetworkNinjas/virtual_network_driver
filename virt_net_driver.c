/*
* virt_net_driver - Virtual network adapter for Linux
*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/nl80211.h>
#include <linux/workqueue.h> // work struct
#include <semaphore.h>
#include "virt_net_driver.h"

#define SSID "PrettyFly4aWiFi"
#define SSID_DEMO_SIZE (sizeof("PrettyFly4aWiFi") - 1)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Craig Opie");
MODULE_AUTHOR("Jake Imanaka");
MODULE_AUTHOR("Lydia Sollis");
MODULE_DESCRIPTION("Virtual network driver for Linux");
MODULE_VERSION("0.01");

typedef unsigned short u16;

/* virtual driver context */
struct virt_net_driver_context {
    struct net_device *ndev; // pointer to network device associated with the driver
    struct wiphy *wiphy; // pointer to wiphy structure that represents the hardware the device is operating on
    /* DEMO FEATURES */
    struct semaphone sem; // synchronize access to shared resources between threads
    struct work_struct ws_connect; // work item to connect to wireless network
    struct work_struct ws_disconnect; // work item to disconnect from wireless network
    struct work_struct ws_scan; // work item to scan for available network
    char connecting_ssid[sizeof(SSID)]; // char array to store SSID
    u16 disconnect_reason_code; // int to store reason for disconnecting from the wireless network
    struct cfg80211_scan_request *scan_request; // pointer to structure that represents the parameters of the scan request
};

/* wiphy private data */
struct virt_net_driver_wiphy_priv_context {
    struct virt_net_driver_context *virtContext;
};

/* private data of net_device */
struct virt_net_driver_ndev_priv_context {
    struct virt_net_driver_context *virtContext;
    struct wireless_dev wirelessDev; // represents wireless device together with net_device
};

static struct net_device *virt_net_dev;
static struct virt_net_driver_context *context;

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
    // TODO: Fill in necessary fields and function pointers in virt_net_dev

    /* Allocate and initialize net_device_driver_context */
    context = kzalloc(sizeof(struct struct virt_net_driver_context), GFP_KERNEL);
    if (!context) {
        printk(KERN_ERR "%s: Failed to allocate context\n", VIRT_NET_DRIVER_NAME);
        return -ENOMEM;
    }

    /* Initialize context fields */
    // TODO: Fill in necessary fields for context

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
    if (context == NULL) {
        return;
    }
    /* Unregister the network device */
    unregister_netdev(virt_net_dev);

    /* Free the net_device memory */
    free_netdev(virt_net_dev);

    /* free driver context */
    wiphy_unregister(context->wiphy);
    wiphy_free(context->wiphy);
    kfree(context);

    printk(KERN_INFO "%s: Virtual network driver unloaded\n", VIRT_NET_DRIVER_NAME);
}

static int virt_net_driver_open(struct net_device *dev)
{
    struct virt_net_dev_priv *priv = netdev_priv(dev);

    /* Initialize any resources required for the virtual network driver */
    // TODO: Initialize resources here

    /* Start the network device's transmit queue */
    netif_start_queue(dev);

    printk(KERN_INFO "%s: Virtual network device opened\n", dev->name);

    return 0;
}

static int virt_net_driver_stop(struct net_device *dev)
{
    /* Stop the network device's transmit queue */
    netif_stop_queue(dev);

    /* Cleanup any resources allocated for the virtual network driver */
    // TODO: Cleanup resources here

    printk(KERN_INFO "%s: Virtual network device closed\n", dev->name);

    return 0;
}

static netdev_tx_t virt_net_driver_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
    /* Process and transmit the packet using the virtual network driver */
    // TODO: Transmit packet here

    /* Update network device statistics */
    dev->stats.tx_packets++;
    dev->stats.tx_bytes += skb->len;

    /* Free the skb */
    dev_kfree_skb(skb);

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
