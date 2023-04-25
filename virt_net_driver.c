#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/kfifo.h>
#include <linux/list.h>
#include "virt_net_driver.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Craig Opie");
MODULE_AUTHOR("Jake Imanaka");
MODULE_AUTHOR("Lydia Sollis");
MODULE_DESCRIPTION("Virtual network driver for Linux");
MODULE_VERSION("0.01");

static struct net_device *virt_net_dev;
static struct virt_adapter_context *context;


static int init_virt_hw_resource(struct net_device *dev)
{
    struct virt_net_dev_priv *priv = netdev_priv(dev);
    int ret;

    /* Initialize the virtual transmit FIFO buffer */
    spin_lock_init(&priv->tx_fifo.lock);
    ret = kfifo_alloc(&priv->tx_fifo.fifo, VIRT_FIFO_SIZE, GFP_KERNEL);
    if (ret) {
        printk(KERN_ERR "%s: Failed to allocate virtual transmit FIFO buffer\n", dev->name);
        return ret;
    }

    /* Initialize the virtual receive FIFO buffer */
    spin_lock_init(&priv->rx_fifo.lock);
    ret = kfifo_alloc(&priv->rx_fifo.fifo, VIRT_FIFO_SIZE, GFP_KERNEL);
    if (ret) {
        printk(KERN_ERR "%s: Failed to allocate virtual receive FIFO buffer\n", dev->name);
        kfifo_free(&priv->tx_fifo.fifo);
        return ret;
    }

    return 0;
}

static void release_virt_hw_resource(struct net_device *dev)
{
    struct virt_net_dev_priv *priv = netdev_priv(dev);

    /* Cancel the delayed work */
    cancel_delayed_work(&priv->work);

    /* Ensure the work function has completed */
    flush_delayed_work(&priv->work);

    /* Release the virtual FIFO buffer */
    kfifo_free(&priv->tx_fifo.fifo);
}

static int is_tx_fifo_full(struct virt_fifo *tx_fifo)
{
    return (kfifo_len(&tx_fifo->fifo) / sizeof(struct sk_buff *)) >= MAX_NUM_PACKETS;
}

static int is_rx_fifo_empty(struct virt_fifo *rx_fifo)
{
    return kfifo_is_empty(&rx_fifo->fifo);
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

    /* Initialize the netdev field */
    priv->netdev = dev;

    /* Initialize delayed work */
    INIT_DELAYED_WORK(&priv->work, virt_net_work_callback);

    /* Schedule the delayed work */
    schedule_delayed_work(&priv->work, msecs_to_jiffies(1000));

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

    printk(KERN_INFO "%s: Virtual network device closed\n", dev->name);

    return 0;
}

static void virt_net_tx_complete(struct net_device *dev, struct sk_buff *skb)
{
    dev->stats.tx_packets++;
    dev->stats.tx_bytes += skb->len;

    /* Wake up all the transmit queues for the device */
    netif_tx_wake_all_queues(dev);

    /* Free the skb */
    dev_kfree_skb(skb);
}

static netdev_tx_t virt_net_driver_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
    struct virt_net_dev_priv *priv = netdev_priv(dev);
    int ret;

    /* Lock the virtual FIFO buffer */
    spin_lock_bh(&priv->tx_fifo.lock);

    /* Check if there is enough space in the buffer */
    if (kfifo_avail(&priv->tx_fifo.fifo) < sizeof(skb)) {
        printk(KERN_ERR "%s: Not enough space in the virtual transmit FIFO buffer\n", dev->name);

        /* Unlock the virtual FIFO buffer */
        spin_unlock_bh(&priv->tx_fifo.lock);

        /* Update network device statistics */
        dev->stats.tx_fifo_errors++;

        /* Stop the network device's transmit queue */
        netif_stop_queue(dev);

        /* Free the skb */
        dev_kfree_skb(skb);

        return NETDEV_TX_BUSY;
    }

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

static void virt_net_rx_packet(struct net_device *dev, struct sk_buff *skb)
{
    struct virt_net_dev_priv *priv = netdev_priv(dev);
    int ret;

    /* Lock the virtual receive FIFO buffer */
    spin_lock_bh(&priv->rx_fifo.lock);

    /* Check if there is enough space in the buffer */
    if (kfifo_avail(&priv->rx_fifo.fifo) < sizeof(skb)) {
        printk(KERN_ERR "%s: Not enough space in the virtual receive FIFO buffer\n", dev->name);

        /* Unlock the virtual receive FIFO buffer */
        spin_unlock_bh(&priv->rx_fifo.lock);

        /* Update network device statistics */
        dev->stats.rx_dropped++;

        /* Free the skb */
        dev_kfree_skb(skb);

        return;
    }

    /* Add the skb to the virtual receive FIFO buffer */
    ret = kfifo_in(&priv->rx_fifo.fifo, &skb, sizeof(skb));
    if (ret != sizeof(skb)) {
        printk(KERN_ERR "%s: Failed to enqueue packet to the virtual receive FIFO buffer\n", dev->name);

        /* Unlock the virtual receive FIFO buffer */
        spin_unlock_bh(&priv->rx_fifo.lock);

        /* Update network device statistics */
        dev->stats.rx_dropped++;

        /* Free the skb */
        dev_kfree_skb(skb);

        return;
    }

    /* Unlock the virtual receive FIFO buffer */
    spin_unlock_bh(&priv->rx_fifo.lock);
}

static void virt_net_work_callback(struct work_struct *work)
{
    struct virt_net_dev_priv *priv = container_of(work, struct virt_net_dev_priv, work.work);
    struct net_device *dev = priv->netdev;
    struct sk_buff *skb;
    int ret;

    /* Increment the counter */
    priv->counter++;

    /* Dequeue the received packets from the receive FIFO buffer */
    while (!is_rx_fifo_empty(&priv->rx_fifo)) {
        spin_lock_bh(&priv->rx_fifo.lock);
        ret = kfifo_out(&priv->rx_fifo.fifo, &skb, sizeof(skb));
        spin_unlock_bh(&priv->rx_fifo.lock);

        if (ret != sizeof(skb)) {
            printk(KERN_ERR "%s: Failed to dequeue packet from the virtual receive FIFO buffer\n", dev->name);
            break;
        }

        /* Process the received packet */
        virt_net_rx_packet(dev, skb);
    }

    /* Reschedule the delayed work */
    schedule_delayed_work(&priv->work, msecs_to_jiffies(1000));
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

    /* ptr to wiphy device's private data structure */
    struct vnet_device * dev = netdev_priv(wiphy);

    /* ptr to virt network device's private data structure */
    struct virt_net_dev_priv * priv = netdev_priv(dev);
    struct cfg80211_scan_info scan_info = {};

    /* Update scan parameters */
    memcpy(&priv->scan_request->ssids, &request->ssids, sizeof(request->ssids));
    priv->scan_request->n_ssids = request->n_ssids;

    /* Initiate scan */
    printk(KERN_INFO "%s: Initiating scan\n", dev->name);
    schedule_work(&priv->scan_work);

    /* Report scan results with cfg80211_scan_done */
    if (scan_info != NULL) {
        scan_info.aborted = false;
        cfg80211_scan_done(priv->scan_request, &scan_info);
    } else {
        scan_info.aborted = true;
        cfg80211_scan_done(request, &scan_info);
        return -EAGAIN;
    }

    printk(KERN_INFO "Virtual Wi-Fi scan initiated\n");

    return 0;
}

static int virt_net_driver_cfg80211_connect(struct wiphy *wiphy, struct net_device *dev, struct cfg80211_connect_params *params)
{
    // TODO: Perform a Wi-Fi connection using the virtual network driver and report the connection
    // status using cfg80211_connect_result()

    /* ptr to virt network device's private data structure */
    struct virt_net_dev_priv * priv = netdev_priv(dev);

    /* check if the virtual network device is already connected */
    if(priv->connected) {
        printk(KERN_ERR "%s: Virtual network device is already connected\n", dev->name);
    }

    /* Check if the requested SSID matches the virtual network device's SSID */
    if(strncmp(params->ssid, priv->ssid, ETH_ALEN != 0)) {
        printk(KERN_ERR "%s: Requested SSID does not match the virtual network device's SSSID\n", dev->name);
        return -EINVAL;
    }

    /* Check if requested channel matches virtual network device's channel */
    if (params->channel->hw_value != priv->channel) {
        printk(KERN_ERR, "%s: Requested channel does not match virtual network device's channel\n", dev->name);
        return -EINVAL;
    }

    /* Set virtual network device's BSSID to AP's MAC address */
    memcpy(priv->bssid, params->bssid, ETH_ALEN);

    printk(KERN_INFO "Virtual Wi-Fi connect initiated\n");

    /* Report the connection status using cfg80211_connect_result() */
    cfg80211_connect_result(dev, params->bssid, params->ie, params->ie_len, WLAN_STATUS_SUCCESS);

    return 0;
}

static int virt_net_driver_cfg80211_disconnect(struct wiphy *wiphy, struct net_device *dev, u16 reason_code)
{
    // TODO: Perform a Wi-Fi disconnection using the virtual network driver and report the disconnection
    // status using cfg80211_disconnected()

    /* ptr to virt network device's private data structure */
    struct virt_net_dev_priv * priv = netdev_priv(dev);

    /* check if the virtual network device is already connected */
    if(!priv->connected) {
        printk(KERN_ERR "%s: Virtual network device is not connected\n", dev->name);
    }

    // TODO: Perform Wi-Fi disconnection using the virtual network driver

    priv->connected = false;

    cfg80211_disconnected(priv->wdev.netdev, reson_code, NULL, 0, true, GFP_KERNEL);

    printk(KERN_INFO "Virtual Wi-Fi disconnect initiated\n");

    return 0;
}

static void virt_net_driver_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
    strlcpy(info->driver, VIRT_NET_DRIVER_NAME, sizeof(info->driver));
    strlcpy(info->version, VIRT_NET_DRIVER_VERSION, sizeof(info->version));
    snprintf(info->bus_info, sizeof(info->bus_info), "virtual (Vendor: 0x%04X, Device: 0x%04X)", VIRT_NET_VENDOR_ID, VIRT_NET_DEVICE_ID);
}

static uint32_t virt_net_driver_get_link(struct net_device *dev)
{
    // Return 1 if the link is up, 0 if it's down
    // This is a virtual driver, so we can return 1 for the link to always be up
    return 1;
}

static int virt_net_driver_get_link_ksettings(struct net_device *dev, struct ethtool_link_ksettings *ks)
{
    // Get the current link settings, e.g., speed, duplex, etc.
    // For a virtual driver, we can set default/fixed values
    ks->base.speed = SPEED_1000;    // 1 Gbps
    ks->base.duplex = DUPLEX_FULL;  // Full duplex
    return 0;
}

static int virt_net_driver_set_link_ksettings(struct net_device *dev, const struct ethtool_link_ksettings *ks)
{
    // Set the link settings based on the provided ethtool_link_ksettings struct
    // As this is a virtual driver, we can ignore the settings and return success
    return 0;
}

static int __init virt_net_driver_init(void)
{
    int ret;

    /* Allocate and initialize adapter context */ 
	printk(KERN_INFO "%d\n", sizeof(context));
    context = kmalloc(sizeof(struct virt_adapter_context), GFP_KERNEL);
    if(!context) {
      printk(KERN_ERR "%s: Failed to allocate adapter_context\n", VIRT_NET_DRIVER_NAME);
      return -ENOMEM;
    }

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
	static const struct ethtool_ops virt_net_ethtool_ops = {
		.get_drvinfo = virt_net_driver_get_drvinfo,
		.get_link = virt_net_driver_get_link,
		.get_link_ksettings = virt_net_driver_get_link_ksettings,
		.set_link_ksettings = virt_net_driver_set_link_ksettings,
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

module_init(virt_net_driver_init);
module_exit(virt_net_driver_exit);
