#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/kfifo.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/prandom.h>
#include <net/cfg80211.h>
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
    kfifo_free(&priv->rx_fifo.fifo);
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
    int i;

    printk(KERN_INFO "Virtual Wi-Fi scan initiated\n");

    /* Simulate a Wi-Fi scan with some fake access points */
    for (i = 0; i < request->n_ssids; i++) {
        struct cfg80211_bss *bss;
        struct ieee80211_channel *channel;
        uint8_t bssid[ETH_ALEN] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
        uint32_t signal = -30;  // dBm
        struct ieee80211_mgmt mgmt = {};

        /* Use the first channel from the request for the fake access point */
        channel = request->channels[0];

        /* Set the BSSID for the fake BSS */
        memcpy(mgmt.bssid, bssid, ETH_ALEN);

        /* Set the frame control field to indicate a beacon frame */
        mgmt.frame_control = cpu_to_le16(IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_BEACON);

        /* Set the timestamp field for the fake BSS */
        mgmt.u.beacon.timestamp = cpu_to_le64(ktime_get_real_ns());

        /* Create a fake BSS */
        bss = cfg80211_inform_bss_frame(wiphy, channel, &mgmt, sizeof(mgmt), signal, GFP_KERNEL);
        if (!bss) {
            printk(KERN_ERR "Failed to create a fake BSS\n");
            continue;
        }

        /* Notify the cfg80211 subsystem about the new BSS */
        cfg80211_put_bss(wiphy, bss);
    }

    /* Notify the cfg80211 subsystem that the scan is complete */
    cfg80211_scan_done(request, false);

    printk(KERN_INFO "Virtual Wi-Fi scan complete\n");

    return 0;
}

static unsigned int simulate_assoc_delay(void) {
    /* Helper function to simulate a random association delay */
    /* Random delay in the range of 100ms to 500ms */
    return 100 + (get_random_u32() % 400);
}

static int virt_wifi_send_assoc(struct net_device *dev, struct cfg80211_connect_params *params) {
    struct virt_net_dev_priv *netdev_priv_data = netdev_priv(dev);
    unsigned int assoc_delay;

    /* Simulate the time it takes to send an association request and receive a response */
    assoc_delay = simulate_assoc_delay();
    msleep(assoc_delay);

    /* Update the state to reflect successful association */
    netdev_priv_data->state = VIRT_WIFI_CONNECTED;

    /* Optionally, you can notify the upper layers about the association event */
    cfg80211_connect_result(dev, params->bssid, params->ie, params->ie_len, NULL, 0, WLAN_STATUS_SUCCESS, GFP_KERNEL);

    /* Return 0 to indicate success */
    return 0;
}

static int virt_net_driver_cfg80211_connect(struct wiphy *wiphy, struct net_device *dev, struct cfg80211_connect_params *params) {
    struct virt_wifi_wiphy_priv *priv = wiphy_priv(wiphy);
    struct virt_net_dev_priv *netdev_priv_data = netdev_priv(dev);
    struct cfg80211_bss *bss;
    struct ieee80211_channel *channel;
    int err;

    if (!params->bssid || !params->ssid) {
        return -EINVAL;
    }

    mutex_lock(&priv->scan_mutex);
    bss = cfg80211_get_bss(wiphy, NULL, params->bssid, params->ssid, params->ssid_len, IEEE80211_BSS_TYPE_ANY, IEEE80211_PRIVACY_ANY);
    mutex_unlock(&priv->scan_mutex);

    if (!bss) {
        return -ENOENT;
    }

    channel = bss->channel;
    if (channel->flags & IEEE80211_CHAN_DISABLED) {
        cfg80211_put_bss(wiphy, bss);
        return -EINVAL;
    }

    netdev_priv_data->assoc_bss = bss;
    netdev_priv_data->channel = channel;
    netdev_priv_data->state = VIRT_WIFI_ASSOCIATING;

    err = virt_wifi_send_assoc(dev, params);
    if (err) {
        cfg80211_put_bss(wiphy, bss);
        netdev_priv_data->assoc_bss = NULL;
        netdev_priv_data->channel = NULL;
        netdev_priv_data->state = VIRT_WIFI_DISCONNECTED;
        return err;
    }

    cfg80211_connect_result(dev, params->bssid, params->ie, params->ie_len, NULL, 0, WLAN_STATUS_SUCCESS, GFP_KERNEL);

    return 0;
}

static int virt_net_driver_cfg80211_disconnect(struct wiphy *wiphy, struct net_device *dev, uint16_t reason_code)
{
    // TODO: Perform a Wi-Fi disconnection using the virtual network driver and report the disconnection
    // status using cfg80211_disconnected()

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
    ks->base.speed = SPEED_2500;    // 2.5 Gbps
    ks->base.duplex = DUPLEX_FULL;  // Full duplex
    return 0;
}

static int virt_net_driver_set_link_ksettings(struct net_device *dev, const struct ethtool_link_ksettings *ks)
{
    // Set the link settings based on the provided ethtool_link_ksettings struct
    // As this is a virtual driver, we can ignore the settings and return success
    return 0;
}

/* Assign the net_device operations */
static const struct net_device_ops virt_net_dev_ops = {
    .ndo_open = virt_net_driver_open,
    .ndo_stop = virt_net_driver_stop,
    .ndo_start_xmit = virt_net_driver_start_xmit,
    .ndo_set_mac_address = virt_net_driver_set_mac_address,
    .ndo_do_ioctl = virt_net_driver_do_ioctl,
};

static const struct ethtool_ops virt_net_ethtool_ops = {
/* Assign the ethtool operations */
    .get_drvinfo = virt_net_driver_get_drvinfo,
    .get_link = virt_net_driver_get_link,
    .get_link_ksettings = virt_net_driver_get_link_ksettings,
    .set_link_ksettings = virt_net_driver_set_link_ksettings,
    // ... other ethtool operations
};

/* Create virtual interface and add to global context */
static int virt_if_add(struct wiphy* wiphy, int identifier)
{
    int error;

    /* New net device */
    struct net_device* virt_net_dev = NULL;

    /* Private values of new net device */
    struct virt_net_dev_priv *priv = NULL;

    /* Allocating new device */
    virt_net_dev = alloc_netdev(sizeof(struct virt_net_dev_priv), NET_DEV_NAME, NET_NAME_ENUM, ether_setup);
    // virt_net_dev = alloc_netdev(sizeof(struct virt_net_dev_priv), NET_DEV_NAME, NET_NAME_ENUM, ether_setup);
    // virt_net_dev = alloc_etherdev(sizeof(struct virt_net_dev_priv));
    if (!virt_net_dev) {
        printk(KERN_ERR "%s: Failed to allocate net_device\n", VIRT_NET_DRIVER_NAME);

        /* Free wiphy object if necessary */
        if (priv->wiphy) {
            wiphy_unregister(priv->wiphy);
            wiphy_free(priv->wiphy);
        }
        return -ENOMEM;
    }
    else {
        printk(KERN_INFO "%s: private virtual net_device allocated\n", VIRT_NET_DRIVER_NAME);
    }

    /* Getting priv values */
    priv = netdev_priv(virt_net_dev);

    /* Setting private values */
    priv->netdev = virt_net_dev;

    /* Wireless_dev values */
    priv->wdev.wiphy = wiphy;
    // STA by default
    priv->wdev.iftype = NL80211_IFTYPE_STATION;
    priv->netdev->ieee80211_ptr = &priv->wdev;
    
    priv->netdev->features |= NETIF_F_HW_CSUM;

	/* Set the device's name */
    char name[ETH_ALEN];
    snprintf(name, ETH_ALEN, "%s%d", VIRT_NET_INTF_NAME, identifier);
    strlcpy(virt_net_dev->name, name, sizeof(virt_net_dev->name));
    memcpy((void*) priv->netdev->dev_addr, name, ETH_ALEN);

    /* Assign ops */
    virt_net_dev->netdev_ops = &virt_net_dev_ops;
    virt_net_dev->ethtool_ops = &virt_net_ethtool_ops;

    /* Register device */ 
    error = register_netdev(priv->netdev);
    if (error) {
        printk(KERN_ERR "%s: Failed to register net_device\n", VIRT_NET_DRIVER_NAME);
        free_netdev(priv->netdev);
        wiphy_unregister(wiphy);
        wiphy_free(wiphy);
        return -ENOMEM;
    }

    /* init mutex */
    mutex_init(&priv->mtx);

    /* init buffers */
    init_virt_hw_resource(priv->netdev);

    /* init connection info */ 
    /* init timers */

    /* Add to interface list */
    mutex_lock(&context->mtx);
    list_add_tail(&priv->if_node, &context->if_list);
    mutex_unlock(&context->mtx);

    // virt_net_driver_open(priv->netdev);

    return 0;
}

/* kernel callback for changing interface type */
/* to be used in struct cfg80211_ops */
static int virt_if_configure(struct wiphy* wiphy, struct net_device* dev, enum nl80211_iftype type, struct vif_params* params)
{
    switch (type) {
        case NL80211_IFTYPE_STATION:
            dev->ieee80211_ptr->iftype = NL80211_IFTYPE_STATION;
            break;
        case NL80211_IFTYPE_AP:
            dev->ieee80211_ptr->iftype = NL80211_IFTYPE_AP;
            break;
        default:
            printk(KERN_ERR "%s: Failed to change interface -- Invalid interface type [%u]\n", VIRT_NET_DRIVER_NAME, type);
            return -EINVAL;
    }
    return 0;
}

/* FullMAC driver functions
 * Represents functionalities of the wiphy device
 */
static struct cfg80211_ops wifi_dev_ops = {
    .change_virtual_intf = virt_if_configure,
    .scan = virt_net_driver_cfg80211_scan,
    .connect = virt_net_driver_cfg80211_connect,
    .disconnect = virt_net_driver_cfg80211_disconnect,
};

/* Supported channels for wifi device */
static struct ieee80211_channel supported_channels[] = {
    {.band = NL80211_BAND_2GHZ, .hw_value = 1, .center_freq = 1412,},
    // add more channels here if needed
};

/* Supported rates for wifi device */
static struct ieee80211_rate supported_rates[] = {
    {.bitrate = 10, .hw_value = 0x1,},
    // add more rates here if needed
};

/* Supported bands for wifi device */
static struct ieee80211_supported_band supported_bands = {
    .ht_cap.cap = IEEE80211_HT_CAP_SGI_20,
    .ht_cap.ht_supported = false,
    .bitrates = supported_rates,
    .n_bitrates = ARRAY_SIZE(supported_rates),
    .channels = supported_channels,
    .n_channels = ARRAY_SIZE(supported_channels),
};

/* create a wiphy device */
static struct wiphy* wiphy_add(void)
{
    int error = 0;
    struct wiphy* wiphy = NULL;

    // setting default wiphy options 

    // allocate new wiphy structure
    wiphy = wiphy_new(&wifi_dev_ops, 0);
    if (!wiphy) {
        printk(KERN_ERR "%s: Failed to allocate new wiphy device\n", VIRT_NET_DRIVER_NAME);
        return NULL;
    }

    // type of interface
    // AP or STA
    wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION) | BIT(NL80211_IFTYPE_AP);

    // supported bands
    wiphy->bands[NL80211_BAND_2GHZ] = &supported_bands;

    // flags
    wiphy->flags |= WIPHY_FLAG_NETNS_OK;

    wiphy->max_scan_ssids = 10;

    // regsiter wiphy 
    error = wiphy_register(wiphy);
    if (error < 0) {
        printk(KERN_ERR "%s: Failed to register new wiphy device\n", VIRT_NET_DRIVER_NAME);
        wiphy_free(wiphy);
        return NULL;
    }

    return wiphy;
}

/* Delete virtual interface and free */
static int virt_if_delete(struct virt_net_dev_priv* priv)
{
    struct wiphy* wiphy = priv->wdev.wiphy;
    //TODO: Error checking -- but I'm just happy it works for now ;_;
    mutex_lock(&priv->mtx);
    // stop transfer queues and queued work
    netif_stop_queue(priv->netdev);

    // stop pending scans
    // TODO: after scan is implemented

    mutex_unlock(&priv->mtx);

    // unregister device
    unregister_netdev(priv->netdev);
    // free device
    free_netdev(priv->netdev);

    // unregister wiphy dev
    wiphy_unregister(wiphy);
    // free wiphy
    wiphy_free(wiphy);

    // free wiphy device
    return 0;
}


static int __init virt_net_driver_init(void)
{
    printk(KERN_INFO "%s: Attempting to load virtual network driver...\n", VIRT_NET_DRIVER_NAME);
    int ret;

    /* Allocate and initialize adapter context */ 
    context = kmalloc(sizeof(struct virt_adapter_context), GFP_KERNEL);
    if(!context) {
      printk(KERN_ERR "%s: Failed to allocate adapter_context\n", VIRT_NET_DRIVER_NAME);
      return -ENOMEM;
    }
    else {
        printk(KERN_INFO "%s: Adapter_context allocated\n", VIRT_NET_DRIVER_NAME);
    }

    printk(KERN_INFO "%s: Initializing context mutex and lists\n", VIRT_NET_DRIVER_NAME);
    /* init global lists */
    mutex_init(&context->mtx);
    INIT_LIST_HEAD(&context->ap_list);
    INIT_LIST_HEAD(&context->if_list);
    printk(KERN_INFO "%s: Context mutex and lists initialized\n", VIRT_NET_DRIVER_NAME);

    printk(KERN_INFO "%s: Creating interfaces\n", VIRT_NET_DRIVER_NAME);
    for (int i = 0; i < MAX_IF_NUM; i++)
    {
        struct wiphy* wiphy = wiphy_add();
        if (wiphy == NULL) {
            return -ENOMEM;
        }
        virt_if_add(wiphy, i);
    }
    printk(KERN_INFO "%s: Interfaces created\n", VIRT_NET_DRIVER_NAME);


    printk(KERN_INFO "%s: Virtual network driver loaded\n", VIRT_NET_DRIVER_NAME);
    return 0;
}

static void __exit virt_net_driver_exit(void)
{

    /* Free each virtual interface */ 
    struct virt_net_dev_priv *priv = NULL, *tmp = NULL;
    list_for_each_entry_safe(priv, tmp, &context->if_list, if_node)
    {
        virt_if_delete(priv);
    }

    /* Free context */
     kfree(context);


    printk(KERN_INFO "%s: Virtual network driver unloaded\n", VIRT_NET_DRIVER_NAME);
}

module_init(virt_net_driver_init);
module_exit(virt_net_driver_exit);
