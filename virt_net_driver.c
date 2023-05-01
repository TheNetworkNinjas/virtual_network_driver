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

/*
 * Function: get_wiphy_priv
 * ________________________
 * Description: gets private data from wiphy structure
 * Parameter(s): wiphy - a pointer to the wiphy structure
 * Returns: a pointer to a virt_wiphy_priv structure
 */
static struct virt_wiphy_priv* get_wiphy_priv(struct wiphy* wiphy)
{
    return (struct virt_wiphy_priv*) wiphy_priv(wiphy);
}

/*
 * Function: init_virt_hw_resource
 * ________________________
 * Description: initializes the virtual transmit and receive FIFO buffers
 * Parameter(s): dev - a pointer to the net_device structure
 * Returns: 0 on success or an error code if memory allocation fails
 */
static int init_virt_hw_resource(struct net_device *dev)
{
    struct virt_net_dev_priv *priv = netdev_priv(dev); // pointer to network devices private data
    int ret; // variable for return value
    
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

/*
 * Function: release_virt_hw_resource
 * ________________________
 * Description: frees the allocated memory for FIFO buffers and cancels and flushes
the delayed work function
 * Parameter(s): dev - a pointer to the net_device structure
 * Returns: void
 */
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

/*
 * Function: is_tx_fifo_full
 * ________________________
 * Description: checks if the virtual transmit FIFO buffer is full
 * Parameter(s): tx_fifo - pointer to the virt_fifo structure for the transmit FIFO buffer
 * Returns: 1 if the buffer is full, 0 otherwise
 */
static int is_tx_fifo_full(struct virt_fifo *tx_fifo)
{
    return (kfifo_len(&tx_fifo->fifo) / sizeof(struct sk_buff *)) >= MAX_NUM_PACKETS;
}

/*
 * Function: is_rx_fifo_empty
 * ________________________
 * Description: checks if the virtual receive FIFO buffer is empty
 * Parameter(s): tx_fifo - pointer to the virt_fifo structure for the receive FIFO buffer
 * Returns: 1 if the buffer is empty, 0 otherwise
 */
static int is_rx_fifo_empty(struct virt_fifo *rx_fifo)
{
    return kfifo_is_empty(&rx_fifo->fifo);
}

/*
 * Function: virt_net_driver_open
 * ________________________
 * Description: initializes any resources required for the virtual network driver and schedules
delayed work for the driver; also starts the network device's transmit queue
 * Parameter(s): dev - pointer to the net_device structure
 * Returns: 0 on success, or an error code if resource initialization fails
 */
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

/*
 * Function: virt_net_driver_stop
 * ________________________
 * Description: stops the network device's transmit queue and cleans up any resources allocated
 * Parameter(s): dev - pointer to the net_device structure
 * Returns: 0 on success
 */
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

/*
 * Function: virt_net_tx_complete
 * ________________________
 * Description: updates network device statistics and wakes up all the transmit queues for the device; frees the skb
 * Parameter(s): dev - pointer to the net_device structure
 *               skb - pointer to the sk_buff structure for the packet
 * Returns: void
 */
static void virt_net_tx_complete(struct net_device *dev, struct sk_buff *skb)
{
    dev->stats.tx_packets++;
    dev->stats.tx_bytes += skb->len;

    /* Wake up all the transmit queues for the device */
    netif_tx_wake_all_queues(dev);

    /* Free the skb */
    dev_kfree_skb(skb);
}

/*
 * Function: virt_net_driver_start_xmit
 * ________________________
 * Description: locks the virtual transmit FIFO buffer, checks if there is enough space in the buffer for the packet,
 *              adds the skb to the buffer, and updates network device statistics; stops transmit queue and frees the
 *              skb if there is not enough space in the buffer or if the packet fails to enqueue; otherwise unlocks the
 *              buffer and calls virt_net_tx_complete to simulate a successful transmission
 * Parameter(s): dev - pointer to the net_device structure
 *               skb - pointer to the sk_buff structure for the packet
 * Returns: NETDEV_TX_OK if transmission is successful, otherwise NETDEV_TX_BUSY
 */
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

/*
 * Function: virt_net_rx_packet
 * ________________________
 * Description: locks the virtual receive FIFO buffer, checks if there is enough space in the buffer for the packet,
 *              and adds the skb to the buffer; updates network device statistics and frees the skb if there is not enough
 *              space in the buffer or if the packet failed to enqueue; otherwise unlocks the buffer and calls
 * Parameter(s): dev - pointer to the net_device structure
 *               skb - pointer to the sk_buff structure for the packet
 * Returns: void
 */
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

/*
 * Function: virt_net_work_callback
 * ________________________
 * Description: dequeues received packets from the receive FIFO buffer and processes them; reschedules the delayed
 * work for the driver
 * Parameter(s): work - pointer to the work_struct structure for the delayed work
 * Returns: void
 */
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

/*
 * Function: virt_net_driver_set_mac_address
 * ________________________
 * Description: validates and sets the device's MAC address to the new address
 * Parameter(s): dev - pointer to the net-device structure
 * Returns: 0 on success, or an error code if the MAC address is invalid
 */
static int virt_net_driver_set_mac_address(struct net_device *dev, void *addr)
{
    struct sockaddr *sa = addr;

    /* Validate the new MAC address */
    if (!is_valid_ether_addr(sa->sa_data)) {
        printk(KERN_ERR "%s: Invalid MAC address\n", dev->name);
        return -EADDRNOTAVAIL;
    }

    /* Update the device's MAC address */
    memcpy((void*)dev->dev_addr, sa->sa_data, dev->addr_len);

    printk(KERN_INFO "%s: MAC address set to %pM\n", dev->name, dev->dev_addr);

    return 0;
}

/*
 * Function: virt_net_driver_do_ioctl
 * ________________________
 * Description: handles custom ioctl commands for the virtual network driver
 * Parameter(s): dev - pointer to the net-device structure
 *               ifr - pointer to the ifreq structure for the ioctl command
 *               cmd - the ioctl command to handle
 * Returns: 0 on success, or an error code if the iotctl command is unsupported
 */
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

/*
 * Function: inform_bss
 * ________________________
 * Description: Sends BSS information to the kernel
 * Parameter(s): priv - pointer to the net_device private data
 * Returns: void
 */
static void inform_bss(struct virt_net_dev_priv* priv)
{
    printk(KERN_INFO "informing bss\n");
    /* Loop through all APs known in context.  There should only be 1 for our purposes */
    struct virt_net_dev_priv* current_ap;
    list_for_each_entry(current_ap, &context->ap_list, ap_node)
    {
        printk(KERN_INFO "%s: doing info stuff\n", VIRT_NET_DRIVER_NAME);
        struct cfg80211_bss* bss = NULL;
        struct cfg80211_inform_bss inform_bss_data = {
            .signal = (s32) -50,
            .chan = &current_ap->wdev.wiphy->bands[NL80211_BAND_2GHZ]->channels[0],
            .scan_width = NL80211_BSS_CHAN_WIDTH_20,
        };

        u8 *ie = kmalloc(current_ap->ssid_len + 2, GFP_KERNEL);
        ie[0] = WLAN_EID_SSID;
        ie[1] = current_ap->ssid_len;
        memcpy(ie + 2, current_ap->ssid, current_ap->ssid_len);

        u64 tsf = div_u64(ktime_get_boottime_ns(), 1000);
        bss = cfg80211_inform_bss_data (priv->wdev.wiphy, &inform_bss_data, CFG80211_BSS_FTYPE_UNKNOWN, current_ap->bssid, tsf, WLAN_CAPABILITY_ESS, 100, ie, sizeof(ie), GFP_KERNEL);

        cfg80211_put_bss(priv->wdev.wiphy, bss);
        kfree(ie);
    }
}

/*
 * Function: virt_net_driver_cfg_80211_scan
 * ________________________
 * Description: initiates a scan of nearby wireless networks
 * Parameter(s): wiphy - pointer to the wireless device struct
 *               request - pointer to the scan request struct
 * Returns: 0 on success, -ERESTARTSYS if the mutex lock is interrupted, -EBUSY if the scan request is already in progress
 */
static int virt_net_driver_cfg80211_scan(struct wiphy *wiphy, struct cfg80211_scan_request *request)
{
    printk(KERN_INFO "Virtual Wi-Fi scan initiated\n");
    struct virt_net_dev_priv* priv = get_wiphy_priv(wiphy)->wiphy_priv;
    if(mutex_lock_interruptible(&priv->mtx)) {
        return -ERESTARTSYS;
    }

    if (priv->scan_request != NULL) {
        mutex_unlock(&priv->mtx);
        return -EBUSY;
    }

    priv->scan_request = request;

    mutex_unlock(&priv->mtx);

    if (!schedule_work(&priv->ws_scan)) {
        return -EBUSY;
    }
    return 0;
}

/*
 * Function: scan_routine
 * ________________________
 * Description: scans nearby wireless networks
 * Parameter(s): work - pointer to the work struct
 * Returns: void
 */
static void scan_routine(struct work_struct* work)
{
    struct virt_net_dev_priv* priv = container_of(work, struct virt_net_dev_priv, ws_scan);
    struct cfg80211_scan_info info = {
        .aborted = false,
    };
    msleep(100);
    inform_bss(priv);

    if (mutex_lock_interruptible(&priv->mtx)) {
        return;
    }
    cfg80211_scan_done(priv->scan_request, &info);
    priv->scan_request = NULL;
    mutex_unlock(&priv->mtx);
}

/*
 * Function: simulate_assoc_delay
 * ________________________
 * Description: helper function to simulate a random association delay
 * Parameter(s): work - pointer to the work struct
 * Returns: a random delay between 100ms and 500ms
 */
static unsigned int simulate_assoc_delay(void) {
    return 100 + (get_random_u32() % 400);
}

/*
 * Function: simulate_assoc_delay
 * ________________________
 * Description: simulates sending an association request to a wireless network
 * Parameter(s): dev - pointer to the network device struct
                 params - pointer to the connect parameters struct
 * Returns: a random delay between 100ms and 500ms
 */
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

/*
 * Function: virt_net_driver_cfg80211_connect
 * ________________________
 * Description: simulates sending an association request to a wireless network
 * Parameter(s): dev - pointer to the network device struct
                 params - pointer to the connect parameters struct
 * Returns: a random delay between 100ms and 500ms
 */
static int virt_net_driver_cfg80211_connect(struct wiphy *wiphy, struct net_device *dev, struct cfg80211_connect_params *params) {

    printk(KERN_INFO "%s: Starting connection...\n", VIRT_NET_DRIVER_NAME);
    struct virt_net_dev_priv *priv = get_wiphy_priv(wiphy)->wiphy_priv;

    printk(KERN_INFO "%s: Init connect 1\n", VIRT_NET_DRIVER_NAME);

    if(params->ssid == NULL) {
        return -EBUSY;
    }
    printk(KERN_INFO "%s: Init connect 2\n", VIRT_NET_DRIVER_NAME);

    if(mutex_lock_interruptible(&priv->mtx)) {
        return -ERESTARTSYS;
    }
    printk(KERN_INFO "%s: Init connect 3\n", VIRT_NET_DRIVER_NAME);

    memcpy(priv->req_ssid, params->ssid, params->ssid_len);
    printk(KERN_INFO "%s: Init connect 4\n", VIRT_NET_DRIVER_NAME);
    priv->ssid[params->ssid_len] = 0;
    priv->connect_request = params;

    printk(KERN_INFO "%s: Init connect 5\n", VIRT_NET_DRIVER_NAME);
    mutex_unlock(&priv->mtx);

    printk(KERN_INFO "%s: Init connect 6\n", VIRT_NET_DRIVER_NAME);
    if (!schedule_work(&priv->ws_connect)) {
        return -EBUSY;
    }

    return 0;
}

/*
 * Function: connect_routine
 * ________________________
 * Description: this function is called when the connection work is scheduled; it loops through all known access points
 *              in the network and compares their SSIDs to the requested SSID. If it finds a match, it informs the BSS
 *              and simulates an association delay. It updates the state and copies the SSID and BSSID into the priv struct
 * Parameter(s): work - the work struct scheduled to run the connect_routine
 * Returns: void
 */
static void connect_routine(struct work_struct* work)
{
    printk(KERN_INFO "%s: CONNECTION ROUTINE 1\n", VIRT_NET_DRIVER_NAME);
    struct virt_net_dev_priv* priv = container_of(work, struct virt_net_dev_priv, ws_connect);
    if (mutex_lock_interruptible(&priv->mtx)) {
        return;
    }

    struct virt_net_dev_priv* ap = NULL;
    int i = 0;
    list_for_each_entry(ap, &context->ap_list, ap_node)
    {
        printk(KERN_INFO "%s: Looping %d\n", VIRT_NET_DRIVER_NAME, i);
        if (!memcmp(ap->ssid, priv->req_ssid, sizeof(priv->req_ssid))) {
            printk(KERN_INFO "%s: CONNECTION ROUTINE 2\n", VIRT_NET_DRIVER_NAME);

            printk(KERN_INFO "%s: CONNECTION ROUTINE 3\n", VIRT_NET_DRIVER_NAME);

            inform_bss(priv);
            printk(KERN_INFO "%s: CONNECTION ROUTINE 4\n", VIRT_NET_DRIVER_NAME);
            cfg80211_connect_bss(priv->netdev, NULL, NULL, NULL, 0, NULL, 0, WLAN_STATUS_SUCCESS, GFP_KERNEL, NL80211_TIMEOUT_UNSPECIFIED);
            printk(KERN_INFO "%s: CONNECTION ROUTINE 5\n", VIRT_NET_DRIVER_NAME);

            priv->state = VIRT_WIFI_CONNECTED;
            // // cfg80211_connect_result(priv->netdev, ap->bssid, NULL, 0, NULL, 0, WLAN_STATUS_SUCCESS, GFP_KERNEL);
            //
            memcpy(priv->ssid, priv->req_ssid, priv->ssid_len);
            printk(KERN_INFO "%s: CONNECTION ROUTINE 6\n", VIRT_NET_DRIVER_NAME);
            memcpy(priv->bssid, ap->bssid, ETH_ALEN);
            printk(KERN_INFO "%s: CONNECTION ROUTINE 7\n", VIRT_NET_DRIVER_NAME);

            if (mutex_lock_interruptible(&ap->mtx)) {
                mutex_unlock(&priv->mtx);
                return;
            }
            printk(KERN_INFO "%s: CONNECTION ROUTINE 8\n", VIRT_NET_DRIVER_NAME);
            list_add_tail(&priv->bss_list, &ap->bss_list);
            printk(KERN_INFO "%s: CONNECTION ROUTINE 9\n", VIRT_NET_DRIVER_NAME);
            mutex_unlock(&ap->mtx);
            mutex_unlock(&priv->mtx);
            printk(KERN_INFO "%s: CONNECTION ROUTINE 10\n", VIRT_NET_DRIVER_NAME);
            return;
        }
    }

    mutex_unlock(&priv->mtx);
    printk(KERN_INFO "%s: CONNECTION ROUTINE ABOUT TO TIMEOUT\n", VIRT_NET_DRIVER_NAME);
    // cfg80211_connect_timeout(priv->netdev, NULL, NULL, 0, GFP_KERNEL, NL80211_TIMEOUT_SCAN);

}

/*
 * Function: virt_get_station
 * ________________________
 * Description: this function returns information about a station on the network; it iterates over all known interfaces
 *              and checks if any match the requested MAC address; if it finds a match, it returns dummy information
 * Parameter(s): wiphy - pointer to the wiphy struct
                 dev - pointer to the net_device struct
                 net_addr - MAC address of the station
                 info - pointer to the station_info struct
 * Returns: void
 */
static int virt_get_station(struct wiphy* wiphy, struct net_device* dev, const u8* net_addr, struct station_info* info)
{
    struct virt_net_dev_priv* priv = netdev_priv(dev);
    struct virt_net_dev_priv* iterator = NULL;

    // iterate over all known interfaces
    list_for_each_entry(iterator, &context->if_list, if_node)
    {
        // if one matches the requested mac address, return dummy info
        if (!memcmp(net_addr, iterator->netdev->dev_addr, ETH_ALEN)) {
            info->filled = BIT_ULL(NL80211_STA_INFO_TX_PACKETS) | BIT_ULL(NL80211_STA_INFO_RX_PACKETS) | BIT_ULL(NL80211_STA_INFO_TX_FAILED) | BIT_ULL(NL80211_STA_INFO_TX_BYTES) | BIT_ULL(NL80211_STA_INFO_RX_BYTES);
            info->tx_packets = 1;
            info->rx_packets = 1;
            info->tx_failed = 1;
            info->tx_bytes = 1;
            info->rx_bytes = 1;
            return 0;
        }
    }

    // if none match requested mac address, return (machine is not on network)
    return -ENONET;
}

/*
 * Function: simulate_disassoc_delay
 * ________________________
 * Description: this function returns a random disassociation delay to simulate sending a disassociation request and
 *              receiving a response
 * Returns: a random delay in the range of 50ms to 250ms
 */
static unsigned int simulate_disassoc_delay(void) {
    /* Helper function to simulate a random disassociation delay */
    /* Random delay in the range of 50ms to 250ms */
    return 50 + (get_random_u32() % 200);
}

/*
 * Function: virt_net_disconnect
 * ________________________
 * Description: this function disconnects the virtual network device; it simulates a disassociation delay and updates the state to reflect a successful disassociation;
it also releases the associated BSS (if any)
 * Parameter(s): priv - pointer to the virt_net_dev_priv struct
 * Returns: void
 */
static void virt_net_disconnect(struct virt_net_dev_priv *priv) {
    unsigned int disassoc_delay;

    if (priv->state != VIRT_WIFI_CONNECTED) {
        printk(KERN_INFO "Virtual Wi-Fi is not connected, nothing to disconnect\n");
        return;
    }

    /* Simulate the time it takes to send a disassociation request and receive a response */
    disassoc_delay = simulate_disassoc_delay();
    msleep(disassoc_delay);

    /* Update the state to reflect successful disassociation */
    priv->state = VIRT_WIFI_DISCONNECTED;

    if (priv->assoc_bss) {
        cfg80211_put_bss(priv->wiphy, priv->assoc_bss);
        priv->assoc_bss = NULL;
    }

    priv->channel = NULL;
}

/*
 * Function: virt_net_driver_cfg80211_disconnect
 * ________________________
 * Description: this function is called to initiate disconnection of the virtual network device; it schedules the
 *              disconnect work and returns 0 on success
 * Parameter(s): wiphy - pointer to the wiphy struct
 *               dev - pointer to the net_device struct
 *               reason_code - reason for the disconnection
 * Returns: 0 on success, -ERESTARTSYS if interrupted, -EBUSY if work is already schedule
 */
static int virt_net_driver_cfg80211_disconnect(struct wiphy *wiphy, struct net_device *dev, u16 reason_code)
{
    struct virt_net_dev_priv* priv = get_wiphy_priv(wiphy)->wiphy_priv;
    if (mutex_lock_interruptible(&priv->mtx)) {
        return -ERESTARTSYS;
    }

    priv->disconnect_code = reason_code;

    mutex_unlock(&priv->mtx);
    if (!schedule_work(&priv->ws_disconnect)) {
        return -EBUSY;
    }
    return 0;
}

/*
 * Function: disconnect_routine
 * ________________________
 * Description: this function is called as a work routine to initiate disconnection of the virtual network device
 * Parameter(s): work - pointer to the work_struct associated with the disconnect work
 * Returns: void
 */
static void disconnect_routine(struct work_struct* work)
{
    struct virt_net_dev_priv* priv = container_of(work, struct virt_net_dev_priv, ws_disconnect);
    if (mutex_lock_interruptible(&priv->mtx)) {
        return;
    }

    cfg80211_disconnected(priv->netdev, priv->disconnect_code, NULL, 0, true, GFP_KERNEL);
    priv->disconnect_code = 0;
    mutex_unlock(&priv->mtx);
}

/*
 * Function: virt_net_driver_get_drvinfo
 * ________________________
 * Description: this function is called by ethtool to retrieve driver information
 * Parameter(s): dev - pointer to the net_device struct
 *               info - pointer to the ethtool_drvinfo struct to be filled with driver information
 * Returns: void
 */
static void virt_net_driver_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
    strlcpy(info->driver, VIRT_NET_DRIVER_NAME, sizeof(info->driver));
    strlcpy(info->version, VIRT_NET_DRIVER_VERSION, sizeof(info->version));
    snprintf(info->bus_info, sizeof(info->bus_info), "virtual (Vendor: 0x%04X, Device: 0x%04X)", VIRT_NET_VENDOR_ID, VIRT_NET_DEVICE_ID);
}

/*
 * Function: virt_net_driver_get_link
 * ________________________
 * Description: this function is called to get the link state of the virtual network device
 * Parameter(s): dev - pointer to the net_device struct
 *               info - pointer to the ethtool_drvinfo struct to be filled with driver information
 * Returns: 1 to indicate that the link is up, since this is a virtual driver
 */
static uint32_t virt_net_driver_get_link(struct net_device *dev)
{
    return 1;
}

/*
 * Function: virt_net_driver_get_link_ksettings
 * ________________________
 * Description: gets the current link settings of the virtual network device and sets default/fixed values
 * Parameter(s): dev - pointer to the net_device struct
 * Returns: 0 on success
 */
static int virt_net_driver_get_link_ksettings(struct net_device *dev, struct ethtool_link_ksettings *ks)
{
    // For a virtual driver, we can set default/fixed values
    ks->base.speed = SPEED_2500;    // 2.5 Gbps
    ks->base.duplex = DUPLEX_FULL;  // Full duplex
    return 0;
}

/*
 * Function: virt_net_driver_set_link_ksettings
 * ________________________
 * Description: sets the current link settings of the virtual network device; ignores settings since this is a virtual
 *              driver
 * Parameter(s): dev - pointer to the net_device struct
 * Returns: 0 on success
 */
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
    // .ndo_set_mac_address = virt_net_driver_set_mac_address,
    // .ndo_do_ioctl = virt_net_driver_do_ioctl,
};

static const struct ethtool_ops virt_net_ethtool_ops = {
/* Assign the ethtool operations */
    .get_drvinfo = virt_net_driver_get_drvinfo,
    .get_link = virt_net_driver_get_link,
    .get_link_ksettings = virt_net_driver_get_link_ksettings,
    .set_link_ksettings = virt_net_driver_set_link_ksettings,
    // ... other ethtool operations
};

/*
 * Function: virt_if_add
 * ________________________
 * Description: this function is called to create a virtual network interface and add it to the global context
 * Parameter(s): wiphy - pointer to the wiphy struct
 * Returns: 0 on success, -ENOMEM on failure to allocate memory or register the device
 */
static int virt_if_add(struct wiphy* wiphy, int identifier)
{
    int error;

    /* New net device */
    struct net_device* virt_net_dev = NULL;

    /* Private values of new net device */
    struct virt_net_dev_priv *priv = NULL;

    struct virt_wiphy_priv* wpriv = NULL;

    /* Allocating new device */
    virt_net_dev = alloc_netdev(sizeof(struct virt_net_dev_priv), NET_DEV_NAME, NET_NAME_ENUM, ether_setup);
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
    wpriv = get_wiphy_priv(wiphy);
    wpriv->wiphy_priv = priv;
    /* Setting private values */
    priv->netdev = virt_net_dev;

    /* Wireless_dev values */
    priv->wdev.wiphy = wiphy;
    priv->wdev.netdev = virt_net_dev;
    priv->wdev.iftype = NL80211_IFTYPE_STATION;
    priv->netdev->ieee80211_ptr = &priv->wdev;
    priv->scan_request = NULL;
    priv->netdev->features |= NETIF_F_HW_CSUM;

	/* Set the device's name */
    char hw_name[ETH_ALEN + 1] = {0};
    snprintf(hw_name + 1, ETH_ALEN, "%s%d", VIRT_NET_INTF_NAME, identifier);
    eth_hw_addr_set(priv->netdev, hw_name);

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

    memset(priv->bssid, 0, ETH_ALEN);
    memset(priv->ssid, 0, IEEE80211_MAX_SSID_LEN);
    priv->state = VIRT_WIFI_DISCONNECTED;
    priv->ap = NULL;
    /* init mutex and work*/
    mutex_init(&priv->mtx);
    INIT_WORK(&priv->ws_scan, scan_routine);
    INIT_WORK(&priv->ws_connect, connect_routine);
    INIT_WORK(&priv->ws_disconnect, disconnect_routine);

    /* init buffers */
    init_virt_hw_resource(priv->netdev);

    /* Add to interface list */
    mutex_lock(&context->mtx);
    list_add_tail(&priv->if_node, &context->if_list);
    mutex_unlock(&context->mtx);

    // virt_net_driver_open(priv->netdev);

    return 0;
}

/*
 * Function: virt_if_configure
 * ________________________
 * Description: kernel callback for changing interface type; to be used in struct cfg80211_ops
 * Parameter(s): wiphy - pointer to the wiphy struct
 *               dev - pointer to the net_device struct
 *               type - the new interface type
 *               params - additional parameters for the new interface type
 * Returns: 0 on success, -EINVAL if an invalid interface type is provided
 */
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

/*
 * Function: virt_if_delete
 * ________________________
 * Description: deletes a virtual interface and frees all associated memory
 * Parameter(s): priv - pointer to the virt_net_dev_priv struct representing the virtual interface
 * Returns: 0 on success, -EINVAL if invalid argument provided
 */
static int virt_if_delete(struct virt_net_dev_priv* priv)
{
    struct wiphy* wiphy = priv->wdev.wiphy;

    // stop AP if device in an AP
    if (priv->is_ap) {
        ap_terminate(wiphy, priv->netdev, 0);
    } 

    cancel_work_sync(&priv->ws_scan);
    cancel_work_sync(&priv->ws_connect);
    cancel_work_sync(&priv->ws_disconnect);

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

/* FullMAC driver functions
 * Represents functionalities of the wiphy device
 */
static struct cfg80211_ops wifi_dev_ops = {
    .change_virtual_intf = virt_if_configure,
    .scan = virt_net_driver_cfg80211_scan,
    .connect = virt_net_driver_cfg80211_connect,
    .disconnect = virt_net_driver_cfg80211_disconnect,
    .get_station = virt_get_station,
    .start_ap = ap_init,
    .stop_ap = ap_terminate,
};

/* Supported channels for wifi device */
/* https://en.wikipedia.org/wiki/List_of_WLAN_channels */
static struct ieee80211_channel supported_channels[] = {
    {.band = NL80211_BAND_2GHZ, .hw_value = 1,  .center_freq = 2412,},
    {.band = NL80211_BAND_2GHZ, .hw_value = 2,  .center_freq = 2417,},
    {.band = NL80211_BAND_2GHZ, .hw_value = 3,  .center_freq = 2422,},
    {.band = NL80211_BAND_2GHZ, .hw_value = 4,  .center_freq = 2427,},
    {.band = NL80211_BAND_2GHZ, .hw_value = 5,  .center_freq = 2432,},
    {.band = NL80211_BAND_2GHZ, .hw_value = 6,  .center_freq = 2437,},
    {.band = NL80211_BAND_2GHZ, .hw_value = 7,  .center_freq = 2442,},
    {.band = NL80211_BAND_2GHZ, .hw_value = 8,  .center_freq = 2447,},
    {.band = NL80211_BAND_2GHZ, .hw_value = 9,  .center_freq = 2452,},
    {.band = NL80211_BAND_2GHZ, .hw_value = 10, .center_freq = 2457,},
    {.band = NL80211_BAND_2GHZ, .hw_value = 11, .center_freq = 2462,},
    {.band = NL80211_BAND_2GHZ, .hw_value = 12, .center_freq = 2467,},
    {.band = NL80211_BAND_2GHZ, .hw_value = 13, .center_freq = 2472,},
    {.band = NL80211_BAND_2GHZ, .hw_value = 14, .center_freq = 2487,},
};

/* Supported rates for wifi device */
static struct ieee80211_rate supported_rates[] = {
    {.bitrate = 10,  .hw_value = 0x1,},
    {.bitrate = 20,  .hw_value = 0x2,},
    {.bitrate = 55,  .hw_value = 0x4,},
    {.bitrate = 110, .hw_value = 0x8,},
    {.bitrate = 60,  .hw_value = 0x10,},
    {.bitrate = 90,  .hw_value = 0x20,},
    {.bitrate = 120, .hw_value = 0x40,},
    {.bitrate = 180, .hw_value = 0x80,},
    {.bitrate = 240, .hw_value = 0x100,},
    {.bitrate = 360, .hw_value = 0x200,},
    {.bitrate = 480, .hw_value = 0x400,},
    {.bitrate = 540, .hw_value = 0x800,},
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

    // allocate new wiphy structure
    wiphy = wiphy_new_nm(&wifi_dev_ops, 0, NULL);
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

/*
 * Function: ap_init
 * ________________________
 * Description: callback to initialize an interface as an AP
 * Parameter(s): dev - pointer to the net_device struct
 *               ap_settings - pointer to the cfg80211_ap_settings struct containing access point settings
 * Returns: 0 on success, negative error code on failure
 */
static int ap_init(struct wiphy* wiphy, struct net_device* dev, struct cfg80211_ap_settings* ap_settings)
{
    struct virt_net_dev_priv* priv = netdev_priv(dev);

    // set ssid and bssid
    priv->ssid_len = ap_settings->ssid_len;
    memcpy(priv->ssid, ap_settings->ssid, ap_settings->ssid_len);
    memcpy(priv->bssid, priv->netdev->dev_addr, ETH_ALEN);
    printk(KERN_INFO "%s: bssid changed: %x\n", priv->bssid);

    // add to ap list
    list_add_tail(&priv->ap_node, &context->ap_list);

    // initialize bss list 
    INIT_LIST_HEAD(&priv->bss_list);

    priv->is_ap = true;

    return 0;
}

/*
 * Function: ap_terminate
 * ________________________
 * Description: callback to terminate an interface as an AP
 * Parameter(s): dev - pointer to the net_device struct
 *               wiphy - pointer to the wiphy struct
 *               link_id - identifier for the link
 * Returns: 0 on success, -ENOMEM on error
 */
static int ap_terminate(struct wiphy* wiphy, struct net_device* dev, unsigned int link_id)
{
    struct virt_net_dev_priv* priv = netdev_priv(dev);

    /* Check if device is an AP */
    if (!priv->is_ap) {
        printk(KERN_ERR "%s: Attempting to terminate AP of a non-AP interface\n", VIRT_NET_DRIVER_NAME);
        // maybe could use a different error code
        return -ENOMEM;
    }

    /* Stop beaconing */
    // we don't really need to do this because we are not really scanning?

    /* Delete BSS list */
    // loop through bss list
    struct virt_net_dev_priv *list_pos = NULL, *tmp = NULL;
    list_for_each_entry_safe(list_pos, tmp, &priv->bss_list, bss_list)
    {
        list_del(&tmp->bss_list);
    }

    /* Remove from context's ap_list */
    mutex_lock(&context->mtx);
    list_del(&priv->ap_node);
    mutex_unlock(&context->mtx);

    /* set interface to non AP */
    priv->is_ap = false;
    return 0;
}

/*
 * Function: virt_net_driver_init
 * ________________________
 * Description:  Initializes the virtual network driver by allocating and initializing the adapter context, creating
 *               virtual interfaces, and adding them to the global context
 * Parameter(s): none
 * Returns: 0 on success, -ENOMEM if failed to allocate adapter context or create virtual interfaces
 */
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

/*
 * Function: virt_net_driver_exit
 * ________________________
 * Description:  function called when unloading the virtual network driver; deletes each virtual interface and frees
 *               the context
 * Parameter(s): none
 * Returns: void
 */
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
