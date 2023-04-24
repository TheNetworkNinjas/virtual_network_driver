#ifndef _VIRT_NET_DRIVER_H_
#define _VIRT_NET_DRIVER_H_

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/wireless.h>
#include <net/cfg80211.h>
#include <linux/kfifo.h>
#include <linux/list.h>

/* Constants */
#define VIRT_NET_DRIVER_NAME "virt_net_driver"
#define VIRT_NET_DRIVER_VERSION "0.01"
#define VIRT_FIFO_SIZE 4096
#define VIRT_NET_DRIVER_MTU 1500
#define VIRT_NET_VENDOR_ID 0x10ec // Realtek Semiconductor Corp.
#define VIRT_NET_DEVICE_ID 0x8125 // RTL8125 2.5GbE Controller

// Virtual FIFO buffer for packet transmission
struct virt_fifo {
    struct kfifo fifo;
    spinlock_t lock;
};

/* Virtual Network Device Private Data */
struct virt_net_dev_priv {
    struct net_device *netdev;
    struct cfg80211_scan_request *scan_request;
    struct cfg80211_connect_params *connect_params;
    struct work_struct scan_work;
    struct work_struct connect_work;
    struct wireless_dev wdev;
    struct delayed_work work;
    struct virt_fifo tx_fifo;
    unsigned long counter;
};

/* Program context */
struct virt_adapter_context {
    struct list_head ap_list;   // List of access points
    struct list_head if_list;   // List of virtual interfaces
    spinlock_t       lock;      // Lock for modifying program context
};

/* Function Prototypes */
static int __init virt_net_driver_init(void);
static void __exit virt_net_driver_exit(void);
static int init_virt_hw_resource(struct net_device *dev);
static void release_virt_hw_resource(struct net_device *dev);

/* Kernel Interaction Functions */
static void virt_net_driver_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info);
static uint32_t virt_net_driver_get_link(struct net_device *dev);
static int virt_net_driver_get_link_ksettings(struct net_device *dev, struct ethtool_link_ksettings *ks);
static int virt_net_driver_set_link_ksettings(struct net_device *dev, const struct ethtool_link_ksettings *ks);

/* Network Device Operations */
static int virt_net_driver_open(struct net_device *dev);
static int virt_net_driver_stop(struct net_device *dev);
static void virt_net_tx_complete(struct net_device *dev, struct sk_buff *skb);
static netdev_tx_t virt_net_driver_start_xmit(struct sk_buff *skb, struct net_device *dev);
static void virt_net_rx_packet(struct net_device *dev, struct sk_buff *skb);
static void virt_net_work_callback(struct work_struct *work);
static int virt_net_driver_set_mac_address(struct net_device *dev, void *addr);
static int virt_net_driver_do_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd);

/* Wireless Operations */
static int virt_net_driver_cfg80211_scan(struct wiphy *wiphy, struct cfg80211_scan_request *request);
static int virt_net_driver_cfg80211_connect(struct wiphy *wiphy, struct net_device *dev, struct cfg80211_connect_params *params);
static int virt_net_driver_cfg80211_disconnect(struct wiphy *wiphy, struct net_device *dev, u16 reason_code);

#endif /* _VIRT_NET_DRIVER_H_ */
