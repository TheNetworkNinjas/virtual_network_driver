#ifndef _VIRT_NET_DRIVER_H_
#define _VIRT_NET_DRIVER_H_

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/wireless.h>
#include <net/cfg80211.h>
#include <linux/kfifo.h>

/* Constants */
#define VIRT_NET_DRIVER_NAME "virt_net_driver"
#define VIRT_NET_DRIVER_VERSION "0.01"
#define VIRT_FIFO_SIZE 4096

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
    struct timer_list timer;
    struct virt_fifo tx_fifo;
    unsigned long counter;
};

/* Function Prototypes */
static int __init virt_net_driver_init(void);
static void __exit virt_net_driver_exit(void);
static int init_virt_hw_resource(struct net_device *dev);
static void release_virt_hw_resource(struct net_device *dev);

/* Network Device Operations */
static void virt_net_timer_callback(struct timer_list *t);
static int virt_net_driver_open(struct net_device *dev);
static int virt_net_driver_stop(struct net_device *dev);
static void virt_net_tx_complete(struct net_device *dev, struct sk_buff *skb);
static netdev_tx_t virt_net_driver_start_xmit(struct sk_buff *skb, struct net_device *dev);
static int virt_net_driver_set_mac_address(struct net_device *dev, void *addr);
static int virt_net_driver_do_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd);

/* Wireless Operations */
static int virt_net_driver_cfg80211_scan(struct wiphy *wiphy, struct cfg80211_scan_request *request);
static int virt_net_driver_cfg80211_connect(struct wiphy *wiphy, struct net_device *dev, struct cfg80211_connect_params *params);
static int virt_net_driver_cfg80211_disconnect(struct wiphy *wiphy, struct net_device *dev, u16 reason_code);

#endif /* _VIRT_NET_DRIVER_H_ */