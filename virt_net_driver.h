#ifndef _VIRT_NET_DRIVER_H_
#define _VIRT_NET_DRIVER_H_

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/wireless.h>
#include <linux/kfifo.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/prandom.h>
#include <net/cfg80211.h>

/* Constants */
#define VIRT_NET_DRIVER_NAME "virt_net_driver"
#define VIRT_NET_DRIVER_VERSION "0.01"
#define MAX_NUM_PACKETS 128
#define VIRT_FIFO_SIZE 4096
#define VIRT_NET_DRIVER_MTU 1500
#define VIRT_NET_VENDOR_ID 0x10ec   // Realtek Semiconductor Corp.
#define VIRT_NET_DEVICE_ID 0x8125   // RTL8125 2.5GbE Controller
#define VIRT_NET_INTF_NAME "vif"
#define NET_DEV_NAME VIRT_NET_INTF_NAME "%d"
#define MAX_IF_NUM 3

/* Virtual FIFO buffer for packet transmission */
struct virt_fifo {
    struct kfifo fifo;
    spinlock_t lock;
};

/* Virtual WiFi State */
enum virt_wifi_state {
    VIRT_WIFI_DISCONNECTED,
    VIRT_WIFI_ASSOCIATING,
    VIRT_WIFI_CONNECTED
};

/* Virtual Network Device Private Data */
struct virt_net_dev_priv {
    struct net_device *netdev;
    struct wiphy *wiphy;
    struct cfg80211_scan_request *scan_request;
    struct cfg80211_connect_params *connect_params;
    struct work_struct scan_work;
    struct work_struct connect_work;
    struct wireless_dev wdev;
    struct delayed_work work;
    struct virt_fifo tx_fifo;
    struct virt_fifo rx_fifo;
    unsigned long counter;
    struct list_head if_node;
    struct mutex mtx;
    struct cfg80211_bss *assoc_bss;
    struct ieee80211_channel *channel;
    enum virt_wifi_state state;
    u8 ssid[IEEE80211_MAX_SSID_LEN];
    size_t ssid_len;
    u8 bssid[ETH_ALEN];
    // list of all interfaces for a bss as head
    struct list_head bss_list;
    // AP node for global AP list
    struct list_head ap_node;
    bool is_ap;
};

/* Virtual WiFi Wiphy Private Data */
struct virt_wifi_wiphy_priv {
    struct mutex scan_mutex;
};

/* Program context */
struct virt_adapter_context {
    struct mutex mtx;
    struct list_head ap_list;   // List of access points
    struct list_head if_list;   // List of virtual interfaces
};

/* Function Prototypes */
static int __init virt_net_driver_init(void);
static void __exit virt_net_driver_exit(void);
static int init_virt_hw_resource(struct net_device *dev);
static void release_virt_hw_resource(struct net_device *dev);
static int is_tx_fifo_full(struct virt_fifo *tx_fifo);
static int is_rx_fifo_empty(struct virt_fifo *rx_fifo);

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

/* Wireless Operations Helper Functions */
static unsigned int simulate_assoc_delay(void);
static unsigned int simulate_disassoc_delay(void);
static void virt_net_disconnect(struct virt_net_dev_priv *priv);
static int virt_wifi_send_assoc(struct net_device *dev, struct cfg80211_connect_params *params);

/* Wireless Operations */
static int virt_net_driver_cfg80211_scan(struct wiphy *wiphy, struct cfg80211_scan_request *request);
static int virt_net_driver_cfg80211_connect(struct wiphy *wiphy, struct net_device *dev, struct cfg80211_connect_params *params);
static int virt_net_driver_cfg80211_disconnect(struct wiphy *wiphy, struct net_device *dev, u16 reason_code);

/* Interface Configuration Operatins */
static int virt_if_add(struct wiphy* wiphy, int identifier);
static int virt_if_configure(struct wiphy* wiphy, struct net_device *dev, enum nl80211_iftype type, struct vif_params* params);
static int virt_if_delete(struct virt_net_dev_priv* priv);

/* Wiphy Configuration Options */
static struct wiphy* wiphy_add(void);

/* Access Point Options */
static int ap_init(struct wiphy*, struct net_device* dev, struct cfg80211_ap_settings*);

#endif /* _VIRT_NET_DRIVER_H_ */
