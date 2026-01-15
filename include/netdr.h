#ifndef NETDR_H 
#define NETDR_H

#include <linux/netdevice.h>
#include <linux/etherdevice.h> 
#include <linux/skbuff.h> 
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>


/*hardware register offsets */ 
#define REG_CTRL            0x0000
#define REG_STATUS          0x0004
#define REG_INT_MASK        0x0008 
#define REG_INT_STATUS      0x000C 
#define REG_TX_DESC_BASE    0x0010 
#define REG_TX_DESC_LEN     0x0014
#define REG_TX_HEAD         0x0018 
#define REG_TX_TAIL         0x001C 
#define REG_RX_DESC_BASE    0x0020 
#define REG_RX_DESC_LEN     0x0024
#define REG_RX_HEAD         0x0028 
#define REG_RX_TAIL         0x002C
#define REG_MAC_ADDR_LOW    0x0030 
#define REG_MAC_ADDR_HIGH   0x0034 

#define CTRL_RESET          (1 << 0)
#define CTRL_TX_ENABLE      (1 << 1)
#define CTRL_RX_ENABLE      (1 << 2) 

#define INT_TX_COMPLETE     (1 << 0)
#define INT_RX_COMPLETE     (1 << 1)
#define INT_LINK_CHANGE     (1 << 2) 
#define INT_ALL             (INT_TX_COMPLETE | INT_RX_COMPLETE | INT_LINK_CHANGE)

#define TX_RING_SIZE        256 
#define RX_RING_SIZE        256 
#define BUFFER_SIZE         2048 


struct netdr_desc 
{
    __le64 buffer_addr; 
    __le16 length; 
    __le16 checksum ;
    __le32 status; 
}__attribute__((packed)); 

/*descriptor status bits */ 
#define DESC_STATUS_DD      (1 << 0)
#define DESC_STATUS_EOP     (1 << 1)
#define DESC_STATUS_ERR     (1 << 2)

/*TX/RX bufferr tracking stucture 
* tracks addtional information for each buffer in the ring */ 
struct netdr_buffer
{
    struct sk_buff *skb; 
    dma_addr_t dma; 
    unsigned int length; 
};

struct netdr_nic 
{
    struct net_device *netdev; 
    struct pci_dev *pdev; 
    void __iomem *hw_addr; 

    struct netdr_desc *tx_ring; 
    dma_addr_t tx_ring_dma; 
    struct netdr_buffer *tx_buffers; 
    unsigned int tx_head; 
    unsigned int tx_tail; 
    spinlock_t tx_lock; 

    struct netdr_desc *rx_ring;
    dma_addr_t rx_ring_dma; 
    struct netdr_buffer *rx_buffers; 
    unsigned int rx_head; 
    unsigned int rx_tail; 

    /*poll-based packet reception to reduce interrupt overhead*/ 
    struct napi_struct napi;

    struct net_device_stats stats; 

    bool link_up; 
    int irq; 
}; 



#endif 
