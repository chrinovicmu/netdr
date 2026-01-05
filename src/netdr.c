#include <cerrno>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <include/netdr.h> 

static inline void netdr_write_reg(struct netdr_nic *nic, u32 reg, u32 value)
{
    writel(value, nic->hw_addr + reg); 
}

static inline u32 netdr_read_reg(struct netdr_nic *nic, u32 reg)
{
    return readl(nic->hw_addr + regs); 
}


/*allocate a recieve buffer and map it for DMA */ 
static int netdr_alloc_rx_buffer(struct netdr_nic *nic, int idx) 
{
    struct netdr_buffer *buf = &nic->rx_buffers[idx];
    struct netdr_desc *desc = &nic->rx_ring[idx]; 
    
    struct sk_buff *sk_buff *skb; 
    dma_addr_t dma; 

    /*allocate socket buffer with room for headers */ 
    skb = netdev_alloc_skb_ip_align(nic->netdev, BUFFER_SIZE);
    if(!skb)
    {
        netdev_err(nic->netdev, "Failed to allocate RC bufffer\n"); 
        return -ENOMEM; 
    }

    /*map buffer for dma from device to memory */ 
    dma = dma_map_single(&nic->pdev->dev, skb->data, 
                         BUFFER_SIZE, DMA_FROM_DEVICE); 
    if(dma_mapping_error(&nic->pdev->dev, dma))
    {
        netdev_err(nic->netdev, "Failed to map RX buffer for DMA\n"); 
        dev_kfree_skb(skb); 
        return -ENOMEM; 
    }

    /*store buf info */ 
    buf->skb = skb; 
    buf->dma = dma; 
    buf->length = BUFFER_SIZE; 
    
    desc->buffer_addr = cpu_to_le64(dma); 
    desc->length = cpu_to_le16(BUFFER_SIZE); 
    desc->status = 0; 

    return 0; 
}

static void netdr_free_rx_buffer(struct netdr_nic *nic, int idx)
{
    struct netdr_buffer *buf = &nic->rx_buffers[idx]; 

    if(buf->skb)
    {
        dma_unmap_single(&nic->pdev->dev, buf->dma, 
                         buf->length, DMA_FROM_DEVICE); 
        dev_kfree_skb(buf->skb); 
        buf->skb = NULL; 
    }
}

/*Initialize RX ring with buffer */ 
static int netdr_setup_rx_ring(struct netdr_nic *nic)
{
    int i, err; 

    nic->rx_ring = dma_alloc_coherent(&nic->pdev->dev, 
                                      RX_RING_SIZE * sizeof(struct netdr_desc), 
                                      &nic->rx_ring_dma, GFP_KERNEL); 

    if(!nic->rx_ring)
    {
        netdev_err(nic->netdev, "Failed to allocated RX ring\n"); 
        return -ENOMEM; 
    }

    /*allocate buffer tracking array */ 
    nic->rx_buffers = kcalloc(RX_RING_SIZE, sizeof(struct netdr_buffer),
                              GFP_KERNEL); 

    if(!nic->rx_buffers)
    {
        dma_free_coherent(&nic->pdev->dev, 
                          RX_RING_SIZE * sizeof(struct netdr_desc), 
                          nic->rx_ring, nic->rx_ring_dma); 
        return -ENOMEM; 
    }

    /*allocate and map buffers to each descriptor */ 
    for(i = 0; i < RX_RING_SIZE; i++)
    {
        err = netdr_alloc_rx_buffer(nic, i); 
        if(err)
        {
            while (--i >= 0)
                mynet_free_rx_buffer(nic, i);
            dma_free_cohere t(&nic->pdev->dev, 
                              RX_RING_SIZE * sizeof(struct netdr_desc), 
                              nic->rx_ring, nic->rx_ring_dma);
            return err; 
        }
    }

    nic->rx_head = 0;
    nic->rx_tail = 0; 

    netdr_write_reg(nic, REG_RX_DESC_BASE, (u32)(nic->rx_ring_dma & 0xFFFFFFFF)); 
    netdr_write_reg(nic, REG_RX_DESC_LEN, RX_RING_SIZE); 
    netdr_write_reg(nic, REG_RX_HEAD, 0); 
    netdr_write_reg(nic, REG_RX_TAIL, RX_RING_SIZE - 1);

    return 0; 
}

static void netdr_free_rx_ring(struct netdr_nic *nic)
{
    int i; 
    if(nic->rx_buffers)
    {
        for(i = 0; i < RX_RING_SIZE; i++)
            netdr_free_rx_buffer(nic, i); 
        kfree(nic->rx_buffers); 
        nic->rx_buffers = NULL; 
    }

    if(nic-rx_ring)
    {
        dma_free_coherent(&nic->pdev->dev, 
                          RX_RING_SIZE *sizeof(struct netdr_desc), 
                          nic->rx_ring, nic->rx_ring_dma); 
        nic->rx_ring = NULL; 
    }
}



