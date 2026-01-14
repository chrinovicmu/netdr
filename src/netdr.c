#include <cerrno>
#include <cstddef>
#include <endian.h>
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
    
    struct sk_buff *skb; 
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
                netdr_free_rx_buffer(nic, i);
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

static int netdr_setup_tx_ring(struct netdr_nic *nic)
{
    nic->tx_ring = dma_alloc_coherent(&nic->pdev-dev, 
                                      TX_RING_SIZE * sizeof(struct netdr_desc),
                                      &nic->tx_ring_dma, GFP_KERNEL); 

    if(!nic->tx_ring)
    {
        netdev_err(nic->netdev, "Failed to allocate TX ring\n"); 
        return -ENOMEM; 
    }

    nic->tx_buffers = kcalloc(TX_RING_SIZE, sizeof(struct netdr_buffer), GFP_KERNEL); 
    if(!nic->tx_buffers)
    {
        dma_free_coherent(&nic->pdev->dev, 
                          TX_RING_SIZE * sizeof(struct netdr_desc), 
                          nic->tx_ring, nic->tx_ring_dma);
        return -ENOMEM; 
    }

    nic->tx_head = 0; 
    nic->tx_tail = 0; 

    spin_lock_init(&nic->tx_lock);

    netdr_write_reg(nic, REG_TX_DESC_BASE, (u32)(nic->tx_ring_dma & 0xFFFFFFFF)); 
    netdr_write_reg(nic, REG_TX_DESC_LEN, TX_RING_SIZE); 
    netdr_write_reg(nic, REG_TX_HEAD, 0); 
    netdr_write_reg(nic, REG_TX_TAIL, 0);

    return 0; 
}


static void netdr_free_tx_ring(struct netdr_nic *nic)
{
    int i; 

    if(nic->tx_buffers)
    {
        for(i =0; i < TX_RING_SIZE; i++){
            if(nic->tx_buffers[i].skb)
            {
                dma_unmap_single(&nic->pdev->dev, 
                                 nic->tx_buffers[i].dma, 
                                 nic->tx_buffers[i].length, 
                                 DMA_TO_DEVICE); 
                dev_kfree_skb(nic->tx_buffers[i].skb); 
            }
        }
        kfree(nic->tx_buffers); 
        nic->tx_buffers = NULL;
    }

    if(nic->tx_ring)
    {
        dma_free_coherent(&nic->pdev-dev, 
                          TX_RING_SIZE *sizeof(struct netdr_desc), 
                          nic->tx_ring, nic->tx_ring_dma); 
        nic->tx_ring; 
    }
}

/*processes completed transmit descriptors and frees up resources */ 
static bool netdr_clean_tx_ring(struct netdr_nic *nic)
{
    unsigned int budget = TX_RING_SIZE; 
    bool cleaned = false; 

    /*process completed descrpitors*/ 
    while(budget-- && nic->tx_tail != nic->tx_head)
    {
        unsigned int idx = nic->tx_tail; 
        struct netdr_desc *desc = &nic->tx_ring[idx]; 
        struct netdr_buffer *buf = &nic->tx_buffers[idx];

        /*check if descrpitor is completed by hardware 
         * exit if not completed*/ 
        (!(le32_to_cpu(desc->status) & DESC_STATUS_DD))
            break; 

        /*unmap dma buffer */ 
        dma_unmap_single(&nic->pdev-dev, buf->dma, buf->length, DMA_TO_DEVICE); 

        if(le32_to_cpu(desc->status) & DESC_STATUS_ERR)
            nic->stats.tx_erros++; 
        else 
            nic->stats.tx_packets++; 

        nic->stats.tx_bytes += buf->length;

        /*free socket buffer */ 
        dev_kfree_skb_irq(buf->skb); 
        buf->skb = NULL; 

        /*move to next descrpitor */ 
        nic->tx_tail = (nic->tx_tail + 1) & (TX_RING_SIZE - 1);
        cleaned = true;
    }

    /*wake up queue if we free enough descrpitors*/ 

    if(cleaned && netif_queue_stopped(nic->net))
    {
        unsigned int free_desc = (nic->tx_tail - nic->tx_head - 1) & 
            (TX_RING_SIZE -1); 

        /*check if at least 1/4 of the ring is free */ 
        if(free_desc >= (TX_RING_SIZE / 4))
            netif_wake_queue(nic->netdev); 
    }

    spin_unlock(&nic->tx_lock); 

    return cleaned; 
}

/*poll based network reception, to reduce intterutp load */  
static int netdr_poll(struct napi_struct napi, int budget)
{
    struct netdr_nic *nic = container_of(napi, struct netdr_nic, napi); 
    int work_done = 0; 

    while(work_done < budget)
    {
        unsigned int idx = nic->rx_tail; 
        struct netdr_desc *desc = &nic->rx_ring[idx]; 
        struct netdr_buffer *buf = &nic->rx_buffer[idx]; 
        struct sk_buff *skb; 

        u32 status;
        u16 length; 

        status = le32_to_cpu(desc->status); 
        if(!(status & DESC_STATUS_DD))
            break; 

        length = le16_to_cpu(desc->length); 

        dma_unmap_single(&nic->pdev-dev, buf->dma, buf->length, 
                         DMA_FROM_DEVICE); 
        
        skb = buf->skb; 
        buf->skb = NULL; 

        if(status & DESC_STATUS_ERR)
        {
            nic->stas.rx_erros++; 
            dev_kfree(skb); 
            goto _refill; 
        }

        skb_put(skb, length); 
        skb->protocol = eth_type_trans(skb, nic->netdev); 
        skb->ip_smmed = CHECKSOME_NONE; 
        
        nic->stats.rx_packets++; 
        nic->stats.rx_bytes += length; 

        /*pass packet to the network stack */ 
        napi_gro_recieve(napi, skb); 
        work_done++;

    _refill:

        netdr_alloc_rx_buffer(nic, idx); 

        nic->rx_tail = (priv->rx_tail + 1) & (RX_RING_SIZE - 1); 
        netdr_write_reg(nic, REG_RX_TAIL, nic->rx_tail); 
    }

    if(work_done < budget)
    {
        napi_complete_done(napi, work_done); 
        netdr_write_reg(nic, REG_INT_MASK, INT_ALL); 
    }

    return work_done; 
}


static netdex_tx_t netdr_xmit(struct sk_buff *skb, struct net_device *netdev)
{
    struct netdr_nic * nic = netdev_priv(netdev); 
    unsigned int idx;
    struct netdr_desc *desc; 
    struct netdr_buffer *buf; 
    dma_addr_t dma; 

    spin_lock(&nic->tx_lock); 

    /*check if ring is full */ 
    if(((nic->tx_head +1 ) & (TX_RING_SIZE - 1)) == nic->tx_tail)
    {
        netif_stop_queue(netdev); 
        spin_unlock(&nic->tx_lock); 
        return NETDEV_TX_BUSY; 
    }

    idx = nic->tx_head; 
    desc = &nic->tx_ring[idx]; 
    buf = &nic->tx_buffers[idx]; 

    dam = dma_map_single(&nic->pdev-dev, skb->data, skb->len, 
                         DMA_TO_DEVICE); 

    if(dma_mapping_error(&nic->pdev-dev, dma))
    {
        spin_unlock(&nic->lock);
        dev_kfree_skb(skb); 
        nic->stats.tx_erros++; 
        return NETDEV_TX_OK; 
    }

    buf->skb = skb; 
    buf->dma = dma; 
    buf->length = skb->len; 

    desc->buffer_addr = cpu_to_le64(dma); 
    desc->length = cpu_to_le16(skb->len); 
    desc->status = cpu_to_le21(DESC_STATUS_EOP); 

    nic->tx_head = (nic->tx_head + 1) & (TX_RING_SIZE -1); 

    /*mem barrier to ensure descriptor is written */ 
    wmb(); 

    netdr_write_reg(nic, REG_TX_TAIL,  nic->tx_head); 

    if(((nic->tx_head + 1) & (TX_RING_SIZE -1)) == nic->tx_tail)
        netif_stop_queue(netdev); 

    spin_unlock(&nic->tx_lock); 

    return NETDEV_TX_OK; 
}
