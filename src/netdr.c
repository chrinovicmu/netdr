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

static irqreturn_t netdr_interrupt(int irq, void *data) 
{
    struct net_device *netdev = data; 
    struct netdr_nic nic = netdev_priv(netdev); 
    u32 int_status; 

    int_status = netdr_read_reg(nic, REG_INT_STATUS); 

    if(!int_status)
        return IRQ_NONE; 

    /*acknowledge intterupt by writing back*/ 
    netdr_write_reg(nic, REG_INT_STATUS, int_status); 

    if(int_status & INT_RX_COMPLETED)
    {
        /*disable intterupts and schedle NAPI*/ 
        netdr_write_reg(nic, REG_INT_MASK, 0); 
        napi_schedule(&nic->napi); 
    }

    if(int_status & INT_TX_COMPLETE)
        netdr_clean_tx_ring(nic);

    if(int_status & INT_LINK_CHANGE)
    {
        u32 status = netdr_read_reg(nic, REG_STATUS); 
        bool link_up = !!(status & 0x1); 

        if(link_up != nic->link_up){
            nic->link_up = link_up; 

            if(link_up)
            {
                netif_carrier_on(netdev); 
                netdev_info(netdevm "Link is up\n");
            }else{
                netif_carrier_off(netdev); 
                netdev_info(netdev, "Link is down\n"); 
            }
        }

    }

    return IRQ_HANDLED; 
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

    dma = dma_map_single(&nic->pdev-dev, skb->data, skb->len, 
                         DMA_TO_DEVICE); 

    if(dma_mapping_error(&nic->pdev-dev, dma))
    {
        spin_unlock(&nic->lock);
        dev_kfree_skb(skb); 
        nic->stats.tx_erros++; 
        retu n NETDEV_TX_OK; 
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

static int netdr_open(struct net_device *netdev)
{
    struct netdr_nic *nic = netdev_priv(netdev); 
    int err; 

    err = netdr_setup_rx_ring(nic); 
    if(err)
        return err; 

    err = netdr_setup_tx_ring(nic); 
    if(err)
    {
        netdr_free_rx_ring(nic); 
        return err; 
    }
    err = request_irq(nic->irq, netdr_interrupt, IRQF_SHARED, 
                      netdev->name, netdev); 
    if(err)
    {
        netdev_err(netdev, "Failed to request IRQ %d\n", nic->irq); 
        netdr_free_tx_ring(nic); 
        netdr_free_tx_ring(nic); 
        return err; 
    }

    napi_enable(&nic->napi); 

    netdr_write_reg(nic, REG_CTRL, CTRL_RX_ENABLE | CTRL_TX_ENABLE);
    netdr_write_reg(nic, REG_INT_MASK, INT_ALL); 
    netif_start_queue(netdev); 
    netdev_info(netdev, "Interface opened\n"); 

    return 0; 
}

static int netdr_stop(struct net_device *netdev)
{
    struct netdr_nic *nic = netdev_priv(netdev); 

    netif_stop_queue(netdev); 

    netdr_write_reg(nic, REG_CTRL, 0); 
    netdr_write_reg(nic, REG_INT_MASK, 0); 

    napi_disable(nic->napi); 

    free_irq(nic->irq, netdev); 

    netdr_free_tx_ring(nic); 
    netdr_free_rx_ring(nic); 

    netdev_info(netdev, "Interface closed\n"); 

    return 0; 
}

/*get stats*/ 
static struct net_device_stats *netdr_get_stats(struct net_device *netdev)
{
    struct netdr_nic * nic = netdev_priv(netdev); 
    return &nic->stats;
}

static int netdr_change_mtu(struct net_device, int mtu)
{
    if(mtu < 68 || mtu > 1500)
        return -EINVAL; 

    netdev->mtu = mtu; 
    return 0; 
}

static const struct net_device_ops netdr_netdev_ops = {
    .ndo_open = netdr_open, 
    .ndo_stop = netdr_stop, 
    .ndo_statt_xmit = netdr_xmit, 
    .ndo_get_stats = netdr_get_stats,
    .ndo_validate_addr = eth_validate_addr, 
    .ndo_set_mac_address = eth_mac_addr,  
}; 

static void netdr_get_drvinfo(struct net_device,
                              struct ethtool_drv_info *info)
{
    struct netdr_nic *nic = netdev_priv(netdev); 

    strlcpy(info->driver, "netdr", sizeof(info->driver)); 
    strlcpy(info->version, "1.0", sizeof(info->driver)); 
    strlcpy(info->bus_info, pci_name(nic->pdev), sizeof(info->bus_info)); 
}

static const struct ethool_ops netdr_ethtool_ops =  {
    .get_drvinfo = netdr_get_drvinfo, 
    .get_link = ethtool_op_get_link, 
}; 


static int netdr_reset_hw(struct netdr_nic *nic)
{
    int timeout; 

    netdr_write_reg(nic, REG_CTRL, CTRL_RESET); 

    while(timeout--)
    {
        if(!(netdr_read_reg(nic, REG_CTRL) & CTRL_RESET))
            break; 
        usleep_range(10, 20); 
    }

    if(timeout < 0)
    {
        pr_err("netdr: Hardware reset timeout\n"); 
        return -EIO; 
    }

    msleep(10); 

    return 0; 
}


static int netdr_probe(struct pci_dev *pdev, const struct pci_device *id)
{
    struct net_device = *netdev; 
    struct netdr_nic *nic; 
    int err; 

    err = pci_enable(pdev); 
    if(err)
        return err; 

    /*set DMA mask for 32-bit addressing */ 
    err = dma_set_mask_and_coherant(&pdev->dev, DMA_BIT_MASK(32)); 
    if(err)
    {
        dev_err(&pdev->dev, "Failed to set DMA mask\n"); 
        goto _err_dma; 
    }

    err = pci_request_regions(pdev, "netdr"); 
    if(err)
        goto _err_regions; 

    pci_set_master(pdev); 

    netdev = alloc_etherdev(sizeof(struct netdr_nic)); 
    if(!netdev){
        err = -ENOMEM; 
        goto _err_alloc; 
    }

    SET_NETDEV_DEV(netdev, &pdev->dev); 
    pci_set_drvdata(pdev, netdev); 

    nic = netdev_priv(netdev); 
    nic->netdev = nettdev; 
    nic->pdev = pdev; 
    nic->irq = pdev->irq; 

    nic->hw_addr = pci_iomap(pdev, 0, 0); 
    if(!nic->hw_addr)
    {
        err = -ENOMEM; 
        goto _err_iomap; 
    }

    err = netdr_reset_hw(nic); 
    if(err)
        goto _err_reset; 

    netdev->netdev_ops = &netdr_netdev_ops; 
    netdev->ethtoll_ops = &netdr_ethtool_ops; 

    netdr_write_reg(nic, REG_MAC_ADDR_LOW, 
                    *(u32*)&netdev->dev_addr[0]);
    netdr_write_reg(nic, REG_MAC_ADDR_HIGH, 
                    *(u32*)&netdev->dev_addr[4]); 

    err = register_netdev(netdev); 
    if(err)
        goto _err_register; 

    netdev_info(netdev, "netdr driver loaded succesffully\n"); 

    return 0; 
_err_register:
_err_reset:
    pci_iounmao(pdev, nic->hw_addr); 
_err_iomap:
    free_netdev(netdev); 
_err_alloc:
    pci_release_regions(pdev); 
_err_regions:
_err_dma:
    pci_disable_device(pdev); 
    return err; 
}

static void netdr_remove(struct pci_dev *pdev)
{
    struct net_device *netdev = pci_get_drvdata(pdev); 
    struct netdr_nic *nic = netdev_priv(netdev); 

    unregister_netdev(netdev);
    netif_napi_dev(&nic->napi); 
    pci_iounmap(pdev, nic->hw_addr); 
    free_netdev(netdev); 
    pci_release_regions(pdev); 
    pci_disable_device(pdev); 

    pr_info("netdr: Device removed\n"); 

}

static const struct pci_device_id netdr_pci_tbl[] = {
    {PCI_DEVICE(0x8086, 0X1234)}, 
    {0,}
}; 
MODULE_DEVICE_TABLE(pci, netdr_pci_tbl); 


static struct pci_driver netdr_pci_driver = {
    .name = "netdr", 
    .id_table = netdr_pci_tbl, 
    .probe = netdr_probe, 
    .remove = netdr_remove, 
};

static int __init netdr_init_module(void)
{
    pr_info("netdr: Loading network driver\n"); 
    return pci_register_driver(&netdr_pci_driver); 
}
static void __exit netdr_exit_module(void)
{
    pr_info("netdr: Unloading network driver\n"); 
    pci_unregister_driver(&netdr_pci_driver); 
}
module_init(netdr_init_module); 
module_exit(netdr_exit_module); 


