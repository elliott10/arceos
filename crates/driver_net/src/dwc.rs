// compatible = "starfive,dwmac", "snps,dwmac-5.10a"
// refer to:
// u-boot/drivers/net/dwc_eth_qos.c
// u-boot/drivers/net/phy/motorcomm.c

pub const ethernet0: u32 = 0x16030000;
pub const ethernet1: u32 = 0x16040000;

pub const serial2: u32 = 0x10020000;

pub const clock_clkgen1: u32 = 0x13020000;
pub const clock_clkgen2: u32 = 0x17000000;

pub const cache_controller: u32 = 0x2010000;


/*  MAC Interrupt bitmap*/
pub const GMAC_INT_RGSMIIS      : u32 =         BIT(0);
pub const GMAC_INT_PCS_LINK     : u32 =         BIT(1);
pub const GMAC_INT_PCS_ANE      : u32 =         BIT(2);
pub const GMAC_INT_PCS_PHYIS    : u32 =         BIT(3);
pub const GMAC_INT_PMT_EN       : u32 =         BIT(4);
pub const GMAC_INT_LPI_EN       : u32 =         BIT(5);
pub const GMAC_INT_TSIE         : u32 =         BIT(12);
pub const GMAC_PCS_IRQ_DEFAULT  : u32 = (GMAC_INT_RGSMIIS | GMAC_INT_PCS_LINK |  GMAC_INT_PCS_ANE);
pub const GMAC_INT_DEFAULT_ENABL: u32 = (GMAC_INT_PMT_EN | GMAC_INT_LPI_EN);


#[derive(Debug)]
#[repr(C)]
pub struct EqosEesc {
     des0: u32,
     des1: u32,
     des2: u32,
     des3: u32,
}

/*
pub fn eqos_mdio_read(struct mii_dev *bus, mdio_addr: u32, mdio_devad: u32, mdio_reg: u32) -> u32
{
	debug!("eqos_mdio_read, addr={:#x}, reg={:#x}", mdio_addr, mdio_reg);

    // Wait MDIO idle
    loop {
        let mut value: u32 = readv(mdio_address);
        let set = false;
        if !set { value = !value; }//对每位二进制取反
        if (value & EQOS_MAC_MDIO_ADDRESS_GB) == EQOS_MAC_MDIO_ADDRESS_GB { break; }
    }

	let mut val: u32 = readv(&eqos.mac_regs.mdio_address);
	val &= EQOS_MAC_MDIO_ADDRESS_SKAP |
		EQOS_MAC_MDIO_ADDRESS_C45E;
	val |= (mdio_addr << EQOS_MAC_MDIO_ADDRESS_PA_SHIFT) |
		(mdio_reg << EQOS_MAC_MDIO_ADDRESS_RDA_SHIFT) |
		(eqos->config->config_mac_mdio <<
		 EQOS_MAC_MDIO_ADDRESS_CR_SHIFT) |
		(EQOS_MAC_MDIO_ADDRESS_GOC_READ <<
		 EQOS_MAC_MDIO_ADDRESS_GOC_SHIFT) |
		EQOS_MAC_MDIO_ADDRESS_GB;

	writev(val, &eqos.mac_regs.mdio_address);

	//udelay(eqos->config->mdio_wait);

    // Wait MDIO idle
    loop {
        let mut value: u32 = readv(mdio_address);
        let set = false;
        if !set { value = !value; } //对每位二进制取反
        if (value & EQOS_MAC_MDIO_ADDRESS_GB) == EQOS_MAC_MDIO_ADDRESS_GB { break; }
    }

	val = readv(&eqos.mac_regs.mdio_data);
	val &= EQOS_MAC_MDIO_DATA_GD_MASK;

	debug!("eqos_mdio_read: mdio_data={:#x}\n", val);

	val
}

pub fn eqos_mdio_write(mdio_addr: u32, mdio_devad: u32, mdio_reg: u32, mdio_val: u16) -> i32
{
	debug!("eqos_mdio_write, addr={:#x}, reg={:#x}, val={:#x})", mdio_addr, mdio_reg, mdio_val);

    // Wait MDIO idle
    loop {
        let mut value = read(ethernet1 + mac_mdio_address);
        let set = false;
        if !set { value = !value; }//对每位二进制取反
        if (value & EQOS_MAC_MDIO_ADDRESS_GB) == EQOS_MAC_MDIO_ADDRESS_GB { break; }
        // udelay(1)
    }

	writev(ethernet1 + mac_mdio_data, mdio_val);

	let mut val: u32 = readv(ethernet1 + mac_mdio_address);

	/*
	.config_mac = EQOS_MAC_RXQ_CTRL0_RXQ0EN_ENABLED_DCB, // 2
	.config_mac_mdio = EQOS_MAC_MDIO_ADDRESS_CR_250_300, // 5
	.axi_bus_width = EQOS_AXI_WIDTH_64, // 8
	 */
	 let eqos_config_config_mac_mdio = 5;
	
	val &= EQOS_MAC_MDIO_ADDRESS_SKAP |
		EQOS_MAC_MDIO_ADDRESS_C45E;
	val |= (mdio_addr << EQOS_MAC_MDIO_ADDRESS_PA_SHIFT) |
		(mdio_reg << EQOS_MAC_MDIO_ADDRESS_RDA_SHIFT) |
		(eqos_config_config_mac_mdio <<
		 EQOS_MAC_MDIO_ADDRESS_CR_SHIFT) |
		(EQOS_MAC_MDIO_ADDRESS_GOC_WRITE <<
		 EQOS_MAC_MDIO_ADDRESS_GOC_SHIFT) |
		EQOS_MAC_MDIO_ADDRESS_GB;

	writev(ethernet1 + mac_mdio_address, val);

	//udelay(eqos->config->mdio_wait);

    // Wait MDIO idle
    loop {
        let mut value: u32 = readv(ethernet1 + mac_mdio_address);
        let set = false;
        if !set { value = !value; }//对每位二进制取反
        if (value & EQOS_MAC_MDIO_ADDRESS_GB) == EQOS_MAC_MDIO_ADDRESS_GB { break; }
    }

    0
}
*/

/// These barriers need to enforce ordering on both devices or memory.
pub fn mb() {
    #[cfg(target_arch = "riscv64")]
    unsafe {
        asm!("fence iorw, iorw");
    }
}

pub fn eqos_start(struct udevice *dev) -> i32
{
	struct eqos_priv *eqos = dev_get_priv(dev);
	int ret, i;
	ulong rate;


	debug!("eqos_start");

	eqos.tx_desc_idx = 0;
	eqos.rx_desc_idx = 0;           

	// eqos_start_clks(dev);

	//eqos_start_resets(dev);

	//udelay(10);

    // EQOS_DMA_MODE_SWR stuck or not
    loop {
        let mut value: u32 = readv(dma_regs.mode); // 0x1000
        let set = false; // arg3
        if !set { value = !value; }//对每位二进制取反
        if (value & EQOS_DMA_MODE_SWR) == EQOS_DMA_MODE_SWR { break; }
    }


	// eqos_calibrate_pads(dev);

	let rate: u32 = eqos_get_tick_clk_rate(dev);

	let mut val: u32 = (rate / 1000000) - 1;
	writev(val, &eqos.mac_regs.us_tic_counter);

	/*
	 if PHY was already connected and configured,
	 don't need to reconnect/reconfigure again

     // eqos->phy = phy_connect(eqos->mii, addr, dev, eqos->config->interface(dev));
	 */
	
	// phy_startup(eqos->phy);

	eqos_adjust_link(dev);

 let setbits_le32 = |addr: u32, set: u32| writev( readv(addr) | set, addr);
 let clrbits_le32 = |addr: u32, clear: u32| writev( readv(addr) & (!clear), addr);
 let clrsetbits_le32 = |addr: u32, clear: u32, set: u32| writev( ( readv(addr) & (!clear)) | set , addr);

	/* Configure MTL */
	/* Enable Store and Forward mode for TX */
	/* Program Tx operating mode */
	setbits_le32(eqos.mtl_regs.txq0_operation_mode,
		     EQOS_MTL_TXQ0_OPERATION_MODE_TSF |
		     (EQOS_MTL_TXQ0_OPERATION_MODE_TXQEN_ENABLED <<
		      EQOS_MTL_TXQ0_OPERATION_MODE_TXQEN_SHIFT));

	/* Transmit Queue weight */
	writev(0x10, eqos.mtl_regs.txq0_quantum_weight);

	/* Enable Store and Forward mode for RX, since no jumbo frame */
	setbits_le32(eqos.mtl_regs.rxq0_operation_mode,
		     EQOS_MTL_RXQ0_OPERATION_MODE_RSF);

	/* Transmit/Receive queue fifo size; use all RAM for 1 queue */
	let val: u32 = readv(eqos.mac_regs.hw_feature1);
	let tx_fifo_sz: u32 = (val >> EQOS_MAC_HW_FEATURE1_TXFIFOSIZE_SHIFT) &
		EQOS_MAC_HW_FEATURE1_TXFIFOSIZE_MASK;
	let rx_fifo_sz: u32 = (val >> EQOS_MAC_HW_FEATURE1_RXFIFOSIZE_SHIFT) &
		EQOS_MAC_HW_FEATURE1_RXFIFOSIZE_MASK;

	/*
	 * r/tx_fifo_sz is encoded as log2(n / 128). Undo that by shifting.
	 * r/tqs is encoded as (n / 256) - 1.
	 */
	let tqs: u32 = (128 << tx_fifo_sz) / 256 - 1;
	let rqs: u32 = (128 << rx_fifo_sz) / 256 - 1;

	clrsetbits_le32(&eqos.mtl_regs.txq0_operation_mode,
			EQOS_MTL_TXQ0_OPERATION_MODE_TQS_MASK <<
			EQOS_MTL_TXQ0_OPERATION_MODE_TQS_SHIFT,
			tqs << EQOS_MTL_TXQ0_OPERATION_MODE_TQS_SHIFT);
	clrsetbits_le32(&eqos.mtl_regs.rxq0_operation_mode,
			EQOS_MTL_RXQ0_OPERATION_MODE_RQS_MASK <<
			EQOS_MTL_RXQ0_OPERATION_MODE_RQS_SHIFT,
			rqs << EQOS_MTL_RXQ0_OPERATION_MODE_RQS_SHIFT);

	/* Flow control used only if each channel gets 4KB or more FIFO */
	if rqs >= ((4096 / 256) - 1) {
        let mut rfd: u32 = 0;
        let mut rfa: u32 = 0;

		setbits_le32(&eqos.mtl_regs.rxq0_operation_mode,
			     EQOS_MTL_RXQ0_OPERATION_MODE_EHFC);

		/*
		 * Set Threshold for Activating Flow Contol space for min 2
		 * frames ie, (1500 * 1) = 1500 bytes.
		 *
		 * Set Threshold for Deactivating Flow Contol for space of
		 * min 1 frame (frame size 1500bytes) in receive fifo
		 */
		if (rqs == ((4096 / 256) - 1)) {
			/*
			 * This violates the above formula because of FIFO size
			 * limit therefore overflow may occur inspite of this.
			 */
			rfd = 0x3;	/* Full-3K */
			rfa = 0x1;	/* Full-1.5K */
		} else if (rqs == ((8192 / 256) - 1)) {
			rfd = 0x6;	/* Full-4K */
			rfa = 0xa;	/* Full-6K */
		} else if (rqs == ((16384 / 256) - 1)) {
			rfd = 0x6;	/* Full-4K */
			rfa = 0x12;	/* Full-10K */
		} else {
			rfd = 0x6;	/* Full-4K */
			rfa = 0x1E;	/* Full-16K */
		}

		clrsetbits_le32(&eqos.mtl_regs.rxq0_operation_mode,
				(EQOS_MTL_RXQ0_OPERATION_MODE_RFD_MASK <<
				 EQOS_MTL_RXQ0_OPERATION_MODE_RFD_SHIFT) |
				(EQOS_MTL_RXQ0_OPERATION_MODE_RFA_MASK <<
				 EQOS_MTL_RXQ0_OPERATION_MODE_RFA_SHIFT),
				(rfd <<
				 EQOS_MTL_RXQ0_OPERATION_MODE_RFD_SHIFT) |
				(rfa <<
				 EQOS_MTL_RXQ0_OPERATION_MODE_RFA_SHIFT));
	}

	/* Configure MAC */

	clrsetbits_le32(&eqos.mac_regs.rxq_ctrl0,
			EQOS_MAC_RXQ_CTRL0_RXQ0EN_MASK <<
			EQOS_MAC_RXQ_CTRL0_RXQ0EN_SHIFT,
			eqos->config->config_mac <<
			EQOS_MAC_RXQ_CTRL0_RXQ0EN_SHIFT);

	/* Multicast and Broadcast Queue Enable */
	setbits_le32(&eqos.mac_regs.unused_0a4, 0x00100000);
	/* enable promise mode */
	setbits_le32(&eqos.mac_regs.unused_004[1], 0x1);

	/* Set TX flow control parameters */
	/* Set Pause Time */
	setbits_le32(&eqos.mac_regs.q0_tx_flow_ctrl,
		     0xffff << EQOS_MAC_Q0_TX_FLOW_CTRL_PT_SHIFT);
	/* Assign priority for TX flow control */
	clrbits_le32(&eqos.mac_regs.txq_prty_map0,
		     EQOS_MAC_TXQ_PRTY_MAP0_PSTQ0_MASK <<
		     EQOS_MAC_TXQ_PRTY_MAP0_PSTQ0_SHIFT);
	/* Assign priority for RX flow control */
	clrbits_le32(&eqos.mac_regs.rxq_ctrl2,
		     EQOS_MAC_RXQ_CTRL2_PSRQ0_MASK <<
		     EQOS_MAC_RXQ_CTRL2_PSRQ0_SHIFT);
	/* Enable flow control */
	setbits_le32(&eqos.mac_regs.q0_tx_flow_ctrl,
		     EQOS_MAC_Q0_TX_FLOW_CTRL_TFE);
	setbits_le32(&eqos.mac_regs.rx_flow_ctrl,
		     EQOS_MAC_RX_FLOW_CTRL_RFE);

	clrsetbits_le32(&eqos.mac_regs.configuration,
			EQOS_MAC_CONFIGURATION_GPSLCE |
			EQOS_MAC_CONFIGURATION_WD |
			EQOS_MAC_CONFIGURATION_JD |
			EQOS_MAC_CONFIGURATION_JE,
			EQOS_MAC_CONFIGURATION_CST |
			EQOS_MAC_CONFIGURATION_ACS);

	eqos_write_hwaddr(dev);

	/* Configure DMA */

	/* Enable OSP mode */
	setbits_le32(&eqos.dma_regs.ch0_tx_control,
		     EQOS_DMA_CH0_TX_CONTROL_OSP);

	/* RX buffer size. Must be a multiple of bus width */
	clrsetbits_le32(&eqos.dma_regs.ch0_rx_control,
			EQOS_DMA_CH0_RX_CONTROL_RBSZ_MASK <<
			EQOS_DMA_CH0_RX_CONTROL_RBSZ_SHIFT,
			EQOS_MAX_PACKET_SIZE <<
			EQOS_DMA_CH0_RX_CONTROL_RBSZ_SHIFT);

	let desc_pad: u32 = (eqos.desc_size - size_of::<EqosDesc>()) /
		   eqos->config->axi_bus_width;

	setbits_le32(&eqos.dma_regs.ch0_control,
		     EQOS_DMA_CH0_CONTROL_PBLX8 |
		     (desc_pad << EQOS_DMA_CH0_CONTROL_DSL_SHIFT));

	/*
	 * Burst length must be < 1/2 FIFO size.
	 * FIFO size in tqs is encoded as (n / 256) - 1.
	 * Each burst is n * 8 (PBLX8) * 16 (AXI width) == 128 bytes.
	 * Half of n * 256 is n * 128, so pbl == tqs, modulo the -1.
	 */
	let pbl: u32 = tqs + 1;
	if pbl > 32 { pbl = 32; }
	clrsetbits_le32(&eqos.dma_regs.ch0_tx_control,
			EQOS_DMA_CH0_TX_CONTROL_TXPBL_MASK <<
			EQOS_DMA_CH0_TX_CONTROL_TXPBL_SHIFT,
			pbl << EQOS_DMA_CH0_TX_CONTROL_TXPBL_SHIFT);

	clrsetbits_le32(&eqos.dma_regs.ch0_rx_control,
			EQOS_DMA_CH0_RX_CONTROL_RXPBL_MASK <<
			EQOS_DMA_CH0_RX_CONTROL_RXPBL_SHIFT,
			8 << EQOS_DMA_CH0_RX_CONTROL_RXPBL_SHIFT);

	/* DMA performance configuration */
	val = (2 << EQOS_DMA_SYSBUS_MODE_RD_OSR_LMT_SHIFT) |
		EQOS_DMA_SYSBUS_MODE_EAME | EQOS_DMA_SYSBUS_MODE_BLEN16 |
		EQOS_DMA_SYSBUS_MODE_BLEN8 | EQOS_DMA_SYSBUS_MODE_BLEN4;
	writev(val, &eqos.dma_regs.sysbus_mode);

    u32 val, tx_fifo_sz, rx_fifo_sz, tqs, rqs, pbl;
	ulong last_rx_desc;
	ulong desc_pad;

	/* Set up descriptors */

	memset(eqos->descs, 0, eqos->desc_size * EQOS_DESCRIPTORS_NUM);

	for i in 0..EQOS_DESCRIPTORS_TX {
		struct eqos_desc *tx_desc = eqos_get_desc(eqos, i, false);
		// eqos_flush_desc(tx_desc);
	}

	for i in 0..EQOS_DESCRIPTORS_RX {
		struct eqos_desc *rx_desc = eqos_get_desc(eqos, i, true);

		rx_desc.des0 = (u32)(ulong)(eqos->rx_dma_buf + (i * EQOS_MAX_PACKET_SIZE));
		rx_desc.des3 = EQOS_DESC3_OWN | EQOS_DESC3_BUF1V;

		mb();
		// eqos_flush_desc(rx_desc);
		/// eqos_inval_buffer(eqos->rx_dma_buf +	(i * EQOS_MAX_PACKET_SIZE),	EQOS_MAX_PACKET_SIZE);
	}

	writev(0, &eqos.dma_regs.ch0_txdesc_list_haddress);
	writev((ulong)eqos_get_desc(eqos, 0, false),
		&eqos.dma_regs.ch0_txdesc_list_address);
	writev(EQOS_DESCRIPTORS_TX - 1,
	       &eqos.dma_regs.ch0_txdesc_ring_length);

	writev(0, &eqos.dma_regs.ch0_rxdesc_list_haddress);
	writev((ulong)eqos_get_desc(eqos, 0, true),
		&eqos.dma_regs.ch0_rxdesc_list_address);
	writev(EQOS_DESCRIPTORS_RX - 1,
	       &eqos.dma_regs.ch0_rxdesc_ring_length);

	/* Enable everything */
	setbits_le32(&eqos.dma_regs.ch0_tx_control,
		     EQOS_DMA_CH0_TX_CONTROL_ST);
	setbits_le32(&eqos.dma_regs.ch0_rx_control,
		     EQOS_DMA_CH0_RX_CONTROL_SR);
	setbits_le32(&eqos.mac_regs.configuration,
		     EQOS_MAC_CONFIGURATION_TE | EQOS_MAC_CONFIGURATION_RE);

	/* TX tail pointer not written until we need to TX a packet */
	/*
	 * Point RX tail pointer at last descriptor. Ideally, we'd point at the
	 * first descriptor, implying all descriptors were available. However,
	 * that's not distinguishable from none of the descriptors being
	 * available.
	 */
	last_rx_desc = (ulong)eqos_get_desc(eqos, EQOS_DESCRIPTORS_RX - 1, true);
	writev(last_rx_desc, &eqos.dma_regs.ch0_rxdesc_tail_pointer);

	debug!("eqos_start: OK");
	0
}

pub fn eqos_get_desc(struct eqos_priv *eqos, num: u32, rx: bool) -> &mut eqos_desc
{
    let desc_offset = if rx {EQOS_DESCRIPTORS_TX}else{0};

eqos.descs + (desc_offset + num) * size_of::<EqosDesc>();
}


pub fn eqos_send(eqos: &mut DwcDevice, packet: &[u8], length: u32) -> i32
{
	debug!("eqos_send, packet={:#p}, length={}", packet, length);

	//memcpy(eqos->tx_dma_buf, packet, length);
    let txbuf = unsafe {
        slice::from_raw_parts_mut(phys_to_virt(eqos.send_buffers[eqos.tx_desc_idx]) as *mut u8, length)
    };
    txbuf.copy_from_slice(packet);
	// eqos_flush_buffer(eqos.send_buffers[eqos.tx_desc_idx], length);

	let tx_desc = eqos.tx_desc[eqos.tx_desc_idx];

	eqos.tx_desc_idx++;
	eqos.tx_desc_idx %= EQOS_DESCRIPTORS_TX;

	// Send packet
	tx_desc->des0 = (ulong)eqos->tx_dma_buf;
	tx_desc->des1 = 0;
	tx_desc->des2 = length;
	/*
	 * Make sure that if HW sees the _OWN write below, it will see all the
	 * writes to the rest of the descriptor too.
	 */
	mb();

	tx_desc->des3 = EQOS_DESC3_OWN | EQOS_DESC3_FD | EQOS_DESC3_LD | length;
	eqos->config->ops->eqos_flush_desc(tx_desc);

	writev(eqos.tx_desc[tx_desc_idx],
		&eqos->dma_regs->ch0_txdesc_tail_pointer);

	loop {
		// eqos_inval_desc(tx_desc);
		if readv(&tx_desc->des3) & EQOS_DESC3_OWN == 0 {break;}

		udelay(1);
	}

    0
}

pub fn eqos_recv(eqos: &mut DwcDevice, flags: u32, packetp: &mut [u8]) -> u32
{

	debug!("eqos_recv, flags={:#x}", flags);

	let rx_desc = eqos.rx_desc[eqos.rx_desc_idx];
	//eqos_inval_desc(rx_desc);

	if (rx_desc.des3 & EQOS_DESC3_OWN) != 0 {
		debug!("eqos_recv: RX packet not available");
		return -11; //EAGAIN
	}

	// RX packet

	//*packetp = eqos->rx_dma_buf + (eqos.rx_desc_idx * EQOS_MAX_PACKET_SIZE);

	let length: usize = rx_desc->des3 & 0x7fff;

    let rx_packets =
    unsafe { slice::from_raw_parts(phys_to_virt(eqos.recv_buffers[eqos.rx_desc_idx]) as *const u8, length) };

    packetp[..length as usize].copy_from_slice(rx_packets);

	debug("eqos_recv: length={}", length);

	//eqos_inval_buffer(*packetp, length);

	return length;
}

pub fn eqos_free_pkt(eqos: &mut DwcDevice, uchar *packet, length: u32) -> i32
{
	struct eqos_priv *eqos = dev_get_priv(dev);
	uchar *packet_expected;
	struct eqos_desc *rx_desc;

	debug("eqos_free_pkt, packet={:#p}, length={}", packet, length);

	packet_expected = eqos->rx_dma_buf +
		(eqos->rx_desc_idx * EQOS_MAX_PACKET_SIZE);

	if packet != packet_expected {
		erro!("eqos_free_pkt: Unexpected packet (expected {:#p})", packet_expected);
		return -22 ;// EINVAL
	}

	// eqos_inval_buffer(packet, length);

	rx_desc = eqos.rx_desc[eqos.rx_desc_idx];

	rx_desc->des0 = 0;
	mb();
	//eqos_flush_desc(rx_desc);
	//eqos_inval_buffer(packet, length);

	rx_desc->des0 = (u32)(ulong)packet;
	rx_desc->des1 = 0;
	rx_desc->des2 = 0;
	/*
	 * Make sure that if HW sees the _OWN write below, it will see all the
	 * writes to the rest of the descriptor too.
	 */
	mb();
	rx_desc->des3 = EQOS_DESC3_OWN | EQOS_DESC3_BUF1V;
	//eqos_flush_desc(rx_desc);

	writev((ulong)rx_desc, &eqos->dma_regs->ch0_rxdesc_tail_pointer);

	eqos.rx_desc_idx++;
	eqos.rx_desc_idx %= EQOS_DESCRIPTORS_RX;

	0
}

pub fn eqos_write_hwaddr(mac_addr: &[u8; 6]) -> u32 {
/*
This function may be called before start() or after stop(). At that
time, on at least some configurations of the EQoS HW, all clocks to
the EQoS HW block will be stopped, and a reset signal applied. If
any register access is attempted in this state, bus timeouts or CPU
hangs may occur. 
*/

let mut val: u32 = 0;

       /* Update the MAC address */
       val = (mac_addr[5] << 8) |
               (mac_addr[4]);
       writev(val, &eqos.mac_regs.address0_high);

       val = (mac_addr[3] << 24) |
               (mac_addr[2] << 16) |
               (mac_addr[1] << 8) |
               (mac_addr[0]);
       writev(val, &eqos.mac_regs.address0_low);

    0
}

// setbits(addr, set) => raw_write(raw_read(addr) | set, addr)
// clrbits(addr, clear) => raw_write(raw_read(addr) & (!clear), addr)
// clrsetbits(addr, clear, set) => raw_write( (raw_read(addr) & (!clear)) | set , addr) 

pub fn eqos_adjust_link(struct udevice *dev) -> i32
{
	int ret;
	bool en_calibration;

	debug("%s(dev=%p):\n", __func__, dev);

	if eqos.phy.duplex {
		// eqos_set_full_duplex(dev);
        let addr = eqos.mac_regs.configuration;
        raw_write(raw_read(addr) | EQOS_MAC_CONFIGURATION_DM, addr);
    }else{
		//eqos_set_half_duplex(dev);

        let addr = eqos.mac_regs.configuration;
        raw_write(raw_read(addr) & (!EQOS_MAC_CONFIGURATION_DM), addr);

        // WARN: Flush TX queue when switching to half-duplex
        let addr = eqos.mtl_regs.txq0_operation_mode;
        raw_write(raw_read(addr) | EQOS_MTL_TXQ0_OPERATION_MODE_FTQ, addr);
    }

    let mut en_calibration=false;
	match eqos.phy.speed {
	SPEED_1000 => {
		en_calibration = true;
		// eqos_set_gmii_speed(dev);
        let addr = eqos.mac_regs.configuration;
        raw_write(raw_read(addr) & !(EQOS_MAC_CONFIGURATION_PS | EQOS_MAC_CONFIGURATION_FES), addr);
    }
	SPEED_100 => {
        en_calibration = true;
		// eqos_set_mii_speed_100(dev);
        let addr = eqos.mac_regs.configuration;
        raw_write(raw_read(addr) | (EQOS_MAC_CONFIGURATION_PS | EQOS_MAC_CONFIGURATION_FES), addr)
    }
	SPEED_10 => {
        en_calibration = false;
		// eqos_set_mii_speed_10(dev);
        let addr = eqos.mac_regs.configuration;
        raw_write( (raw_read(addr) & !EQOS_MAC_CONFIGURATION_FES) | EQOS_MAC_CONFIGURATION_PS , addr) 
    }

	 
	eqos_set_tx_clk_speed(dev);

	0
}


// eqos_set_tx_clk_speed_jh7110
// case SPEED_1000 => txclk_rate = 125 * 1000 * 1000;
// SPEED_100 => txclk_rate = 25 * 1000 * 1000;
// SPEED_10 => txclk_rate = 2.5 * 1000 * 1000;

pub fn eqos_set_tx_clk_speed_jh7110(struct udevice *dev) -> u32
{
	ulong rate;
	int ret;

	match eqos.phy.speed {
	SPEED_1000 => 	rate = 125 * 1000 * 1000,
	SPEED_100 => 	rate = 25 * 1000 * 1000,
	SPEED_10 => 	rate = 2.5 * 1000 * 1000,
	_ => {
        error!("invalid speed {}", eqos.phy.speed);
		return -22;
    },
	}

	jh7110_eqos_txclk_set_rate(dev, rate);

	clk_set_rate(eqos.rmii_rtx, rate);

	0
}

