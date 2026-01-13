use super::dwc_const::*;

use log::*;

pub const ethernet1: u32 = 0x16040000;

pub const clock_clkgen1: u32 = 0x13020000;
pub const clock_clkgen2: u32 = 0x17000000;

pub const AONCRG_RESET_ASSERT: u32 = 0x38;
pub const AONCRG_RESET_STATUS: u32 = 0x3C;
pub const ISPCRG_RESET_ASSERT: u32 = 0x38;
pub const ISPCRG_RESET_STATUS: u32 = 0x3C;
pub const VOUTCRG_RESET_ASSERT: u32 = 0x48;
pub const VOUTCRG_RESET_STATUS: u32 = 0x4C;
pub const STGCRG_RESET_ASSERT: u32 = 0x74;
pub const STGCRG_RESET_STATUS: u32 = 0x78;
pub const SYSCRG_RESET_ASSERT0: u32 = 0x2F8;
pub const SYSCRG_RESET_ASSERT1: u32 = 0x2FC;
pub const SYSCRG_RESET_ASSERT2: u32 = 0x300;
pub const SYSCRG_RESET_ASSERT3: u32 = 0x304;
pub const SYSCRG_RESET_STATUS0: u32 = 0x308;
pub const SYSCRG_RESET_STATUS1: u32 = 0x30C;
pub const SYSCRG_RESET_STATUS2: u32 = 0x310;
pub const SYSCRG_RESET_STATUS3: u32 = 0x314;

pub fn jh7110_reset_trigger(id: u32, assert: bool) -> u32 {
    let syscrg = 0x13020000;
    let stgcrg = 0x10230000;
    let aoncrg = 0x17000000;
    let ispcrg = 0x19810000;
    let voutcrg = 0x295c0000;

    info!("jh7110_reset_trigger reseting ...");

    let group: u32 = id / 32;
    let mask: u32 = BIT(id % 32);

    // jh7110_get_reset(priv, &reset, group);
    let reset_assert = syscrg + SYSCRG_RESET_ASSERT2;
    let reset_status = syscrg + SYSCRG_RESET_STATUS2;
    /*
    let reset_assert = aoncrg + AONCRG_RESET_ASSERT;
    let reset_status = aoncrg + AONCRG_RESET_STATUS;
    */

    let mut done: u32 = 0;
    if !assert {
        done ^= mask;
    }

    let mut value: u32 = readv(reset_assert);
    if assert {
        value |= mask;
    } else {
        value &= !mask;
    }

    writev(reset_assert, value);

    loop {
        value = readv(reset_status);
        if (value & mask) == done {
            break;
        }
    }

    0
}

pub fn jh7110_clock_reset() {
    log::info!("--------- jh7110_clock_reset");

    /* 会导致发包数少发！
    log::info!("---------init clk");
    for i in 97..112 {
        writev(clock_clkgen1 + i * 4, 0x80000000);
    }
    for i in 221..228 {
        writev(clock_clkgen2 + (i - 219) * 4, 0x80000000);
    }
    */

    // 重要！
    // jh7110_reset_trigger
    // -value=ffe5afc4 reset.assert=13020300
    // -value=ffe5afc0 reset.assert=13020300
    writev(clock_clkgen1 + SYSCRG_RESET_ASSERT2, 0xffe5afc4);
    writev(clock_clkgen1 + SYSCRG_RESET_ASSERT2, 0xffe5afc0);

    writev(clock_clkgen2 + AONCRG_RESET_ASSERT, 0xe1);
    writev(clock_clkgen2 + AONCRG_RESET_ASSERT, 0xe0);
    writev(clock_clkgen2 + 0x0038, 0xe2);
    writev(clock_clkgen2 + 0x0038, 0xe3);

    /*
    writev(clock_clkgen1 + 0x0190, 0x8);
    writev(clock_clkgen1 + 0x0194, 0x1);
    */
}

pub fn phy_config() {
    info!("PHY MII MDIO write");
    // PHY
    const YTPHY_EXTREG_CHIP_CONFIG: u32 = 0xa001;
    const YTPHY_EXTREG_RGMII_CONFIG1: u32 = 0xa003;
    const YTPHY_PAD_DRIVES_STRENGTH_CFG: u32 = 0xa010;

    // ytphy_of_config() ?
    mdio_write_cl(ethernet1, YTPHY_EXTREG_CHIP_CONFIG, 0x8020);
    mdio_write_cl(ethernet1, YTPHY_PAD_DRIVES_STRENGTH_CFG, 0xcbff);
    mdio_write_cl(ethernet1, YTPHY_EXTREG_RGMII_CONFIG1, 0x850);

    log::info!("-------------------phylink_start phylink_speed_up--------------");
    log::info!("-------------------phy_config_aneg--------------");
    mdio_write_cl(ethernet1, 0x1de1, 0x300);
}

fn mdio_write_cl(ioaddr: u32, data: u32, value: u32) {
    //const MII_BUSY: u32 = 1 << 0;

    loop {
        let value = readv(ioaddr + 0x10);

        if value & MII_BUSY != 1 {
            break;
        }
    }

    writev((ioaddr + 0x14), data);
    writev((ioaddr + 0x10), value);

    loop {
        let value = readv(ioaddr + 0x10);

        if value & MII_BUSY != 1 {
            break;
        }
    }
}

const MII_BUSY: u32 = 1 << 0;
const MII_WRITE: u32 = 1 << 1;
const MII_CLKRANGE_60_100M: u32 = 0;
const MII_CLKRANGE_100_150M: u32 = 0x4;
const MII_CLKRANGE_20_35M: u32 = 0x8;
const MII_CLKRANGE_35_60M: u32 = 0xC;
const MII_CLKRANGE_150_250M: u32 = 0x10;
const MII_CLKRANGE_250_300M: u32 = 0x14;
const MIIADDRSHIFT: u32 = 11;
const MIIREGSHIFT: u32 = 6;
const MII_REGMSK: u32 = 0x1F << 6;
const MII_ADDRMSK: u32 = 0x1F << 11;

fn mdio_read_cl(iobase: u32, reg: u32) -> u32 {
    let addr = 0x3;
    let mut miiaddr = ((addr << MIIADDRSHIFT) & MII_ADDRMSK) | ((reg << MIIREGSHIFT) & MII_REGMSK);
    miiaddr = miiaddr | MII_CLKRANGE_150_250M | MII_BUSY;
    log::info!("dw_mdio_read  reg={:#x?}", reg);
    writev((iobase + 0x10), miiaddr);
    loop {
        let value = readv(iobase + 0x10);
        if value & MII_BUSY != 1 {
            let value = readv(iobase + 0x14);
            return value;
        }
    }
}

//        log::info!("-------------------open--------------");

pub fn dwmac_dma_reset() {
    log::info!("-------------dwmac_dma_reset");
    let mut value = readv(ethernet1 + DMA_BUS_MODE);
    writev(ethernet1 + DMA_BUS_MODE, value | DMA_BUS_MODE_SFT_RESET);

    loop {
        let val = readv(ethernet1 + DMA_BUS_MODE);
        if value & DMA_BUS_MODE_SFT_RESET == 0 {
            info!("DMA reset Okay!");
            break;
        }
    }

    dma_status_read();
}

pub fn dwmac_dma_init_rxtx_chan(
    rx_ring_len: u32,
    rdes_base: u32,
    rdes_end: u32,
    tx_ring_len: u32,
    tdes_base: u32,
    tdes_end: u32,
) {
    log::info!("---------------dwmac4_dma_init");
    writev(ethernet1 + DMA_BUS_MODE, 0x1);

    // f0f08f1
    log::info!("---------------axi");
    writev(ethernet1 + DMA_BUS_MODE, 0xf0f08f1);

    log::info!("------------------dwmac410_dma_init_channel");
    writev(ethernet1 + dma_ch0_control, 0);

    //RX
    log::info!("------------------dwmac4_dma_init_rx_chan");
    writev(ethernet1 + dma_ch0_rx_control, 0x100000);

    log::info!("-------------set rx base");
    writev(ethernet1 + dma_ch0_rxdesc_list_address, rdes_base);

    log::info!("-------------set rx end");
    writev(ethernet1 + dma_ch0_rxdesc_tail_pointer, rdes_end);

    //TX
    log::info!("------------------dwmac4_dma_init_tx_chan");
    writev(ethernet1 + dma_ch0_tx_control, 0x100010);

    log::info!("-------------set tx base");
    writev(ethernet1 + dma_ch0_txdesc_list_address, tdes_base);

    // 设置收发ring个数长度
    dwmac4_set_rxtx_ring_len(rx_ring_len, tx_ring_len);
}

pub fn set_mac_addr() {
    log::info!("set mac addr");
    // let mac_id: [u8; 6] = [0xaa, 0xbb, 0xcc, 0xdd, 0x05, 0x06];

    let macid_lo = 0xddccbbaa;
    let macid_hi = 0x0605;

    writev(ethernet1 + mac_address0_high, macid_hi);
    writev(ethernet1 + mac_address0_low, macid_lo);
}

pub fn dwmac4_core_init() {
    log::info!("--------- dwmac4_core_init");
    writev(ethernet1, 0x78200);

    /* Enable GMAC interrupts */
    // value = GMAC_INT_DEFAULT_ENABLE;
    // writev(value, ioaddr + GMAC_INT_EN);
}

pub fn dwmac_mtl_queue_set() {
    log::info!("------------------dwmac4_map_mtl_dma");
    writev(ethernet1 + MTL_RXQ_DMA_MAP0, 0x0); // queue < 4

    log::info!("------------------dwmac4_rx_queue_enable");
    writev(ethernet1 + mac_rxq_ctrl0, 0x2);

    log::info!("------------------dwmac4_dma_rx_chan_op_mode");
    writev(ethernet1 + mtl_rxq0_operation_mode, 0x700000);

    log::info!("------------------dwmac4_dma_tx_chan_op_mode");
    writev(ethernet1 + mtl_txq0_operation_mode, 0x70018);
}

pub fn dwmac4_set_rxtx_ring_len(rx_ring_len: u32, tx_ring_len: u32) {
    log::info!("-------------dwmac4_set_tx_ring_len");
    let tx_ring_len = 64; // chanel = 0;
    writev(ethernet1 + dma_ch0_txdesc_ring_length, tx_ring_len);

    log::info!("-------------dwmac4_set_rx_ring_len");
    let rx_ring_len = 64; // chanel = 0;
    writev(ethernet1 + dma_ch0_rxdesc_ring_length, rx_ring_len);
}

pub fn dwmac4_flow_ctrl() {
    /* MAC Flow Control RX */
    pub const GMAC_RX_FLOW_CTRL_RFE: u32 = BIT(0);
    /* MAC Flow Control TX */
    pub const GMAC_TX_FLOW_CTRL_TFE: u32 = BIT(1);
    pub const GMAC_TX_FLOW_CTRL_PT_SHIFT: u32 = 16;

    info!("Just disable flow ctrl");
    // Just disable flow ctrl
    writev(ethernet1 + mac_rx_flow_ctrl, 0);
    writev(ethernet1 + mac_q0_tx_flow_ctrl, 0);

    /*
        // dwmac4_flow_ctrl ?
    log::info!("--------------tx flow contrl");
        writev((ethernet1) + mac_q0_tx_flow_ctrl, 0xffff0000);

    log::info!("--------------tx flow contrl");
    // not duplex now?
        writev((ethernet1) + mac_q0_tx_flow_ctrl, GMAC_TX_FLOW_CTRL_TFE);

    log::info!("--------------tx flow contrl");
    let pause_time = 0xffff;
    let flow = GMAC_TX_FLOW_CTRL_TFE | (pause_time << GMAC_TX_FLOW_CTRL_PT_SHIFT);
    writev((ethernet1) + mac_q0_tx_flow_ctrl, flow); //0xffff0002

    let mut mac_config = readv(regs::mac::CONFIG);
    mac_config = mac_config & !(EQOS_MAC_CONFIGURATION_GPSLCE | EQOS_MAC_CONFIGURATION_WD | EQOS_MAC_CONFIGURATION_JD | EQOS_MAC_CONFIGURATION_JE);
    writev(regs::mac::CONFIG, mac_config | (EQOS_MAC_CONFIGURATION_CST | EQOS_MAC_CONFIGURATION_ACS));

    */
}

pub fn dma_start_rxtx() {
    info!("--------- dma start RX");
    let mut value = readv(ethernet1 + dma_ch0_rx_control);
    writev(
        ethernet1 + dma_ch0_rx_control,
        value | EQOS_DMA_CH0_RX_CONTROL_SR,
    );

    let mut value = readv(ethernet1);
    writev((ethernet1), value | EQOS_MAC_CONFIGURATION_RE);

    info!("--------- dma start TX");
    let mut value = readv(ethernet1 + dma_ch0_tx_control);
    writev(
        ethernet1 + dma_ch0_tx_control,
        value | EQOS_DMA_CH0_TX_CONTROL_ST,
    );
    let mut value = readv(ethernet1);
    writev((ethernet1), value | EQOS_MAC_CONFIGURATION_TE);
}

pub fn stmmac_mac_link_up() {
    // phy_config() ?

    log::info!("--------------stmmac_mac_link_up");
    writev(ethernet1, 0x8072203);
}

/// Enable disable MAC RX/TX
pub fn stmmac_set_mac(enable: bool) {
    /* Common MAC defines */
    const MAC_CTRL_REG: u32 = 0x00000000; /* MAC Control */
    const MAC_ENABLE_TX: u32 = 0x00000008; /* Transmitter Enable */
    const MAC_ENABLE_RX: u32 = 0x00000004; /* Receiver Enable */

    let enable = true;
    info!("Set MAC {:?}", enable);

    let mut value: u32 = readv(ethernet1 + MAC_CTRL_REG);

    if enable {
        value |= MAC_ENABLE_RX | MAC_ENABLE_TX; // ?
    } else {
        value &= !(MAC_ENABLE_TX | MAC_ENABLE_RX);
    }

    writev(ethernet1, value);
}

///////// dwmac4_lib.c

/// Enable disable MAC RX/TX
pub fn stmmac_dwmac4_set_mac(enable: bool) {
    let ioaddr = ethernet1;

    let mut value: u32 = readv(ioaddr + GMAC_CONFIG);

    if enable {
        value |= GMAC_CONFIG_RE | GMAC_CONFIG_TE;
    } else {
        value &= !(GMAC_CONFIG_TE | GMAC_CONFIG_RE);
    }

    writev(ioaddr + GMAC_CONFIG, value);
}

pub fn stmmac_dwmac4_set_mac_addr(addr: &[u8; 6]) {
    const STMMAC_CHAN0: u32 = 0;
    info!("stmmac_dwmac4_set_mac_addr: {:x?}", addr);
    let ioaddr = ethernet1;

    let mut data: u32 = ((addr[5] as u32) << 8) | (addr[4] as u32);
    /* For MAC Addr registers se have to set the Address Enable (AE)
     * bit that has no effect on the High Reg 0 where the bit 31 (MO)
     * is RO.
     */
    data |= (STMMAC_CHAN0 << GMAC_HI_DCS_SHIFT);
    writev(ioaddr + mac_address0_high, data | GMAC_HI_REG_AE);

    let data = ((addr[3] as u32) << 24)
        | ((addr[2] as u32) << 16)
        | ((addr[1] as u32) << 8)
        | (addr[0] as u32);
    writev(ioaddr + mac_address0_low, data);
}

pub fn stmmac_dwmac4_get_mac_addr(addr: &mut [u8; 6]) {
    let ioaddr = ethernet1;
    debug!("stmmac_dwmac4_get_mac_addr");

    /* Read the MAC address from the hardware */
    let hi_addr: u32 = readv(ioaddr + mac_address0_high);
    let lo_addr: u32 = readv(ioaddr + mac_address0_low);

    /* Extract the MAC address from the high and low words */
    addr[0] = (lo_addr & 0xff) as u8;
    addr[1] = ((lo_addr >> 8) & 0xff) as u8;
    addr[2] = ((lo_addr >> 16) & 0xff) as u8;
    addr[3] = ((lo_addr >> 24) & 0xff) as u8;
    addr[4] = (hi_addr & 0xff) as u8;
    addr[5] = ((hi_addr >> 8) & 0xff) as u8;
}

// dwmac4_enable_dma_irq OR dwmac410_enable_dma_irq ?
pub fn dwmac4_enable_dma_irq(rx: bool, tx: bool) {
    let chan: u32 = 0;
    let ioaddr = ethernet1;
    let mut value: u32 = readv(ioaddr + DMA_CHAN_INTR_ENA);

    if rx {
        value |= DMA_CHAN_INTR_DEFAULT_RX;
    }
    if tx {
        value |= DMA_CHAN_INTR_DEFAULT_TX;
    }

    writev(ioaddr + DMA_CHAN_INTR_ENA, value);
}

pub fn dwmac_enable_dma_irq(rx: bool, tx: bool) {
    /* DMA Normal interrupt */
    const DMA_INTR_ENA_NIE: u32 = 0x00010000; /* Normal Summary */
    const DMA_INTR_ENA_TIE: u32 = 0x00000001; /* Transmit Interrupt */
    const DMA_INTR_ENA_TUE: u32 = 0x00000004; /* Transmit Buffer Unavailable */
    const DMA_INTR_ENA_RIE: u32 = 0x00000040; /* Receive Interrupt */
    const DMA_INTR_ENA_ERE: u32 = 0x00004000; /* Early Receive */
    const DMA_INTR_NORMAL: u32 = (DMA_INTR_ENA_NIE | DMA_INTR_ENA_RIE | DMA_INTR_ENA_TIE);

    const DMA_INTR_ENA: u32 = 0x101c; /* Interrupt Enable */
    const DMA_INTR_DEFAULT_RX: u32 = DMA_INTR_ENA_RIE;
    const DMA_INTR_DEFAULT_TX: u32 = DMA_INTR_ENA_TIE;

    let mut value: u32 = readv(ethernet1 + DMA_INTR_ENA);

    if rx {
        value |= DMA_INTR_DEFAULT_RX;
    }
    if tx {
        value |= DMA_INTR_DEFAULT_TX;
    }
    writev(ethernet1 + DMA_INTR_ENA, value);
}

/// Show DMA status ?
pub fn dma_status_read() {
    let ioaddr = ethernet1;

    // 1. MAC status:
    let us_tic: u32 = readv(ioaddr + mac_us_tic_counter);
    let hw0: u32 = readv(ioaddr + mac_hw_feature0);
    let hw1: u32 = readv(ioaddr + mac_hw_feature1);
    let hw2: u32 = readv(ioaddr + mac_hw_feature2);
    let mac_high: u32 = readv(ioaddr + mac_address0_high);
    let mac_low: u32 = readv(ioaddr + mac_address0_low);

    debug!(
        "MAC STATUS: us_tic_counter:{:#x}, hw_feature0:{:#x}; hw_feature1:{:#x}, hw_feature2:{:#x}, mac_high:{:#x}, mac_low:{:#x}",
        us_tic, hw0, hw1, hw2, mac_high, mac_low
    );

    // 2. DMA status:
    let cur_rx_desc: u32 = readv(ioaddr + DMA_CHAN_CUR_RX_DESC);
    let cur_tx_desc: u32 = readv(ioaddr + DMA_CHAN_CUR_TX_DESC);
    let cur_rx_buf_addr: u32 = readv(ioaddr + DMA_CHAN_CUR_RX_BUF_ADDR);
    let cur_tx_buf_addr: u32 = readv(ioaddr + DMA_CHAN_CUR_TX_BUF_ADDR);

    debug!(
        "DMA_CUR_RX: BUF@{:#x}, DESC@{:#x}; DMA_CUR_TX: BUF@{:#x}, DESC@{:#x}",
        cur_rx_buf_addr, cur_rx_desc, cur_tx_buf_addr, cur_tx_desc
    );

    let txdesc_addr: u32 = readv(ioaddr + dma_ch0_txdesc_list_address);
    let rxdesc_addr: u32 = readv(ioaddr + dma_ch0_rxdesc_list_address);
    let tx_tail_addr: u32 = readv(ioaddr + dma_ch0_txdesc_tail_pointer);
    let rx_tail_addr: u32 = readv(ioaddr + dma_ch0_rxdesc_tail_pointer);
    debug!(
        "DMA RX DESC ADDR: {:#x}~{:#x}; DMA TX DESC ADDR: {:#x}~{:#x}",
        rxdesc_addr, rx_tail_addr, txdesc_addr, tx_tail_addr
    );


    /*
   for i in (0x1100..0x1200).step_by(0x4) {
   let val: u32 = readv(ioaddr + i);
   debug!("DMA REGS: {:#x}={:#X}", i, val);
   }
    */

    // 3. PHY status ?
    const REG_PHY_SPEC_STATUS: u32 = 0x11;
    const YTPHY_EXTREG_CHIP_CONFIG: u32 = 0xa001;
    const YTPHY_EXTREG_RGMII_CONFIG1: u32 = 0xa003;
    const YTPHY_PAD_DRIVES_STRENGTH_CFG: u32 = 0xa010;
    const YTPHY_DUPLEX: u32 = 0x2000;
    const YTPHY_DUPLEX_BIT: u32 = 13;
    const YTPHY_SPEED_MODE: u32 = 0xc000;
    const YTPHY_SPEED_MODE_BIT: u32 = 14;

    // phy_read(phydev, MDIO_DEVAD_NONE, REG_PHY_SPEC_STATUS);
    // eqos_mdio_read(mdio_addr: u32, mdio_devad: u32, mdio_reg: u32) -> u32
    // ? to test phy/motorcomm.c
    let phy_addr = 0;
    let phy_stat: u32 = eqos_mdio_read(phy_addr, 0, REG_PHY_SPEC_STATUS);
    let duplex = (phy_stat & YTPHY_DUPLEX) >> YTPHY_DUPLEX_BIT;
    let speed_mode = (phy_stat & YTPHY_SPEED_MODE) >> YTPHY_SPEED_MODE_BIT;
    debug!(
        "PHY status: {:#x},  deplex(0:half, 1:full)={}, speed(2:1000M, 1:100M)={}",
        phy_stat, duplex, speed_mode
    );

    ////////////////////

    /* read the status register (CSR5) */
    let intr_status: u32 = readv(ioaddr + DMA_STATUS);

    /* Enable it to monitor DMA rx/tx status in case of critical problems */
    debug!("dma_status_read: [DMA_STATUS: {:#x}]", intr_status);

    let intr_status0: u32 = readv(ioaddr + DMA_DEBUG_STATUS_0);
    debug!("dma_status_read: [DMA_STATUS0: {:#x}]", intr_status0);

    let intr_status1: u32 = readv(ioaddr + DMA_DEBUG_STATUS_1);
    debug!("dma_status_read: [DMA_STATUS1: {:#x}]", intr_status1);

    let intr_status2: u32 = readv(ioaddr + DMA_DEBUG_STATUS_2);
    debug!("dma_status_read: [DMA_STATUS2: {:#x}]", intr_status2);

    /*
    // 可能这个状态位没用？
    show_tx_process_state(intr_status);
    show_rx_process_state(intr_status);
    */
}

const DMA_STATUS_TS_MASK: u32 = 0x00700000; /* Transmit Process State */
const DMA_STATUS_TS_SHIFT: u32 = 20;
const DMA_STATUS_RS_MASK: u32 = 0x000e0000; /* Receive Process State */
const DMA_STATUS_RS_SHIFT: u32 = 17;

pub fn show_tx_process_state(status: u32) {
    let state: u32 = (status & DMA_STATUS_TS_MASK) >> DMA_STATUS_TS_SHIFT;

    match state {
        0 => {
            debug!("- TX (Stopped): Reset or Stop command");
        }
        1 => {
            debug!("- TX (Running): Fetching the Tx desc");
        }
        2 => {
            debug!("- TX (Running): Waiting for end of tx");
        }
        3 => {
            debug!("- TX (Running): Reading the data and queuing the data into the Tx buf");
        }
        6 => {
            debug!("- TX (Suspended): Tx Buff Underflow or an unavailable Transmit descriptor");
        }
        7 => {
            debug!("- TX (Running): Closing Tx descriptor");
        }
        _ => {
            warn!("Unknown TX state: {:#x}", state);
        }
    }
}

pub fn show_rx_process_state(status: u32) {
    let state: u32 = (status & DMA_STATUS_RS_MASK) >> DMA_STATUS_RS_SHIFT;

    match state {
        0 => {
            debug!("- RX (Stopped): Reset or Stop command");
        }
        1 => {
            debug!("- RX (Running): Fetching the Rx desc");
        }
        2 => {
            debug!("- RX (Running): Checking for end of pkt");
        }
        3 => {
            debug!("- RX (Running): Waiting for Rx pkt");
        }
        4 => {
            debug!("- RX (Suspended): Unavailable Rx buf");
        }
        5 => {
            debug!("- RX (Running): Closing Rx descriptor");
        }
        6 => {
            debug!("- RX(Running): Flushing the current frame from the Rx buf");
        }
        7 => {
            debug!("- RX (Running): Queuing the Rx frame from the Rx buf into memory");
        }
        _ => {
            warn!("Unknown RX state: {:#x}", state);
        }
    }
}

pub fn eqos_mdio_write(mdio_addr: u32, mdio_reg: u32, mdio_val: u16) -> i32 {
    debug!(
        "mdio_write_cl, addr={:#x}, reg={:#x}, val={:#x})",
        mdio_addr, mdio_reg, mdio_val
    );
    /*
    .config_mac = EQOS_MAC_RXQ_CTRL0_RXQ0EN_ENABLED_DCB, // 2
    .config_mac_mdio = EQOS_MAC_MDIO_ADDRESS_CR_250_300, // 5
    .axi_bus_width = EQOS_AXI_WIDTH_64, // 8
     */

    // Wait MDIO idle
    loop {
        let mut value = readv(ethernet1 + mac_mdio_address);
        let set = false;
        if !set {
            value = !value;
        } //对每位二进制取反

        if (value & EQOS_MAC_MDIO_ADDRESS_GB) == EQOS_MAC_MDIO_ADDRESS_GB {
            break;
        }
        // udelay(1)
    }

    writev(ethernet1 + mac_mdio_data, mdio_val as u32);

    let mut val: u32 = readv(ethernet1 + mac_mdio_address);
    let eqos_config_config_mac_mdio = 5;

    val &= EQOS_MAC_MDIO_ADDRESS_SKAP | EQOS_MAC_MDIO_ADDRESS_C45E;
    val |= (mdio_addr << EQOS_MAC_MDIO_ADDRESS_PA_SHIFT)
        | (mdio_reg << EQOS_MAC_MDIO_ADDRESS_RDA_SHIFT)
        | (eqos_config_config_mac_mdio << EQOS_MAC_MDIO_ADDRESS_CR_SHIFT)
        | (EQOS_MAC_MDIO_ADDRESS_GOC_WRITE << EQOS_MAC_MDIO_ADDRESS_GOC_SHIFT)
        | EQOS_MAC_MDIO_ADDRESS_GB;

    writev(ethernet1 + mac_mdio_address, val);

    //udelay(eqos->config->mdio_wait);

    // Wait MDIO idle
    loop {
        let mut value: u32 = readv(ethernet1 + mac_mdio_address);
        let set = false;
        if !set {
            value = !value;
        }
        if (value & EQOS_MAC_MDIO_ADDRESS_GB) == EQOS_MAC_MDIO_ADDRESS_GB {
            break;
        }
    }

    0
}

pub fn eqos_mdio_read(mdio_addr: u32, mdio_devad: u32, mdio_reg: u32) -> u32 {
    debug!("eqos_mdio_read, addr={:#x}, reg={:#x}", mdio_addr, mdio_reg);

    // Wait MDIO idle
    loop {
        let mut value: u32 = readv(ethernet1 + mac_mdio_address);
        let set = false;
        if !set {
            value = !value;
        }
        if (value & EQOS_MAC_MDIO_ADDRESS_GB) == EQOS_MAC_MDIO_ADDRESS_GB {
            break;
        }
    }

    let eqos_config_config_mac_mdio = 5;

    let mut val: u32 = readv(ethernet1 + mac_mdio_address);
    val &= EQOS_MAC_MDIO_ADDRESS_SKAP | EQOS_MAC_MDIO_ADDRESS_C45E;
    val |= (mdio_addr << EQOS_MAC_MDIO_ADDRESS_PA_SHIFT)
        | (mdio_reg << EQOS_MAC_MDIO_ADDRESS_RDA_SHIFT)
        | (eqos_config_config_mac_mdio << EQOS_MAC_MDIO_ADDRESS_CR_SHIFT)
        | (EQOS_MAC_MDIO_ADDRESS_GOC_READ << EQOS_MAC_MDIO_ADDRESS_GOC_SHIFT)
        | EQOS_MAC_MDIO_ADDRESS_GB;

    writev(ethernet1 + mac_mdio_address, val);

    //udelay(eqos->config->mdio_wait);

    // Wait MDIO idle
    loop {
        let mut value: u32 = readv(ethernet1 + mac_mdio_address);
        let set = false;
        if !set {
            value = !value;
        }
        if (value & EQOS_MAC_MDIO_ADDRESS_GB) == EQOS_MAC_MDIO_ADDRESS_GB {
            break;
        }
    }

    val = readv(ethernet1 + mac_mdio_data);
    val &= EQOS_MAC_MDIO_DATA_GD_MASK;

    debug!("eqos_mdio_read: mdio_data={:#x}", val);

    val
}

pub fn readv(src: u32) -> u32 {
    unsafe { core::ptr::read_volatile(phys_to_virt(src as usize) as *const u32) }
}

// pub fn writev0<T>(dst: *mut T, value: T)
pub fn writev(dst: u32, value: u32) {
    unsafe {
        core::ptr::write_volatile(phys_to_virt(dst as usize) as *mut u32, value);
    }
}

/*
#[linkage = "weak"]
#[export_name = "phys_to_virt"]
*/
pub fn phys_to_virt(addr: usize) -> usize {
    let va = addr + (0xffffffc0 << 32);
    //info!("phys_to_virt: {:#x}", va);
    va
}
