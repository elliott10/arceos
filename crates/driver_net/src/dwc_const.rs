#![allow(unused)]

/* Core registers */

/*  MAC registers */
pub const GMAC_CONFIG: u32 = 0;

pub const EQOS_MAC_REGS_BASE: u32 = 0x000;
// struct eqos_mac_regs

pub const mac_configuration: u32 = 0x000;
// unused_004[(0x070 - 0x004) / 4];	/* 0x004 */
pub const mac_q0_tx_flow_ctrl: u32 = 0x070;
// unused_070[(0x090 - 0x074) / 4];	/* 0x074 */
pub const mac_rx_flow_ctrl: u32 = 0x090;
pub const unused_094: u32 = 0x094;
pub const mac_txq_prty_map0: u32 = 0x098;
pub const unused_09c: u32 = 0x09c;
pub const mac_rxq_ctrl0: u32 = 0x0a0;
pub const unused_0a4: u32 = 0x0a4;
pub const mac_rxq_ctrl2: u32 = 0x0a8;

pub const GMAC_INT_STATUS: u32 = 0x00b0;
pub const GMAC_INT_EN: u32 = 0x00b4;

// unused_0ac[(0x0dc - 0x0ac) / 4];	/* 0x0ac */
pub const mac_us_tic_counter: u32 = 0x0dc;
// unused_0e0[(0x11c - 0x0e0) / 4];	/* 0x0e0 */
pub const mac_hw_feature0: u32 = 0x11c;
pub const mac_hw_feature1: u32 = 0x120;
pub const mac_hw_feature2: u32 = 0x124;
// unused_128[(0x200 - 0x128) / 4];	/* 0x128 */
pub const mac_mdio_address: u32 = 0x200;
pub const mac_mdio_data: u32 = 0x204;
// unused_208[(0x300 - 0x208) / 4];	/* 0x208 */
pub const GMAC_ARP_ADDR: u32 = 0x0210;

pub const mac_address0_high: u32 = 0x300;
pub const mac_address0_low: u32 = 0x304;

/* MAC config */
pub const GMAC_CONFIG_ARPEN       : u32 =   BIT(31);
pub const GMAC_CONFIG_SARC        : u32 =   GENMASK(30, 28);
pub const GMAC_CONFIG_SARC_SHIFT  : u32 =   28;
pub const GMAC_CONFIG_IPC         : u32 =   BIT(27);
pub const GMAC_CONFIG_IPG         : u32 =   GENMASK(26, 24);
pub const GMAC_CONFIG_IPG_SHIFT   : u32 =   24;
pub const GMAC_CONFIG_2K          : u32 =   BIT(22);
pub const GMAC_CONFIG_ACS         : u32 =   BIT(20);
pub const GMAC_CONFIG_BE          : u32 =   BIT(18);
pub const GMAC_CONFIG_JD          : u32 =   BIT(17);
pub const GMAC_CONFIG_JE          : u32 =   BIT(16);
pub const GMAC_CONFIG_PS          : u32 =   BIT(15);
pub const GMAC_CONFIG_FES         : u32 =   BIT(14);
pub const GMAC_CONFIG_FES_SHIFT   : u32 =   14;
pub const GMAC_CONFIG_DM          : u32 =   BIT(13);
pub const GMAC_CONFIG_LM          : u32 =   BIT(12);
pub const GMAC_CONFIG_DCRS        : u32 =   BIT(9);
pub const GMAC_CONFIG_TE          : u32 =   BIT(1);
pub const GMAC_CONFIG_RE          : u32 =   BIT(0);

/* MAC HW ADDR regs */
pub const GMAC_HI_DCS        : u32 =   GENMASK(18, 16);
pub const GMAC_HI_DCS_SHIFT  : u32 =   16;
pub const GMAC_HI_REG_AE     : u32 =   BIT(31);

/*  MAC Interrupt bitmap*/
pub const GMAC_INT_RGSMIIS: u32 = BIT(0);
pub const GMAC_INT_PCS_LINK: u32 = BIT(1);
pub const GMAC_INT_PCS_ANE: u32 = BIT(2);
pub const GMAC_INT_PCS_PHYIS: u32 = BIT(3);
pub const GMAC_INT_PMT_EN: u32 = BIT(4);
pub const GMAC_INT_LPI_EN: u32 = BIT(5);
pub const GMAC_INT_TSIE: u32 = BIT(12);

pub const GMAC_PCS_IRQ_DEFAULT: u32 = (GMAC_INT_RGSMIIS | GMAC_INT_PCS_LINK | GMAC_INT_PCS_ANE);
pub const GMAC_INT_DEFAULT_ENAB: u32 = (GMAC_INT_PMT_EN | GMAC_INT_LPI_EN);

pub const EQOS_MAC_CONFIGURATION_GPSLCE: u32 = BIT(23);
pub const EQOS_MAC_CONFIGURATION_CST: u32 = BIT(21);
pub const EQOS_MAC_CONFIGURATION_ACS: u32 = BIT(20);
pub const EQOS_MAC_CONFIGURATION_WD: u32 = BIT(19);
pub const EQOS_MAC_CONFIGURATION_JD: u32 = BIT(17);
pub const EQOS_MAC_CONFIGURATION_JE: u32 = BIT(16);
pub const EQOS_MAC_CONFIGURATION_PS: u32 = BIT(15);
pub const EQOS_MAC_CONFIGURATION_FES: u32 = BIT(14);
pub const EQOS_MAC_CONFIGURATION_DM: u32 = BIT(13);
pub const EQOS_MAC_CONFIGURATION_LM: u32 = BIT(12);
pub const EQOS_MAC_CONFIGURATION_TE: u32 = BIT(1);
pub const EQOS_MAC_CONFIGURATION_RE: u32 = BIT(0);

pub const EQOS_MAC_Q0_TX_FLOW_CTRL_PT_SHIFT: u32 = 16;
pub const EQOS_MAC_Q0_TX_FLOW_CTRL_PT_MASK: u32 = 0xffff;
pub const EQOS_MAC_Q0_TX_FLOW_CTRL_TFE: u32 = BIT(1);

pub const EQOS_MAC_RX_FLOW_CTRL_RFE: u32 = BIT(0);

pub const EQOS_MAC_TXQ_PRTY_MAP0_PSTQ0_SHIFT: u32 = 0;
pub const EQOS_MAC_TXQ_PRTY_MAP0_PSTQ0_MASK: u32 = 0xff;

pub const EQOS_MAC_RXQ_CTRL0_RXQ0EN_SHIFT: u32 = 0;
pub const EQOS_MAC_RXQ_CTRL0_RXQ0EN_MASK: u32 = 3;
pub const EQOS_MAC_RXQ_CTRL0_RXQ0EN_NOT_ENABLED: u32 = 0;
pub const EQOS_MAC_RXQ_CTRL0_RXQ0EN_ENABLED_DCB: u32 = 2;
pub const EQOS_MAC_RXQ_CTRL0_RXQ0EN_ENABLED_AV: u32 = 1;

pub const EQOS_MAC_RXQ_CTRL2_PSRQ0_SHIFT: u32 = 0;
pub const EQOS_MAC_RXQ_CTRL2_PSRQ0_MASK: u32 = 0xff;

pub const EQOS_MAC_HW_FEATURE0_MMCSEL_SHIFT: u32 = 8;
pub const EQOS_MAC_HW_FEATURE0_HDSEL_SHIFT: u32 = 2;
pub const EQOS_MAC_HW_FEATURE0_GMIISEL_SHIFT: u32 = 1;
pub const EQOS_MAC_HW_FEATURE0_MIISEL_SHIFT: u32 = 0;

pub const EQOS_MAC_HW_FEATURE1_TXFIFOSIZE_SHIFT: u32 = 6;
pub const EQOS_MAC_HW_FEATURE1_TXFIFOSIZE_MASK: u32 = 0x1f;
pub const EQOS_MAC_HW_FEATURE1_RXFIFOSIZE_SHIFT: u32 = 0;
pub const EQOS_MAC_HW_FEATURE1_RXFIFOSIZE_MASK: u32 = 0x1f;

pub const EQOS_MAC_HW_FEATURE3_ASP_SHIFT: u32 = 28;
pub const EQOS_MAC_HW_FEATURE3_ASP_MASK: u32 = 0x3;

pub const EQOS_MAC_MDIO_ADDRESS_PA_SHIFT: u32 = 21;
pub const EQOS_MAC_MDIO_ADDRESS_RDA_SHIFT: u32 = 16;
pub const EQOS_MAC_MDIO_ADDRESS_CR_SHIFT: u32 = 8;
pub const EQOS_MAC_MDIO_ADDRESS_CR_20_35: u32 = 2;
pub const EQOS_MAC_MDIO_ADDRESS_CR_250_300: u32 = 5;
pub const EQOS_MAC_MDIO_ADDRESS_SKAP: u32 = BIT(4);
pub const EQOS_MAC_MDIO_ADDRESS_GOC_SHIFT: u32 = 2;
pub const EQOS_MAC_MDIO_ADDRESS_GOC_READ: u32 = 3;
pub const EQOS_MAC_MDIO_ADDRESS_GOC_WRITE: u32 = 1;
pub const EQOS_MAC_MDIO_ADDRESS_C45E: u32 = BIT(1);
pub const EQOS_MAC_MDIO_ADDRESS_GB: u32 = BIT(0);

pub const EQOS_MAC_MDIO_DATA_GD_MASK: u32 = 0xffff;

pub const MTL_RXQ_DMA_MAP0: u32 = 0x0c30; /* queue 0 to 3 */
pub const MTL_RXQ_DMA_MAP1: u32 = 0x0c34; /* queue 4 to 7 */
pub const MTL_CHAN_BASE_ADDR: u32 = 0x0d00;
pub const MTL_CHAN_BASE_OFFSET: u32 = 0x0040;

/*
MTL(MAC Transaction Layer)
MTL层主要管理TX/RX的fifo，这些fifo作为MAC层与系统内存之间的缓冲;
fifo的传输规则，一般是设置一个阈值，当fifo的数据超过该阈值，则触发下一步的操作;
TX fifo会将数据发送给MAC层，传输到网络中;RX fifo会将数据交给DMA，传输到系统内存中，等待CPU处理;
 */
pub const EQOS_MTL_REGS_BASE: u32 = 0xd00;
// struct eqos_mtl_regs

pub const mtl_txq0_operation_mode: u32 = 0xd00;
pub const unused_d04: u32 = 0xd04;
pub const mtl_txq0_debug: u32 = 0xd08;
// unused_d0c[(0xd18 - 0xd0c) / 4];	/* 0xd0c */
pub const mtl_txq0_quantum_weight: u32 = 0xd18;
// unused_d1c[(0xd30 - 0xd1c) / 4];	/* 0xd1c */
pub const mtl_rxq0_operation_mode: u32 = 0xd30;
pub const unused_d34: u32 = 0xd34;
pub const mtl_rxq0_debug: u32 = 0xd38;

pub const EQOS_MTL_TXQ0_OPERATION_MODE_TQS_SHIFT: u32 = 16;
pub const EQOS_MTL_TXQ0_OPERATION_MODE_TQS_MASK: u32 = 0x1ff;
pub const EQOS_MTL_TXQ0_OPERATION_MODE_TXQEN_SHIFT: u32 = 2;
pub const EQOS_MTL_TXQ0_OPERATION_MODE_TXQEN_MASK: u32 = 3;
pub const EQOS_MTL_TXQ0_OPERATION_MODE_TXQEN_ENABLED: u32 = 2;
pub const EQOS_MTL_TXQ0_OPERATION_MODE_TSF: u32 = BIT(1);
pub const EQOS_MTL_TXQ0_OPERATION_MODE_FTQ: u32 = BIT(0);

pub const EQOS_MTL_TXQ0_DEBUG_TXQSTS: u32 = BIT(4);
pub const EQOS_MTL_TXQ0_DEBUG_TRCSTS_SHIFT: u32 = 1;
pub const EQOS_MTL_TXQ0_DEBUG_TRCSTS_MASK: u32 = 3;

pub const EQOS_MTL_RXQ0_OPERATION_MODE_RQS_SHIFT: u32 = 20;
pub const EQOS_MTL_RXQ0_OPERATION_MODE_RQS_MASK: u32 = 0x3ff;
pub const EQOS_MTL_RXQ0_OPERATION_MODE_RFD_SHIFT: u32 = 14;
pub const EQOS_MTL_RXQ0_OPERATION_MODE_RFD_MASK: u32 = 0x3f;
pub const EQOS_MTL_RXQ0_OPERATION_MODE_RFA_SHIFT: u32 = 8;
pub const EQOS_MTL_RXQ0_OPERATION_MODE_RFA_MASK: u32 = 0x3f;
pub const EQOS_MTL_RXQ0_OPERATION_MODE_EHFC: u32 = BIT(7);
pub const EQOS_MTL_RXQ0_OPERATION_MODE_RSF: u32 = BIT(5);

pub const EQOS_MTL_RXQ0_DEBUG_PRXQ_SHIFT: u32 = 16;
pub const EQOS_MTL_RXQ0_DEBUG_PRXQ_MASK: u32 = 0x7fff;
pub const EQOS_MTL_RXQ0_DEBUG_RXQSTS_SHIFT: u32 = 4;
pub const EQOS_MTL_RXQ0_DEBUG_RXQSTS_MASK: u32 = 3;

pub const EQOS_DMA_REGS_BASE: u32 = 0x1000;
// struct eqos_dma_regs

pub const dma_mode: u32 = 0x1000;
pub const dma_sysbus_mode: u32 = 0x1004;
// unused_1008[(0x1100 - 0x1008) / 4];	/* 0x1008 */
pub const dma_ch0_control: u32 = 0x1100;
pub const dma_ch0_tx_control: u32 = 0x1104;
pub const dma_ch0_rx_control: u32 = 0x1108;
pub const unused_110c: u32 = 0x110c;
pub const dma_ch0_txdesc_list_haddress: u32 = 0x1110;
pub const dma_ch0_txdesc_list_address: u32 = 0x1114;
pub const dma_ch0_rxdesc_list_haddress: u32 = 0x1118;
pub const dma_ch0_rxdesc_list_address: u32 = 0x111c;
pub const dma_ch0_txdesc_tail_pointer: u32 = 0x1120;
pub const unused_1124: u32 = 0x1124;
pub const dma_ch0_rxdesc_tail_pointer: u32 = 0x1128;
pub const dma_ch0_txdesc_ring_length: u32 = 0x112c;
pub const dma_ch0_rxdesc_ring_length: u32 = 0x1130;
// pub const DMA_CHAN_INTR_ENA(x) (DMA_CHAN_BASE_ADDR + 0x34)

// 注，请参考dwmac4_dma.h头文件，而dwmac_dma.h中的变量不可用于visionfive2网卡, 如 Current Host Tx/Rx Buffer

pub const DMA_BUS_MODE: u32 = 0x00001000;
pub const DMA_SYS_BUS_MODE: u32 = 0x00001004;
pub const DMA_STATUS: u32 = 0x00001008;
pub const DMA_DEBUG_STATUS_0: u32 = 0x0000100c;
pub const DMA_DEBUG_STATUS_1: u32 = 0x00001010;
pub const DMA_DEBUG_STATUS_2: u32 = 0x00001014;
pub const DMA_AXI_BUS_MODE: u32 = 0x00001028;
pub const DMA_TBS_CTRL: u32 = 0x00001050;

/* DMA Bus Mode bitmap */
pub const DMA_BUS_MODE_DCHE: u32 = BIT(19);
pub const DMA_BUS_MODE_INTM_MASK: u32 = GENMASK(17, 16);
pub const DMA_BUS_MODE_INTM_SHIFT: u32 = 16;
pub const DMA_BUS_MODE_INTM_MODE1: u32 = 0x1;
pub const DMA_BUS_MODE_SFT_RESET: u32 = BIT(0);

/* DMA SYS Bus Mode bitmap */
pub const DMA_BUS_MODE_SPH: u32 = BIT(24);
pub const DMA_BUS_MODE_PBL: u32 = BIT(16);
pub const DMA_BUS_MODE_PBL_SHIFT: u32 = 16;
pub const DMA_BUS_MODE_RPBL_SHIFT: u32 = 16;
pub const DMA_BUS_MODE_MB: u32 = BIT(14);
pub const DMA_BUS_MODE_FB: u32 = BIT(0);

/* DMA Interrupt top status */
pub const DMA_STATUS_MAC: u32 = BIT(17);
pub const DMA_STATUS_MTL: u32 = BIT(16);
pub const DMA_STATUS_CHAN7: u32 = BIT(7);
pub const DMA_STATUS_CHAN6: u32 = BIT(6);
pub const DMA_STATUS_CHAN5: u32 = BIT(5);
pub const DMA_STATUS_CHAN4: u32 = BIT(4);
pub const DMA_STATUS_CHAN3: u32 = BIT(3);
pub const DMA_STATUS_CHAN2: u32 = BIT(2);
pub const DMA_STATUS_CHAN1: u32 = BIT(1);
pub const DMA_STATUS_CHAN0: u32 = BIT(0);

/* DMA debug status bitmap */
pub const DMA_DEBUG_STATUS_TS_MASK: u32 = 0xf;
pub const DMA_DEBUG_STATUS_RS_MASK: u32 = 0xf;

/* DMA AXI bitmap */
pub const DMA_AXI_EN_LPI: u32 = BIT(31);
pub const DMA_AXI_LPI_XIT_FRM: u32 = BIT(30);
pub const DMA_AXI_WR_OSR_LMT: u32 = GENMASK(27, 24);
pub const DMA_AXI_WR_OSR_LMT_SHIFT: u32 = 24;
pub const DMA_AXI_RD_OSR_LMT: u32 = GENMASK(19, 16);
pub const DMA_AXI_RD_OSR_LMT_SHIFT: u32 = 16;

pub const DMA_AXI_OSR_MAX: u32 = 0xf;
pub const DMA_AXI_MAX_OSR_LIMIT: u32 =
    ((DMA_AXI_OSR_MAX << DMA_AXI_WR_OSR_LMT_SHIFT) | (DMA_AXI_OSR_MAX << DMA_AXI_RD_OSR_LMT_SHIFT));

pub const DMA_SYS_BUS_MB: u32 = BIT(14);
pub const DMA_AXI_1KBBE: u32 = BIT(13);
pub const DMA_SYS_BUS_AAL: u32 = BIT(12);
pub const DMA_SYS_BUS_EAME: u32 = BIT(11);
pub const DMA_AXI_BLEN256: u32 = BIT(7);
pub const DMA_AXI_BLEN128: u32 = BIT(6);
pub const DMA_AXI_BLEN64: u32 = BIT(5);
pub const DMA_AXI_BLEN32: u32 = BIT(4);
pub const DMA_AXI_BLEN16: u32 = BIT(3);
pub const DMA_AXI_BLEN8: u32 = BIT(2);
pub const DMA_AXI_BLEN4: u32 = BIT(1);
pub const DMA_SYS_BUS_FB: u32 = BIT(0);

pub const DMA_BURST_LEN_DEFAULT: u32 = (DMA_AXI_BLEN256
    | DMA_AXI_BLEN128
    | DMA_AXI_BLEN64
    | DMA_AXI_BLEN32
    | DMA_AXI_BLEN16
    | DMA_AXI_BLEN8
    | DMA_AXI_BLEN4);

pub const DMA_AXI_BURST_LEN_MASK: u32 = 0x000000FE;

/* DMA TBS Control */
pub const DMA_TBS_FTOS: u32 = GENMASK(31, 8);
pub const DMA_TBS_FTOV: u32 = BIT(0);
pub const DMA_TBS_DEF_FTOS: u32 = (DMA_TBS_FTOS | DMA_TBS_FTOV);

/* Following DMA defines are chanels oriented */
pub const DMA_CHAN_BASE_ADDR: u32 = 0x1100;
pub const DMA_CHAN_BASE_OFFSET: u32 = 0x80;
//# define DMA_CHAN_BASE_ADDR		(DMA_CHAN_BASE_ADDR + (x * DMA_CHAN_BASE_OFFSET))
pub const DMA_CHAN_REG_NUMBER: u32 = 17;

pub const DMA_CHAN_CONTROL: u32 = DMA_CHAN_BASE_ADDR;
pub const DMA_CHAN_TX_CONTROL: u32 = (DMA_CHAN_BASE_ADDR + 0x4);
pub const DMA_CHAN_RX_CONTROL: u32 = (DMA_CHAN_BASE_ADDR + 0x8);
pub const DMA_CHAN_TX_BASE_ADDR_HI: u32 = (DMA_CHAN_BASE_ADDR + 0x10);
pub const DMA_CHAN_TX_BASE_ADDR: u32 = (DMA_CHAN_BASE_ADDR + 0x14);
pub const DMA_CHAN_RX_BASE_ADDR_HI: u32 = (DMA_CHAN_BASE_ADDR + 0x18);
pub const DMA_CHAN_RX_BASE_ADDR: u32 = (DMA_CHAN_BASE_ADDR + 0x1c);
pub const DMA_CHAN_TX_END_ADDR: u32 = (DMA_CHAN_BASE_ADDR + 0x20);
pub const DMA_CHAN_RX_END_ADDR: u32 = (DMA_CHAN_BASE_ADDR + 0x28);
pub const DMA_CHAN_TX_RING_LEN: u32 = (DMA_CHAN_BASE_ADDR + 0x2c);
pub const DMA_CHAN_RX_RING_LEN: u32 = (DMA_CHAN_BASE_ADDR + 0x30);
pub const DMA_CHAN_INTR_ENA: u32 = (DMA_CHAN_BASE_ADDR + 0x34);
pub const DMA_CHAN_RX_WATCHDOG: u32 = (DMA_CHAN_BASE_ADDR + 0x38);
pub const DMA_CHAN_SLOT_CTRL_STATUS: u32 = (DMA_CHAN_BASE_ADDR + 0x3c);
pub const DMA_CHAN_CUR_TX_DESC: u32 = (DMA_CHAN_BASE_ADDR + 0x44);
pub const DMA_CHAN_CUR_RX_DESC: u32 = (DMA_CHAN_BASE_ADDR + 0x4c);
pub const DMA_CHAN_CUR_TX_BUF_ADDR: u32 = (DMA_CHAN_BASE_ADDR + 0x54);
pub const DMA_CHAN_CUR_RX_BUF_ADDR: u32 = (DMA_CHAN_BASE_ADDR + 0x5c);
pub const DMA_CHAN_STATUS: u32 = (DMA_CHAN_BASE_ADDR + 0x60);

/* DMA Control X */
pub const DMA_CONTROL_SPH: u32 = BIT(24);
pub const DMA_CONTROL_MSS_MASK: u32 = GENMASK(13, 0);

/* DMA Tx Channel X Control register defines */
pub const DMA_CONTROL_EDSE: u32 = BIT(28);
pub const DMA_CONTROL_TSE: u32 = BIT(12);
pub const DMA_CONTROL_OSP: u32 = BIT(4);
pub const DMA_CONTROL_ST: u32 = BIT(0);

/* DMA Rx Channel X Control register defines */
pub const DMA_CONTROL_SR: u32 = BIT(0);
pub const DMA_RBSZ_MASK: u32 = GENMASK(14, 1);
pub const DMA_RBSZ_SHIFT: u32 = 1;

/* Interrupt status per channel */
pub const DMA_CHAN_STATUS_REB: u32 = GENMASK(21, 19);
pub const DMA_CHAN_STATUS_REB_SHIFT: u32 = 19;
pub const DMA_CHAN_STATUS_TEB: u32 = GENMASK(18, 16);
pub const DMA_CHAN_STATUS_TEB_SHIFT: u32 = 16;
pub const DMA_CHAN_STATUS_NIS: u32 = BIT(15);
pub const DMA_CHAN_STATUS_AIS: u32 = BIT(14);
pub const DMA_CHAN_STATUS_CDE: u32 = BIT(13);
pub const DMA_CHAN_STATUS_FBE: u32 = BIT(12);
pub const DMA_CHAN_STATUS_ERI: u32 = BIT(11);
pub const DMA_CHAN_STATUS_ETI: u32 = BIT(10);
pub const DMA_CHAN_STATUS_RWT: u32 = BIT(9);
pub const DMA_CHAN_STATUS_RPS: u32 = BIT(8);
pub const DMA_CHAN_STATUS_RBU: u32 = BIT(7);
pub const DMA_CHAN_STATUS_RI: u32 = BIT(6);
pub const DMA_CHAN_STATUS_TBU: u32 = BIT(2);
pub const DMA_CHAN_STATUS_TPS: u32 = BIT(1);
pub const DMA_CHAN_STATUS_TI: u32 = BIT(0);

pub const DMA_CHAN_STATUS_MSK_COMMON: u32 =
    (DMA_CHAN_STATUS_NIS | DMA_CHAN_STATUS_AIS | DMA_CHAN_STATUS_CDE | DMA_CHAN_STATUS_FBE);
pub const DMA_CHAN_STATUS_MSK_RX: u32 = (DMA_CHAN_STATUS_REB
    | DMA_CHAN_STATUS_ERI
    | DMA_CHAN_STATUS_RWT
    | DMA_CHAN_STATUS_RPS
    | DMA_CHAN_STATUS_RBU
    | DMA_CHAN_STATUS_RI
    | DMA_CHAN_STATUS_MSK_COMMON);
pub const DMA_CHAN_STATUS_MSK_TX: u32 = (DMA_CHAN_STATUS_ETI
    | DMA_CHAN_STATUS_TBU
    | DMA_CHAN_STATUS_TPS
    | DMA_CHAN_STATUS_TI
    | DMA_CHAN_STATUS_MSK_COMMON);

/* Interrupt enable bits per channel */
pub const DMA_CHAN_INTR_ENA_NIE: u32 = BIT(16);
pub const DMA_CHAN_INTR_ENA_AIE: u32 = BIT(15);
pub const DMA_CHAN_INTR_ENA_NIE_4_10: u32 = BIT(15);
pub const DMA_CHAN_INTR_ENA_AIE_4_10: u32 = BIT(14);
pub const DMA_CHAN_INTR_ENA_CDE: u32 = BIT(13);
pub const DMA_CHAN_INTR_ENA_FBE: u32 = BIT(12);
pub const DMA_CHAN_INTR_ENA_ERE: u32 = BIT(11);
pub const DMA_CHAN_INTR_ENA_ETE: u32 = BIT(10);
pub const DMA_CHAN_INTR_ENA_RWE: u32 = BIT(9);
pub const DMA_CHAN_INTR_ENA_RSE: u32 = BIT(8);
pub const DMA_CHAN_INTR_ENA_RBUE: u32 = BIT(7);
pub const DMA_CHAN_INTR_ENA_RIE: u32 = BIT(6);
pub const DMA_CHAN_INTR_ENA_TBUE: u32 = BIT(2);
pub const DMA_CHAN_INTR_ENA_TSE: u32 = BIT(1);
pub const DMA_CHAN_INTR_ENA_TIE: u32 = BIT(0);

pub const DMA_CHAN_INTR_NORMAL: u32 =
    (DMA_CHAN_INTR_ENA_NIE | DMA_CHAN_INTR_ENA_RIE | DMA_CHAN_INTR_ENA_TIE);

pub const DMA_CHAN_INTR_ABNORMAL: u32 = (DMA_CHAN_INTR_ENA_AIE | DMA_CHAN_INTR_ENA_FBE);
/* DMA default interrupt mask for 4.00 */
pub const DMA_CHAN_INTR_DEFAULT_MASK: u32 = (DMA_CHAN_INTR_NORMAL | DMA_CHAN_INTR_ABNORMAL);
pub const DMA_CHAN_INTR_DEFAULT_RX: u32 = (DMA_CHAN_INTR_ENA_RIE);
pub const DMA_CHAN_INTR_DEFAULT_TX: u32 = (DMA_CHAN_INTR_ENA_TIE);

pub const DMA_CHAN_INTR_NORMAL_4_10: u32 =
    (DMA_CHAN_INTR_ENA_NIE_4_10 | DMA_CHAN_INTR_ENA_RIE | DMA_CHAN_INTR_ENA_TIE);

pub const DMA_CHAN_INTR_ABNORMAL_4_10: u32 = (DMA_CHAN_INTR_ENA_AIE_4_10 | DMA_CHAN_INTR_ENA_FBE);
/* DMA default interrupt mask for 4.10a */
pub const DMA_CHAN_INTR_DEFAULT_MASK_4_10: u32 =
    (DMA_CHAN_INTR_NORMAL_4_10 | DMA_CHAN_INTR_ABNORMAL_4_10);
pub const DMA_CHAN_INTR_DEFAULT_RX_4_10: u32 = (DMA_CHAN_INTR_ENA_RIE);
pub const DMA_CHAN_INTR_DEFAULT_TX_4_10: u32 = (DMA_CHAN_INTR_ENA_TIE);

/* channel 0 specific fields */
pub const DMA_CHAN0_DBG_STAT_TPS: u32 = GENMASK(15, 12);
pub const DMA_CHAN0_DBG_STAT_TPS_SHIFT: u32 = 12;
pub const DMA_CHAN0_DBG_STAT_RPS: u32 = GENMASK(11, 8);
pub const DMA_CHAN0_DBG_STAT_RPS_SHIFT: u32 = 8;
/////////

pub const EQOS_DMA_MODE_SWR: u32 = BIT(0);

pub const EQOS_DMA_SYSBUS_MODE_RD_OSR_LMT_SHIFT: u32 = 16;
pub const EQOS_DMA_SYSBUS_MODE_RD_OSR_LMT_MASK: u32 = 0xf;
pub const EQOS_DMA_SYSBUS_MODE_EAME: u32 = BIT(11);
pub const EQOS_DMA_SYSBUS_MODE_BLEN16: u32 = BIT(3);
pub const EQOS_DMA_SYSBUS_MODE_BLEN8: u32 = BIT(2);
pub const EQOS_DMA_SYSBUS_MODE_BLEN4: u32 = BIT(1);

pub const EQOS_DMA_CH0_CONTROL_DSL_SHIFT: u32 = 18;
pub const EQOS_DMA_CH0_CONTROL_PBLX8: u32 = BIT(16);

pub const EQOS_DMA_CH0_TX_CONTROL_TXPBL_SHIFT: u32 = 16;
pub const EQOS_DMA_CH0_TX_CONTROL_TXPBL_MASK: u32 = 0x3f;
pub const EQOS_DMA_CH0_TX_CONTROL_OSP: u32 = BIT(4);
pub const EQOS_DMA_CH0_TX_CONTROL_ST: u32 = BIT(0);

pub const EQOS_DMA_CH0_RX_CONTROL_RXPBL_SHIFT: u32 = 16;
pub const EQOS_DMA_CH0_RX_CONTROL_RXPBL_MASK: u32 = 0x3f;
pub const EQOS_DMA_CH0_RX_CONTROL_RBSZ_SHIFT: u32 = 1;
pub const EQOS_DMA_CH0_RX_CONTROL_RBSZ_MASK: u32 = 0x3fff;
pub const EQOS_DMA_CH0_RX_CONTROL_SR: u32 = BIT(0);

/* These registers are Tegra186-specific */
pub const EQOS_TEGRA186_REGS_BASE: u32 = 0x8800;
// struct eqos_tegra186_regs

pub const tegra186_sdmemcomppadctrl: u32 = 0x8800;
pub const tegra186_auto_cal_config: u32 = 0x8804;
pub const unused_8808: u32 = 0x8808;
pub const tegra186_auto_cal_status: u32 = 0x880c;

pub const EQOS_SDMEMCOMPPADCTRL_PAD_E_INPUT_OR_E_PWRD: u32 = BIT(31);

pub const EQOS_AUTO_CAL_CONFIG_START: u32 = BIT(31);
pub const EQOS_AUTO_CAL_CONFIG_ENABLE: u32 = BIT(29);

pub const EQOS_AUTO_CAL_STATUS_ACTIVE: u32 = BIT(31);

/* Descriptors */
pub const EQOS_DESCRIPTORS_TX: u32 = 4;
pub const EQOS_DESCRIPTORS_RX: u32 = 4;
pub const EQOS_DESCRIPTORS_NUM: u32 = (EQOS_DESCRIPTORS_TX + EQOS_DESCRIPTORS_RX);
pub const EQOS_BUFFER_ALIGN: u32 = ARCH_DMA_MINALIGN;
pub const EQOS_MAX_PACKET_SIZE: u32 = ALIGN(1568, ARCH_DMA_MINALIGN);
pub const EQOS_RX_BUFFER_SIZE: u32 = (EQOS_DESCRIPTORS_RX * EQOS_MAX_PACKET_SIZE);

// starfive-visionfive2 CONFIG_SYS_CACHELINE_SIZE
pub const ARCH_DMA_MINALIGN: u32 = 64;

/*
struct eqos_desc {
    u32 des0;
    u32 des1;
    u32 des2;
    u32 des3;
}; */

pub const EQOS_DESC3_OWN: u32 = BIT(31);
pub const EQOS_DESC3_FD: u32 = BIT(29);
pub const EQOS_DESC3_LD: u32 = BIT(28);
pub const EQOS_DESC3_BUF1V: u32 = BIT(24);

pub const EQOS_AXI_WIDTH_32: u32 = 4;
pub const EQOS_AXI_WIDTH_64: u32 = 8;
pub const EQOS_AXI_WIDTH_128: u32 = 16;

pub const fn BIT(n: u32) -> u32 {
    1 << n
}

pub const fn ALIGN(x: u32, a: u32) -> u32 {
    (x + (a - 1)) & !(a - 1)
}

pub const BITS_PER_LONG: u32 = 64;
pub const fn GENMASK(h: u32, l: u32) -> u32 {
    ((!(0 as u64) - (1 << l) + 1) & (!(0 as u64) >> (BITS_PER_LONG - 1 - h))) as u32
}
