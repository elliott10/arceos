use alloc::boxed::Box;
use core::slice::from_raw_parts_mut;
use core::{fmt::write, marker::PhantomData};

use driver_common::{BaseDriverOps, DevError, DevResult, DeviceType};

use crate::{EthernetAddress, NetBufPtr, NetDriverOps};

use super::dwc_const::*;

use log::{debug, error, info, warn};

extern crate alloc;

unsafe impl<A: StarfiveHal> Sync for StarfiveNic<A> {}
unsafe impl<A: StarfiveHal> Send for StarfiveNic<A> {}

pub const DMA_BUS_MODE: usize = 0x00001000;

/* SW Reset */
pub const DMA_BUS_MODE_SFT_RESET: usize = 0x1; /* Software Reset */

/* AXI Master Bus Mode */
pub const DMA_AXI_BUS_MODE: usize = 0x00001028;

pub const DMA_RCV_BASE_ADDR: usize = 0x0000100c; /* Receive List Base */

/* Ctrl (Operational Mode) */
pub const DMA_CONTROL: usize = 0x00001018;

pub const DMA_CONTROL_SR: usize = 0x00000002;

pub const MAC_ENABLE_TX: u32 = 1 << 3; /* Transmitter Enable */
pub const MAC_ENABLE_RX: u32 = 1 << 2; /* Receiver Enable */

/* Received Poll Demand */
pub const DMA_XMT_POLL_DEMAND: u32 = 0x00001004;

/* Received Poll Demand */
pub const DMA_RCV_POLL_DEMAND: u32 = 0x00001008;

pub const DMA_CONTROL_ST: u32 = 0x00002000;

pub const SIFIVE_CCACHE_WAY_ENABLE: usize = 0x8;

use core::ptr::{read_volatile, write_volatile};

pub struct StarfiveNic<A>
where
    A: StarfiveHal,
{
    ioaddr: usize,
    phantom: PhantomData<A>,
}

pub fn sifive_ccache_flush_range<A: StarfiveHal>(start: usize, end: usize) {
    // let start_pa = A::virt_to_phys(start) as u32;
    // let end_pa: u32 = A::virt_to_phys(end) as u32;
    log::info!(
        "sifive_ccache_flush_range---------start:{:#x} end:{:#x?}",
        start,
        end
    );
    let start_pa = start as usize;
    let end_pa = end as usize;

    let mut s = start_pa;

    let cache_line_size = 0x40;

    let cache_flush = A::phys_to_virt(0x201_0000);

    A::fence();

    unsafe { core::arch::asm!("fence") };

    let addr = cache_flush + 0x200 as usize;

    // let va = A::phys_to_virt(addr);

    // let ptr = &va as _ as usize;
    // let ptr = &va as *const usize as usize;

    while s < end_pa as usize {
        // let flush64 = *((cache_flush + 0x200) as *mut u32);
        unsafe {
            write_volatile((cache_flush + 0x200) as *mut usize, s);
        }
        unsafe {
            write_volatile((cache_flush + 0x200) as *mut usize, A::phys_to_virt(s));
        }

        s += cache_line_size;
    }
    A::fence();

    unsafe { core::arch::asm!("fence") };
}

impl<A: StarfiveHal> StarfiveNic<A> {
    pub fn init1() -> Self {
        Self {
            ioaddr: 0x10020000,
            phantom: PhantomData,
        }
    }
    pub fn init() -> Self {
        log::warn!("######### StarfiveNic init() #########");

        let ioaddr = A::phys_to_virt(0x16040000);

        let mut mac_addr: [u8; 6] = [0; 6];
        crate::dwc_init::stmmac_dwmac4_get_mac_addr(&mut mac_addr);
        info!("stmmac_dwmac4_get_mac_addr original: {:x?}", mac_addr);

        crate::dwc_init::jh7110_clock_reset();
        //crate::dwc_init::phy_config();

        log::info!("-------------------open--------------");
        log::info!("init_dma_rx_desc_rings");

        const EQOS_DESC3_OWN: u32 = (1 << 31);
        const EQOS_DESC3_FD: u32 = (1 << 29);
        const EQOS_DESC3_LD: u32 = (1 << 28);
        const EQOS_DESC3_BUF1V: u32 = (1 << 24);
        /*

          .des0 = (u32)(void *)rx_dma_buf
          .des3 = EQOS_DESC3_OWN | EQOS_DESC3_BUF1V;

          ch0_rxdesc_list_haddress = 0;
          ch0_rxdesc_list_address ==> rx_desc0;
          ch0_rxdesc_ring_length = EQOS_DESCRIPTORS_RX - 1; // 64 - 1

          ch0_rxdesc_tail_pointer = rx_desc63; //rx_desc(EQOS_DESCRIPTORS_RX - 1)


          if (rx_desc->des3 & EQOS_DESC3_OWN) != 0 {
              error!("RX packet not available");
          }

        */

        // dma start 0x8200_1000
        let mut rx_ring = RxRing::<A>::new(0x8200_4000, 64);
        A::fence();
        let rdes_base = rx_ring.rd.phy_addr as u32;

        let size = mem::size_of::<RxDes>() * 64;

        let rdes_end = rdes_base + size as u32;

        let skb_start = 0x8208_0000 as usize;
        for i in 0..64 {
            let buff_addr = skb_start + 4096 * i;

            let va = A::phys_to_virt(buff_addr);
            unsafe {
                core::slice::from_raw_parts_mut(va as *mut u8, 4096).fill(0);
            }
            rx_ring.init_rx_desc(i, buff_addr);
        }

        /*
           tx_desc->des0 = (ulong)(void *)tx_dma_buf;
           tx_desc->des1 = 0;
           tx_desc->des2 = length_packet;
           tx_desc->des3 = EQOS_DESC3_OWN | EQOS_DESC3_FD | EQOS_DESC3_LD | length_packet;

           ch0_txdesc_list_haddress = 0;
           ch0_txdesc_list_address ==>  tx_desc0
           ch0_txdesc_ring_length = EQOS_DESCRIPTORS_TX - 1; // 64 - 1

          // TX tail pointer not written until we need to TX a packet
           ch0_txdesc_tail_pointer --> tx_desc


           if (readl(&tx_desc->des3) & EQOS_DESC3_OWN) == 0 {
               debug!(">>>>>>>>> TX Okay!");
           }


        */

        log::info!("init_dma_tx_desc_rings");
        // dma start 0x8200_2000
        let mut tx_ring = TxRing::<A>::new(0x8200_0000, 64);
        A::fence();

        let tdes_base = tx_ring.td.phy_addr as u32;
        let tskb_start = 0x8202_0000 as usize;
        for i in 0..64 {
            tx_ring.init_tx_desc(i, false);
        }

        A::fence();
        dump_reg(ioaddr);

        unsafe {
            core::arch::asm!("fence iorw, iorw");
        }

        /*

        #define EQOS_DMA_REGS_BASE 0x1000
        struct eqos_dma_regs {
                uint32_t mode;                                  /* 0x1000 */
                uint32_t sysbus_mode;                           /* 0x1004 */
                uint32_t unused_1008[(0x1100 - 0x1008) / 4];    /* 0x1008 */
                uint32_t ch0_control;                           /* 0x1100 */
                uint32_t ch0_tx_control;                        /* 0x1104 */
                uint32_t ch0_rx_control;                        /* 0x1108 */
                uint32_t unused_110c;                           /* 0x110c */
                uint32_t ch0_txdesc_list_haddress;              /* 0x1110 */
                uint32_t ch0_txdesc_list_address;               /* 0x1114 */
                uint32_t ch0_rxdesc_list_haddress;              /* 0x1118 */
                uint32_t ch0_rxdesc_list_address;               /* 0x111c */
                uint32_t ch0_txdesc_tail_pointer;               /* 0x1120 */
                uint32_t unused_1124;                           /* 0x1124 */
                uint32_t ch0_rxdesc_tail_pointer;               /* 0x1128 */
                uint32_t ch0_txdesc_ring_length;                /* 0x112c */
                uint32_t ch0_rxdesc_ring_length;                /* 0x1130 */
            };
        */

        crate::dwc_init::dwmac_dma_reset();
        crate::dwc_init::dwmac_dma_init_rxtx_chan(64, rdes_base, rdes_end, 64, tdes_base, 0);

        crate::dwc_init::set_mac_addr();

        let mut mac_addr: [u8; 6] = [0; 6];
        crate::dwc_init::stmmac_dwmac4_get_mac_addr(&mut mac_addr);
        info!("stmmac_dwmac4_get_mac_addr current: {:x?}", mac_addr);

        crate::dwc_init::dwmac4_core_init();
        crate::dwc_init::dwmac_mtl_queue_set();
        crate::dwc_init::dwmac4_flow_ctrl();
        crate::dwc_init::dma_start_rxtx();
        crate::dwc_init::stmmac_mac_link_up();

        crate::dwc_init::stmmac_dwmac4_set_mac(true); // todo test
                                                      // stmmac_set_mac(true);

        dump_reg(ioaddr);

        // -----------------------------recv

        for i in 0..16 {
            A::mdelay(500);

            let rd = rx_ring.rd.dma_read(i).unwrap();
            log::debug!("rd{} {:x?}", i, rd);

            if rd.rdes3 & EQOS_DESC3_OWN == 0 {
                let length = rd.rdes3 & 0x7fff;

                let buff_addr = A::phys_to_virt(skb_start + 4096 * i);
                warn!(
                    "<<<<<<<<<<<< RX packet length={}, buff@{:#x}",
                    length, buff_addr
                );

                //packet --> eqos->rx_dma_buf
                let mbuf = unsafe { from_raw_parts_mut(buff_addr as *mut u8, length as usize) };

                info!("<<<<<<<<<<<< Buff: {:x?}", mbuf);
            }

            let value = unsafe { read_volatile((ioaddr + 0x115c) as *mut u32) };
            log::info!("Current Host rx buffer ----- 0x115c={:#x?}", value);
        }

        log::info!("-------------------sending------------------------");
        let packet: Box<[u8]> = Box::new([
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xaa, 0xbb, 0xcc, 0xdd, 0x05, 0x06, 0x08, 0x06,
            0x00, 0x01, 0x08, 0x00, 0x06, 0x04, 0x00, 0x01, 0xaa, 0xbb, 0xcc, 0xdd, 0x05, 0x06,
            0xc0, 0xa8, 0x0a, 0x68, // 192.168.10.104, local IP
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xa8, 0x0a, 0x32, // 192.168.10.50
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        ]);

        // for i in 0..64{
        for i in 0..16 {
            //
            // tskb_start = 0x8202_0000
            let buff_addr = tskb_start + 4096 * i;

            let packet_pa: usize = tskb_start + 4096 * i;
            let packet_va = A::phys_to_virt(packet_pa);

            /*
            let raw_pointer = packet.as_mut_ptr();
            unsafe {
                core::ptr::copy_nonoverlapping(raw_pointer as *const u8, buff as *mut u8, 0x3c);
            }
            */

            let pbufs_len = packet.len();
            let pbuf = unsafe { from_raw_parts_mut(packet_va as *mut u8, pbufs_len) };
            pbuf.copy_from_slice(&packet);

            sifive_ccache_flush_range::<A>(0x8200_0000, 0x820c_0000);

            ///////////
            let mut td = tx_ring.td.dma_read(i).unwrap();

            td.tdes0 = buff_addr as u32;
            td.tdes1 = 0;
            td.tdes2 = pbufs_len as u32;
            td.tdes3 = EQOS_DESC3_OWN | EQOS_DESC3_FD | EQOS_DESC3_LD | (pbufs_len as u32);

            unsafe {
                core::arch::asm!("fence	iorw,iorw");
            }

            sifive_ccache_flush_range::<A>(0x8200_0000, 0x820c_0000);

            tx_ring.td.dma_write(i, &td);
            unsafe {
                core::arch::asm!("fence	ow,ow");
            }
            A::fence();

            let tail_ptr = tdes_base + (mem::size_of::<TxDes>() * (i + 1)) as u32;
            log::info!("td {:x?}, tail_ptr={:#x}", td, tail_ptr);

            // ch0_txdesc_tail_pointer
            unsafe {
                write_volatile((ioaddr + 0x1120) as *mut u32, tail_ptr);
            }

            loop {
                unsafe {
                    core::arch::asm!("fence	ow,ow");
                }

                let tx_desc = tx_ring.td.dma_read(i).unwrap();
                if (tx_desc.tdes3 & EQOS_DESC3_OWN) == 0 {
                    warn!(
                        ">>>>>>>> TX {} lengh={}, tx_desc:{:x?}",
                        i, pbufs_len, tx_desc
                    );

                    break;
                }
                A::mdelay(50);
            }

            let value = unsafe { read_volatile((ioaddr + 0x1154) as *mut u32) };
            log::info!("Current Host tx buffer={:#x?} >>>>>>>>>", value);

            {
                let cur_tx_desc = unsafe { read_volatile((ioaddr + 0x1144) as *mut u32) };
                let cur_rx_desc = unsafe { read_volatile((ioaddr + 0x114c) as *mut u32) };

                log::info!(
                    "Current  tx desc@{:#x?} rxdesc@{:#x}",
                    cur_tx_desc,
                    cur_rx_desc
                );
            }

            crate::dwc_init::dma_status_read();

            let mut last = 0;
            for i in 0..64 {
                //recv
                A::mdelay(50);

                let rd = rx_ring.rd.dma_read(i).unwrap();

                if rd.rdes3 & EQOS_DESC3_OWN == 0 {
                    last = i;
                    continue;
                } else {
                    let rd = rx_ring.rd.dma_read(last).unwrap();

                    let length = rd.rdes3 & 0x7fff;

                    let buff_addr = A::phys_to_virt(skb_start + 4096 * last);
                    warn!(
                        "<<<<<<<<<<<< RX {} packet length={}, buff@{:#x}",
                        last, length, buff_addr
                    );

                    //packet --> eqos->rx_dma_buf
                    let mbuf = unsafe { from_raw_parts_mut(buff_addr as *mut u8, length as usize) };
                    info!("RX Buff: {:x?}", mbuf);

                    break;
                }
            }
        }

        log::warn!("######### StarfiveNic init return #########");

        Self {
            ioaddr,
            phantom: PhantomData,
        }
    }
}

impl<A: StarfiveHal> BaseDriverOps for StarfiveNic<A> {
    fn device_name(&self) -> &str {
        "starfive"
    }

    fn device_type(&self) -> DeviceType {
        DeviceType::Net
    }
}

impl<A: StarfiveHal> NetDriverOps for StarfiveNic<A> {
    fn mac_address(&self) -> crate::EthernetAddress {
        crate::EthernetAddress([0xaa, 0xbb, 0xcc, 0xdd, 0x05, 0x06])
    }

    fn tx_queue_size(&self) -> usize {
        1
    }

    fn rx_queue_size(&self) -> usize {
        1
    }

    fn can_receive(&self) -> bool {
        true
    }

    fn can_transmit(&self) -> bool {
        true
    }

    fn recycle_rx_buffer(&mut self, rx_buf: NetBufPtr) -> DevResult {
        Err(DevError::Unsupported)
    }

    fn recycle_tx_buffers(&mut self) -> DevResult {
        Err(DevError::Unsupported)
    }

    fn receive(&mut self) -> DevResult<NetBufPtr> {
        Err(DevError::Unsupported)
    }

    fn transmit(&mut self, tx_buf: NetBufPtr) -> DevResult {
        Err(DevError::Unsupported)
    }

    fn alloc_tx_buffer(&mut self, size: usize) -> DevResult<NetBufPtr> {
        Err(DevError::Unsupported)
    }
}

pub const MII_BUSY: u32 = 1 << 0;
pub const MII_WRITE: u32 = 1 << 1;
pub const MII_CLKRANGE_60_100M: u32 = 0;
pub const MII_CLKRANGE_100_150M: u32 = 0x4;
pub const MII_CLKRANGE_20_35M: u32 = 0x8;
pub const MII_CLKRANGE_35_60M: u32 = 0xC;
pub const MII_CLKRANGE_150_250M: u32 = 0x10;
pub const MII_CLKRANGE_250_300M: u32 = 0x14;
pub const MIIADDRSHIFT: u32 = 11;
pub const MIIREGSHIFT: u32 = 6;
pub const MII_REGMSK: u32 = 0x1F << 6;
pub const MII_ADDRMSK: u32 = 0x1F << 11;

use alloc::vec::Vec;
use core::mem;

#[derive(Debug)]
pub struct Dma<T> {
    pub count: usize,
    pub phy_addr: usize,
    pub cpu_addr: *mut T,
}

impl<T> Dma<T> {
    pub fn new(cpu_addr: *mut T, phy_addr: usize, count: usize) -> Self {
        Self {
            count: count,
            phy_addr: phy_addr,
            cpu_addr: cpu_addr,
        }
    }

    pub fn dma_read(&self, index: usize) -> Option<T> {
        if index >= self.count {
            error!("Dma read_volatile index:{:?} count:{:?}", index, self.count);
            return None;
        }
        let ptr = self.cpu_addr.wrapping_add(index);
        Some(unsafe { ptr.read_volatile() })
    }

    pub fn dma_write(&self, index: usize, value: &T) -> bool
    where
        T: Copy,
    {
        if index >= self.count {
            error!(
                "Dma write_volatile index:{:?} count:{:?}",
                index, self.count
            );
            return false;
        }
        let ptr = self.cpu_addr.wrapping_add(index);
        unsafe { ptr.write_volatile(*value) };
        true
    }
}

#[derive(Copy, Clone, Debug)]
#[repr(C, packed)]
pub struct RxDes {
    pub rdes0: u32,
    pub rdes1: u32,
    pub rdes2: u32,
    pub rdes3: u32,
}

pub struct RxRing<A> {
    pub rd: Dma<RxDes>,
    pub idx: usize,
    pub skbuf: Vec<usize>,
    phantom: PhantomData<A>,
}

impl<A: StarfiveHal> RxRing<A> {
    pub fn new(pa: usize, count: usize) -> Self {
        // let size = mem::size_of::<RxDes>() * count;
        // let pages = (size + 0x1000 - 1) / 0x1000;
        // let (va, pa) = A::dma_alloc_pages(pages);

        //let count = 64;
        //let pa = 0x8200_1000;
        let va = A::phys_to_virt(pa);

        let rd_dma = Dma::new(va as _, pa, count);
        let skbuf = Vec::new();

        Self {
            rd: rd_dma,
            idx: 0,
            skbuf: skbuf,
            phantom: PhantomData,
        }
    }

    pub fn init_rx_desc(&mut self, idx: usize, skb_phys_addr: usize) {
        let mut rd = RxDes {
            rdes0: skb_phys_addr as u32,
            rdes1: 0,
            rdes2: 0,
            rdes3: 0x81000000, // EQOS_DESC3_OWN | EQOS_DESC3_BUF1V;
        };

        self.rd.dma_write(idx, &rd);
    }
}

#[derive(Copy, Clone, Debug)]
#[repr(C, packed)]
pub struct TxDes {
    pub tdes0: u32,
    pub tdes1: u32,
    pub tdes2: u32,
    pub tdes3: u32,
    // pub tdes4: u32,
    // pub tdes5: u32,
    // pub tdes6: u32,
    // pub tdes7: u32,
}

pub struct TxRing<A> {
    pub td: Dma<TxDes>,
    pub idx: usize,
    pub skbuf: Vec<usize>,
    phantom: PhantomData<A>,
}

impl<A: StarfiveHal> TxRing<A> {
    pub fn new(pa: usize, count: usize) -> Self {
        //let count = 64;
        //let pa = 0x8200_2000;
        let va = A::phys_to_virt(pa);

        let td_dma = Dma::new(va as *mut TxDes, pa, count);
        let skbuf = Vec::new();

        Self {
            td: td_dma,
            idx: 0,
            skbuf: skbuf,
            phantom: PhantomData,
        }
    }

    pub fn init_tx_desc(&mut self, idx: usize, end: bool) {
        let mut td: TxDes = TxDes {
            tdes0: 0,
            tdes1: 0,
            tdes2: 0,
            tdes3: 0,
        };

        // td.tdes3 &= !(1 << 31);

        // if end {
        //     td.tdes3 |= 1 << 21;
        // }

        self.td.dma_write(idx, &td);
    }

    pub fn set_skb_addr(&mut self, idx: usize, skb_addr: usize) {
        let mut td = self.td.dma_read(idx).unwrap();
        td.tdes0 = skb_addr as u32;
        self.td.dma_write(idx, &td);
    }
}

pub fn stmmac_set_mac(ioaddr: usize, enable: bool) {
    let old_val: u32;
    let mut value: u32;

    log::info!("stmmac_set_mac--------------------enable={:?}", enable);

    old_val = unsafe { read_volatile(ioaddr as *mut u32) };
    value = old_val;

    if enable {
        value |= MAC_ENABLE_RX | MAC_ENABLE_TX;
    } else {
        value &= !(MAC_ENABLE_TX | MAC_ENABLE_RX);
    }

    if value != old_val {
        unsafe { write_volatile(ioaddr as *mut u32, value) }
    }
}

pub trait StarfiveHal {
    fn phys_to_virt(pa: usize) -> usize {
        pa
    }
    fn virt_to_phys(va: usize) -> usize {
        va
    }

    fn dma_alloc_pages(pages: usize) -> (usize, usize);

    fn dma_free_pages(vaddr: usize, pages: usize);

    fn mdelay(m_times: usize);

    fn fence();
}

pub fn dump_reg(ioaddr: usize) {
    log::info!("------------------------------dumpreg--------------------------------------");
    for i in 0..10 {
        let value = unsafe { read_volatile((ioaddr + 0x110 + i * 4) as *mut u32) };
        log::info!("MAC REGS 0x110+{:#x} = {:#x?}", i * 4, value);
    }
    /*
    for i in 0..25 {
        let value = unsafe { read_volatile((ioaddr + 0x1000 + i * 4) as *mut u32) };
        log::info!("DMA REGS 0x1000+{:#x} = {:#x?}", i * 4, value);
    } */
    for i in 0..25 {
        let value = unsafe { read_volatile((ioaddr + 0x1100 + i * 4) as *mut u32) };
        log::info!("DMA CHAN REGS 0x1100+{:#x} = {:#x?}", i * 4, value);
    }
}

// pub fn ytphy_read_ext<A: StarfiveHal>(iobase: usize, reg: u32) -> u32 {

//     dw_mdio_write::<A>(iobase, 0x1e, reg);

//     let value = dw_mdio_read::<A>(iobase, 0x1f);

//     value

// }

// pub fn ytphy_write_ext<A: StarfiveHal>(iobase: usize, reg: u32, value: u32) {

//     dw_mdio_write::<A>(iobase, 0x1e, reg);

//     dw_mdio_write::<A>(iobase, 0x1f, value);

// }

// pub fn dw_mdio_write<A: StarfiveHal>(iobase: usize, reg: u32, value: u32) {

//     let addr = 0x3;

//     unsafe {
//         write_volatile((iobase + 0x14) as *mut u32, value);
//     }

//     let mut miiaddr =
//         ((addr << MIIADDRSHIFT) & MII_ADDRMSK) | ((reg << MIIREGSHIFT) & MII_REGMSK) | MII_WRITE;

//     miiaddr = miiaddr | MII_CLKRANGE_150_250M | MII_BUSY;
//     log::info!(
//         "dw_mdio_write  addr={:#x?} reg={:#x?} val_0x14={:#x?}, val_0x10={:#x?}\n",
//         addr,
//         reg,
//         value,
//         miiaddr | MII_CLKRANGE_150_250M | MII_BUSY
//     );

//     unsafe {
//         write_volatile((iobase + 0x10) as *mut u32, miiaddr);
//     }

//     loop {
//         let value = unsafe { read_volatile((iobase + 0x10) as *mut u32) };

//         if value & MII_BUSY != 1 {
//             break;
//         }
//         A::mdelay(10);
//     }
// }

// pub fn dw_mdio_read<A: StarfiveHal>(iobase: usize, reg: u32) -> u32 {

//     let addr = 0x3;

//     let mut miiaddr = ((addr << MIIADDRSHIFT) & MII_ADDRMSK) | ((reg << MIIREGSHIFT) & MII_REGMSK);

//     miiaddr = miiaddr | MII_CLKRANGE_150_250M | MII_BUSY;

//     log::info!("dw_mdio_read  reg={:#x?}", reg);

//     unsafe {
//         write_volatile((iobase + 0x10) as *mut u32, miiaddr);
//     }

//     loop {
//         let value = unsafe { read_volatile((iobase + 0x10) as *mut u32) };

//         if value & MII_BUSY != 1 {
//             let value = unsafe { read_volatile((iobase + 0x14) as *mut u32) };
//             return value;
//         }
//         A::mdelay(10);
//     }
// }

// pub const DESC_TXSTS_OWNBYDMA		:u32 = (1 << 31);
// pub const DESC_TXSTS_TXINT		:u32 = (1 << 30);
// pub const DESC_TXSTS_TXLAST		:u32 = (1 << 29);
// pub const DESC_TXSTS_TXFIRST		:u32 = (1 << 28);
// pub const DESC_TXSTS_TXCRCDIS		:u32 = (1 << 27);

// pub const DESC_TXSTS_TXPADDIS		    :u32 = (1 << 26);
// pub const DESC_TXSTS_TXCHECKINSCTRL	:u32 = (3 << 22);
// pub const DESC_TXSTS_TXRINGEND		:u32 = (1 << 21);
// pub const DESC_TXSTS_TXCHAIN		    :u32 = (1 << 20);
// pub const DESC_TXSTS_MSK			    :u32 = (0x1FFFF << 0);

// pub const DESC_TXCTRL_SIZE1MASK	:u32 = 	(0x7FF << 0);
// pub const DESC_TXCTRL_SIZE1SHFT	:u32 = 	(0);
// pub const DESC_TXCTRL_SIZE2MASK	:u32 = 	(0x7FF << 11);
// pub const DESC_TXCTRL_SIZE2SHFT	:u32 = 	(11);

pub fn mdio_write<A: StarfiveHal>(ioaddr: usize, data: u32, value: u32) {
    loop {
        let value = unsafe { read_volatile((ioaddr + 0x10) as *mut u32) };

        if value & MII_BUSY != 1 {
            break;
        }
        A::mdelay(10);
    }

    unsafe {
        write_volatile((ioaddr + 0x14) as *mut u32, data);
        write_volatile((ioaddr + 0x10) as *mut u32, value);
    }

    loop {
        let value = unsafe { read_volatile((ioaddr + 0x10) as *mut u32) };

        if value & MII_BUSY != 1 {
            break;
        }
        A::mdelay(10);
    }
}
