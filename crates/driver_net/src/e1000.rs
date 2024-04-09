use alloc::boxed::Box;
use alloc::vec::Vec;
use core::convert::From;
use core::{mem::ManuallyDrop, ptr::NonNull};

use alloc::{collections::VecDeque, sync::Arc};
use driver_common::{BaseDriverOps, DevError, DevResult, DeviceType};
use e1000_driver::e1000::E1000Device;

use crate::{EthernetAddress, NetBufBox, NetBufPool, NetBufPtr, NetDriverOps};
use log::info;

pub use e1000_driver::e1000::KernelFunc;

extern crate alloc;

const QS: usize = 64;
const NET_BUF_LEN: usize = 1526;

pub struct E1000Nic<'a, K: KernelFunc> {
    inner: E1000Device<'a, K>,
    rx_buffer_queue: VecDeque<NetBufPtr>,
}

unsafe impl<'a, K: KernelFunc> Sync for E1000Nic<'a, K> {}
unsafe impl<'a, K: KernelFunc> Send for E1000Nic<'a, K> {}

impl<'a, K: KernelFunc> E1000Nic<'a, K> {
    pub fn init(mut kfn: K, mapped_regs: usize) -> DevResult<Self> {
        info!("E1000Nic init");
        const NONE_BUF: Option<NetBufBox> = None;
        let rx_buffer_queue = VecDeque::with_capacity(QS);
        let inner = E1000Device::<K>::new(kfn, mapped_regs).map_err(|err| {
            log::error!("Failed to initialize e1000 device: {:?}", err);
            DevError::BadState
        })?;
        let mut dev = Self {
            inner,
            rx_buffer_queue,
        };
        Ok(dev)
    }
}

impl<'a, K: KernelFunc> BaseDriverOps for E1000Nic<'a, K> {
    fn device_name(&self) -> &str {
        "e1000:Intel-82540EP/EM"
    }

    fn device_type(&self) -> DeviceType {
        DeviceType::Net
    }
}

impl<'a, K: KernelFunc> NetDriverOps for E1000Nic<'a, K> {
    fn mac_address(&self) -> EthernetAddress {
        EthernetAddress([0x52, 0x54, 0x00, 0x12, 0x34, 0x56])
    }

    fn rx_queue_size(&self) -> usize {
        QS
    }

    fn tx_queue_size(&self) -> usize {
        QS
    }

    fn can_receive(&self) -> bool {
        !self.rx_buffer_queue.is_empty()
    }

    fn can_transmit(&self) -> bool {
        //!self.free_tx_bufs.is_empty()
        true
    }

    fn recycle_rx_buffer(&mut self, rx_buf: NetBufPtr) -> DevResult {
        unsafe {
            drop(Box::from_raw(rx_buf.raw_ptr::<u8>()));
        }
        drop(rx_buf);
        Ok(())
    }

    fn recycle_tx_buffers(&mut self) -> DevResult {
        // drop tx_buf
        Ok(())
    }

    fn receive(&mut self) -> DevResult<NetBufPtr> {
        if !self.rx_buffer_queue.is_empty() {
            // RX buffer have received packets.
            Ok(self.rx_buffer_queue.pop_front().unwrap())
        } else {
            match self.inner.e1000_recv() {
                None => Err(DevError::Again),
                Some(packets) => {
                    for packet in packets {
                        info!("received packet number={}", packet.len());
                        let mut buf = Box::new(packet);
                        let buf_ptr = buf.as_mut_ptr() as *mut u8;
                        let buf_len = buf.len();
                        let rx_buf = NetBufPtr::new(
                            NonNull::new(Box::into_raw(buf) as *mut u8).unwrap(),
                            NonNull::new(buf_ptr).unwrap(),
                            buf_len,
                        );

                        self.rx_buffer_queue.push_back(rx_buf);
                    }

                    Ok(self.rx_buffer_queue.pop_front().unwrap())
                }
            }
        }
    }

    fn transmit(&mut self, tx_buf: NetBufPtr) -> DevResult {
        let ret = self.inner.e1000_transmit(tx_buf.packet());
        unsafe {
            drop(Box::from_raw(tx_buf.raw_ptr::<u8>()));
        }
        if ret < 0 {
            Err(DevError::Again)
        } else {
            Ok(())
        }
    }

    fn alloc_tx_buffer(&mut self, size: usize) -> DevResult<NetBufPtr> {
        let mut tx_buf = Box::new(alloc::vec![0; size]);
        let tx_buf_ptr = tx_buf.as_mut_ptr();

        Ok(NetBufPtr::new(
            NonNull::new(Box::into_raw(tx_buf) as *mut u8).unwrap(),
            NonNull::new(tx_buf_ptr).unwrap(),
            size,
        ))
    }
}
