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

const RECV_BATCH_SIZE: usize = 64;
const RX_BUFFER_SIZE: usize = 4096;
const NET_BUF_LEN: usize = 4096;

pub struct E1000Nic<'a, K: KernelFunc> {
    inner: E1000Device<'a, K>,
    // rx_buffer_queue: VecDeque<NetBufPtr>,
    buf_pool: Arc<NetBufPool>,
    free_tx_bufs: Vec<NetBufBox>,
}

unsafe impl<'a, K: KernelFunc> Sync for E1000Nic<'a, K> {}
unsafe impl<'a, K: KernelFunc> Send for E1000Nic<'a, K> {}

impl<'a, K: KernelFunc> E1000Nic<'a, K> {
    pub fn init(mut kfn: K, mapped_regs: usize) -> DevResult<Self> {
        info!("E1000Nic init");
        const NONE_BUF: Option<NetBufBox> = None;
        // let rx_buffers = [NONE_BUF; RX_BUFFER_SIZE];
        // let tx_buffers = [NONE_BUF; RX_BUFFER_SIZE];
        let buf_pool = NetBufPool::new(2 * RX_BUFFER_SIZE, NET_BUF_LEN)?;
        let free_tx_bufs = Vec::with_capacity(RX_BUFFER_SIZE);
        let inner = E1000Device::<K>::new(kfn, mapped_regs).map_err(|err| {
            log::error!("Failed to initialize e1000 device: {:?}", err);
            DevError::BadState
        })?;

        let mut dev = Self {
            // rx_buffers,
            // tx_buffers,
            buf_pool,
            free_tx_bufs,
            inner,
        };

        for _ in 0..RX_BUFFER_SIZE {
            let mut tx_buf = dev.buf_pool.alloc_boxed().ok_or(DevError::NoMemory)?;
            tx_buf.set_header_len(20); // ipv4 header length
            dev.free_tx_bufs.push(tx_buf);
        }
        Ok(dev)
    }
}

impl<'a, K: KernelFunc> BaseDriverOps for E1000Nic<'a, K> {
    fn device_name(&self) -> &str {
        "e1000:Intel 82540EP/EM"
    }

    fn device_type(&self) -> DeviceType {
        DeviceType::Net
    }
}

impl<'a, K: KernelFunc> NetDriverOps for E1000Nic<'a, K> {
    fn mac_address(&self) -> EthernetAddress {
        EthernetAddress([0x00, 0x0c, 0x29, 0x3e, 0x4f, 0x50])
    }

    fn rx_queue_size(&self) -> usize {
        256
    }

    fn tx_queue_size(&self) -> usize {
        256
    }

    fn can_receive(&self) -> bool {
        true
    }

    fn can_transmit(&self) -> bool {
        true
    }

    fn recycle_rx_buffer(&mut self, rx_buf: NetBufPtr) -> DevResult {
        drop(rx_buf);
        Ok(())
    }

    fn recycle_tx_buffers(&mut self) -> DevResult {
        Ok(())
    }

    fn receive(&mut self) -> DevResult<NetBufPtr> {
        match self.inner.e1000_recv() {
            None => Err(DevError::Again),
            Some(packets) => {
                let total_len = packets.iter().map(|p| p.len()).sum();
                let mut buf = Box::new(Vec::<u8>::with_capacity(total_len));
                let mut offset = 0;
                for packet in packets {
                    //buf[offset..offset + packet.len()].copy_from_slice(&packet);
                    buf.extend_from_slice(&packet[..]);
                    offset += packet.len();
                }
                Ok(NetBufPtr::new(
                    NonNull::dangling(),
                    NonNull::new(Box::into_raw(buf) as *mut u8).unwrap(),
                    total_len,
                ))
            }
        }
    }

    fn transmit(&mut self, tx_buf: NetBufPtr) -> DevResult {
        self.inner.e1000_transmit(tx_buf.packet());
        Ok(())
    }

    fn alloc_tx_buffer(&mut self, size: usize) -> DevResult<NetBufPtr> {
        info!("alloc_tx_buffer size={}", size);
        // 0. Allocate a buffer from the queue.
        let mut net_buf = self.free_tx_bufs.pop().ok_or(DevError::NoMemory)?;
        let pkt_len = size;

        // 1. Check if the buffer is large enough.
        let hdr_len = net_buf.header_len();
        if hdr_len + pkt_len > net_buf.capacity() {
            return Err(DevError::InvalidParam);
        }
        net_buf.set_packet_len(pkt_len);

        // 2. Return the buffer.
        Ok(net_buf.into_buf_ptr())
        // Err(DevError::NoMemory)
    }
}
