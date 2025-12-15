use alloc::vec;

use axdriver_base::{BaseDriverOps, DevError, DevResult, DeviceType};
use gpt_disk_io::{
    BlockIo, Disk, DiskError,
    gpt_disk_types::{BlockSize, GptPartitionEntry, Lba, LbaRangeInclusive},
};
use log::{debug, info};

use super::prelude::*;

struct BlockDriverAdapter<'a, T>(&'a mut T);

impl<T: BlockDriverOps> BlockIo for BlockDriverAdapter<'_, T> {
    type Error = DevError;

    fn block_size(&self) -> BlockSize {
        BlockSize::from_usize(self.0.block_size()).unwrap()
    }

    fn num_blocks(&mut self) -> Result<u64, Self::Error> {
        Ok(self.0.num_blocks())
    }

    fn read_blocks(&mut self, start_lba: Lba, dst: &mut [u8]) -> Result<(), Self::Error> {
        self.block_size().assert_valid_block_buffer(dst);
        for (i, chunk) in dst.chunks_exact_mut(self.0.block_size()).enumerate() {
            self.0.read_block(start_lba.to_u64() + i as u64, chunk)?;
        }
        Ok(())
    }

    fn write_blocks(&mut self, start_lba: Lba, src: &[u8]) -> Result<(), Self::Error> {
        self.block_size().assert_valid_block_buffer(src);
        for (i, chunk) in src.chunks_exact(self.0.block_size()).enumerate() {
            self.0.write_block(start_lba.to_u64() + i as u64, chunk)?;
        }
        Ok(())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.0.flush()
    }
}

fn map_disk_error(err: DiskError<DevError>) -> DevError {
    match err {
        DiskError::BufferTooSmall => DevError::InvalidParam,
        DiskError::Overflow => DevError::BadState,
        DiskError::BlockSizeSmallerThanPartitionEntry => DevError::InvalidParam,
        DiskError::Io(e) => e,
    }
}

/// A GPT partition.
pub struct GptPartitionDev<T> {
    inner: T,
    range: LbaRangeInclusive,
}

impl<T: BlockDriverOps> GptPartitionDev<T> {
    /// Creates a new GPT partition device from the given block storage device
    /// driver.
    ///
    /// Will use the first partition that matches the given selection criteria.
    pub fn new<F>(mut inner: T, mut predicate: F) -> DevResult<Self>
    where
        F: FnMut(&GptPartitionEntry) -> bool,
    {
        let mut block_buf = vec![0u8; 512];

        let block_io = BlockDriverAdapter(&mut inner);
        let mut disk = Disk::new(block_io).map_err(map_disk_error)?;

        let primary_header = disk
            .read_primary_gpt_header(&mut block_buf)
            .map_err(map_disk_error)?;
        debug!("{}", primary_header);
        assert!(primary_header.is_signature_valid());

        let mut range = None;
        let layout = primary_header.get_partition_entry_array_layout().unwrap();
        for entry in disk
            .gpt_partition_entry_array_iter(layout, &mut block_buf)
            .map_err(map_disk_error)?
        {
            let entry = entry.map_err(map_disk_error)?;
            if entry.is_used() {
                debug!("{}", entry);
                if predicate(&entry) {
                    info!("Selected partition: {}", entry.name);
                    range = entry.lba_range();
                    break;
                }
            }
        }
        drop(disk);
        let range = range.ok_or(DevError::Io)?;
        Ok(Self { inner, range })
    }
}

impl<T: BlockDriverOps> BaseDriverOps for GptPartitionDev<T> {
    fn device_name(&self) -> &str {
        self.inner.device_name()
    }

    fn device_type(&self) -> DeviceType {
        DeviceType::Block
    }
}

impl<T: BlockDriverOps> BlockDriverOps for GptPartitionDev<T> {
    fn num_blocks(&self) -> u64 {
        self.range.num_blocks()
    }

    fn block_size(&self) -> usize {
        self.inner.block_size()
    }

    fn read_block(&mut self, block_id: u64, buf: &mut [u8]) -> DevResult {
        if block_id > (self.range.end().to_u64() - self.range.start().to_u64()) {
            return Err(DevError::InvalidParam);
        }
        self.inner
            .read_block(self.range.start().to_u64() + block_id, buf)
    }

    fn write_block(&mut self, block_id: u64, buf: &[u8]) -> DevResult {
        if block_id > (self.range.end().to_u64() - self.range.start().to_u64()) {
            return Err(DevError::InvalidParam);
        }
        self.inner
            .write_block(self.range.start().to_u64() + block_id, buf)
    }

    fn flush(&mut self) -> DevResult {
        self.inner.flush()
    }
}
