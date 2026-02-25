/// W25Q128FV external SPI flash driver (async, embassy)
///
/// Commands used:
///   0x06 — Write Enable
///   0x05 — Read Status Register 1
///   0x03 — Read Data
///   0x02 — Page Program (256-byte pages)
///   0x20 — Sector Erase (4K)
///   0x9F — JEDEC ID
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::spi;
use embassy_time::{Duration, Timer};

const CMD_WRITE_ENABLE: u8 = 0x06;
const CMD_READ_STATUS1: u8 = 0x05;
const CMD_PAGE_PROGRAM: u8 = 0x02;
const CMD_SECTOR_ERASE: u8 = 0x20;
const CMD_READ_DATA: u8 = 0x03;
const CMD_JEDEC_ID: u8 = 0x9F;

const STATUS_BUSY: u8 = 0x01;

/// Config storage address on the W25Q (first 4K sector)
pub const CONFIG_ADDR: u32 = 0x0000_0000;

pub struct W25q {
    spi: spi::Spi<'static, Async>,
    cs: Output<'static>,
}

impl W25q {
    pub fn new(spi: spi::Spi<'static, Async>, cs: Output<'static>) -> Self {
        Self { spi, cs }
    }

    /// Read JEDEC manufacturer + device ID (3 bytes)
    pub async fn read_jedec_id(&mut self) -> Result<[u8; 3], spi::Error> {
        self.cs.set_low();
        self.spi.write(&[CMD_JEDEC_ID]).await?;
        let mut id = [0u8; 3];
        self.spi.read(&mut id).await?;
        self.cs.set_high();
        Ok(id)
    }

    async fn write_enable(&mut self) -> Result<(), spi::Error> {
        self.cs.set_low();
        self.spi.write(&[CMD_WRITE_ENABLE]).await?;
        self.cs.set_high();
        Ok(())
    }

    async fn wait_busy(&mut self) -> Result<(), spi::Error> {
        loop {
            self.cs.set_low();
            self.spi.write(&[CMD_READ_STATUS1]).await?;
            let mut status = [0u8; 1];
            self.spi.read(&mut status).await?;
            self.cs.set_high();
            if status[0] & STATUS_BUSY == 0 {
                return Ok(());
            }
            Timer::after(Duration::from_millis(1)).await;
        }
    }

    /// Erase a 4K sector (addr must be 4K-aligned)
    pub async fn sector_erase(&mut self, addr: u32) -> Result<(), spi::Error> {
        self.write_enable().await?;
        self.cs.set_low();
        self.spi
            .write(&[
                CMD_SECTOR_ERASE,
                (addr >> 16) as u8,
                (addr >> 8) as u8,
                addr as u8,
            ])
            .await?;
        self.cs.set_high();
        self.wait_busy().await
    }

    /// Program up to 256 bytes within a single page
    async fn page_program(&mut self, addr: u32, data: &[u8]) -> Result<(), spi::Error> {
        self.write_enable().await?;
        self.cs.set_low();
        self.spi
            .write(&[
                CMD_PAGE_PROGRAM,
                (addr >> 16) as u8,
                (addr >> 8) as u8,
                addr as u8,
            ])
            .await?;
        self.spi.write(data).await?;
        self.cs.set_high();
        self.wait_busy().await
    }

    /// Read data from any address
    pub async fn read(&mut self, addr: u32, buf: &mut [u8]) -> Result<(), spi::Error> {
        self.cs.set_low();
        self.spi
            .write(&[
                CMD_READ_DATA,
                (addr >> 16) as u8,
                (addr >> 8) as u8,
                addr as u8,
            ])
            .await?;
        self.spi.read(buf).await?;
        self.cs.set_high();
        Ok(())
    }

    /// Write data, handling 256-byte page boundaries automatically
    pub async fn write(&mut self, addr: u32, data: &[u8]) -> Result<(), spi::Error> {
        let mut offset = 0usize;
        let mut current_addr = addr;

        while offset < data.len() {
            let page_remaining = 256 - (current_addr as usize % 256);
            let chunk_len = core::cmp::min(page_remaining, data.len() - offset);
            self.page_program(current_addr, &data[offset..offset + chunk_len])
                .await?;
            offset += chunk_len;
            current_addr += chunk_len as u32;
        }

        Ok(())
    }

    /// Erase sector, then write data
    pub async fn erase_and_write(&mut self, addr: u32, data: &[u8]) -> Result<(), spi::Error> {
        self.sector_erase(addr).await?;
        self.write(addr, data).await
    }
}
