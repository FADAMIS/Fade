use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::OutputPin;
use mpu6000::bus::SpiBus;
use mpu6000::Error;
use mpu6000::MPU6000 as Mpu6000;

pub struct Gyro<SPI, CS, D> {
    mpu: Mpu6000<SpiBus<SPI, CS, D>>,
}

impl<SPI, CS, D, E> Gyro<SPI, CS, D>
where
    SPI: Write<u8, Error = E> + Transfer<u8, Error = E>,
    CS: OutputPin,
    D: DelayUs<u8>,
{
    pub fn new(spi: SPI, cs: CS, delay: D) -> Self {
        let bus = SpiBus::new(spi, cs, delay);
        let mpu = Mpu6000::new(bus);
        Self { mpu }
    }

    pub fn init(&mut self) -> Result<(), Error<E>> {
        self.mpu.init()
    }

    pub fn get_gyro(&mut self) -> Result<[f32; 3], Error<E>> {
        self.mpu.get_gyro()
    }
}
