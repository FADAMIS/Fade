use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use mpu6000::bus::RegAccess;
use mpu6000::registers::{AccelerometerSensitive, GyroSensitive};
use mpu6000::{ClockSource, MPU6000};

/// Wrapper to adapt a u32 delay to also satisfy DelayMs<u8>
pub struct DelayCompat<D>(pub D);

impl<D> DelayMs<u8> for DelayCompat<D>
where
    D: DelayMs<u32>,
{
    fn delay_ms(&mut self, ms: u8) {
        self.0.delay_ms(ms as u32);
    }
}

impl<D> DelayMs<u32> for DelayCompat<D>
where
    D: DelayMs<u32>,
{
    fn delay_ms(&mut self, ms: u32) {
        self.0.delay_ms(ms);
    }
}

impl<D> DelayUs<u32> for DelayCompat<D>
where
    D: DelayUs<u32>,
{
    fn delay_us(&mut self, us: u32) {
        self.0.delay_us(us);
    }
}

pub struct GyroManager<BUS, D> {
    mpu: MPU6000<BUS>,
    #[allow(dead_code)]
    delay: DelayCompat<D>,
}

impl<BUS, D, E> GyroManager<BUS, D>
where
    BUS: RegAccess<Error = E>,
    D: DelayUs<u32> + DelayMs<u32>,
{
    pub fn new(mut mpu: MPU6000<BUS>, delay: D) -> Self {
        let mut delay = DelayCompat(delay);

        mpu.reset(&mut delay).ok();
        delay.delay_ms(100u32);

        mpu.set_sleep(false).ok();
        mpu.set_clock_source(ClockSource::PLLGyroZ).ok();
        mpu.set_accelerometer_sensitive(AccelerometerSensitive::Sensitive16384)
            .ok();
        mpu.set_gyro_sensitive(GyroSensitive::Sensitive65_5).ok();

        Self { mpu, delay }
    }

    pub fn read_gyro(&mut self) -> Result<[i16; 3], E> {
        self.mpu.read_gyro().map(|g| g.0)
    }

    pub fn read_accel(&mut self) -> Result<[i16; 3], E> {
        self.mpu.read_acceleration().map(|a| a.0)
    }
}
