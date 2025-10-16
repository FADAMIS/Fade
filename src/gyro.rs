use embassy_time::{Duration, Timer};
use icm426xx::{Ready, Uninitialized, ICM42688};

/// Configuration for ICM42688 gyroscope
#[derive(Clone, Copy)]
pub struct Icm42688Config {
    /// Gyro full-scale range in degrees per second
    pub gyro_range: GyroRange,
    /// Output data rate in Hz
    pub odr: OutputDataRate,
    /// Low-pass filter bandwidth
    pub filter_bw: FilterBandwidth,
}

impl Default for Icm42688Config {
    fn default() -> Self {
        Self {
            gyro_range: GyroRange::Dps2000,
            odr: OutputDataRate::Hz8000,
            filter_bw: FilterBandwidth::Bw180Hz,
        }
    }
}

/// Gyroscope full-scale range options
#[derive(Clone, Copy)]
pub enum GyroRange {
    Dps2000 = 0,
    Dps1000 = 1,
    Dps500 = 2,
    Dps250 = 3,
}

impl GyroRange {
    /// Get sensitivity in LSB per degree/second
    pub fn sensitivity(&self) -> f32 {
        match self {
            GyroRange::Dps2000 => 16.4,
            GyroRange::Dps1000 => 32.8,
            GyroRange::Dps500 => 65.5,
            GyroRange::Dps250 => 131.0,
        }
    }
}

/// Output data rate options
#[derive(Clone, Copy)]
pub enum OutputDataRate {
    Hz8000 = 0x03,
    Hz4000 = 0x04,
    Hz2000 = 0x05,
    Hz1000 = 0x06,
}

/// Filter bandwidth options
#[derive(Clone, Copy)]
pub enum FilterBandwidth {
    Bw180Hz = 0x0D, // Low latency
    Bw121Hz = 0x0C,
    Bw73Hz = 0x0B,
    Bw53Hz = 0x0A,
}

/// ICM42688 Manager for handling initialization and data reading
pub struct Icm42688Manager<SPI> {
    sensor: ICM42688<SPI, Ready>,
    config: Icm42688Config,
}

impl<SPI, E> Icm42688Manager<SPI>
where
    SPI: embedded_hal_async::spi::SpiDevice<Error = E>,
{
    /// Initialize the ICM42688 sensor from an uninitialized state
    ///
    /// This performs initialization and configures the sensor for gyro and accelerometer operation.
    pub async fn new<D>(
        sensor: ICM42688<SPI, Uninitialized>,
        delay: D,
    ) -> Result<Self, &'static str>
    where
        D: embedded_hal_async::delay::DelayNs,
    {
        Self::with_config(sensor, Icm42688Config::default(), delay).await
    }

    /// Initialize the ICM42688 sensor with custom configuration
    pub async fn with_config<D>(
        sensor: ICM42688<SPI, Uninitialized>,
        config: Icm42688Config,
        delay: D,
    ) -> Result<Self, &'static str>
    where
        D: embedded_hal_async::delay::DelayNs,
    {
        // Initialize using the library's method
        let mut sensor = sensor
            .initialize(delay)
            .await
            .map_err(|_| "Failed to initialize sensor")?;

        // Wait for sensor to stabilize
        Timer::after(Duration::from_millis(50)).await;

        // Configure using low-level register access
        {
            let ll = sensor.ll();
            let mut bank = ll.bank::<0>();

            // Configure gyro: range and ODR
            bank.gyro_config0()
                .async_write(|w| {
                    w.gyro_fs_sel(config.gyro_range as u8)
                        .gyro_odr(config.odr as u8)
                })
                .await
                .map_err(|_| "Failed to configure gyro")
                .unwrap();

            // Configure accel: 16g range and ODR
            bank.accel_config0()
                .async_write(|w| w.accel_fs_sel(0x00).accel_odr(config.odr as u8))
                .await
                .map_err(|_| "Failed to configure accel")?;

            // Configure gyro filter bandwidth
            bank.gyro_config1()
                .async_write(|w| w.gyro_ui_filt_ord(config.filter_bw as u8))
                .await
                .map_err(|_| "Failed to configure gyro filter")?;

            // Configure accel filter bandwidth
            bank.accel_config1()
                .async_write(|w| w.accel_ui_filt_ord(config.filter_bw as u8))
                .await
                .map_err(|_| "Failed to configure accel filter")?;

            Self::init(&mut sensor).await?;
        }

        Timer::after(Duration::from_millis(50)).await;

        Ok(Self { sensor, config })
    }

    /// Power on the gyro and accelerometer
    async fn init(sensor: &mut ICM42688<SPI, Ready>) -> Result<(), &'static str> {
        let ll = sensor.ll();
        let mut bank = ll.bank::<0>();

        // Turn on gyro and accelerometer
        bank.pwr_mgmt0()
            .async_write(|w| {
                w.gyro_mode(0b11) // Set gyro mode to standby
                    .accel_mode(0b11) // Set accel mode to standby
            })
            .await
            .map_err(|_| "Failed to set power config")?;

        Ok(())
    }

    /// Read gyroscope data in degrees per second [x, y, z]
    pub async fn read_gyro(&mut self) -> Result<[f32; 3], &'static str> {
        let ll = self.sensor.ll();
        let mut bank = ll.bank::<0>();

        // Read gyro X high byte
        let gx_h = bank
            .gyro_data_x1()
            .async_read()
            .await
            .map_err(|_| "Failed to read gyro X high")?;

        // Read gyro X low byte
        let gx_l = bank
            .gyro_data_x0()
            .async_read()
            .await
            .map_err(|_| "Failed to read gyro X low")?;

        // Read gyro Y high byte
        let gy_h = bank
            .gyro_data_y1()
            .async_read()
            .await
            .map_err(|_| "Failed to read gyro Y high")?;

        // Read gyro Y low byte
        let gy_l = bank
            .gyro_data_y0()
            .async_read()
            .await
            .map_err(|_| "Failed to read gyro Y low")?;

        // Read gyro Z high byte
        let gz_h = bank
            .gyro_data_z1()
            .async_read()
            .await
            .map_err(|_| "Failed to read gyro Z high")?;

        // Read gyro Z low byte
        let gz_l = bank
            .gyro_data_z0()
            .async_read()
            .await
            .map_err(|_| "Failed to read gyro Z low")?;

        // Convert to signed 16-bit values
        let gx_raw = i16::from_be_bytes([gx_h.gyro_data_x_15_8(), gx_l.gyro_data_x_7_0()]);
        let gy_raw = i16::from_be_bytes([gy_h.gyro_data_y_15_8(), gy_l.gyro_data_y_7_0()]);
        let gz_raw = i16::from_be_bytes([gz_h.gyro_data_z_15_8(), gz_l.gyro_data_z_7_0()]);

        // Convert to degrees per second using configured sensitivity
        let sensitivity = self.config.gyro_range.sensitivity();
        let gx = gx_raw as f32 / sensitivity;
        let gy = gy_raw as f32 / sensitivity;
        let gz = gz_raw as f32 / sensitivity;

        Ok([gx, gy, gz])
    }

    /// Read accelerometer data in g [x, y, z]
    pub async fn read_accel(&mut self) -> Result<[f32; 3], &'static str> {
        let ll = self.sensor.ll();
        let mut bank = ll.bank::<0>();

        // Read accel X
        let ax_h = bank
            .accel_data_x1()
            .async_read()
            .await
            .map_err(|_| "Failed to read accel X high")?;
        let ax_l = bank
            .accel_data_x0()
            .async_read()
            .await
            .map_err(|_| "Failed to read accel X low")?;

        // Read accel Y
        let ay_h = bank
            .accel_data_y1()
            .async_read()
            .await
            .map_err(|_| "Failed to read accel Y high")?;
        let ay_l = bank
            .accel_data_y0()
            .async_read()
            .await
            .map_err(|_| "Failed to read accel Y low")?;

        // Read accel Z
        let az_h = bank
            .accel_data_z1()
            .async_read()
            .await
            .map_err(|_| "Failed to read accel Z high")?;
        let az_l = bank
            .accel_data_z0()
            .async_read()
            .await
            .map_err(|_| "Failed to read accel Z low")?;

        // Convert to signed 16-bit values
        let ax_raw = i16::from_be_bytes([ax_h.accel_data_x_15_8(), ax_l.accel_data_x_7_0()]);
        let ay_raw = i16::from_be_bytes([ay_h.accel_data_y_15_8(), ay_l.accel_data_y_7_0()]);
        let az_raw = i16::from_be_bytes([az_h.accel_data_z_15_8(), az_l.accel_data_z_7_0()]);

        // Convert to g (16g range, sensitivity = 2048 LSB/g)
        let sensitivity = 2048.0;
        let ax = ax_raw as f32 / sensitivity;
        let ay = ay_raw as f32 / sensitivity;
        let az = az_raw as f32 / sensitivity;

        Ok([ax, ay, az])
    }

    /// Read temperature in degrees Celsius
    pub async fn read_temperature(&mut self) -> Result<f32, &'static str> {
        let ll = self.sensor.ll();
        let mut bank = ll.bank::<0>();

        let temp_h = bank
            .temp_data1()
            .async_read()
            .await
            .map_err(|_| "Failed to read temp high")?;

        let temp_l = bank
            .temp_data0()
            .async_read()
            .await
            .map_err(|_| "Failed to read temp low")?;

        let temp_raw = i16::from_be_bytes([temp_h.temp_data_15_8(), temp_l.temp_data_7_0()]);

        // Convert to Celsius: Temperature (°C) = (TEMP_DATA / 132.48) + 25
        let temp_c = (temp_raw as f32 / 132.48) + 25.0;

        Ok(temp_c)
    }

    /// Get current configuration
    pub fn config(&self) -> Icm42688Config {
        self.config
    }

    /// Get reference to the underlying sensor
    pub fn sensor(&self) -> &ICM42688<SPI, Ready> {
        &self.sensor
    }

    /// Get mutable reference to the underlying sensor
    pub fn sensor_mut(&mut self) -> &mut ICM42688<SPI, Ready> {
        &mut self.sensor
    }
}
