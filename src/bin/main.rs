#![no_std]
#![no_main]

use fade as _;
use fade::board_config;

use fade::filters::GyroFilter;
use fade::gyro::GyroManager;
use fade::led::LedManager;
use fade::msp::MspManager;
use fade::pid::Axis;
use fade::pid::PidControllers;
use fade::pid::Vector3;
use fade::usb::UsbManager;
use fugit::RateExtU32;
use mpu6000::bus::SpiBus;
use mpu6000::MPU6000;
use mpu6000::SPI_MODE;

use board_config::hal;
use hal::pac;
use hal::prelude::*;

#[cortex_m_rt::entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_pll48clk(hal::rcc::PLL48CLK::Pllq)
        .sysclk(RateExtU32::MHz(216))
        .use_pll()
        .freeze();

    let dl = cp.SYST.delay(&clocks);
    let dl2 = dp.TIM2.delay_us(&clocks);

    let pins = board_config::setup_pins(dp.GPIOA, dp.GPIOB);

    let spi = hal::spi::Spi::new(dp.SPI1, (pins.sck, pins.miso, pins.mosi)).enable(
        SPI_MODE,
        RateExtU32::MHz(8),
        &clocks,
        &mut rcc.apb2,
    );

    let spi_bus = SpiBus::new(spi, pins.cs, dl);

    let mpu6000 = MPU6000::new(spi_bus);

    let mut gyro_manager = GyroManager::new(mpu6000, dl2);

    let mut usb_manager = UsbManager::new(
        dp.OTG_FS_GLOBAL,
        dp.OTG_FS_DEVICE,
        dp.OTG_FS_PWRCLK,
        pins.usb_dm,
        pins.usb_dp,
        &clocks,
    );

    let mut led_manager = LedManager::new(pins.led);
    let mut msp_manager = MspManager::new();

    let mut pid_controllers = PidControllers::new(8000.0); // Match your sample rate
    pid_controllers.set_gains(Axis::Pitch, 0.4, 0.05, 0.2);
    pid_controllers.set_gains(Axis::Roll, 0.4, 0.05, 0.2);
    pid_controllers.set_gains(Axis::Yaw, 0.5, 0.02, 0.0);
    pid_controllers.set_output_limits(100.0); // Reasonable limit

    pid_controllers.set_derivative_lpf(50.0, 8000.0);

    let mut gyro_filter = GyroFilter::new(8000.0);
    gyro_filter.setup_lowpass1(8000.0, 200.0);
    gyro_filter.setup_lowpass2(8000.0, 100.0);
    gyro_filter.set_calibration_params(
        300,    // samples
        2.0,    // threshold (deg/s)
        0.5,    // deadband (deg/s)
        200.0,  // stability_time_ms
        1000.0, // startup_delay_ms
        8000.0, // sample_rate
    );

    let rate_setpoint = Vector3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    loop {
        usb_manager.poll();
        led_manager.update();

        if let Ok(gyro) = gyro_manager.read_gyro() {
            let filtered_gyro = gyro_filter.update(gyro);
            let filtered_gyro_vect = Vector3 {
                x: filtered_gyro[0],
                y: filtered_gyro[1],
                z: filtered_gyro[2],
            };

            // Calculate PID corrections
            let pid_correction = pid_controllers.calculate_rate(
                &rate_setpoint,
                &filtered_gyro_vect,
                0.0, // throttle (0.0 = no TPA effect)
            );

            msp_manager.handle_msp(
                &mut usb_manager,
                &mut pid_controllers,
                &filtered_gyro_vect,
                &pid_correction,
            );
        }
    }
}
