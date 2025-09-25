#![no_std]
#![no_main]

use core::fmt::Write;
use fade as _;
use fade::filters::GyroFilter;
use fade::gyro::GyroManager;
use fade::led::LedManager;
use fade::pid::Axis;
use fade::pid::PidControllers;
use fade::pid::Vector3;
use fade::usb::UsbManager;
use fugit::RateExtU32;
use hal::pac;
use heapless::String;
use mpu6000::bus::SpiBus;
use mpu6000::MPU6000;
use mpu6000::SPI_MODE;
use stm32f7xx_hal::rcc::PLL48CLK;
use stm32f7xx_hal::spi::Spi;

use stm32f7xx_hal::{
    self as hal,
    gpio::GpioExt,
    rcc::RccExt,
    timer::{SysTimerExt, TimerExt},
};

#[cortex_m_rt::entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_pll48clk(PLL48CLK::Pllq)
        .sysclk(RateExtU32::MHz(216))
        .use_pll()
        .freeze();

    let dl = cp.SYST.delay(&clocks);
    let dl2 = dp.TIM2.delay_us(&clocks);

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let led_pin = gpioa.pa14.into_push_pull_output();
    let pin_dm = gpioa.pa11.into_alternate();
    let pin_dp = gpioa.pa12.into_alternate();
    let pin_sck = gpioa.pa5.into_alternate::<5>();
    let pin_miso = gpioa.pa6.into_alternate::<5>();
    let pin_mosi = gpioa.pa7.into_alternate::<5>();
    let pin_cs = gpiob.pb2.into_push_pull_output();

    let spi = Spi::new(dp.SPI1, (pin_sck, pin_miso, pin_mosi)).enable(
        SPI_MODE,
        RateExtU32::MHz(8),
        &clocks,
        &mut rcc.apb2,
    );

    let spi_bus = SpiBus::new(spi, pin_cs, dl);

    let mpu6000 = MPU6000::new(spi_bus);

    let mut gyro_manager = GyroManager::new(mpu6000, dl2);

    let mut usb_manager = UsbManager::new(
        dp.OTG_FS_GLOBAL,
        dp.OTG_FS_DEVICE,
        dp.OTG_FS_PWRCLK,
        pin_dm,
        pin_dp,
        &clocks,
    );

    let mut led_manager = LedManager::new(led_pin);

    let mut _counter = 0;

    let mut pid_controllers = PidControllers::new(1000.0); // Match your sample rate
    pid_controllers.set_gains(Axis::Pitch, 0.4, 0.05, 0.02); // I: 0.3 → 0.05
    pid_controllers.set_gains(Axis::Roll, 0.4, 0.05, 0.02); // I: 0.3 → 0.05
    pid_controllers.set_gains(Axis::Yaw, 0.5, 0.02, 0.0); // I: 0.1 → 0.02
    pid_controllers.set_output_limits(100.0); // Reasonable limit

    pid_controllers.set_derivative_lpf(50.0, 1000.0);

    let mut gyro_filter = GyroFilter::new(1000.0);
    gyro_filter.setup_lowpass1(1000.0, 200.0);
    gyro_filter.setup_lowpass2(1000.0, 100.0);
    gyro_filter.set_calibration_params(
        300,    // samples
        2.0,    // threshold (deg/s)
        0.5,    // deadband (deg/s)
        200.0,  // stability_time_ms
        1000.0, // startup_delay_ms
        1000.0, // sample_rate
    );

    let rate_setpoint = Vector3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    loop {
        usb_manager.poll();
        led_manager.update();

        let mut buf: String<128> = String::new(); // Bigger buffer for more data

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

            if gyro_filter.is_waiting_for_stability() {
                let _ = write!(buf, "Waiting for stability...");
                usb_manager.write_string(&buf).ok();
            } else if gyro_filter.is_calibrating() {
                let _ = write!(
                    buf,
                    "Calibration Progress: {:.1}%",
                    gyro_filter.get_calibration_progress()
                );
                usb_manager.write_string(&buf).ok();
            } else {
                // Display both gyro readings AND PID corrections
                let _ = write!(
                    buf,
                    "Gyro R:{:.1} P:{:.1} Y:{:.1} | PID R:{:.1} P:{:.1} Y:{:.1}",
                    filtered_gyro_vect.x,
                    filtered_gyro_vect.y,
                    filtered_gyro_vect.z,
                    pid_correction.x,
                    pid_correction.y,
                    pid_correction.z
                );
                usb_manager.write_string(&buf).ok();
            }
        }
        _counter += 1;
    }
}
