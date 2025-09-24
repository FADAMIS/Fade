#![no_std]
#![no_main]

use core::fmt::Write;
use fade as _;
use fade::gyro::GyroManager;
use fade::led::LedManager;
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

    let mut counter = 0;

    loop {
        usb_manager.poll();
        led_manager.update();

        if counter % 10000 == 0 {
            let mut buf: String<64> = String::new();
            if let Ok(gyro) = gyro_manager.read_gyro() {
                let _ = write!(
                    buf,
                    "Angle: roll {:.1}, pitch {:.1}, yaw {:.1}",
                    gyro[0], gyro[1], gyro[2]
                );
                usb_manager.write_string(&buf).ok();
            }
        }
        counter += 1;
    }
}
