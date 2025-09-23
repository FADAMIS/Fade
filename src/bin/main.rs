#![no_std]
#![no_main]

use core::fmt::Write;
use fade as _;
use fade::led::LedManager;
use fade::usb::UsbManager;
use fugit::RateExtU32;
use hal::pac;
use heapless::String;
use stm32f7xx_hal::rcc::PLL48CLK;
use stm32f7xx_hal::{self as hal, gpio::GpioExt, rcc::RccExt};

#[cortex_m_rt::entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_pll48clk(PLL48CLK::Pllq)
        .sysclk(RateExtU32::MHz(216))
        .use_pll()
        .freeze();

    let gpioa = dp.GPIOA.split();
    let led_pin = gpioa.pa14.into_push_pull_output();
    let pin_dm = gpioa.pa11.into_alternate();
    let pin_dp = gpioa.pa12.into_alternate();

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
            let mut buf: String<32> = String::new();
            write!(buf, "test{}", counter).unwrap();
            usb_manager.write_string(&buf).ok();
        }
        counter += 1;
    }
}
