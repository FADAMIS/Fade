// src/config.rs

// Re-export the HAL crate for easy access from other modules
pub use stm32f7xx_hal as hal;

use hal::gpio::{Alternate, GpioExt, Output, Pin, PushPull};
use hal::pac::{GPIOA, GPIOB};

// Pin type definitions
pub type LedPin = Pin<'A', 14, Output<PushPull>>;
pub type UsbDmPin = Pin<'A', 11, Alternate<10>>;
pub type UsbDpPin = Pin<'A', 12, Alternate<10>>;
pub type SpiSckPin = Pin<'A', 5, Alternate<5>>;
pub type SpiMisoPin = Pin<'A', 6, Alternate<5>>;
pub type SpiMosiPin = Pin<'A', 7, Alternate<5>>;
pub type SpiCsPin = Pin<'B', 2, Output<PushPull>>;

// A struct to hold all the pins
pub struct Pins {
    pub led: LedPin,
    pub usb_dm: UsbDmPin,
    pub usb_dp: UsbDpPin,
    pub sck: SpiSckPin,
    pub miso: SpiMisoPin,
    pub mosi: SpiMosiPin,
    pub cs: SpiCsPin,
}

// Function to set up and return the pins
pub fn setup_pins(gpioa: GPIOA, gpiob: GPIOB) -> Pins {
    let gpioa = gpioa.split();
    let gpiob = gpiob.split();

    Pins {
        led: gpioa.pa14.into_push_pull_output(),
        usb_dm: gpioa.pa11.into_alternate(),
        usb_dp: gpioa.pa12.into_alternate(),
        sck: gpioa.pa5.into_alternate::<5>(),
        miso: gpioa.pa6.into_alternate::<5>(),
        mosi: gpioa.pa7.into_alternate::<5>(),
        cs: gpiob.pb2.into_push_pull_output(),
    }
}
