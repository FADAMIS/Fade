// src/board_config.rs

use embassy_stm32::rcc::*;
use embassy_stm32::time::Hertz;

/// Board-specific configuration
pub struct BoardConfig;

impl BoardConfig {
    /// Get the board name
    pub const fn name() -> &'static str {
        "HGLRC F722"
    }

    /// Get HSE (High Speed External) oscillator configuration
    pub const fn hse_config() -> Hse {
        Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Oscillator,
        }
    }

    /// Get PLL configuration
    pub const fn pll_config() -> Pll {
        Pll {
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL216,
            divp: Some(PllPDiv::DIV2), // 216 MHz
            divq: Some(PllQDiv::DIV9), // 48 MHz for USB
            divr: None,
        }
    }

    /// Get AHB prescaler
    pub const fn ahb_prescaler() -> AHBPrescaler {
        AHBPrescaler::DIV1
    }

    /// Get APB1 prescaler
    pub const fn apb1_prescaler() -> APBPrescaler {
        APBPrescaler::DIV4
    }

    /// Get APB2 prescaler
    pub const fn apb2_prescaler() -> APBPrescaler {
        APBPrescaler::DIV2
    }

    /// Get SPI1 frequency (for gyro)
    pub const fn spi1_frequency() -> Hertz {
        Hertz(8_000)
    }

    /// Get SPI3 frequency (for flash)
    pub const fn spi3_frequency() -> Hertz {
        Hertz(1_000_000)
    }
}

/// Pin definitions for HGLRC F722
pub mod pins {
    /// SPI1 pins (Gyro)
    pub mod spi1 {
        use embassy_stm32::peripherals::*;
        pub type Sck = PA5;
        pub type Mosi = PA7;
        pub type Miso = PA6;
    }

    /// SPI3 pins (Flash)
    pub mod spi3 {
        use embassy_stm32::peripherals::*;
        pub type Sck = PC10;
        pub type Mosi = PC12;
        pub type Miso = PC11;
    }

    /// Gyro chip select pin
    pub type GyroCs = embassy_stm32::peripherals::PB2;

    /// Flash pins
    pub type FlashCs = embassy_stm32::peripherals::PD2;
    pub type FlashHold = embassy_stm32::peripherals::PC8;
    pub type FlashWp = embassy_stm32::peripherals::PC9;

    /// LED pin
    pub type Led = embassy_stm32::peripherals::PA14;

    /// USB pins
    pub type UsbDp = embassy_stm32::peripherals::PA12;
    pub type UsbDm = embassy_stm32::peripherals::PA11;
}

/// Get complete clock configuration for the board
pub fn get_clock_config() -> embassy_stm32::Config {
    let mut config = embassy_stm32::Config::default();

    config.rcc.hse = Some(BoardConfig::hse_config());
    config.rcc.pll_src = PllSource::HSE;
    config.rcc.pll = Some(BoardConfig::pll_config());
    config.rcc.ahb_pre = BoardConfig::ahb_prescaler();
    config.rcc.apb1_pre = BoardConfig::apb1_prescaler();
    config.rcc.apb2_pre = BoardConfig::apb2_prescaler();
    config.rcc.sys = Sysclk::PLL1_P;

    config
}

// Example: Alternative board configuration (commented out)
// To add support for another board, uncomment and modify as needed:

/*
/// Alternative board configuration example
pub struct AlternativeBoard;

impl AlternativeBoard {
    pub const fn name() -> &'static str {
        "Alternative F4 Board"
    }

    pub const fn hse_config() -> Hse {
        Hse {
            freq: Hertz(25_000_000), // Different HSE frequency
            mode: HseMode::Oscillator,
        }
    }

    pub const fn pll_config() -> Pll {
        Pll {
            prediv: PllPreDiv::DIV25,
            mul: PllMul::MUL336,
            divp: Some(PllPDiv::DIV2), // 168 MHz for F4
            divq: Some(PllQDiv::DIV7), // 48 MHz for USB
            divr: None,
        }
    }

    pub const fn ahb_prescaler() -> AHBPrescaler {
        AHBPrescaler::DIV1
    }

    pub const fn apb1_prescaler() -> APBPrescaler {
        APBPrescaler::DIV4
    }

    pub const fn apb2_prescaler() -> APBPrescaler {
        APBPrescaler::DIV2
    }

    pub const fn spi1_frequency() -> Hertz {
        Hertz(10_000) // Different SPI frequency
    }

    pub const fn spi3_frequency() -> Hertz {
        Hertz(2_000_000) // Different flash frequency
    }
}

/// Alternative pin definitions
pub mod alt_pins {
    /// SPI1 pins for alternative board
    pub mod spi1 {
        use embassy_stm32::peripherals::*;
        pub type Sck = PA5;   // Could be different pins
        pub type Mosi = PA7;  // for different board
        pub type Miso = PA6;
    }

    pub mod spi3 {
        use embassy_stm32::peripherals::*;
        pub type Sck = PB3;   // Different pins example
        pub type Mosi = PB5;
        pub type Miso = PB4;
    }

    pub type GyroCs = embassy_stm32::peripherals::PC4;    // Different CS pin
    pub type FlashCs = embassy_stm32::peripherals::PC0;   // Different flash pins
    pub type FlashHold = embassy_stm32::peripherals::PC1;
    pub type FlashWp = embassy_stm32::peripherals::PC2;
    pub type Led = embassy_stm32::peripherals::PC13;      // Different LED pin
    pub type UsbDp = embassy_stm32::peripherals::PA12;    // USB pins usually same
    pub type UsbDm = embassy_stm32::peripherals::PA11;
}

/// To switch to alternative board:
/// 1. Replace BoardConfig usage with AlternativeBoard
/// 2. Replace pins module usage with alt_pins
/// 3. Update main.rs pin references accordingly
*/
