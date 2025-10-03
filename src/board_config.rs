// src/board_config.rs

use embassy_stm32::rcc::*;
use embassy_stm32::time::Hertz;

/// Board configuration for HGLRC F722
pub struct HGLRCF722;

impl HGLRCF722 {
    pub const NAME: &'static str = "HGLRC F722";

    pub fn hse_config() -> Hse {
        Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Oscillator,
        }
    }

    pub fn pll_config() -> Pll {
        Pll {
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL216,
            divp: Some(PllPDiv::DIV2), // 216 MHz
            divq: Some(PllQDiv::DIV9), // 48 MHz for USB
            divr: None,
        }
    }

    pub fn ahb_prescaler() -> AHBPrescaler {
        AHBPrescaler::DIV1
    }

    pub fn apb1_prescaler() -> APBPrescaler {
        APBPrescaler::DIV4
    }

    pub fn apb2_prescaler() -> APBPrescaler {
        APBPrescaler::DIV2
    }

    pub fn spi1_frequency() -> Hertz {
        Hertz(8_000)
    }

    pub fn spi3_frequency() -> Hertz {
        Hertz(1_000_000)
    }

    pub fn get_clock_config() -> embassy_stm32::Config {
        let mut config = embassy_stm32::Config::default();
        config.rcc.hse = Some(Self::hse_config());
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Self::pll_config());
        config.rcc.ahb_pre = Self::ahb_prescaler();
        config.rcc.apb1_pre = Self::apb1_prescaler();
        config.rcc.apb2_pre = Self::apb2_prescaler();
        config.rcc.sys = Sysclk::PLL1_P;
        config
    }
}

/// Hardcoded pin constants for HGLRC F722
pub mod hglrcf722_pins {
    // Macro to extract pins - this defines which physical pins to use
    #[macro_export]
    macro_rules! hglrcf722_get_pins {
        ($p:ident) => {
            (
                // SPI1 pins (Gyro)
                $p.PA5, // SPI1_SCK
                $p.PA7, // SPI1_MOSI
                $p.PA6, // SPI1_MISO
                // SPI3 pins (Flash)
                $p.PC10, // SPI3_SCK
                $p.PC12, // SPI3_MOSI - HGLRC specific
                $p.PC11, // SPI3_MISO
                // Chip selects
                $p.PB2, // GYRO_CS - HGLRC specific
                $p.PD2, // FLASH_CS - HGLRC specific
                // Flash control pins
                $p.PC8, // FLASH_HOLD
                $p.PC9, // FLASH_WP
                // LED pin
                $p.PA14, // LED
                // USB pins
                $p.PA12, // USB_DP
                $p.PA11, // USB_DM
            )
        };
    }
    pub use hglrcf722_get_pins as get_pins;
}

/// Board configuration for SKYSTARS F7 HD PRO
pub struct SkystarsF7HdPro;

impl SkystarsF7HdPro {
    pub const NAME: &'static str = "SKYSTARS F7 HD PRO";

    pub fn hse_config() -> Hse {
        Hse {
            freq: Hertz(8_000_000), // 8 MHz crystal
            mode: HseMode::Oscillator,
        }
    }

    pub fn pll_config() -> Pll {
        Pll {
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL216,
            divp: Some(PllPDiv::DIV2), // 216 MHz system clock
            divq: Some(PllQDiv::DIV9), // 48 MHz USB
            divr: None,
        }
    }

    pub fn ahb_prescaler() -> AHBPrescaler {
        AHBPrescaler::DIV1
    }

    pub fn apb1_prescaler() -> APBPrescaler {
        APBPrescaler::DIV4
    }

    pub fn apb2_prescaler() -> APBPrescaler {
        APBPrescaler::DIV2
    }

    pub fn spi1_frequency() -> Hertz {
        Hertz(8_000_000) // gyro
    }

    pub fn spi3_frequency() -> Hertz {
        Hertz(1_000_000) // flash
    }

    pub fn get_clock_config() -> embassy_stm32::Config {
        let mut config = embassy_stm32::Config::default();
        config.rcc.hse = Some(Self::hse_config());
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Self::pll_config());
        config.rcc.ahb_pre = Self::ahb_prescaler();
        config.rcc.apb1_pre = Self::apb1_prescaler();
        config.rcc.apb2_pre = Self::apb2_prescaler();
        config.rcc.sys = Sysclk::PLL1_P;
        config
    }
}

/// Hardcoded pin constants for SKYSTARS F7 HD PRO - Based on Betaflight configuration
pub mod skystars_pins {
    // Macro to extract pins - this defines which physical pins to use
    #[macro_export]
    macro_rules! skystars_get_pins {
        ($p:ident) => {
            (
                // SPI1 pins (Gyro) - From Betaflight config
                $p.PA5, // SPI1_SCK_PIN
                $p.PA7, // SPI1_SDO_PIN (MOSI)
                $p.PA6, // SPI1_SDI_PIN (MISO)
                // SPI3 pins (Flash) - From Betaflight config
                $p.PC10, // SPI3_SCK_PIN
                $p.PB5,  // SPI3_SDO_PIN (MOSI) - KEY DIFFERENCE! SKYSTARS uses PB5
                $p.PC11, // SPI3_SDI_PIN (MISO)
                // Chip selects - From Betaflight config
                $p.PA4,  // GYRO_1_CS_PIN - KEY DIFFERENCE! SKYSTARS uses PA4
                $p.PA15, // FLASH_CS_PIN - KEY DIFFERENCE! SKYSTARS uses PA15
                // Flash control pins - From Betaflight config (reusing motor pins)
                $p.PC8, // MOTOR1_PIN (FLASH_HOLD)
                $p.PC9, // MOTOR2_PIN (FLASH_WP)
                // LED pin - From Betaflight config
                $p.PA14, // PINIO1_PIN (LED)
                // USB pins (standard)
                $p.PA12, // USB_DP
                $p.PA11, // USB_DM
            )
        };
    }
    pub use skystars_get_pins as get_pins;
}

// Legacy compatibility functions
pub fn get_clock_config() -> embassy_stm32::Config {
    HGLRCF722::get_clock_config()
}

pub fn get_skystars_clock_config() -> embassy_stm32::Config {
    SkystarsF7HdPro::get_clock_config()
}

// Additional pin definitions from Betaflight configuration for reference
pub mod betaflight_pins {
    use embassy_stm32::peripherals::*;

    // Motors - MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN, MOTOR4_PIN
    pub type Motor1 = PC8;
    pub type Motor2 = PC9;
    pub type Motor3 = PB6;
    pub type Motor4 = PB7;

    // LEDs - LED0_PIN, LED1_PIN
    pub type Led0 = PC15;
    pub type Led1 = PC14;

    // Beeper - BEEPER_PIN
    pub type Beeper = PB2;

    // SPI2 (OSD/Baro) - SPI2_SCK_PIN, SPI2_SDO_PIN, SPI2_SDI_PIN
    pub type Spi2Sck = PB13;
    pub type Spi2Mosi = PB15;
    pub type Spi2Miso = PB14;

    // Additional chip selects
    pub type Gyro2Cs = PC13; // GYRO_2_CS_PIN
    pub type BaroCs = PB1; // BARO_CS_PIN
    pub type OsdCs = PB12; // MAX7456_SPI_CS_PIN

    // Gyro interrupts
    pub type Gyro1Int = PC4; // GYRO_1_EXTI_PIN
    pub type Gyro2Int = PC0; // GYRO_2_EXTI_PIN

    // LED strip - LED_STRIP_PIN
    pub type LedStrip = PB3;

    // Camera control - CAMERA_CONTROL_PIN
    pub type CameraControl = PA8;

    // ADC inputs
    pub type VBat = PC1; // ADC_VBAT_PIN
    pub type Rssi = PC2; // ADC_RSSI_PIN
    pub type Curr = PC3; // ADC_CURR_PIN

    // I2C - I2C1_SCL_PIN, I2C1_SDA_PIN
    pub type I2cScl = PB8;
    pub type I2cSda = PB9;

    // UARTs
    pub type Uart1Tx = PA9; // UART1_TX_PIN
    pub type Uart1Rx = PA10; // UART1_RX_PIN
    pub type Uart2Tx = PA2; // UART2_TX_PIN
    pub type Uart2Rx = PA3; // UART2_RX_PIN
    pub type Uart3Tx = PB10; // UART3_TX_PIN
    pub type Uart3Rx = PB11; // UART3_RX_PIN
    pub type Uart4Tx = PA0; // UART4_TX_PIN
    pub type Uart4Rx = PA1; // UART4_RX_PIN
    pub type Uart5Tx = PC12; // UART5_TX_PIN
    pub type Uart5Rx = PD2; // UART5_RX_PIN
    pub type Uart6Tx = PC6; // UART6_TX_PIN
    pub type Uart6Rx = PC7; // UART6_RX_PIN

    // PPM RX - RX_PPM_PIN
    pub type RxPpm = PB4;
}

// Example: Custom F4 Board configuration
// Uncomment this to add support for a new board
/*
pub struct CustomF4Board;

impl CustomF4Board {
    pub const NAME: &'static str = "Custom F4 Board";

    pub fn hse_config() -> Hse {
        Hse {
            freq: Hertz(25_000_000), // Different 25MHz crystal
            mode: HseMode::Oscillator,
        }
    }

    pub fn pll_config() -> Pll {
        Pll {
            prediv: PllPreDiv::DIV25,
            mul: PllMul::MUL336,
            divp: Some(PllPDiv::DIV2), // 168 MHz for F4
            divq: Some(PllQDiv::DIV7), // 48 MHz for USB
            divr: None,
        }
    }

    pub fn ahb_prescaler() -> AHBPrescaler {
        AHBPrescaler::DIV1
    }

    pub fn apb1_prescaler() -> APBPrescaler {
        APBPrescaler::DIV4
    }

    pub fn apb2_prescaler() -> APBPrescaler {
        APBPrescaler::DIV2
    }

    pub fn spi1_frequency() -> Hertz {
        Hertz(10_000) // Different frequency
    }

    pub fn spi3_frequency() -> Hertz {
        Hertz(2_000_000) // Different frequency
    }

    pub fn get_clock_config() -> embassy_stm32::Config {
        let mut config = embassy_stm32::Config::default();
        config.rcc.hse = Some(Self::hse_config());
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Self::pll_config());
        config.rcc.ahb_pre = Self::ahb_prescaler();
        config.rcc.apb1_pre = Self::apb1_prescaler();
        config.rcc.apb2_pre = Self::apb2_prescaler();
        config.rcc.sys = Sysclk::PLL1_P;
        config
    }
}

/// Pin configuration for Custom F4 Board
pub mod custom_f4_pins {
    #[macro_export]
    macro_rules! custom_f4_get_pins {
        ($p:ident) => {
            (
                // SPI1 pins (same as others)
                $p.PA5, // SPI1_SCK
                $p.PA7, // SPI1_MOSI
                $p.PA6, // SPI1_MISO
                // SPI3 pins (DIFFERENT from other boards)
                $p.PB3, // SPI3_SCK - Different!
                $p.PB5, // SPI3_MOSI - Different!
                $p.PB4, // SPI3_MISO - Different!
                // Chip selects (DIFFERENT)
                $p.PC4, // GYRO_CS - Different!
                $p.PC0, // FLASH_CS - Different!
                // Flash control pins
                $p.PC1, // FLASH_HOLD - Different!
                $p.PC2, // FLASH_WP - Different!
                // LED pin (DIFFERENT)
                $p.PC13, // LED - Different LED pin!
                // USB pins (same)
                $p.PA12, // USB_DP
                $p.PA11, // USB_DM
            )
        };
    }
    pub use custom_f4_get_pins as get_pins;
}

// To use this board, change main.rs imports to:
// use fade::board_config::custom_f4_pins as pins;
// use fade::board_config::CustomF4Board as Board;
*/
