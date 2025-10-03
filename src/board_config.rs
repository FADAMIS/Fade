use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPreDiv, PllSource, Sysclk,
};
#[cfg(feature = "h743")]
use embassy_stm32::rcc::{PllDiv, VoltageScale};
#[cfg(feature = "f722")]
use embassy_stm32::rcc::{PllPDiv, PllQDiv};
use embassy_stm32::time::Hertz;

/// Trait for a board's specific configuration
pub trait BoardConfig {
    const NAME: &'static str;
    const GYRO_SPI: &'static str;

    fn get_clock_config() -> embassy_stm32::Config;
    fn spi_frequency() -> Hertz;
}

//--------------------------------------------------------------------------------

/// Board configuration for HGLRC F722
pub struct HGLRCF722;

#[cfg(feature = "f722")]
impl BoardConfig for HGLRCF722 {
    const NAME: &'static str = "HGLRC F722";
    const GYRO_SPI: &'static str = "SPI1";

    fn get_clock_config() -> embassy_stm32::Config {
        let mut config = embassy_stm32::Config::default();
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL216,
            divp: Some(PllPDiv::DIV2),
            divq: Some(PllQDiv::DIV9),
            divr: None,
        });
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV4;
        config.rcc.apb2_pre = APBPrescaler::DIV2;
        config.rcc.sys = Sysclk::PLL1_P;
        config
    }

    fn spi_frequency() -> Hertz {
        Hertz(8_000_000)
    }
}

#[cfg(feature = "f722")]
pub mod hglrcf722_pins {
    #[macro_export]
    macro_rules! hglrcf722_get_pins {
        ($p:ident) => {
            (
                $p.SPI1,
                $p.DMA2_CH3,
                $p.DMA2_CH0, // Gyro SPI
                $p.SPI3,
                $p.DMA1_CH5,
                $p.DMA1_CH0, // Flash SPI
                $p.PA5,
                $p.PA7,
                $p.PA6, // Gyro pins
                $p.PC10,
                $p.PC12,
                $p.PC11, // Flash pins
                $p.PB2,
                $p.PD2, // Chip selects
                $p.PC8,
                $p.PC9,  // Flash control
                $p.PA14, // LED
                $p.PA12,
                $p.PA11, // USB
            )
        };
    }
    pub use hglrcf722_get_pins as get_pins;
}

//--------------------------------------------------------------------------------

/// Board configuration for SKYSTARS F7 HD PRO
pub struct SkystarsF7HdPro;

#[cfg(feature = "f722")]
impl BoardConfig for SkystarsF7HdPro {
    const NAME: &'static str = "SKYSTARS F7 HD PRO";
    const GYRO_SPI: &'static str = "SPI1";

    fn get_clock_config() -> embassy_stm32::Config {
        let mut config = embassy_stm32::Config::default();
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL216,
            divp: Some(PllPDiv::DIV2),
            divq: Some(PllQDiv::DIV9),
            divr: None,
        });
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV4;
        config.rcc.apb2_pre = APBPrescaler::DIV2;
        config.rcc.sys = Sysclk::PLL1_P;
        config
    }

    fn spi_frequency() -> Hertz {
        Hertz(8_000_000)
    }
}

#[cfg(feature = "f722")]
pub mod skystars_pins {
    #[macro_export]
    macro_rules! skystars_get_pins {
        ($p:ident) => {
            (
                $p.SPI1,
                $p.DMA2_CH3,
                $p.DMA2_CH0, // Gyro SPI
                $p.SPI3,
                $p.DMA1_CH5,
                $p.DMA1_CH0, // Flash SPI
                $p.PA5,
                $p.PA7,
                $p.PA6, // Gyro pins
                $p.PC10,
                $p.PB5,
                $p.PC11, // Flash pins
                $p.PA4,
                $p.PA15, // Chip selects
                $p.PC8,
                $p.PC9,  // Flash control
                $p.PA14, // LED
                $p.PA12,
                $p.PA11, // USB
            )
        };
    }
    pub use skystars_get_pins as get_pins;
}

//--------------------------------------------------------------------------------

/// Board configuration for SEQUREH7V2
pub struct SEQUREH7V2;

#[cfg(feature = "h743")]
impl BoardConfig for SEQUREH7V2 {
    const NAME: &'static str = "SEQUREH7V2";
    const GYRO_SPI: &'static str = "SPI2";

    fn get_clock_config() -> embassy_stm32::Config {
        let mut config = embassy_stm32::Config::default();
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV2,
            mul: PllMul::MUL200,
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV4),
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV2;
        config.rcc.apb1_pre = APBPrescaler::DIV2;
        config.rcc.apb2_pre = APBPrescaler::DIV2;
        config.rcc.apb3_pre = APBPrescaler::DIV2;
        config.rcc.apb4_pre = APBPrescaler::DIV2;
        config.rcc.voltage_scale = VoltageScale::Scale1;
        config
    }

    fn spi_frequency() -> Hertz {
        Hertz(20_000_000)
    }
}

#[cfg(feature = "h743")]
pub mod sequreh7v2_pins {
    #[macro_export]
    macro_rules! sequreh7v2_get_pins {
        ($p:ident) => {
            (
                $p.SPI2,
                $p.DMA1_CH1,
                $p.DMA1_CH0, // Gyro SPI (Board uses SPI2)
                $p.SPI3,
                $p.DMA1_CH2,
                $p.DMA1_CH3, // Flash SPI
                $p.PB13,
                $p.PB15,
                $p.PB14, // Gyro pins (PB13-SCK, PB15-MOSI, PB14-MISO)
                $p.PC10,
                $p.PC12,
                $p.PC11, // Flash pins
                $p.PB12,
                $p.PA15, // Chip selects
                $p.PE9,
                $p.PE10, // Flash control (TODO: Verify)
                $p.PC13, // LED
                $p.PA12,
                $p.PA11, // USB
            )
        };
    }
    pub use sequreh7v2_get_pins as get_pins;
}
