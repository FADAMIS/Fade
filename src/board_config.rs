use embassy_stm32::peripherals::*;
use embassy_stm32::Peri;
use embassy_stm32::Peripherals;

pub struct SequireH7V2Pins {
    pub led_pin: Peri<'static, PC13>,
    pub usb_dm: Peri<'static, PA11>,
    pub usb_dp: Peri<'static, PA12>,
    pub usb_otg_fs: Peri<'static, USB_OTG_FS>,
    pub gyro_cs: Peri<'static, PB12>,
    pub gyro_spi: Peri<'static, SPI2>,
    pub gyro_sck: Peri<'static, PB13>,
    pub gyro_mosi: Peri<'static, PB15>,
    pub gyro_miso: Peri<'static, PB14>,
    pub gyro_tx_dma: Peri<'static, DMA1_CH0>,
    pub gyro_rx_dma: Peri<'static, DMA1_CH1>,
    pub uart1: Peri<'static, USART1>,
    pub uart1_tx: Peri<'static, PA9>,
    pub uart1_rx: Peri<'static, PA10>,
    pub uart1_tx_dma: Peri<'static, DMA1_CH2>,
    pub uart1_rx_dma: Peri<'static, DMA1_CH3>,
    // Motor pins — all on TIM3
    pub motor_tim: Peri<'static, TIM3>,
    pub motor1_pin: Peri<'static, PB4>,     // M1 = TIM3_CH1
    pub motor2_pin: Peri<'static, PB5>,     // M2 = TIM3_CH2
    pub motor3_pin: Peri<'static, PB0>,     // M3 = TIM3_CH3
    pub motor4_pin: Peri<'static, PB1>,     // M4 = TIM3_CH4
    pub motor_dma: Peri<'static, DMA1_CH4>, // TIM3_UP DMA channel
}

impl SequireH7V2Pins {
    pub fn new(p: Peripherals) -> Self {
        Self {
            led_pin: p.PC13,
            usb_dm: p.PA11,
            usb_dp: p.PA12,
            usb_otg_fs: p.USB_OTG_FS,
            gyro_cs: p.PB12,
            gyro_spi: p.SPI2,
            gyro_sck: p.PB13,
            gyro_mosi: p.PB15,
            gyro_miso: p.PB14,
            gyro_tx_dma: p.DMA1_CH0,
            gyro_rx_dma: p.DMA1_CH1,
            uart1: p.USART1,
            uart1_tx: p.PA9,
            uart1_rx: p.PA10,
            uart1_tx_dma: p.DMA1_CH2,
            uart1_rx_dma: p.DMA1_CH3,
            motor_tim: p.TIM3,
            motor1_pin: p.PB4,
            motor2_pin: p.PB5,
            motor3_pin: p.PB0,
            motor4_pin: p.PB1,
            motor_dma: p.DMA1_CH4,
        }
    }
}

pub struct HGLRCF722Pins {
    pub led_pin: Peri<'static, PA14>,
    pub usb_dm: Peri<'static, PA11>,
    pub usb_dp: Peri<'static, PA12>,
    pub usb_otg_fs: Peri<'static, USB_OTG_FS>,
    pub gyro_cs: Peri<'static, PB2>,
    pub gyro_spi: Peri<'static, SPI1>,
    pub gyro_sck: Peri<'static, PA5>,
    pub gyro_mosi: Peri<'static, PA7>,
    pub gyro_miso: Peri<'static, PA6>,
    pub gyro_tx_dma: Peri<'static, DMA2_CH3>,
    pub gyro_rx_dma: Peri<'static, DMA2_CH0>,
    pub uart1: Peri<'static, USART1>,
    pub uart1_tx: Peri<'static, PA9>,
    pub uart1_rx: Peri<'static, PA10>,
    pub uart1_tx_dma: Peri<'static, DMA2_CH7>,
    pub uart1_rx_dma: Peri<'static, DMA2_CH2>,
}

impl HGLRCF722Pins {
    pub fn new(p: Peripherals) -> Self {
        Self {
            led_pin: p.PA14,
            usb_dm: p.PA11,
            usb_dp: p.PA12,
            usb_otg_fs: p.USB_OTG_FS,
            gyro_cs: p.PB2,
            gyro_spi: p.SPI1,
            gyro_sck: p.PA5,
            gyro_mosi: p.PA7,
            gyro_miso: p.PA6,
            gyro_tx_dma: p.DMA2_CH3,
            gyro_rx_dma: p.DMA2_CH0,
            uart1: p.USART1,
            uart1_tx: p.PA9,
            uart1_rx: p.PA10,
            uart1_tx_dma: p.DMA2_CH7,
            uart1_rx_dma: p.DMA2_CH2,
        }
    }
}
