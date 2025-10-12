#![no_main]
#![no_std]

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_stm32::usb;
use embassy_stm32::usb::Driver;
use embassy_stm32::{bind_interrupts, init};
use embassy_stm32::{peripherals, spi};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Delay, Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, UsbDevice};
use fade as _;
use fade::filters::GyroFilter;
use fade::gyro::Mpu6000Manager;
use mpu6000::bus::SpiBus;
use mpu6000::MPU6000;

#[cfg(feature = "stm32h7")]
use fade::board_config::SequireH7V2Pins as BoardPins;

#[cfg(feature = "stm32f")]
use fade::board_config::HGLRCF722Pins as BoardPins;

bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

// =================== CHANNELS ===================
static MSP_GYRO_CHANNEL: Channel<ThreadModeRawMutex, [f32; 3], 4> = Channel::new();

// =================== USB CDC GLOBAL ===================
static mut CDC_CLASS: Option<CdcAcmClass<'static, Driver<'static, peripherals::USB_OTG_FS>>> = None;

// =================== CHIP SELECT PIN WRAPPER ===================
struct CsPin(Output<'static>);
impl embedded_hal::digital::v2::OutputPin for CsPin {
    type Error = ();
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.0.set_low();
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.0.set_high();
        Ok(())
    }
}

// =================== TASKS ===================

#[embassy_executor::task]
async fn led_task(mut led: Output<'static>) {
    loop {
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}

#[embassy_executor::task]
async fn gyro_task(
    mut gyro: Mpu6000Manager<
        SpiBus<spi::Spi<'static, embassy_stm32::mode::Async>, CsPin, Delay>,
        Delay,
    >,
    mut filter: GyroFilter,
) {
    loop {
        if let Ok(raw_gyro) = gyro.read_gyro() {
            let filtered_gyro = filter.update(raw_gyro);
            MSP_GYRO_CHANNEL.send(filtered_gyro).await;
        }
    }
}

#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice<'static, Driver<'static, peripherals::USB_OTG_FS>>) {
    usb.run().await;
}

#[embassy_executor::task]
async fn msp_task() {
    loop {
        let gyro = MSP_GYRO_CHANNEL.receive().await;

        let mut buf = heapless::String::<64>::new();
        use core::fmt::Write;
        let _ = write!(&mut buf, "{:.3},{:.3},{:.3}\r\n", gyro[0], gyro[1], gyro[2]);

        #[allow(static_mut_refs)]
        if let Some(class) = unsafe { CDC_CLASS.as_mut() } {
            let _ = class.write_packet(buf.as_bytes()).await;
        }
    }
}

// =================== MAIN ===================
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = init(clock_config());
    let pins = BoardPins::new(p);

    // LED setup
    let led = Output::new(pins.led_pin, Level::Low, Speed::Low);

    // Gyro setup
    let gyro_spi_config = spi::Config::default();

    let gyro_spi = Spi::new(
        pins.gyro_spi,
        pins.gyro_sck,
        pins.gyro_mosi,
        pins.gyro_miso,
        pins.gyro_tx_dma,
        pins.gyro_rx_dma,
        gyro_spi_config,
    );

    let cs_pin = Output::new(pins.gyro_cs, Level::High, Speed::VeryHigh);
    let bus = SpiBus::new(gyro_spi, CsPin(cs_pin), Delay);
    let mpu6000 = MPU6000::new(bus);
    let mpu6000_manager = Mpu6000Manager::new(mpu6000, Delay);

    // Gyro Filters setup
    let mut gyro_filter = GyroFilter::new(8000.0);
    gyro_filter.set_calibration_params(200, 5.0, 0.2, 100.0, 100.0);

    // USB setup
    let mut usb_cfg = embassy_usb::Config::new(0x16c0, 0x27dd);
    usb_cfg.manufacturer = Some("Fade");
    usb_cfg.product = Some("Flight Controller");
    usb_cfg.serial_number = Some("80085");
    usb_cfg.max_power = 100;
    usb_cfg.max_packet_size_0 = 64;

    static mut EP_MEMORY: [u8; 1024] = [0; 1024];
    static mut DEVICE_DESCRIPTOR: [u8; 256] = [0; 256];
    static mut CONFIG_DESCRIPTOR: [u8; 256] = [0; 256];
    static mut BOS_DESCRIPTOR: [u8; 256] = [0; 256];
    static mut CONTROL_BUF: [u8; 64] = [0; 64];
    static mut STATE: State = State::new();

    let driver_config = embassy_stm32::usb::Config::default();
    let usb_driver = unsafe {
        Driver::new_fs(
            pins.usb_otg_fs,
            Irqs,
            pins.usb_dp,
            pins.usb_dm,
            #[allow(static_mut_refs)]
            &mut EP_MEMORY,
            driver_config,
        )
    };
    #[allow(static_mut_refs)]
    let mut builder = unsafe {
        Builder::new(
            usb_driver,
            usb_cfg,
            &mut DEVICE_DESCRIPTOR,
            &mut CONFIG_DESCRIPTOR,
            &mut BOS_DESCRIPTOR,
            &mut CONTROL_BUF,
        )
    };

    #[allow(static_mut_refs)]
    unsafe {
        CDC_CLASS = Some(CdcAcmClass::new(&mut builder, &mut STATE, 256));
    }

    let usb = builder.build();

    // Spawn tasks
    spawner.spawn(led_task(led)).unwrap();
    spawner
        .spawn(gyro_task(mpu6000_manager, gyro_filter))
        .unwrap();
    spawner.spawn(usb_task(usb)).unwrap();
    spawner.spawn(msp_task()).unwrap();
}

// =================== CLOCK CONFIG ===================
pub fn clock_config() -> embassy_stm32::Config {
    let mut config = embassy_stm32::Config::default();
    use embassy_stm32::rcc;

    #[cfg(feature = "stm32f")]
    use embassy_stm32::rcc::Sysclk;

    config.rcc.hse = Some(rcc::Hse {
        freq: Hertz(8_000_000),
        mode: rcc::HseMode::Oscillator,
    });

    #[cfg(feature = "stm32f")]
    {
        config.rcc.pll_src = rcc::PllSource::HSE;
        config.rcc.pll = Some(rcc::Pll {
            prediv: rcc::PllPreDiv::DIV4,
            mul: rcc::PllMul::MUL216,
            divp: Some(rcc::PllPDiv::DIV2),
            divq: Some(rcc::PllQDiv::DIV9),
            divr: None,
        });
        config.rcc.ahb_pre = rcc::AHBPrescaler::DIV1;
        config.rcc.apb1_pre = rcc::APBPrescaler::DIV4;
        config.rcc.apb2_pre = rcc::APBPrescaler::DIV2;
        config.rcc.sys = Sysclk::PLL1_P;
    }

    #[cfg(feature = "stm32h7")]
    {
        use embassy_stm32::rcc::{mux::Usbsel, *};

        config.rcc.hsi = Some(HSIPrescaler::DIV1);
        config.rcc.csi = true;

        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL48,
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV16),
            divr: None,
        });
        config.rcc.mux.usbsel = Usbsel::PLL1_Q;

        config.rcc.sys = Sysclk::PLL1_P; // 384 MHz
        config.rcc.ahb_pre = AHBPrescaler::DIV2; // 192 MHz
        config.rcc.apb1_pre = APBPrescaler::DIV2; // 96 MHz
        config.rcc.apb2_pre = APBPrescaler::DIV2; // 96 MHz
        config.rcc.apb3_pre = APBPrescaler::DIV2; // 96 MHz
        config.rcc.apb4_pre = APBPrescaler::DIV2; // 96 MHz
        config.rcc.voltage_scale = VoltageScale::Scale1;
    }

    config
}
