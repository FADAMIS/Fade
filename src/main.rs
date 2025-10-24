#![no_main]
#![no_std]

use core::sync::atomic::{AtomicBool, Ordering};
use crsf::PacketParser;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::spi;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart;
use embassy_stm32::usart::Uart;
use embassy_stm32::usb;
use embassy_stm32::{bind_interrupts, init};
use embassy_stm32::{peripherals, usb::Driver};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::Delay;
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, UsbDevice};
use fade as _;
use fade::filters::GyroFilter;
use fade::gyro::Icm42688Manager;
use fade::ngchl2::NgChl2Parser;
use fade::ngchl2::NgChl2Responses;
use icm426xx::ICM42688;
use static_cell::StaticCell;

#[cfg(feature = "stm32h7")]
use fade::board_config::SequireH7V2Pins as BoardPins;

#[cfg(feature = "stm32f")]
use fade::board_config::HGLRCF722Pins as BoardPins;

bind_interrupts!(struct IrqsUsb {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

bind_interrupts!(struct IrqsUart {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

// =================== BOOTLOADER ===================

// =================== CHANNELS ===================
static GYRO_CHANNEL: Channel<ThreadModeRawMutex, [f32; 3], 4> = Channel::new();
static RC_CHANNEL: Channel<ThreadModeRawMutex, [u16; 16], 16> = Channel::new();
static LINK_STATS_CHANNEL: Channel<ThreadModeRawMutex, u8, 4> = Channel::new();
static SHUTDOWN_FLAG: AtomicBool = AtomicBool::new(false);
static SHUTDOWN_SIGNAL: Signal<ThreadModeRawMutex, ()> = Signal::new();

// =================== USB CDC GLOBAL ===================
static mut CDC_CLASS: Option<CdcAcmClass<'static, Driver<'static, peripherals::USB_OTG_FS>>> = None;

// =================== TASKS ===================

#[embassy_executor::task]
async fn led_task(mut led: Output<'static>) {
    loop {
        let work = async {
            led.set_high();
            Timer::after(Duration::from_millis(500)).await;
            led.set_low();
            Timer::after(Duration::from_millis(500)).await;
        };
        match select(work, SHUTDOWN_SIGNAL.wait()).await {
            Either::First(_) => { /* did one blink cycle */ }
            Either::Second(_) => break,
        }
    }
}

#[embassy_executor::task]
async fn icm42688_task(
    mut gyro: Icm42688Manager<
        embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice<
            'static,
            NoopRawMutex,
            spi::Spi<'static, embassy_stm32::mode::Async>,
            Output<'static>,
        >,
    >,
    mut filter: GyroFilter,
) {
    loop {
        // race gyro read vs shutdown
        match select(gyro.read_gyro(), SHUTDOWN_SIGNAL.wait()).await {
            Either::First(Ok(raw_gyro)) => {
                let filtered = filter.update(raw_gyro);
                // race send vs shutdown
                match select(GYRO_CHANNEL.send(filtered), SHUTDOWN_SIGNAL.wait()).await {
                    Either::First(_) => {}
                    Either::Second(_) => break,
                }
            }
            Either::First(Err(_e)) => {
                // read error; small backoff but still cancellable
                match select(
                    Timer::after(Duration::from_millis(5)),
                    SHUTDOWN_SIGNAL.wait(),
                )
                .await
                {
                    Either::First(_) => {}
                    Either::Second(_) => break,
                }
            }
            Either::Second(_) => break, // shutdown
        }
    }
}

#[embassy_executor::task]
async fn reciever_task(
    mut uart_rx: usart::UartRx<'static, Async>,
    mut _uart_tx: usart::UartTx<'static, Async>,
) {
    let mut parser = PacketParser::<256>::new();
    let mut buf = [0u8; 1];

    loop {
        match select(uart_rx.read(&mut buf), SHUTDOWN_SIGNAL.wait()).await {
            Either::First(Ok(())) => {
                parser.push_bytes(&buf);

                let mut packet_count = 0;
                while let Some(packet_result) = parser.next_packet() {
                    if SHUTDOWN_FLAG.load(Ordering::Relaxed) {
                        break;
                    }

                    if let Ok((_consumed, packet)) = packet_result {
                        match packet {
                            crsf::Packet::RcChannels(ch) => {
                                // send is awaitable; make it cancelable
                                match select(RC_CHANNEL.send(ch.0), SHUTDOWN_SIGNAL.wait()).await {
                                    Either::First(_) => {}
                                    Either::Second(_) => break,
                                }
                            }
                            crsf::Packet::LinkStatistics(stats) => {
                                match select(
                                    LINK_STATS_CHANNEL.send(stats.uplink_link_quality),
                                    SHUTDOWN_SIGNAL.wait(),
                                )
                                .await
                                {
                                    Either::First(_) => {}
                                    Either::Second(_) => break,
                                }
                            }
                            _ => {
                                match select(RC_CHANNEL.send([0; 16]), SHUTDOWN_SIGNAL.wait()).await
                                {
                                    Either::First(_) => {}
                                    Either::Second(_) => break,
                                }
                            }
                        }
                    }

                    packet_count += 1;
                    if packet_count >= 3 {
                        // short cancelable delay so shutdown can be observed
                        match select(
                            Timer::after(Duration::from_micros(10)),
                            SHUTDOWN_SIGNAL.wait(),
                        )
                        .await
                        {
                            Either::First(_) => {}
                            Either::Second(_) => break,
                        }
                        break;
                    }
                }
            }
            Either::First(Err(_)) => {
                // read error -> short backoff, but cancellable
                match select(
                    Timer::after(Duration::from_millis(1)),
                    SHUTDOWN_SIGNAL.wait(),
                )
                .await
                {
                    Either::First(_) => {}
                    Either::Second(_) => break,
                }
            }
            Either::Second(_) => break,
        }
    }
}

#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice<'static, Driver<'static, peripherals::USB_OTG_FS>>) {
    // race the usb.run future with shutdown
    match select(usb.run(), SHUTDOWN_SIGNAL.wait()).await {
        Either::First(_) => {}
        Either::Second(_) => {
            drop(usb);
        }
    }
}

#[embassy_executor::task]
async fn ngchl_task() {
    let mut ngchl2_parser = NgChl2Parser::new();
    loop {
        let _gyro = GYRO_CHANNEL.receive().await;
        let _rx_channels = RC_CHANNEL.receive().await;
        let _rx_lq = LINK_STATS_CHANNEL.receive().await;

        let mut buf = [0u8; 4];

        #[allow(static_mut_refs)]
        if let Some(class) = unsafe { CDC_CLASS.as_mut() } {
            class.read_packet(&mut buf).await.unwrap();
            let res = ngchl2_parser.process_byte(buf[0]);
            let command = ngchl2_parser.get_command();
            if command.ready {}
            if res != NgChl2Responses::None {
                let res_byte = [res as u8];
                class.write_packet(&res_byte).await.unwrap();
            }
        }
        Timer::after(Duration::from_millis(1)).await;
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
    let mut gyro_spi_config = spi::Config::default();
    gyro_spi_config.frequency = Hertz(24_000_000);

    static BUS: StaticCell<Mutex<NoopRawMutex, spi::Spi<embassy_stm32::mode::Async>>> =
        StaticCell::new();
    let bus = BUS.init(Mutex::new(spi::Spi::new(
        pins.gyro_spi,
        pins.gyro_sck,
        pins.gyro_mosi,
        pins.gyro_miso,
        pins.gyro_tx_dma,
        pins.gyro_rx_dma,
        gyro_spi_config,
    )));

    let cs_pin = Output::new(pins.gyro_cs, Level::High, Speed::VeryHigh);
    let spi_dev = embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice::new(bus, cs_pin);

    let icm42688 = ICM42688::new(spi_dev);
    let icm42688_manager = Icm42688Manager::new(icm42688, Delay).await.unwrap();

    // Gyro Filters setup
    let mut gyro_filter = GyroFilter::new(8000.0);
    gyro_filter.set_calibration_params(200, 2.0, 0.2, 100.0, 100.0);

    let mut uart_config = usart::Config::default();
    uart_config.baudrate = 420_000;
    uart_config.parity = usart::Parity::ParityNone;
    uart_config.stop_bits = usart::StopBits::STOP1;
    uart_config.data_bits = usart::DataBits::DataBits8;

    // Reciever setup
    let (uart1_tx, uart1_rx) = Uart::new(
        pins.uart1,
        pins.uart1_rx,
        pins.uart1_tx,
        IrqsUart,
        pins.uart1_tx_dma,
        pins.uart1_rx_dma,
        uart_config,
    )
    .unwrap()
    .split();

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
            IrqsUsb,
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
        .spawn(icm42688_task(icm42688_manager, gyro_filter))
        .unwrap();
    spawner.spawn(reciever_task(uart1_rx, uart1_tx)).unwrap();
    spawner.spawn(ngchl_task()).unwrap();
    spawner.spawn(usb_task(usb)).unwrap();
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
