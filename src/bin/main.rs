#![no_main]
#![no_std]

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;

use embassy_stm32::time::Hertz;

static MSP_GYRO_CHANNEL: Channel<ThreadModeRawMutex, [f32; 3], 4> = Channel::new();
use embassy_futures::select::{select, Either};
use embassy_stm32::usb::Driver;
use embassy_stm32::{bind_interrupts, peripherals, spi, usb, Config};
use embassy_time::Delay;
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, UsbDevice};
use embedded_hal::digital::v2::OutputPin;
use fade::filters::GyroFilter;
use fade::gyro::GyroManager;
use fade::msp::MspManager;
use fade::pid::{PidControllers, Vector3};
use mpu6000::bus::SpiBus;
use mpu6000::MPU6000;

type GyroBus = SpiBus<spi::Spi<'static, embassy_stm32::mode::Async>, CsPin<'static>, Delay>;
type Gyro = GyroManager<GyroBus, Delay>;

#[embassy_executor::task]
async fn gyro_task(mut gyro: Gyro, mut filter: GyroFilter) {
    loop {
        if let Ok(gyro_raw) = gyro.read_gyro() {
            let gyro_filtered = filter.update(gyro_raw);

            // Send filtered gyro data to MSP system
            MSP_GYRO_CHANNEL.send(gyro_filtered).await;
        }

        // Remove delay for maximum responsiveness
    }
}

struct CsPin<'a>(Output<'a>);

impl<'a> OutputPin for CsPin<'a> {
    type Error = core::convert::Infallible;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.0.set_low();
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.0.set_high();
        Ok(())
    }
}

bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Configure system clock to 216 MHz
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL216,
            divp: Some(PllPDiv::DIV2), // 216 MHz
            divq: Some(PllQDiv::DIV9), // 48 MHz for USB
            divr: None,
        });
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV4;
        config.rcc.apb2_pre = APBPrescaler::DIV2;
        config.rcc.sys = Sysclk::PLL1_P;
    }

    let p = embassy_stm32::init(config);

    // SPI configuration
    let mut spi_config = spi::Config::default();
    spi_config.frequency = Hertz(8_000);
    let spi = spi::Spi::new(
        p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA2_CH3, p.DMA2_CH0, spi_config,
    );

    // Gyro setup
    let cs_pin = Output::new(p.PB2, Level::High, Speed::VeryHigh);
    let cs = CsPin(cs_pin);
    let mpu_bus = SpiBus::new(spi, cs, Delay);
    let mpu6000 = MPU6000::new(mpu_bus);
    let gyro_manager = GyroManager::new(mpu6000, Delay);

    //Gyro Filters setup
    let mut gyro_filter = GyroFilter::new(8000.0);
    gyro_filter.setup_lowpass1(8000.0, 200.0);
    gyro_filter.setup_lowpass2(8000.0, 100.0);
    gyro_filter.set_calibration_params(
        200,    // samples - reasonable amount for accuracy
        8.0,    // threshold (deg/s) - more lenient for real conditions
        0.2,    // deadband (deg/s) - small deadband
        100.0,  // stability_time_ms - reasonable stability time
        250.0,  // startup_delay_ms - short but allows sensor to settle
        8000.0, // sample_rate
    );

    // LED setup
    let led = Output::new(p.PA14, Level::High, Speed::Low);

    // USB configuration
    let mut usb_config = embassy_usb::Config::new(0x16c0, 0x27dd);
    usb_config.manufacturer = Some("Fade");
    usb_config.product = Some("Flight Controller");
    usb_config.serial_number = Some("12345678");
    usb_config.max_power = 100;
    usb_config.max_packet_size_0 = 64;

    static mut EP_MEMORY: [u8; 1024] = [0; 1024];
    static mut DEVICE_DESCRIPTOR: [u8; 256] = [0; 256];
    static mut CONFIG_DESCRIPTOR: [u8; 256] = [0; 256];
    static mut BOS_DESCRIPTOR: [u8; 256] = [0; 256];
    static mut CONTROL_BUF: [u8; 64] = [0; 64];
    static mut STATE: State = State::new();

    // USB driver
    let driver_config = usb::Config::default();
    #[allow(static_mut_refs)]
    let driver = Driver::new_fs(
        p.USB_OTG_FS,
        Irqs,
        p.PA12, // D+
        p.PA11, // D-
        unsafe { &mut EP_MEMORY },
        driver_config,
    );

    #[allow(static_mut_refs)]
    let mut builder = Builder::new(
        driver,
        usb_config,
        unsafe { &mut DEVICE_DESCRIPTOR },
        unsafe { &mut CONFIG_DESCRIPTOR },
        unsafe { &mut BOS_DESCRIPTOR },
        unsafe { &mut CONTROL_BUF },
    );

    #[allow(static_mut_refs)]
    let class = CdcAcmClass::new(&mut builder, unsafe { &mut STATE }, 64);

    let usb = builder.build();

    // Spawn tasks
    spawner.spawn(usb_task(usb)).unwrap();
    spawner.spawn(msp_task(class)).unwrap();
    spawner.spawn(led_task(led)).unwrap();
    spawner.spawn(gyro_task(gyro_manager, gyro_filter)).unwrap();

    loop {
        Timer::after_secs(1).await;
    }
}

#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice<'static, Driver<'static, peripherals::USB_OTG_FS>>) {
    usb.run().await;
}

#[embassy_executor::task]
async fn msp_task(mut class: CdcAcmClass<'static, Driver<'static, peripherals::USB_OTG_FS>>) {
    let mut buf = [0; 64];
    let mut msp_manager = MspManager::new();
    let mut pid_controllers = PidControllers::new(8000.0);
    let mut current_gyro = Vector3::zero();
    let pid_correction = Vector3::zero();

    loop {
        class.wait_connection().await;
        loop {
            let from_usb = class.read_packet(&mut buf);
            let from_gyro = MSP_GYRO_CHANNEL.receive();

            match select(from_usb, from_gyro).await {
                Either::First(Ok(len)) => {
                    let data = &buf[..len];

                    // Handle MSP protocol only
                    if len >= 3 && data[0] == b'$' && data[1] == b'X' && data[2] == b'<' {
                        let responses = msp_manager.handle_msp(
                            data,
                            &mut pid_controllers,
                            &current_gyro,
                            &pid_correction,
                        );
                        for response in responses {
                            let mut out_buf = [0u8; 256];
                            if let Ok(response_len) = response.serialize(&mut out_buf) {
                                class.write_packet(&out_buf[..response_len]).await.ok();
                            }
                        }
                    }
                }
                Either::First(Err(_)) => break, // USB disconnected
                Either::Second(gyro_data) => {
                    // Update gyro data immediately when available
                    current_gyro.x = gyro_data[0];
                    current_gyro.y = gyro_data[1];
                    current_gyro.z = gyro_data[2];
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn led_task(mut led: Output<'static>) {
    loop {
        led.set_high();
        Timer::after_millis(500).await;
        led.set_low();
        Timer::after_millis(500).await;
    }
}
