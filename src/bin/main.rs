#![no_main]
#![no_std]

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;

// 🎯 BOARD SELECTION - Change these lines to switch boards!
use fade::board_config::BoardConfig;
// This single change will reconfigure all pins, clocks, and frequencies.
//
// To use SEQUREH7V2 (H743):
// use fade::board_config::{sequreh7v2_pins as pins, SEQUREH7V2 as Board};
//
// To use HGLRCF722 (F722):
// use fade::board_config::{hglrcf722_pins as pins, HGLRCF722 as Board};
//
use fade::board_config::{sequreh7v2_pins as pins, SEQUREH7V2 as Board};

static MSP_GYRO_CHANNEL: Channel<ThreadModeRawMutex, [f32; 3], 4> = Channel::new();
use embassy_futures::select::{select, Either};
use embassy_stm32::usb::Driver;
use embassy_stm32::{bind_interrupts, peripherals, spi, usb};
use embassy_time::Delay;
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, UsbDevice};

use fade::filters::GyroFilter;
use fade::gyro::GyroManager;
use fade::msp::MspManager;
use fade::pid::{PidControllers, Vector3};
use mpu6000::bus::SpiBus;
use mpu6000::MPU6000;
use w25q32jv::W25q32jv;

type GyroBus = SpiBus<spi::Spi<'static, embassy_stm32::mode::Async>, CsPin, Delay>;
type Gyro = GyroManager<GyroBus, Delay>;

#[embassy_executor::task]
async fn gyro_task(mut gyro: Gyro, mut filter: GyroFilter) {
    loop {
        if let Ok(gyro_raw) = gyro.read_gyro() {
            let gyro_filtered = filter.update(gyro_raw);
            MSP_GYRO_CHANNEL.sender().send(gyro_filtered).await;
        }
        Timer::after_millis(1).await;
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

bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

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

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let config = Board::get_clock_config();
    let p = embassy_stm32::init(config);

    defmt::info!("Initializing board: {}", Board::NAME);
    defmt::info!("Gyro on: {}", Board::GYRO_SPI);

    // Abstracted peripherals, DMA, and pins from board config
    let (
        gyro_spi,
        gyro_dma_tx,
        gyro_dma_rx,
        flash_spi,
        flash_dma_tx,
        flash_dma_rx,
        gyro_sck,
        gyro_mosi,
        gyro_miso,
        flash_sck,
        flash_mosi,
        flash_miso,
        gyro_cs,
        flash_cs,
        flash_hold,
        flash_wp,
        led_pin,
        usb_dp,
        usb_dm,
    ) = pins::get_pins!(p);

    // Gyro SPI configuration
    let mut gyro_spi_config = spi::Config::default();
    gyro_spi_config.frequency = Board::spi_frequency();
    let spi_gyro = spi::Spi::new(
        gyro_spi,
        gyro_sck,
        gyro_mosi,
        gyro_miso,
        gyro_dma_tx,
        gyro_dma_rx,
        gyro_spi_config,
    );

    // Flash SPI configuration
    let mut flash_spi_config = spi::Config::default();
    flash_spi_config.frequency = Board::spi_frequency(); // Using the same frequency for now
    let spi_flash = spi::Spi::new(
        flash_spi,
        flash_sck,
        flash_mosi,
        flash_miso,
        flash_dma_tx,
        flash_dma_rx,
        flash_spi_config,
    );

    // Flash setup
    let _flash_cs = Output::new(flash_cs, Level::High, Speed::VeryHigh);
    let hold = Output::new(flash_hold, Level::High, Speed::VeryHigh);
    let wp = Output::new(flash_wp, Level::High, Speed::VeryHigh);
    let _flash = W25q32jv::new(spi_flash, hold, wp);

    // Gyro setup
    let cs_pin = Output::new(gyro_cs, Level::High, Speed::VeryHigh);
    let cs = CsPin(cs_pin);
    let mpu_bus = SpiBus::new(spi_gyro, cs, Delay);
    let mpu6000 = MPU6000::new(mpu_bus);
    let gyro_manager = GyroManager::new(mpu6000, Delay);

    let mut gyro_filter = GyroFilter::new(8000.0);
    gyro_filter.setup_lowpass1(8000.0, 200.0);
    gyro_filter.setup_lowpass2(8000.0, 100.0);

    let led = Output::new(led_pin, Level::High, Speed::Low);

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

    let driver_config = usb::Config::default();
    let driver = unsafe {
        Driver::new_fs(
            p.USB_OTG_FS,
            Irqs,
            usb_dp,
            usb_dm,
            #[allow(static_mut_refs)]
            &mut EP_MEMORY,
            driver_config,
        )
    };
    #[allow(static_mut_refs)]
    let mut builder = unsafe {
        Builder::new(
            driver,
            usb_config,
            &mut DEVICE_DESCRIPTOR,
            &mut CONFIG_DESCRIPTOR,
            &mut BOS_DESCRIPTOR,
            &mut CONTROL_BUF,
        )
    };
    #[allow(static_mut_refs)]
    let class = unsafe { CdcAcmClass::new(&mut builder, &mut STATE, 256) };

    let usb = builder.build();

    // Spawn tasks
    spawner.spawn(usb_task(usb)).unwrap();
    spawner.spawn(msp_task(class)).unwrap();
    spawner.spawn(led_task(led)).unwrap();
    spawner.spawn(gyro_task(gyro_manager, gyro_filter)).unwrap();

    loop {
        Timer::after_millis(100).await;
    }
}

#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice<'static, Driver<'static, peripherals::USB_OTG_FS>>) {
    usb.run().await;
}

#[embassy_executor::task]
async fn msp_task(
    mut class: CdcAcmClass<'static, Driver<'static, embassy_stm32::peripherals::USB_OTG_FS>>,
) {
    let mut pid_controllers = PidControllers::new(8000.0);
    let mut msp_manager = MspManager::new();
    let gyro_receiver = MSP_GYRO_CHANNEL.receiver();

    if !msp_manager.load_config_from_flash(&mut pid_controllers) {
        let default_config = fade::config::FlightConfig::new();
        default_config.apply_to_pid_controllers(&mut pid_controllers);
    }

    let mut current_gyro = Vector3::zero();
    let mut pid_correction = Vector3::zero();

    loop {
        let mut buf = [0u8; 256];

        if let Ok(gyro_data) = gyro_receiver.try_receive() {
            current_gyro.x = gyro_data[0];
            current_gyro.y = gyro_data[1];
            current_gyro.z = gyro_data[2];

            let setpoint = Vector3::zero();
            pid_correction = pid_controllers.calculate_rate(&setpoint, &current_gyro, 0.0);
        }

        match select(class.read_packet(&mut buf), Timer::after_millis(10)).await {
            Either::First(Ok(len)) => {
                let data = &buf[..len];

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
            Either::First(Err(_)) => {
                Timer::after_millis(10).await;
            }
            Either::Second(_) => {}
        }
    }
}
