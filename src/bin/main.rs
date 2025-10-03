#![no_main]
#![no_std]

//! # Board-Agnostic Flight Controller Firmware
//!
//! This firmware uses a board abstraction system that allows switching between
//! different flight controller boards by changing a single line of code.
//!
//! ## How to switch boards:
//! 1. Change the board import below to your target board
//! 2. All pin mappings, clock configurations, and frequencies will automatically
//!    be configured for that specific board based on Betaflight configurations
//!
//! ## Available boards:
//! - `HGLRCF722` - HGLRC F722 board
//! - `SkystarsF7HdPro` - SKYSTARS F7 HD PRO board (based on Betaflight config)
//!
//! ## Example board switching:
//! ```rust
//! // For HGLRC F722:
//! use fade::board_config::HGLRCF722 as Board;
//!
//! // For SKYSTARS F7 HD PRO:
//! use fade::board_config::SkystarsF7HdPro as Board;
//! ```
//!
//! The rest of the code remains exactly the same - pins, frequencies, and
//! configurations are automatically adjusted for the selected board.

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;

// 🎯 BOARD SELECTION - Change these lines to switch boards!
// This single change will reconfigure all pins, clocks, and frequencies
//
// For SKYSTARS F7 HD PRO (current):
use fade::board_config::skystars_pins as pins;
use fade::board_config::HGLRCF722 as Board;
//
// To switch to HGLRC F722, just change to:
// use fade::board_config::hglrcf722_pins as pins;
// use fade::board_config::HGLRCF722 as Board;
//
// Key differences that are automatically handled:
// - SKYSTARS: SPI3_MOSI = PB5, GYRO_CS = PA4, FLASH_CS = PA15
// - HGLRC:    SPI3_MOSI = PC12, GYRO_CS = PB2, FLASH_CS = PD2

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
use heapless::Vec;
use mpu6000::bus::SpiBus;
use mpu6000::MPU6000;
use w25q32jv::W25q32jv;

type GyroBus = SpiBus<spi::Spi<'static, embassy_stm32::mode::Async>, CsPin, Delay>;
#[allow(dead_code)]
type FlashType =
    W25q32jv<spi::Spi<'static, embassy_stm32::mode::Async>, Output<'static>, Output<'static>>;
type Gyro = GyroManager<GyroBus, Delay>;

// Flash communication types
#[derive(Debug)]
pub enum FlashCommand {
    Write { address: u32, data: Vec<u8, 256> },
    Read { address: u32, length: usize },
}

#[derive(Debug)]
pub enum FlashResponse {
    WriteResult(bool),
    ReadResult(Vec<u8, 256>),
}

// Static resources for flash communication
#[allow(dead_code)]
static FLASH_CHANNEL: Channel<
    ThreadModeRawMutex,
    (
        FlashCommand,
        embassy_sync::channel::Sender<'static, ThreadModeRawMutex, FlashResponse, 1>,
    ),
    4,
> = Channel::new();

#[allow(dead_code)]
static FLASH_MUTEX: Mutex<ThreadModeRawMutex, Option<FlashType>> = Mutex::new(None);

#[embassy_executor::task]
async fn flash_task() {
    loop {
        Timer::after_millis(1000).await;
        // Flash task placeholder - can be expanded later for actual flash operations
    }
}

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

    // Extract all board-specific pins at once using the pin mapping macro
    let (
        spi1_sck, // SPI1 pins for gyro
        spi1_mosi,
        spi1_miso,
        spi3_sck,  // SPI3 pins for flash
        spi3_mosi, // KEY DIFFERENCE: PB5 for SKYSTARS, PC12 for HGLRC
        spi3_miso,
        gyro_cs,    // KEY DIFFERENCE: PA4 for SKYSTARS, PB2 for HGLRC
        flash_cs,   // KEY DIFFERENCE: PA15 for SKYSTARS, PD2 for HGLRC
        flash_hold, // Flash control pins
        flash_wp,
        led_pin, // LED pin
        usb_dp,  // USB pins
        usb_dm,
    ) = pins::get_pins!(p);

    // SPI1 configuration for Gyro - using hardcoded board-specific pins
    let mut spi1_config = spi::Config::default();
    spi1_config.frequency = Board::spi1_frequency();
    let spi1 = spi::Spi::new(
        p.SPI1,
        spi1_sck,  // SCK - board-specific pin (PA5 for both boards in this case)
        spi1_mosi, // MOSI - board-specific pin (PA7 for both boards)
        spi1_miso, // MISO - board-specific pin (PA6 for both boards)
        p.DMA2_CH3,
        p.DMA2_CH0,
        spi1_config,
    );

    // SPI3 configuration for Flash - using hardcoded board-specific pins
    let mut spi3_config = spi::Config::default();
    spi3_config.frequency = Board::spi3_frequency();
    let spi3 = spi::Spi::new(
        p.SPI3,
        spi3_sck,   // SCK - board-specific pin (PC10 for both boards)
        spi3_mosi,  // MOSI - board abstracted (PB5 for SKYSTARS, PC12 for HGLRC)
        spi3_miso,  // MISO - board-specific pin (PC11 for both boards)
        p.DMA1_CH5, // TX DMA for SPI3
        p.DMA1_CH0, // RX DMA for SPI3
        spi3_config,
    );

    // Flash setup - using hardcoded board-specific pins
    let _flash_cs = Output::new(flash_cs, Level::High, Speed::VeryHigh); // Flash CS - board abstracted (PA15 vs PD2)
    let hold = Output::new(flash_hold, Level::High, Speed::VeryHigh); // Flash Hold - board-specific
    let wp = Output::new(flash_wp, Level::High, Speed::VeryHigh); // Flash WP - board-specific

    // Initialize flash - basic setup for W25Q32JV on SPI3
    let _flash = W25q32jv::new(spi3, hold, wp);
    defmt::info!("Flash configured on SPI3");

    // Gyro setup - using hardcoded board-specific pins
    let cs_pin = Output::new(gyro_cs, Level::High, Speed::VeryHigh); // Gyro CS - board abstracted (PA4 vs PB2)
    let cs = CsPin(cs_pin);
    let mpu_bus = SpiBus::new(spi1, cs, Delay);
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

    // LED setup - using hardcoded board-specific pins
    let led = Output::new(led_pin, Level::High, Speed::Low); // LED - board-specific (PA14 for both boards)

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

    // USB driver - using hardcoded board-specific pins
    let driver_config = usb::Config::default();
    #[allow(static_mut_refs)]
    let driver = Driver::new_fs(
        p.USB_OTG_FS,
        Irqs,
        usb_dp, // D+ - board-specific (PA12 for both boards)
        usb_dm, // D- - board-specific (PA11 for both boards)
        unsafe { &mut EP_MEMORY },
        driver_config,
    );

    // Create USB builder
    #[allow(static_mut_refs)]
    let mut builder = Builder::new(
        driver,
        usb_config,
        unsafe { &mut DEVICE_DESCRIPTOR },
        unsafe { &mut CONFIG_DESCRIPTOR },
        unsafe { &mut BOS_DESCRIPTOR },
        unsafe { &mut CONTROL_BUF },
    );

    // CDC ACM class
    #[allow(static_mut_refs)]
    let class = CdcAcmClass::new(&mut builder, unsafe { &mut STATE }, 256);

    let usb = builder.build();

    // Spawn tasks
    spawner.spawn(usb_task(usb)).unwrap();
    spawner.spawn(flash_task()).unwrap();
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

    // Try to load configuration from flash, otherwise use defaults
    if !msp_manager.load_config_from_flash(&mut pid_controllers) {
        defmt::info!("No saved configuration found, using defaults");
        // Apply default values to PID controllers
        let default_config = fade::config::FlightConfig::new();
        default_config.apply_to_pid_controllers(&mut pid_controllers);
    }

    let mut current_gyro = Vector3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    let mut pid_correction = Vector3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    loop {
        let mut buf = [0u8; 256];

        // Try to receive new gyro data
        if let Ok(gyro_data) = gyro_receiver.try_receive() {
            current_gyro.x = gyro_data[0];
            current_gyro.y = gyro_data[1];
            current_gyro.z = gyro_data[2];

            // Update PID controllers
            let setpoint = Vector3::zero();
            pid_correction = pid_controllers.calculate_rate(&setpoint, &current_gyro, 0.0);
        }

        match select(class.read_packet(&mut buf), Timer::after_millis(10)).await {
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
            Either::First(Err(_)) => {
                // USB read error, wait a bit
                Timer::after_millis(10).await;
            }
            Either::Second(_) => {
                // Timeout, continue loop
            }
        }
    }
}
