#![no_std]
#![no_main]

pub mod board_config;
pub mod config;
pub mod dshot;
pub mod filters;
pub mod flash;
pub mod flight_control;
pub mod fsp;
pub mod gyro;
pub mod mixer;
pub mod pid;

use embassy_stm32 as _;

use defmt_rtt as _; // global logger

// TODO(5) adjust HAL import
// use some_hal as _; // memory layout

use panic_probe as _;

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

/// Terminates the application by resetting the MCU.
/// This is safe without a debug probe — semihosting would hang.
pub fn exit() -> ! {
    cortex_m::peripheral::SCB::sys_reset();
}

/// Hardfault handler.
///
/// Resets the MCU so the FC recovers from crashes.
/// Using semihosting here would hang when running from battery only.
#[cortex_m_rt::exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    cortex_m::peripheral::SCB::sys_reset();
}

// defmt-test 0.3.0 has the limitation that this `#[tests]` attribute can only be used
// once within a crate. the module can be in any file but there can only be at most
// one `#[tests]` module in this library crate
#[cfg(test)]
#[defmt_test::tests]
mod unit_tests {
    use defmt::assert;

    #[test]
    fn it_works() {
        assert!(true)
    }
}
