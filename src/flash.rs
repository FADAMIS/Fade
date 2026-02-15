// src/flash.rs
//
// Flash storage driver for persisting FlightConfig to STM32 internal flash.
// Uses the last sector of flash memory to avoid conflicts with firmware.

use crate::config::FlightConfig;

/// Flash sector configuration for config storage.
/// On STM32H743: last 128K sector (sector 7) at 0x080E0000
/// On STM32F722: last 128K sector at 0x080E0000
///
/// The user's firmware occupies the beginning of flash, so the last sector
/// is typically safe for config storage.

#[cfg(feature = "stm32h7")]
pub const CONFIG_FLASH_OFFSET: u32 = 0x000E0000; // Offset from flash base (0x08000000)

#[cfg(feature = "stm32f")]
pub const CONFIG_FLASH_OFFSET: u32 = 0x000E0000; // Offset from flash base (0x08000000)

pub const CONFIG_FLASH_BASE: u32 = 0x0800_0000;

/// Read a FlightConfig from flash memory.
///
/// Returns `Some(FlightConfig)` if valid config is found, `None` otherwise
/// (falls back to defaults at the call site).
pub fn load_config_from_flash() -> Option<FlightConfig> {
    let flash_addr = (CONFIG_FLASH_BASE + CONFIG_FLASH_OFFSET) as *const u8;
    let size = FlightConfig::size();

    // Safety: we are reading from a known flash memory address
    let bytes = unsafe { core::slice::from_raw_parts(flash_addr, size) };

    FlightConfig::from_bytes(bytes)
}

/// Save a FlightConfig to flash memory.
///
/// This performs a sector erase followed by a write. The flash peripheral
/// must be passed in because embassy_stm32::flash::Flash requires ownership
/// of the FLASH peripheral.
///
/// Returns Ok(()) on success, Err on flash operation failure.
pub fn save_config_to_flash(
    flash: &mut embassy_stm32::flash::Flash<'_, embassy_stm32::flash::Blocking>,
    config: &FlightConfig,
) -> Result<(), embassy_stm32::flash::Error> {
    let offset = CONFIG_FLASH_OFFSET;
    let bytes = config.to_bytes();

    // Erase the sector containing our config
    flash.blocking_erase(offset, offset + erase_size())?;

    // Write the config bytes
    // Flash writes must be aligned to write size (typically 32 bytes on H7, 4 bytes on F7)
    // We pad the data to the required alignment
    let write_sz = write_size();
    let aligned_len = (bytes.len() + write_sz - 1) / write_sz * write_sz;
    let mut aligned_buf = [0xFFu8; 512]; // Max config size is < 512
    aligned_buf[..bytes.len()].copy_from_slice(bytes);

    flash.blocking_write(offset, &aligned_buf[..aligned_len])?;

    Ok(())
}

/// Get the erase size for the current platform.
#[cfg(feature = "stm32h7")]
const fn erase_size() -> u32 {
    128 * 1024 // 128K sectors on H743
}

#[cfg(feature = "stm32f")]
const fn erase_size() -> u32 {
    128 * 1024 // 128K sectors on F722
}

/// Get the write granularity for the current platform.
#[cfg(feature = "stm32h7")]
const fn write_size() -> usize {
    32 // H7 requires 256-bit (32-byte) aligned writes
}

#[cfg(feature = "stm32f")]
const fn write_size() -> usize {
    4 // F7 requires 32-bit (4-byte) aligned writes
}
