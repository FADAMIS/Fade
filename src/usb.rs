use crate::config::hal;
use crate::config::{UsbDmPin, UsbDpPin};
use hal::otg_fs::{UsbBus, USB};
use hal::pac::{OTG_FS_DEVICE, OTG_FS_GLOBAL, OTG_FS_PWRCLK};
use hal::rcc::Clocks;
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::SerialPort;

pub type UsbBusType = UsbBusAllocator<UsbBus<USB>>;
pub type SerialPortType = SerialPort<'static, UsbBus<USB>>;
pub type UsbDeviceType = UsbDevice<'static, UsbBus<USB>>;

pub struct UsbManager {
    serial: SerialPortType,
    usb_dev: UsbDeviceType,
}

impl UsbManager {
    /// Create a new USB manager with minimal setup
    /// Just pass the USB peripherals, pins, and clocks
    pub fn new(
        usb_global: OTG_FS_GLOBAL,
        usb_device: OTG_FS_DEVICE,
        usb_pwrclk: OTG_FS_PWRCLK,
        pin_dm: UsbDmPin,
        pin_dp: UsbDpPin,
        clocks: &Clocks,
    ) -> Self {
        // Create USB peripheral
        let usb = USB::new(usb_global, usb_device, usb_pwrclk, (pin_dm, pin_dp), clocks);

        // Static memory for USB endpoints
        static mut EP_MEMORY: [u32; 1024] = [0; 1024];
        static mut USB_BUS: Option<UsbBusType> = None;

        // Initialize USB bus (this is safe as it only happens once)
        #[allow(static_mut_refs)]
        unsafe {
            EP_MEMORY = [0; 1024];
            USB_BUS = Some(UsbBus::new(usb, &mut EP_MEMORY));
        }

        // Get reference to USB bus
        #[allow(static_mut_refs)]
        let usb_bus = unsafe { USB_BUS.as_ref().unwrap() };

        // Create serial port
        let serial = SerialPort::new(usb_bus);

        // Create USB device
        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .strings(&[StringDescriptors::default()
                .manufacturer("Fake company")
                .product("Serial port")
                .serial_number("TEST")])
            .unwrap()
            .device_class(usbd_serial::USB_CLASS_CDC)
            .max_packet_size_0(64)
            .unwrap()
            .build();

        Self { serial, usb_dev }
    }

    /// Call this in your main loop to handle USB communication
    pub fn poll(&mut self) {
        self.usb_dev.poll(&mut [&mut self.serial]);
    }

    /// Write a string to USB
    pub fn write_string(&mut self, data: &str) -> Result<usize, usb_device::UsbError> {
        let mut written = self.serial.write(data.as_bytes())?;
        written += self.serial.write(b"\n")?;
        Ok(written)
    }

    /// Write bytes to USB
    pub fn write_bytes(&mut self, data: &[u8]) -> Result<usize, usb_device::UsbError> {
        self.serial.write(data)
    }
}
