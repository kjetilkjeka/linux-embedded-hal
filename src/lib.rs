//! Implementation of [`embedded-hal`] traits for Linux devices
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal
//!
//! # Drivers
//!
//! This crate lets you use a bunch of platform agnostic drivers that are based on the
//! `embedded-hal` traits. You can find them on crates.io by [searching for the embedded-hal
//! keyword][0].
//!
//! [0]: https://crates.io/keywords/embedded-hal

#![deny(missing_docs)]
#![deny(warnings)]

extern crate cast;
extern crate embedded_hal as hal;
extern crate nb;
pub extern crate i2cdev;
pub extern crate spidev;
pub extern crate sysfs_gpio;
pub extern crate socketcan;

use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::time::Duration;
use std::{ops, thread};

use cast::{u32, u64};
use i2cdev::core::I2CDevice;
use spidev::SpidevTransfer;

/// Empty struct that provides delay functionality on top of `thread::sleep`
pub struct Delay;

impl hal::blocking::delay::DelayUs<u8> for Delay {
    fn delay_us(&mut self, n: u8) {
        thread::sleep(Duration::new(0, u32(n) * 1000))
    }
}

impl hal::blocking::delay::DelayUs<u16> for Delay {
    fn delay_us(&mut self, n: u16) {
        thread::sleep(Duration::new(0, u32(n) * 1000))
    }
}

impl hal::blocking::delay::DelayUs<u32> for Delay {
    fn delay_us(&mut self, n: u32) {
        let secs = n / 1_000_000;
        let nsecs = (n % 1_000_000) * 1_000;

        thread::sleep(Duration::new(u64(secs), nsecs))
    }
}

impl hal::blocking::delay::DelayUs<u64> for Delay {
    fn delay_us(&mut self, n: u64) {
        let secs = n / 1_000_000;
        let nsecs = ((n % 1_000_000) * 1_000) as u32;

        thread::sleep(Duration::new(secs, nsecs))
    }
}

impl hal::blocking::delay::DelayMs<u8> for Delay {
    fn delay_ms(&mut self, n: u8) {
        thread::sleep(Duration::from_millis(u64(n)))
    }
}

impl hal::blocking::delay::DelayMs<u16> for Delay {
    fn delay_ms(&mut self, n: u16) {
        thread::sleep(Duration::from_millis(u64(n)))
    }
}

impl hal::blocking::delay::DelayMs<u32> for Delay {
    fn delay_ms(&mut self, n: u32) {
        thread::sleep(Duration::from_millis(u64(n)))
    }
}

impl hal::blocking::delay::DelayMs<u64> for Delay {
    fn delay_ms(&mut self, n: u64) {
        thread::sleep(Duration::from_millis(n))
    }
}

/// Newtype around [`sysfs_gpio::Pin`] that implements the `embedded-hal` traits
///
/// [`sysfs_gpio::Pin`]: https://docs.rs/sysfs_gpio/0.5.1/sysfs_gpio/struct.Pin.html
pub struct Pin(pub sysfs_gpio::Pin);

impl Pin {
    /// See [`sysfs_gpio::Pin::new`][0] for details.
    ///
    /// [0]: https://docs.rs/sysfs_gpio/0.5.1/sysfs_gpio/struct.Pin.html#method.new
    pub fn new(pin_num: u64) -> Pin {
        Pin(sysfs_gpio::Pin::new(pin_num))
    }

    /// See [`sysfs_gpio::Pin::from_path`][0] for details.
    ///
    /// [0]: https://docs.rs/sysfs_gpio/0.5.1/sysfs_gpio/struct.Pin.html#method.from_path
    pub fn from_path<P>(path: P) -> sysfs_gpio::Result<Pin>
    where
        P: AsRef<Path>,
    {
        sysfs_gpio::Pin::from_path(path).map(Pin)
    }
}

impl hal::digital::OutputPin for Pin {
    fn set_low(&mut self) {
        self.0.set_value(0).unwrap()
    }

    fn set_high(&mut self) {
        self.0.set_value(1).unwrap()
    }
}

impl ops::Deref for Pin {
    type Target = sysfs_gpio::Pin;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl ops::DerefMut for Pin {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

/// Newtype around [`i2cdev::linux::LinuxI2CDevice`] that implements the `embedded-hal` traits
///
/// [`i2cdev::linux::LinuxI2CDevice`]: https://docs.rs/i2cdev/0.3.1/i2cdev/linux/struct.LinuxI2CDevice.html
pub struct I2cdev {
    inner: i2cdev::linux::LinuxI2CDevice,
    path: PathBuf,
    address: Option<u8>,
}

impl I2cdev {
    /// See [`i2cdev::linux::LinuxI2CDevice::new`][0] for details.
    ///
    /// [0]: https://docs.rs/i2cdev/0.3.1/i2cdev/linux/struct.LinuxI2CDevice.html#method.new
    pub fn new<P>(path: P) -> Result<Self, i2cdev::linux::LinuxI2CError>
    where
        P: AsRef<Path>,
    {
        let dev = I2cdev {
            path: path.as_ref().to_path_buf(),
            inner: i2cdev::linux::LinuxI2CDevice::new(path, 0)?,
            address: None,
        };
        Ok(dev)
    }

    fn set_address(&mut self, address: u8) -> Result<(), i2cdev::linux::LinuxI2CError> {
        if self.address != Some(address) {
            self.inner = i2cdev::linux::LinuxI2CDevice::new(&self.path, address as u16)?;
            self.address = Some(address);
        }
        Ok(())
    }
}

impl hal::blocking::i2c::Read for I2cdev {
    type Error = i2cdev::linux::LinuxI2CError;

    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.set_address(address)?;
        self.inner.read(buffer)
    }
}

impl hal::blocking::i2c::Write for I2cdev {
    type Error = i2cdev::linux::LinuxI2CError;

    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.set_address(address)?;
        self.inner.write(bytes)
    }
}

impl hal::blocking::i2c::WriteRead for I2cdev {
    type Error = i2cdev::linux::LinuxI2CError;

    fn write_read(
        &mut self,
        address: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.set_address(address)?;
        self.inner.write(bytes)?;
        self.inner.read(buffer)
    }
}

impl ops::Deref for I2cdev {
    type Target = i2cdev::linux::LinuxI2CDevice;

    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl ops::DerefMut for I2cdev {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}

/// Newtype around [`spidev::Spidev`] that implements the `embedded-hal` traits
///
/// [`spidev::Spidev`]: https://docs.rs/spidev/0.3.0/spidev/struct.Spidev.html
pub struct Spidev(pub spidev::Spidev);

impl Spidev {
    /// See [`spidev::Spidev::open`][0] for details.
    ///
    /// [0]: https://docs.rs/spidev/0.3.0/spidev/struct.Spidev.html#method.open
    pub fn open<P>(path: P) -> io::Result<Self>
    where
        P: AsRef<Path>,
    {
        spidev::Spidev::open(path).map(Spidev)
    }
}

impl hal::blocking::spi::Transfer<u8> for Spidev {
    type Error = io::Error;

    fn transfer<'b>(&mut self, buffer: &'b mut [u8]) -> io::Result<&'b [u8]> {
        let tx = buffer.to_owned();
        self.0
            .transfer(&mut SpidevTransfer::read_write(&tx, buffer))?;
        Ok(buffer)
    }
}

impl hal::blocking::spi::Write<u8> for Spidev {
    type Error = io::Error;

    fn write(&mut self, buffer: &[u8]) -> io::Result<()> {
        self.0.write_all(buffer)
    }
}

impl ops::Deref for Spidev {
    type Target = spidev::Spidev;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl ops::DerefMut for Spidev {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

/// Newtype around [`socketcan::CANSocket`] that implements the `embedded-hal::can::Receiver` trait
///
/// [`socketcan::CANSocket`]: https://docs.rs/socketcan/1.7.0/socketcan/struct.CANSocket.html
#[derive(Debug)]
pub struct CanSocket(socketcan::CANSocket);

/// Newtype around [`socketcan::CANFrame`] that implements the `embedded-hal` traits
///
/// [`socketcan::CANFrame`]: https://docs.rs/socketcan/1.7.0/socketcan/struct.CANFrane.html
#[derive(Debug, Clone, Copy)]
pub struct CanFrame(socketcan::CANFrame);

/// Newtype around [`socketcan::CANFilter`] that implements the `embedded-hal` traits
///
/// [`socketcan::CANFilter`]: https://docs.rs/socketcan/1.7.0/socketcan/struct.CANFilter.html
#[derive(Debug, Clone, Copy)]
pub struct CanFilter(socketcan::CANFilter);

/// A CAN-ID type
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum CanId {
    /// The CAN 11-bit ID
    Base(u32),
    /// The CAN 29-bit ID
    Extended(u32),
}

impl From<socketcan::CANSocket> for CanSocket {
    fn from(interface: socketcan::CANSocket) -> CanSocket {
        interface.set_nonblocking(true).unwrap();
        CanSocket(interface)
    }
}

impl From<CanId> for u32 {
    fn from(id: CanId) -> u32 {
        match id {
            CanId::Base(x) => x,
            CanId::Extended(x) => x,
        }
    }
}

impl std::cmp::Ord for CanId {
    // If numeric value of Id is the same BaseId will win aribtration and therefore sorted lower.
    fn cmp(&self, other: &CanId) -> std::cmp::Ordering {
        match (self, other) {
            (&CanId::Base(ref self_id), &CanId::Base(ref other_id)) => std::cmp::Ord::cmp(self_id, other_id),
            (&CanId::Extended(ref self_id), &CanId::Extended(ref other_id)) => std::cmp::Ord::cmp(self_id, other_id),
            (&CanId::Base(ref self_id), &CanId::Extended(ref other_id)) => {
                match std::cmp::Ord::cmp(self_id, other_id) {
                    std::cmp::Ordering::Equal => std::cmp::Ordering::Less,
                    x => x
                }
            },
            (&CanId::Extended(ref self_id), &CanId::Base(ref other_id)) => {
                match std::cmp::Ord::cmp(self_id, other_id) {
                    std::cmp::Ordering::Equal => std::cmp::Ordering::Greater,
                    x => x
                }
            }
        }
    }
}

impl std::cmp::PartialOrd for CanId {
    fn partial_cmp(&self, other: &CanId) -> Option<std::cmp::Ordering> {
        Some(std::cmp::Ord::cmp(self, other))
    }
}

impl hal::can::Id for CanId {
    type BaseId = u32;
    type ExtendedId = u32;

    fn base_id(&self) -> Option<Self::BaseId> {
        if let &CanId::Base(id) = self {
            Some(id)
        } else {
            None
        }
    }

    fn extended_id(&self) -> Option<Self::ExtendedId> {
        if let &CanId::Extended(id) = self {
            Some(id)
        } else {
            None
        }
    }
}

impl hal::can::Filter for CanFilter {
    type Id = CanId;

    fn from_id(id: Self::Id) -> Self {
        let id_socketcan = match id {
            CanId::Base(id) => id,
            CanId::Extended(id) => id | socketcan::EFF_FLAG,
        };
        let mask_socketcan = socketcan::EFF_MASK | socketcan::EFF_FLAG;

        CanFilter(socketcan::CANFilter::new(id_socketcan, mask_socketcan).unwrap())
    }

    fn accept_all() -> Self {
        CanFilter(socketcan::CANFilter::new(0, 0).unwrap())
    }

    fn from_mask(mask: u32, filter: u32) -> Self {
        CanFilter(socketcan::CANFilter::new(mask, filter).unwrap())
    }
}

impl hal::can::Frame for CanFrame {
    type Id = CanId;

    fn is_remote_frame(&self) -> bool {
        self.0.is_rtr()
    }

    fn is_data_frame(&self) -> bool {
        !self.0.is_error() && !self.0.is_rtr()
    }

    fn id(&self) -> CanId {
        if self.0.is_extended() {
            CanId::Extended(self.0.id())
        } else {
            CanId::Base(self.0.id())
        }
    }

    fn data(&self) -> Option<&[u8]> {
        if self.0.is_rtr() {
            None
        } else {
            Some(self.0.data())
        }
    }
}

impl hal::can::Interface for CanSocket {
    type Id = CanId;
    type Frame = CanFrame;
    type Error = std::io::Error;
    type Filter = CanFilter;
}

impl hal::can::Receiver for CanSocket {
    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
        Ok(CanFrame(self.0.read_frame().map_err(|e| {
            match e.kind() {
                std::io::ErrorKind::WouldBlock => nb::Error::WouldBlock,
                _ => nb::Error::Other(e),
            }
        })?))
    }

    fn set_filter(&mut self, filter: Self::Filter) {
        self.0.set_filter( &[filter.0] ).unwrap()
    }

    fn clear_filter(&mut self) {
        self.0.filter_accept_all().unwrap()
    }
}

impl hal::can::Transmitter for CanSocket {
    fn transmit(&mut self, frame: &Self::Frame) -> Result<Option<Self::Frame>, nb::Error<Self::Error>> {
        self.0.write_frame(&frame.0).map_err(|e| {
            match e.kind() {
                std::io::ErrorKind::WouldBlock => nb::Error::WouldBlock,
                _ => nb::Error::Other(e),
            }
        })?;
        Ok(None)
    }
}
