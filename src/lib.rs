/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_std]

use embedded_hal as hal;
use hal::blocking::delay::DelayMs;
use hal::digital::v2::OutputPin;

mod interface;
pub use interface::{I2cInterface, SensorInterface, SpiInterface};

/// Errors in this crate
#[derive(Debug)]
pub enum Error<CommE, PinE> {
    /// Sensor communication error
    Comm(CommE),
    /// Pin setting error
    Pin(PinE),

    /// Sensor is not responding
    Unresponsive,
}

pub struct Builder {}

impl Builder {
    /// Create a new driver using I2C interface
    pub fn new_accel_i2c<I2C, CommE>(
        &self,
        i2c: I2C,
        address: u8,
    ) -> Accelerometer<I2cInterface<I2C>>
    where
        I2C: hal::blocking::i2c::Write<Error = CommE>
            + hal::blocking::i2c::Read<Error = CommE>
            + hal::blocking::i2c::WriteRead<Error = CommE>,
        CommE: core::fmt::Debug,
    {
        let iface = interface::I2cInterface::new(i2c, address);
        Accelerometer::new_with_interface(iface)
    }

    /// Create a new driver using SPI interface
    pub fn new_accel_spi<SPI, CSN, CommE, PinE>(
        spi: SPI,
        csn: CSN,
    ) -> Accelerometer<SpiInterface<SPI, CSN>>
    where
        SPI: hal::blocking::spi::Transfer<u8, Error = CommE>
            + hal::blocking::spi::Write<u8, Error = CommE>,
        CSN: OutputPin<Error = PinE>,
        CommE: core::fmt::Debug,
        PinE: core::fmt::Debug,
    {
        let iface = interface::SpiInterface::new(spi, csn);
        Accelerometer::new_with_interface(iface)
    }

    /// Create a new driver using I2C interface
    pub fn new_gyro_i2c<I2C, CommE>(&self, i2c: I2C, address: u8) -> Gyroscope<I2cInterface<I2C>>
    where
        I2C: hal::blocking::i2c::Write<Error = CommE>
            + hal::blocking::i2c::Read<Error = CommE>
            + hal::blocking::i2c::WriteRead<Error = CommE>,
        CommE: core::fmt::Debug,
    {
        let iface = interface::I2cInterface::new(i2c, address);
        Gyroscope::new_with_interface(iface)
    }

    /// Create a new driver using SPI interface
    pub fn new_gyro_spi<SPI, CSN, CommE, PinE>(
        spi: SPI,
        csn: CSN,
    ) -> Gyroscope<SpiInterface<SPI, CSN>>
    where
        SPI: hal::blocking::spi::Transfer<u8, Error = CommE>
            + hal::blocking::spi::Write<u8, Error = CommE>,
        CSN: OutputPin<Error = PinE>,
        CommE: core::fmt::Debug,
        PinE: core::fmt::Debug,
    {
        let iface = interface::SpiInterface::new(spi, csn);
        Gyroscope::new_with_interface(iface)
    }
}

pub struct Accelerometer<SI> {
    pub(crate) sensor_interface: SI,
}

impl<SI> Accelerometer<SI>
where
    SI: SensorInterface,
    SI::InterfaceError: core::fmt::Debug,
{
    const REG_CHIP_ID: u8 = 0x00;
    const KNOWN_CHIP_ID: u8 = 0x1E;

    const REG_SOFT_RESET: u8 = 0x7E;
    const CMD_SOFT_RESET: u8 = 0xB6;

    pub(crate) fn new_with_interface(sensor_interface: SI) -> Self {
        Self { sensor_interface }
    }

    /// Read the sensor identifiers and
    /// return true if they match the expected value
    pub fn probe(
        &mut self,
        delay_source: &mut impl DelayMs<u8>,
    ) -> Result<bool, SI::InterfaceError> {
        let mut chip_id = 0;
        for _ in 0..5 {
            chip_id = self.sensor_interface.register_read(Self::REG_CHIP_ID)?;
            if chip_id == Self::KNOWN_CHIP_ID {
                break;
            }
            delay_source.delay_ms(10);
        }

        Ok(chip_id == Self::KNOWN_CHIP_ID)
    }

    /// Perform a soft reset on the chip
    pub fn soft_reset(
        &mut self,
        delay_source: &mut impl DelayMs<u8>,
    ) -> Result<(), SI::InterfaceError> {
        self.sensor_interface
            .register_write(Self::REG_SOFT_RESET, Self::CMD_SOFT_RESET)?;
        delay_source.delay_ms(100);
        Ok(())
    }

    /// Give the sensor interface a chance to set up
    pub fn setup(&mut self, delay_source: &mut impl DelayMs<u8>) -> Result<(), SI::InterfaceError> {
        self.soft_reset(delay_source)?;

        let probe_success = self.probe(delay_source)?;
        if probe_success {
            Ok(())
        } else {
            panic!("no luck");
            // Err(Error::Unresponsive)
        }
    }
}

pub struct Gyroscope<SI> {
    pub(crate) sensor_interface: SI,
}

impl<SI> Gyroscope<SI>
where
    SI: SensorInterface,
    SI::InterfaceError: core::fmt::Debug,
{
    const REG_CHIP_ID: u8 = 0x00;
    const KNOWN_CHIP_ID: u8 = 0x0F;

    const REG_SOFT_RESET: u8 = 0x14;
    const CMD_SOFT_RESET: u8 = 0xB6;

    pub(crate) fn new_with_interface(sensor_interface: SI) -> Self {
        Self { sensor_interface }
    }

    /// Read the sensor identifiers and
    /// return true if they match the expected value
    pub fn probe(
        &mut self,
        delay_source: &mut impl DelayMs<u8>,
    ) -> Result<bool, SI::InterfaceError> {
        let mut chip_id = 0;
        for _ in 0..5 {
            chip_id = self.sensor_interface.register_read(Self::REG_CHIP_ID)?;
            if chip_id == Self::KNOWN_CHIP_ID {
                break;
            }
            delay_source.delay_ms(10);
        }

        Ok(chip_id == Self::KNOWN_CHIP_ID)
    }

    /// Perform a soft reset on the chip
    pub fn soft_reset(
        &mut self,
        delay_source: &mut impl DelayMs<u8>,
    ) -> Result<(), SI::InterfaceError> {
        self.sensor_interface
            .register_write(Self::REG_SOFT_RESET, Self::CMD_SOFT_RESET)?;
        delay_source.delay_ms(100);
        Ok(())
    }

    /// Give the sensor interface a chance to set up
    pub fn setup(&mut self, delay_source: &mut impl DelayMs<u8>) -> Result<(), SI::InterfaceError> {
        self.soft_reset(delay_source)?;

        let probe_success = self.probe(delay_source)?;
        if probe_success {
            Ok(())
        } else {
            panic!("no luck");
            // Err(Error::Unresponsive)
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
