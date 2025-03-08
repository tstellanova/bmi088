/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_std]

use embedded_hal as hal;
use hal::delay::DelayNs;

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
        I2C: hal::i2c::I2c<Error = CommE>,
        CommE: core::fmt::Debug,
    {
        let iface = interface::I2cInterface::new(i2c, address);
        Accelerometer::new_with_interface(iface)
    }

    /// Create a new driver using SPI interface
    pub fn new_accel_spi<SPI, CommE>(
        spi: SPI,
    ) -> Accelerometer<SpiInterface<SPI>>
    where
        SPI: hal::spi::SpiDevice<u8, Error = CommE>,
        CommE: core::fmt::Debug,
    {
        //accel part requires "sloppy reads"
        //see section 6.1.2 in the BMI088 datasheet
        let iface = interface::SpiInterface::new(spi, true);
        Accelerometer::new_with_interface(iface)
    }

    /// Create a new driver using I2C interface
    pub fn new_gyro_i2c<I2C, CommE>(&self, i2c: I2C, address: u8) -> Gyroscope<I2cInterface<I2C>>
    where
        I2C: hal::i2c::I2c<Error = CommE>,
        CommE: core::fmt::Debug,
    {
        let iface = interface::I2cInterface::new(i2c, address);
        Gyroscope::new_with_interface(iface)
    }

    /// Create a new driver using SPI interface
    pub fn new_gyro_spi<SPI, CommE>(
        spi: SPI,
    ) -> Gyroscope<SpiInterface<SPI>>
    where
        SPI: hal::spi::SpiDevice<u8, Error = CommE>,
        CommE: core::fmt::Debug,
    {
        let iface = interface::SpiInterface::new(spi, false);
        Gyroscope::new_with_interface(iface)
    }
}

pub struct Accelerometer<SI> {
    pub(crate) si: SI,
}

impl<SI, CommE, PinE> Accelerometer<SI>
where
    SI: SensorInterface<InterfaceError = Error<CommE, PinE>>,
{
    const REG_CHIP_ID: u8 = 0x00;
    const KNOWN_CHIP_ID: u8 = 0x1E;

    const REG_SOFT_RESET: u8 = 0x7E;
    const CMD_SOFT_RESET: u8 = 0xB6;

    const REG_ACC_PWR_CTRL: u8 = 0x7D;
    const ACC_PWR_CTRL_EN: u8 = 0x04;

    const REG_ACC_X_LSB: u8 = 0x12;
    const REG_ACCEL_DATA_START: u8 = Self::REG_ACC_X_LSB;

    pub(crate) fn new_with_interface(sensor_interface: SI) -> Self {
        Self { si: sensor_interface }
    }

    /// Read the sensor identifiers and
    /// return true if they match the expected value
    pub fn probe(
        &mut self,
        delay_source: &mut impl DelayNs,
    ) -> Result<bool, SI::InterfaceError> {
        let mut chip_id = 0;
        for _ in 0..5 {
            chip_id = self.si.register_read(Self::REG_CHIP_ID)?;
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
        delay_source: &mut impl DelayNs,
    ) -> Result<(), SI::InterfaceError> {
        self.si.register_write(Self::REG_SOFT_RESET, Self::CMD_SOFT_RESET)?;
        delay_source.delay_ms(5);
        Ok(())
    }

    /// Give the sensor interface a chance to set up
    pub fn setup(&mut self, delay_source: &mut impl DelayNs) -> Result<(), SI::InterfaceError> {
        // see datasheet section:
        // "3. Quick Start Guide – Device Initialization"
        self.soft_reset(delay_source)?;

        let probe_success = self.probe(delay_source)?;
        if !probe_success {
            return Err(Error::Unresponsive)
        }

        // enable the accelerometer
        self.si.register_write(Self::REG_ACC_PWR_CTRL, Self::ACC_PWR_CTRL_EN)?;
        delay_source.delay_ms(50);

        Ok(())
    }

    pub fn get_accel(&mut self) -> Result<[i16; 3], SI::InterfaceError> {
        let sample = self.si.read_vec3_i16(Self::REG_ACCEL_DATA_START)?;
        Ok(sample)
    }
}

pub struct Gyroscope<SI> {
    pub(crate) si: SI,
}

impl<SI, CommE, PinE> Gyroscope<SI>
where
    SI: SensorInterface<InterfaceError = Error<CommE, PinE>>,
{
    const REG_CHIP_ID: u8 = 0x00;
    const KNOWN_CHIP_ID: u8 = 0x0F;

    const REG_SOFT_RESET: u8 = 0x14;
    const CMD_SOFT_RESET: u8 = 0xB6;

    const REG_RATE_X_LSB: u8 = 0x02;
    const REG_GYRO_START: u8 = Self::REG_RATE_X_LSB;

    const REG_GYRO_RANGE: u8 = 0x0F;
    const REG_GYRO_BANDWIDTH: u8 = 0x10;

    const REG_GYRO_LPM1: u8 = 0x11;
    const POWER_MODE_NORMAL: u8 = 0x00;

    /// Bandwidth in Hz
    const MAX_RATE_HZ: u32 = 2000;
    const DEFAULT_RATE_HZ: u32 = (Self::MAX_RATE_HZ / 2);
    /// Max range in degrees per second
    const MAX_RANGE_DPS: u32 = 2000;
    const DEFAULT_RANGE_DPS: u32 = Self::MAX_RANGE_DPS;

    pub(crate) fn new_with_interface(sensor_interface: SI) -> Self {
        Self {
            si: sensor_interface,
        }
    }

    /// Read the sensor identifiers and
    /// return true if they match the expected value
    pub fn probe(
        &mut self,
        delay_source: &mut impl DelayNs,
    ) -> Result<bool, SI::InterfaceError> {
        let mut chip_id = 0;
        for _ in 0..5 {
            chip_id = self.si.register_read(Self::REG_CHIP_ID)?;
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
        delay_source: &mut impl DelayNs,
    ) -> Result<(), SI::InterfaceError> {
        self.si
            .register_write(Self::REG_SOFT_RESET, Self::CMD_SOFT_RESET)?;
        delay_source.delay_ms(100);
        Ok(())
    }

    /// Give the sensor interface a chance to set up
    pub fn setup(&mut self, delay_source: &mut impl DelayNs) -> Result<(), SI::InterfaceError> {
        self.soft_reset(delay_source)?;

        let probe_success = self.probe(delay_source)?;
        if !probe_success {
            return Err(Error::Unresponsive);
        }

        self.set_range(Self::DEFAULT_RANGE_DPS)?;
        self.set_bandwidth(Self::DEFAULT_RATE_HZ)?;

        //TODO support DRDY and INT pins

        // enable the gyro in normal mode
        self.si.register_write(Self::REG_GYRO_LPM1, Self::POWER_MODE_NORMAL)?;

        Ok(())
    }

    pub fn set_range(&mut self, dps: u32) -> Result<(), SI::InterfaceError> {
        const RANGE_2000_DPS: u8 = 0x00;
        const RANGE_1000_DPS: u8 = 0x01;

        let new_val = if dps < 2000 {
            RANGE_1000_DPS
        } else {
            RANGE_2000_DPS
        };

        self.si.register_write(Self::REG_GYRO_RANGE, new_val)
    }

    pub fn set_bandwidth(&mut self, output_data_rate_hz: u32) -> Result<(), SI::InterfaceError> {
        //const GYRO_ODR_BW_0: u8 = 0x00;//ODR 2000 Hz, filter 532 Hz
        const GYRO_ODR_2000: u8 = 0x01;//ODR 2000 Hz, filter 230 Hz
        const GYRO_ODR_1000: u8 = 0x02;//ODR 1000 Hz, filter 116 Hz

        //REG_GYRO_BANDWIDTH
        let new_val = if output_data_rate_hz < 2000 {
            GYRO_ODR_2000
        }
        else {
            GYRO_ODR_1000
        };
        self.si.register_write(Self::REG_GYRO_BANDWIDTH, new_val)
    }

    pub fn get_gyro(&mut self) -> Result<[i16; 3], SI::InterfaceError> {
        let sample = self.si.read_vec3_i16(Self::REG_GYRO_START)?;
        Ok(sample)
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
