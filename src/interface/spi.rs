use embedded_hal as hal;
use hal::digital::v2::OutputPin;

use super::SensorInterface;
use crate::Error;

/// This combines the SPI peripheral and
/// associated control pins such as:
/// - CSN : Chip Select (aka SS or Slave Select)
/// - DRDY: Data Ready: Sensor uses this to indicate it had data available for read
pub struct SpiInterface<SPI, CSN> {
    /// the SPI port to use when communicating
    spi: SPI,
    /// the Chip Select pin (GPIO output) to use when communicating
    csn: CSN,
    /// should we use the sloppy padded prefix reads? eg for accel part on SPI?
    /// see section 6.1.2 in the BMI088 datasheet
    use_sloppy_reads: bool,
}

impl<SPI, CSN, CommE, PinE> SpiInterface<SPI, CSN>
where
    SPI: hal::blocking::spi::Write<u8, Error = CommE>
        + hal::blocking::spi::Transfer<u8, Error = CommE>,
    CSN: OutputPin<Error = PinE>,
{
    /// Combined with register address for reading single byte register
    const DIR_READ: u8 = 0x80;

    pub fn new(spi: SPI, csn: CSN, use_sloppy_reads: bool) -> Self {
        let mut inst = Self { spi, csn, use_sloppy_reads };
        let _ = inst.csn.set_low();
        let _ = inst.csn.set_high();
        inst
    }

    fn transfer_block(&mut self, block: &mut [u8]) -> Result<(), Error<CommE, PinE>> {
        let _ = self.csn.set_low();
        let rc = self.spi.transfer(block);
        let _ = self.csn.set_high();
        let _ = rc.map_err(Error::Comm)?;

        Ok(())
    }
}

impl<SPI, CSN, CommE, PinE> SensorInterface for SpiInterface<SPI, CSN>
where
    SPI: hal::blocking::spi::Write<u8, Error = CommE>
        + hal::blocking::spi::Transfer<u8, Error = CommE>,
    CSN: OutputPin<Error = PinE>,
    //DRDY: InputPin<Error = PinE>,
{
    type InterfaceError = Error<CommE, PinE>;

    fn register_read(&mut self, reg: u8) -> Result<u8, Self::InterfaceError> {
        /// Combined with register address for reading single byte register
        const DIR_READ: u8 = 0x80;

        if self.use_sloppy_reads {
            // Section 6.1.2: "In case of read operations of the accelerometer part,
            // the requested data is not sent immediately, but instead first a dummy
            // byte is sent, and after this dummy byte the actual reqested register
            // content is transmitted."
            let mut cmd: [u8; 3] = [reg | DIR_READ, 0, 0];
            self.transfer_block(&mut cmd)?;
            Ok(cmd[2])
        }
        else {
            let mut cmd: [u8; 2] = [reg | DIR_READ, 0];
            self.transfer_block(&mut cmd)?;
            Ok(cmd[1])
        }
    }

    fn register_write(&mut self, reg: u8, val: u8) -> Result<(), Self::InterfaceError> {
        let mut cmd: [u8; 2] = [reg, val];
        self.transfer_block(&mut cmd)?;

        Ok(())
    }

    fn read_vec3_i16(&mut self, reg: u8) -> Result<[i16; 3], Self::InterfaceError> {
        let mut resp: [u8; 6] = [0; 6];
        if self.use_sloppy_reads {
            // Section 6.1.2: "In case of read operations of the accelerometer part,
            // the requested data is not sent immediately, but instead first a dummy
            // byte is sent, and after this dummy byte the actual reqested register
            // content is transmitted."
            let mut block: [u8; 8] = [0; 8];
            block[0] = reg | Self::DIR_READ;
            self.transfer_block(&mut block)?;
            // [0] - response to read flags
            // [1] - garbage dummy byte
            resp.copy_from_slice(&block[2..8]);
            //&block[2..8]
        }
        else {
            let mut block: [u8; 7] = [0; 7];
            block[0] = reg | Self::DIR_READ;
            self.transfer_block(&mut block)?;
            // [0] - response to read flags
            resp.copy_from_slice(&block[1..7]);
            //&block[1..7]
        };

        //This is a little-endian device, eg:
        // 0x02 RATE_X_LSB
        // 0x03 RATE_X_MSB
        Ok([
            (resp[1] as i16) << 8 | (resp[0] as i16),
            (resp[3] as i16) << 8 | (resp[2] as i16),
            (resp[5] as i16) << 8 | (resp[4] as i16),
        ])
    }
}
