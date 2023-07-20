//! This library aims to provide support for older legacy Arducam cameras such as ArduCAM Mini 2MP Plus
//! It provides `embedded-hal` compatible API
//! 
//! # Example
//! ```rust
//! #![no_std]
//! #![no_main]
//! 
//! use stm32_hal2::{pac, gpio::{Pin, Port, PinMode, OutputType}, spi::{Spi, BaudRate}, i2c::I2c, timer::Timer};
//! use cortex_m::delay::Delay;
//! 
//! use arducam_legacy::Arducam;
//! 
//! fn main() -> ! {
//!     let cp = cortex_m::Peripherals::take().unwrap();
//!     let dp = pac::Peripherals::take().unwrap();
//! 
//!     // Clocks setup
//!     let clock_cfg = stm32_hal2::clocks::Clocks::default();
//!     clock_cfg.setup().unwrap();
//!     let mut delay = Delay::new(cp.SYST, clock_cfg.systick());
//!     let mut mono_timer = Timer::new_tim2(dp.TIM2, 100.0, Default::default(), &clock_cfg);
//! 
//!     // Example pinout configuration
//!     // Adapt to your HAL crate
//!     let _arducam_spi_mosi = Pin::new(Port::D, 4, PinMode::Alt(5));
//!     let _arducam_spi_miso = Pin::new(Port::D, 3, PinMode::Alt(5));
//!     let _arducam_spi_sck = Pin::new(Port::D, 1, PinMode::Alt(5));
//!     let arducam_cs = Pin::new(Port::D, 0, PinMode::Output);
//!     let arducam_spi = Spi::new(dp.SPI2, Default::default(), BaudRate::Div32);
//!     let mut arducam_i2c_sda = Pin::new(Port::F, 0, PinMode::Alt(4));
//!     arducam_i2c_sda.output_type(OutputType::OpenDrain);
//!     let mut arducam_i2c_scl = Pin::new(Port::F, 1, PinMode::Alt(4));
//!     arducam_i2c_scl.output_type(OutputType::OpenDrain);
//!     let arducam_i2c = I2c::new(dp.I2C2, Default::default(), &clock_cfg);
//! 
//!     let mut arducam = Arducam::new(
//!         arducam_spi,
//!         arducam_i2c,
//!         arducam_cs,
//!         arducam_legacy::Resolution::Res320x240, arducam_legacy::ImageFormat::JPEG
//!         );
//!     arducam.init(&mut delay).unwrap();
//!     
//!     arducam.start_capture().unwrap();
//!     while !arducam.is_capture_done().unwrap() { delay.delay_ms(1) }
//!     let mut image = [0; 8192];
//!     let length = arducam.get_fifo_length().unwrap();
//!     let final_length = arducam.read_captured_image(&mut image).unwrap();
//! 
//!     loop {}
//! }
//! ```

#![no_std]
#![no_main]

use core::fmt;

use embedded_hal::{blocking::{spi::{self, Transfer}, i2c, delay::DelayMs}, digital::v2::OutputPin};
use ov2640_registers::*;

mod ov2640_registers;

const ARDUCHIP_TEST1: u8 = 0x00;
const ARDUCHIP_FIFO: u8 = 0x04;
const ARDUCHIP_TRIG: u8 = 0x41;
const OV2640_ADDR: u8 = 0x60 >> 1;
const OV2640_CHIPID_HIGH: u8 = 0x0A;
const OV2640_CHIPID_LOW: u8 = 0x0B;
const FIFO_CLEAR_MASK: u8 = 0x01;
const FIFO_START_MASK: u8 = 0x02;
const FIFO_BURST: u8 = 0x3C;
const FIFO_SIZE1: u8 = 0x42;
const FIFO_SIZE2: u8 = 0x43;
const FIFO_SIZE3: u8 = 0x44;
const CAP_DONE_MASK: u8 = 0x08;

#[derive(fmt::Debug)]
/// Possible errors which can happen during communication
pub enum Error<SpiErr, I2cErr, PinErr> {
    Spi(SpiErr),
    I2c(I2cErr),
    Pin(PinErr),
    OutOfBounds
}

#[derive(Debug)]
// Image resolutions
pub enum Resolution {
    Res160x120,
    Res176x144,
    Res320x240,
    Res352x288,
    Res640x480,
    Res800x600,
    Res1024x768,
    Res1280x1024,
    Res1600x1200
}

#[derive(PartialEq, Eq, Debug)]
/// Image formats which Arducam can handle
pub enum ImageFormat {
    // BMP,
    // RAW,
    JPEG
}

/// Main struct responsible for communicating with Arducam
pub struct Arducam<SPI, I2C, CS> {
    spi: SPI,
    spi_cs: CS,
    i2c: I2C,
    format: ImageFormat,
    resolution: Resolution
}

impl<SPI, I2C, CS, SpiErr, I2cErr, PinErr> Arducam<SPI, I2C, CS>
where
    SPI: Transfer<u8, Error = SpiErr> + spi::Write<u8, Error = SpiErr>,
    I2C: i2c::Write<Error = I2cErr> + i2c::WriteRead<Error = I2cErr>,
    CS: OutputPin<Error = PinErr>
{
    /// Creates a new Arducam instance but doesn't initialize it
    pub fn new(spi: SPI, i2c: I2C, cs_pin: CS, resolution: Resolution, format: ImageFormat) -> Arducam<SPI, I2C, CS> {
        Arducam {
            spi,
            spi_cs: cs_pin,
            i2c,
            format,
            resolution,
        }
    }

    /// Initializes Arducam to resetted state
    pub fn init<D>(&mut self, delay: &mut D) -> Result<(), Error<SpiErr, I2cErr, PinErr>>
    where
        D: DelayMs<u32>
    {
        self.arduchip_write_reg(0x07, 0x80)?;
        delay.delay_ms(100);
        self.arduchip_write_reg(0x07, 0x00)?;
        delay.delay_ms(100);
        self.sensor_writereg8_8(0xFF, 0x01)?;
        self.sensor_writereg8_8(0x12, 0x80)?;
        delay.delay_ms(100);

        // if self.format == ImageFormat::JPEG {
            unsafe {
                self.sensor_writeregs8_8(&OV2640_JPEG_INIT)?;
                self.sensor_writeregs8_8(&OV2640_YUV422)?;
            }
            self.sensor_writereg8_8(0xFF, 0x01)?;
            self.sensor_writereg8_8(0x15, 0x00)?;
            self.send_resolution()?;
        // }
        // else {
        //     unsafe { self.sensor_writeregs8_8(&OV2640_QVGA)?; }
        // }

        Ok(())
    }

    /// Sets camera resolution
    pub fn set_resolution(&mut self, resolution: Resolution) -> Result<(), Error<SpiErr, I2cErr, PinErr>> {
        self.resolution = resolution;
        self.send_resolution()?;
        Ok(())
    }

    /// Checks if Arducam is still connected to SPI bus
    pub fn is_connected(&mut self) -> Result<bool, Error<SpiErr, I2cErr, PinErr>> {
        let test_value = 0x52;
        self.arduchip_write_reg(ARDUCHIP_TEST1, test_value)?;
        let result = self.arduchip_read_reg(ARDUCHIP_TEST1)?;

        let valid_ov2640_chipid1 = [0x26, 0x41];
        let valid_ov2640_chipid2 = [0x26, 0x42];
        let chipid = self.get_sensor_chipid()?;

        if test_value == result && chipid == valid_ov2640_chipid1 || chipid == valid_ov2640_chipid2 {
            Ok(true)
        }
        else {
            Ok(false)
        }
    }

    /// Sends image capture request
    pub fn start_capture(&mut self) -> Result<(), Error<SpiErr, I2cErr, PinErr>> {
        self.flush_fifo()?;
        self.start_fifo()?;
        Ok(())
    }

    /// Checks if image capture is done
    pub fn is_capture_done(&mut self) -> Result<bool, Error<SpiErr, I2cErr, PinErr>> {
        self.arduchip_read_reg(ARDUCHIP_TRIG).map(|result| { result & CAP_DONE_MASK != 0 })
    }

    /// Saves captured image to provided mutable slice
    /// It is important to be sure if that slice will be big enough for image data
    /// otherwise data will be cut
    /// 
    /// # Returns
    /// Actual image size
    pub fn read_captured_image(&mut self, data_out: &mut [u8]) -> Result<usize, Error<SpiErr, I2cErr, PinErr>> {
        let length = self.get_fifo_length()?;
        let mut final_length = 0;
        // Check if borrowed array is too small for image
        if length > data_out.len() as u32 {
            return Err(Error::OutOfBounds);
        }
        self.spi_cs.set_low().map_err(Error::Pin)?;
        self.set_fifo_burst()?;
        let mut curr_byte = 0;
        #[allow(unused_assignments)]
        let mut prev_byte = 0;
        for (i, b) in data_out.iter_mut().enumerate() {
            prev_byte = curr_byte;
            curr_byte = self.spi.transfer(&mut [0x00]).map_err(Error::Spi)?[0];
            *b = curr_byte;
            if prev_byte == 0xFF && curr_byte == 0xD9 || i as u32 > length {
                final_length = i;
                break;
            }
        }
        Ok(final_length)
    }

    /// Returns image length reported by arduchip in FIFO
    pub fn get_fifo_length(&mut self) -> Result<u32, Error<SpiErr, I2cErr, PinErr>> {
        let mut len_builder = (0u32, 0u32, 0u32);
        len_builder.0 = self.arduchip_read_reg(FIFO_SIZE1)?.into();
        len_builder.1 = self.arduchip_read_reg(FIFO_SIZE2)?.into();
        len_builder.2 = (self.arduchip_read_reg(FIFO_SIZE3)? & 0x7F).into();
        Ok((len_builder.2 << 16 | len_builder.1 << 8 | len_builder.0) as u32 & 0x7FFFFFu32)
    }

    /// Returns sensor vendor and product ID
    pub fn get_sensor_chipid(&mut self) -> Result<[u8; 2], Error<SpiErr, I2cErr, PinErr>> {
        let mut chipid: [u8; 2] = [0; 2];
        self.sensor_writereg8_8(0xFF, 0x01)?;
        self.sensor_readreg8_8(OV2640_CHIPID_HIGH, &mut chipid[0..1])?;
        self.sensor_readreg8_8(OV2640_CHIPID_LOW, &mut chipid[1..2])?;
        Ok(chipid)
    }

    fn send_resolution(&mut self) -> Result<(), Error<SpiErr, I2cErr, PinErr>> {
        unsafe {
            match self.resolution {
                Resolution::Res160x120 => { self.sensor_writeregs8_8(&OV2640_160x120_JPEG)? },
                Resolution::Res1024x768 => { self.sensor_writeregs8_8(&OV2640_1024x768_JPEG)? },
                Resolution::Res1280x1024 => { self.sensor_writeregs8_8(&OV2640_1280x1024_JPEG)? },
                Resolution::Res1600x1200 => { self.sensor_writeregs8_8(&OV2640_1600x1200_JPEG)? },
                Resolution::Res176x144 => { self.sensor_writeregs8_8(&OV2640_176x144_JPEG)? },
                Resolution::Res320x240 => { self.sensor_writeregs8_8(&OV2640_320x240_JPEG)? },
                Resolution::Res352x288 => { self.sensor_writeregs8_8(&OV2640_352x288_JPEG)? },
                Resolution::Res640x480 => { self.sensor_writeregs8_8(&OV2640_640x480_JPEG)? },
                Resolution::Res800x600 => { self.sensor_writeregs8_8(&OV2640_800x600_JPEG)? },
            }
        }

        Ok(())
    }

    fn flush_fifo(&mut self) -> Result<(), Error<SpiErr, I2cErr, PinErr>> {
        self.arduchip_write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK)
    }

    fn start_fifo(&mut self) -> Result<(), Error<SpiErr, I2cErr, PinErr>> {
        self.arduchip_write_reg(ARDUCHIP_FIFO, FIFO_START_MASK)
    }

    fn set_fifo_burst(&mut self) -> Result<(), Error<SpiErr, I2cErr, PinErr>> {
        self.spi.write(&[FIFO_BURST]).map_err(Error::Spi)
    }

    fn arduchip_write(&mut self, addr: u8, data: u8) -> Result<(), Error<SpiErr, I2cErr, PinErr>> {
        self.spi_cs.set_low().map_err(Error::Pin)?;
        self.spi.write(&[addr; 1]).map_err(Error::Spi)?;
        self.spi.write(&[data; 1]).map_err(Error::Spi)?;
        self.spi_cs.set_high().map_err(Error::Pin)?;
        Ok(())
    }

    fn arduchip_read(&mut self, addr: u8) -> Result<u8, Error<SpiErr, I2cErr, PinErr>> {
        self.spi_cs.set_low().map_err(Error::Pin)?;
        self.spi.transfer(&mut [addr; 1]).map_err(Error::Spi)?;
        let value = self.spi.transfer(&mut [0; 1]).map_err(Error::Spi)?[0];
        self.spi_cs.set_high().map_err(Error::Pin)?;
        Ok(value)
    }

    fn arduchip_write_reg(&mut self, addr: u8, data: u8) -> Result<(), Error<SpiErr, I2cErr, PinErr>> {
        self.arduchip_write(addr | 0x80, data)
    }

    fn arduchip_read_reg(&mut self, addr: u8) -> Result<u8, Error<SpiErr, I2cErr, PinErr>> {
        self.arduchip_read(addr & 0x7F)
    }

    fn sensor_writeregs8_8(&mut self, regs: &[[u8; 2]]) -> Result<(), Error<SpiErr, I2cErr, PinErr>> {
        for reg in regs {
            self.sensor_writereg8_8(reg[0], reg[1])?;
        }
        Ok(())
    }

    fn sensor_writereg8_8(&mut self, reg: u8, data: u8) -> Result<(), Error<SpiErr, I2cErr, PinErr>> {
        self.i2c.write(OV2640_ADDR, &[reg & 0xFF, data & 0xFF]).map_err(Error::I2c)
    }

    fn sensor_readreg8_8(&mut self, reg: u8, out: &mut [u8]) -> Result<(), Error<SpiErr, I2cErr, PinErr>> {
        self.i2c.write_read(OV2640_ADDR, &[reg & 0xFF], out).map_err(Error::I2c)
    }
}

impl<SPI, I2C, CS> fmt::Debug for Arducam<SPI, I2C, CS>
where
    SPI: fmt::Debug,
    I2C: fmt::Debug,
    CS: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Arducam")
            .field("Spi", &self.spi)
            .field("I2C", &self.i2c)
            .field("Resolution", &self.resolution)
            .field("Image format", &self.format)
            .finish()
    }
}