[arducam]: https://docs.arducam.com/Arduino-SPI-camera/Legacy-SPI-camera/Introduction/
[embedded-hal]: https://github.com/rust-embedded/embedded-hal

# `arducam-legacy`
This Rust library provides [embedded-hal][embedded-hal] support for [legacy Arducam SPI-based cameras][arducam] like e.g. ArduCAM MINI 2MP Plus. It has been rewritten from original Arducam library written in C.

## Milestones
- [x] OV2640 support
- [x] JPEG image support
- [x] Different resolutions
- [ ] More sensor models support
- [ ] DMA API

## Model support
Currently this library is only tested with Arducam Mini 2MP Plus model. Contributions for other models are welcome!

## MCU support
Any microcontroller HAL with [embedded-hal][embedded-hal] support should work with this driver

## Usage
Add this line to `Cargo.toml` under ```[dependencies]``` of your project
```toml
[dependencies]
arducam-legacy = "0.1.0"
```

## Example
This example shows how to capture image and store it in array buffer

```rust
#![no_std]
#![no_main]

use stm32_hal2::{pac, gpio::{Pin, Port, PinMode, OutputType}, spi::{Spi, BaudRate}, i2c::I2c, timer::Timer};
use cortex_m::delay::Delay;

use arducam_legacy::Arducam;

fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // Clocks setup
    let clock_cfg = stm32_hal2::clocks::Clocks::default();
    clock_cfg.setup().unwrap();
    let mut delay = Delay::new(cp.SYST, clock_cfg.systick());
    let mut mono_timer = Timer::new_tim2(dp.TIM2, 100.0, Default::default(), &clock_cfg);

    // Example pinout configuration
    // Adapt to your HAL crate
    let _arducam_spi_mosi = Pin::new(Port::D, 4, PinMode::Alt(5));
    let _arducam_spi_miso = Pin::new(Port::D, 3, PinMode::Alt(5));
    let _arducam_spi_sck = Pin::new(Port::D, 1, PinMode::Alt(5));
    let arducam_cs = Pin::new(Port::D, 0, PinMode::Output);
    let arducam_spi = Spi::new(dp.SPI2, Default::default(), BaudRate::Div32);
    let mut arducam_i2c_sda = Pin::new(Port::F, 0, PinMode::Alt(4));
    arducam_i2c_sda.output_type(OutputType::OpenDrain);
    let mut arducam_i2c_scl = Pin::new(Port::F, 1, PinMode::Alt(4));
    arducam_i2c_scl.output_type(OutputType::OpenDrain);
    let arducam_i2c = I2c::new(dp.I2C2, Default::default(), &clock_cfg);

    let mut arducam = Arducam::new(
        arducam_spi,
        arducam_i2c,
        arducam_cs,
        arducam_legacy::Resolution::Res320x240, arducam_legacy::ImageFormat::JPEG
        );
    arducam.init(&mut delay).unwrap();

    arducam.start_capture().unwrap();
    while !arducam.is_capture_done().unwrap() { delay.delay_ms(1) }
    let mut image = [0; 8192];
    let length = arducam.get_fifo_length().unwrap();
    let final_length = arducam.read_captured_image(&mut image).unwrap();

    loop {}
}
```