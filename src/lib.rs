#![no_std]
#![deny(missing_docs)]
#![doc = include_str!("../README.md")]
#![doc(issue_tracker_base_url = "https://github.com/Finomnis/BluefruitSPI/issues")]
#![cfg_attr(docsrs, feature(doc_cfg))]

mod delays;
pub mod sdep;

/// SPI Bus the Adafruit Bluefruit LE SPI Friend is attached to.
///
/// The bus should run at 4MHz, SPI Mode 0, MSB first.
///
/// It should be connected to MOSI, MISO and SCK.
///
/// **Important:** It should **not** use a CS pin, the [`BluefruitSPI`] driver controls
/// CS through GPIO.
pub trait SpiBus {
    /// The errors that can happen in BUS communication
    type Error;

    /// Sends data to the bus.
    ///
    /// # Arguments
    ///
    /// * `data` - The data to be sent to the bus.
    fn send(&mut self, data: &[u8]) -> Result<(), Self::Error>;
}

/// Driver for Adafruit Bluefruit LE SPI Friend.
pub struct BluefruitSPI<SPI, CS, RST, IRQ, DELAY> {
    spi: SPI,
    cs: CS,
    reset: RST,
    irq: IRQ,
    delay: DELAY,
    buffer: [u8; 20],
}

impl<SPI, CS, RST, IRQ, DELAY> BluefruitSPI<SPI, CS, RST, IRQ, DELAY>
where
    SPI: SpiBus,
    CS: embedded_hal::digital::OutputPin,
    RST: embedded_hal::digital::OutputPin,
    IRQ: embedded_hal::digital::InputPin + embedded_hal_async::digital::Wait,
    DELAY: embedded_hal_async::delay::DelayNs,
{
    /// Create a new instance of this driver.
    ///
    /// # Arguments
    ///
    /// * `spi` - The SPI driver for MOSI, MISO and SCK.
    /// * `cs` - The CS pin. Should be push-pull.
    /// * `rst` - The RST pin. Should be push-pull.
    /// * `irq` - The interrupt pin. Should be have a pull-down.
    /// * `delay` - A time source.
    pub fn new(spi: SPI, mut cs: CS, mut reset: RST, irq: IRQ, mut delay: DELAY) -> Self {
        cs.set_high().unwrap();

        reset.set_low().unwrap();
        cassette::block_on(delay.delay_ms(delays::RESET_PULSE_LENGTH_MS));
        reset.set_high().unwrap();
        cassette::block_on(delay.delay_ms(delays::WAIT_AFTER_RESET_MS));

        Self {
            spi,
            cs,
            reset,
            irq,
            delay,
            buffer: [0u8; 20],
        }
    }

    //pub async fn init() {}
}
