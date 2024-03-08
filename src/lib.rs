#![no_std]
#![deny(missing_docs)]
#![doc = include_str!("../README.md")]
#![doc(issue_tracker_base_url = "https://github.com/Finomnis/BluefruitSPI/issues")]
#![cfg_attr(docsrs, feature(doc_cfg))]

use core::fmt::Debug;

use snafu::prelude::*;

mod delays;
pub mod sdep;

/// The recommended SPI bitrate for communication
pub const RECOMMENDED_SPI_BITRATE_HZ: u32 = 4_000_000;

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
    type Error: snafu::AsErrorSource;

    /// Sends data to the bus.
    ///
    /// Blocks until the data was completely sent on the bus
    /// and the device is idle again.
    ///
    /// # Arguments
    ///
    /// * `data` - The data to be sent to the bus.
    fn transmit(&mut self, data: &[u8]) -> Result<(), Self::Error>;

    /// Reads data from the bus.
    ///
    /// Blocks until the entire buffer was filled with data and the device is idle again.
    ///
    /// # Arguments
    ///
    /// * `buffer` - The buffer the data should be read into.
    fn receive(&mut self, buffer: &mut [u8; 20]) -> Result<(), Self::Error>;
}

/// Driver for Adafruit Bluefruit LE SPI Friend.
pub struct BluefruitSPI<SPI, CS, RST, IRQ, DELAY> {
    spi: SPI,
    cs: CS,
    _reset: RST,
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
        cassette::block_on(delay.delay_ms(delays::AFTER_RESET_MS));

        Self {
            spi,
            cs,
            _reset: reset,
            irq,
            delay,
            buffer: [0u8; 20],
        }
    }

    /// Write a raw SDEP message to the device
    async fn write_sdep_message<'a>(
        &mut self,
        msg: sdep::Message<'a>,
    ) -> Result<(), Error<SPI::Error>> {
        self.cs.set_low().unwrap();
        self.delay.delay_us(delays::CS_TO_SCK_US).await;

        self.spi
            .transmit(msg.to_bytes(&mut self.buffer))
            .map_err(|source| Error::SpiBus { source })?;

        self.cs.set_high().unwrap();
        self.delay.delay_us(delays::AFTER_SDEP_WRITE_US).await;

        Ok(())
    }

    /// Write a raw SDEP message to the device
    async fn read_sdep_message(&mut self) -> Result<sdep::Message, Error<SPI::Error>> {
        self.cs.set_low().unwrap();
        self.delay.delay_us(delays::CS_TO_SCK_US).await;

        let buffer = &mut self.buffer;
        self.spi
            .receive(buffer)
            .map_err(|source| Error::SpiBus { source })?;

        self.cs.set_high().unwrap();
        self.delay.delay_us(delays::AFTER_SDEP_READ_US).await;

        sdep::Message::from_bytes(buffer).map_err(|source| Error::Sdep { source })
    }

    /// Sends the SDEP initialize command, which causes the board to reset.
    /// This command should complete in under 1s.
    pub async fn init(&mut self) -> Result<(), Error<SPI::Error>> {
        let result = self
            .write_sdep_message(sdep::Message::Command {
                id: sdep::CommandType::Initialize.into(),
                payload: &[],
                more_data: false,
            })
            .await;

        self.delay.delay_ms(delays::AFTER_INIT_MS).await;

        result
    }
}

/// The error type of this crate
#[derive(Debug, Snafu)]
pub enum Error<SpiError: snafu::AsErrorSource> {
    /// An SPI bus error
    SpiBus {
        /// The underlying error
        source: SpiError,
    },
    /// An SDEP protocol error
    Sdep {
        /// The underlying error
        source: sdep::Error,
    },
}
