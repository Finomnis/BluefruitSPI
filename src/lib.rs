#![no_std]
#![deny(missing_docs)]
#![doc = include_str!("../README.md")]
#![doc(issue_tracker_base_url = "https://github.com/Finomnis/BluefruitSPI/issues")]
#![cfg_attr(docsrs, feature(doc_cfg))]

use core::fmt::Debug;

use snafu::prelude::*;

mod delays;
pub mod sdep;
mod sdep_driver;

#[cfg(feature = "imxrt")]
#[cfg_attr(docsrs, doc(cfg(feature = "imxrt")))]
pub mod imxrt;

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
    /// * `data` - The data to be sent to the bus. Guaranteed to be 20 bytes or less.
    ///
    /// # Return
    ///
    /// The first byte of the received data, which contains the immediate result code.
    fn transmit(&mut self, data: &[u8]) -> Result<u8, Self::Error>;

    /// Reads data from the bus.
    ///
    /// Blocks until the entire buffer was filled with data and the device is idle again.
    ///
    /// # Arguments
    ///
    /// * `buffer` - The buffer the data should be read into.
    fn receive(
        &mut self,
        buffer: &mut [u8; sdep::SDEP_MAX_MESSAGE_SIZE],
    ) -> Result<(), Self::Error>;
}

/// Driver for Adafruit Bluefruit LE SPI Friend.
pub struct BluefruitSPI<SPI, CS, RST, IRQ, DELAY> {
    sdep: sdep_driver::SdepDriver<IRQ, SPI, CS>,
    _reset: RST,
    delay: DELAY,
    command_buffer: [u8; 256],
}

impl<SPI, CS, RST, IRQ, DELAY> BluefruitSPI<SPI, CS, RST, IRQ, DELAY>
where
    SPI: SpiBus,
    CS: embedded_hal::digital::OutputPin,
    RST: embedded_hal::digital::OutputPin,
    IRQ: embedded_hal::digital::InputPin,
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
            sdep: sdep_driver::SdepDriver::new(irq, spi, cs),
            _reset: reset,
            delay,
            command_buffer: [0u8; 256],
        }
    }

    /// Sends the SDEP initialize command, which causes the board to reset.
    /// This command should complete in under 1s.
    pub async fn init(&mut self) -> Result<(), Error<SPI::Error>> {
        let result = self
            .sdep
            .write(
                sdep::Message::Command {
                    id: sdep::CommandType::Initialize.into(),
                    payload: &[],
                    more_data: false,
                },
                &mut self.delay,
            )
            .await;

        self.delay.delay_ms(delays::AFTER_INIT_MS).await;

        result
    }

    async fn raw_command(
        &mut self,
        data_iter: &mut dyn Iterator<Item = u8>,
    ) -> Result<&[u8], Error<SPI::Error>> {
        self.sdep
            .execute_command(
                sdep::CommandType::AtWrapper,
                data_iter,
                &mut self.command_buffer,
                &mut self.delay,
            )
            .await
    }

    async fn raw_uart_tx(
        &mut self,
        data_iter: &mut dyn Iterator<Item = u8>,
    ) -> Result<(), Error<SPI::Error>> {
        self.sdep
            .execute_command(
                sdep::CommandType::BleUartTx,
                data_iter,
                &mut [],
                &mut self.delay,
            )
            .await
            .map(|_| ())
    }

    /// Send a fully formed bytestring AT command, and check
    /// whether we got an `OK` back.
    ///
    /// # Returns
    ///
    /// The response payload, if there is any.
    pub async fn command(&mut self, command: &[u8]) -> Result<&[u8], Error<SPI::Error>> {
        let msg = self
            .raw_command(&mut command.iter().chain(b"\n").copied())
            .await?;
        msg.strip_suffix(b"OK\r\n").ok_or(Error::NotOk)
    }

    /// Whether the Bluefruit module is connected to the central
    pub async fn connected(&mut self) -> Result<bool, Error<SPI::Error>> {
        self.command(b"AT+GAPGETCONN")
            .await
            .map(|val| val == b"1\r\n")
    }

    /// Sends the specific bytestring out over BLE UART.
    ///
    /// # Arguments
    ///
    /// * `data` - The bytestring to send.
    pub async fn uart_tx(
        &mut self,
        data: impl IntoIterator<Item = u8>,
    ) -> Result<(), Error<SPI::Error>> {
        /// Completely heuristic ...
        /// Seems to break at somewhere around ~300, so let's be safe and choose a smaller value
        const UART_TX_MAX_BURST_SIZE: usize = 128;

        let mut data = data.into_iter().peekable();

        let mut finished = false;
        while !finished {
            let tx_fifo_space = {
                let response = self.command(b"AT+BLEUARTFIFO=TX").await?;
                core::str::from_utf8(response)
                    .map_err(|_| Error::ResponseInvalid)?
                    .trim_end()
                    .parse::<usize>()
                    .map_err(|_| Error::ResponseInvalid)?
            }
            .min(UART_TX_MAX_BURST_SIZE);

            if tx_fifo_space == 0 {
                continue;
            }

            let mut num_transmitted = 0;
            if data.peek().is_some() {
                self.raw_uart_tx(
                    &mut (&mut data)
                        .take(tx_fifo_space)
                        .inspect(|_| num_transmitted += 1),
                )
                .await?;
            }

            if num_transmitted < tx_fifo_space {
                finished = true;
            }
        }

        Ok(())
    }
}

/// The error type of this crate
#[derive(Debug, Snafu)]
pub enum Error<SpiError: snafu::AsErrorSource> {
    /// An SPI bus error.
    SpiBus {
        /// The underlying error.
        source: SpiError,
    },
    /// An SDEP protocol error.
    Sdep {
        /// The underlying error.
        source: sdep::Error,
    },
    /// The payload of a the command was too long.
    CommandPayloadTooLong,
    /// The payload of a the response was too long.
    ResponsePayloadTooLong,
    /// The device stopped answering before the payload was complete.
    ResponsePayloadIncomplete,
    /// The device answered with an invalid response.
    ResponseInvalid,
    /// The device answered with an invalid response.
    WriteResponseInvalid {
        /// The value of the write response
        response: u8,
    },
    /// The device answered with an error code.
    ErrorResponse {
        /// The error id.
        id: u16,
    },
    /// The command did not return the `OK` message.
    NotOk,
    /// Timeout while waiting for device response.
    Timeout,
}

/// An impossible error; this error can never happen.
///
/// Same as [`core::convert::Infallible`], but with [`snafu`] support.
#[derive(Debug, Snafu)]
pub enum Infallible {}
