#![no_std]
#![deny(missing_docs)]
#![doc = include_str!("../README.md")]
#![doc(issue_tracker_base_url = "https://github.com/Finomnis/BluefruitSPI/issues")]
#![cfg_attr(docsrs, feature(doc_cfg))]

use core::fmt::Debug;

use snafu::prelude::*;

mod delays;
pub mod sdep;

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
    fn receive(&mut self, buffer: &mut [u8; 20]) -> Result<(), Self::Error>;
}

struct SdepDriver<SPI, CS> {
    spi: SPI,
    cs: CS,
    buffer: [u8; 20],
}

/// Driver for Adafruit Bluefruit LE SPI Friend.
pub struct BluefruitSPI<SPI, CS, RST, IRQ, DELAY> {
    sdep: SdepDriver<SPI, CS>,
    _reset: RST,
    irq: IRQ,
    delay: DELAY,
    command_buffer: [u8; 256],
}

impl<SPI, CS> SdepDriver<SPI, CS>
where
    SPI: SpiBus,
    CS: embedded_hal::digital::OutputPin,
{
    /// Write a raw SDEP message to the device
    async fn write<'a, DELAY: embedded_hal_async::delay::DelayNs>(
        &mut self,
        msg: sdep::Message<'a>,
        delay: &mut DELAY,
    ) -> Result<(), Error<SPI::Error>> {
        let data = msg.to_bytes(&mut self.buffer);

        let mut iteration = 0;
        loop {
            self.cs.set_low().unwrap();
            delay.delay_us(delays::CS_TO_SCK_US).await;

            let result = self.spi.transmit(data);
            self.cs.set_high().unwrap();

            let write_response = result.map_err(|source| Error::SpiBus { source })?;
            match sdep::MessageType::from_repr(write_response) {
                Some(sdep::MessageType::DeviceReadOverflow) => break,
                Some(sdep::MessageType::DeviceNotReady) => Ok(()),
                _ => Err(Error::WriteResponseInvalid {
                    response: write_response,
                }),
            }?;

            log::warn!("Write retry!");
            delay.delay_us(delays::WRITE_RETRY_DELAY_US).await;

            iteration += 1;
            if iteration >= delays::WRITE_RETRY_COUNT {
                return Err(Error::Sdep {
                    source: sdep::Error::DeviceNotReady,
                });
            }
        }

        if msg.more_data() {
            delay.delay_us(delays::BETWEEN_SDEP_WRITE_US).await;
        } else {
            delay.delay_us(delays::AFTER_SDEP_WRITE_US).await;
        }

        Ok(())
    }

    /// Write a raw SDEP message to the device
    async fn read<DELAY: embedded_hal_async::delay::DelayNs>(
        &mut self,
        delay: &mut DELAY,
    ) -> Result<sdep::Message, Error<SPI::Error>> {
        self.cs.set_low().unwrap();
        delay.delay_us(delays::CS_TO_SCK_US).await;

        let buffer = &mut self.buffer;
        let spi_result = self.spi.receive(buffer);
        self.cs.set_high().unwrap();

        spi_result.map_err(|source| Error::SpiBus { source })?;

        let result = sdep::Message::from_bytes(buffer).map_err(|source| Error::Sdep { source });

        let delay_us = match &result {
            Ok(msg) => {
                if msg.more_data() {
                    delays::BETWEEN_SDEP_READ_US
                } else {
                    delays::AFTER_SDEP_READ_US
                }
            }
            Err(e) => match e {
                Error::Sdep {
                    source: sdep::Error::DeviceNotReady,
                } => delays::BETWEEN_SDEP_READ_US,
                _ => delays::AFTER_SDEP_READ_US,
            },
        };

        delay.delay_us(delay_us).await;

        result
    }
}

impl<SPI, CS, RST, IRQ, DELAY> BluefruitSPI<SPI, CS, RST, IRQ, DELAY>
where
    SPI: SpiBus,
    CS: embedded_hal::digital::OutputPin,
    RST: embedded_hal::digital::OutputPin,
    IRQ: embedded_hal::digital::InputPin, //TODO: use interrupts + embedded_hal_async::digital::Wait,
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
            sdep: SdepDriver {
                spi,
                cs,
                buffer: [0u8; 20],
            },
            _reset: reset,
            irq,
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
        // Prepare data
        let mut data = {
            let mut data_size = 0;
            let data = &mut self.command_buffer[..127];

            for elem in data.iter_mut() {
                match data_iter.next() {
                    Some(val) => {
                        *elem = val;
                        data_size += 1;
                    }
                    None => {
                        break;
                    }
                }
            }

            if data_iter.next().is_some() {
                return Err(Error::CommandPayloadTooLong);
            }

            &data[..data_size]
        };

        // Send Command
        while !data.is_empty() {
            let more_data = data.len() > 16;
            let payload_len = data.len().min(16);
            let (payload, leftover) = data.split_at(payload_len);
            data = leftover;

            self.sdep
                .write(
                    sdep::Message::Command {
                        id: sdep::CommandType::AtWrapper.into(),
                        payload,
                        more_data,
                    },
                    &mut self.delay,
                )
                .await?;
        }

        // Wait for response
        let mut timeout_left = delays::RESPONSE_TIMEOUT_MS;
        while !self.irq.is_high().unwrap() {
            if timeout_left == 0 {
                return Err(Error::Timeout);
            }

            self.delay.delay_ms(delays::IRQ_POLL_PERIOD_MS).await;
            timeout_left = timeout_left.saturating_sub(delays::IRQ_POLL_PERIOD_MS);
        }

        // Read response
        let data = &mut self.command_buffer;
        let mut data_size = 0;

        loop {
            if !self.irq.is_high().unwrap() {
                return Err(Error::ResponsePayloadIncomplete);
            }

            match self.sdep.read(&mut self.delay).await {
                Ok(msg) => match msg {
                    sdep::Message::Command { .. } => return Err(Error::ResponseInvalid),
                    sdep::Message::Response {
                        id,
                        payload,
                        more_data,
                    } => {
                        if id != sdep::CommandType::AtWrapper.into() {
                            return Err(Error::ResponseInvalid);
                        }
                        for word in payload {
                            *data
                                .get_mut(data_size)
                                .ok_or(Error::ResponsePayloadTooLong)? = *word;
                            data_size += 1;
                        }
                        if !more_data {
                            break;
                        }
                    }
                    sdep::Message::Alert { .. } => return Err(Error::ResponseInvalid),
                    sdep::Message::Error { id } => return Err(Error::ErrorResponse { id }),
                },
                Err(Error::Sdep {
                    source: sdep::Error::DeviceNotReady,
                }) => {
                    continue;
                }
                Err(e) => return Err(e),
            }
        }

        Ok(&data[..data_size])
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
    pub async fn uart_tx(&mut self, data: &[u8]) -> Result<(), Error<SPI::Error>> {
        // let response = self
        //     .raw_command(&mut b"AT+BLEUARTTX=".iter().chain(data).chain(b"\r\n").copied())
        //     .await?;
        // if response == b"OK\r\n" {
        //     Ok(())
        // } else {
        //     Err(Error::NotOk)
        // }

        // Send data
        self.sdep
            .write(
                sdep::Message::Command {
                    id: sdep::CommandType::BleUartTx.into(),
                    payload: data,
                    more_data: false,
                },
                &mut self.delay,
            )
            .await?;

        // Wait for response
        let mut timeout_left = delays::RESPONSE_TIMEOUT_MS;
        while !self.irq.is_high().unwrap() {
            if timeout_left == 0 {
                return Err(Error::Timeout);
            }

            self.delay.delay_ms(delays::IRQ_POLL_PERIOD_MS).await;
            timeout_left = timeout_left.saturating_sub(delays::IRQ_POLL_PERIOD_MS);
        }

        // Read response
        loop {
            if !self.irq.is_high().unwrap() {
                return Err(Error::ResponsePayloadIncomplete);
            }

            match self.sdep.read(&mut self.delay).await {
                Ok(msg) => match msg {
                    sdep::Message::Command { .. } => return Err(Error::ResponseInvalid),
                    sdep::Message::Response {
                        id,
                        payload,
                        more_data,
                    } => {
                        if id != sdep::CommandType::BleUartTx.into()
                            || !payload.is_empty()
                            || more_data
                        {
                            return Err(Error::ResponseInvalid);
                        }
                        break;
                    }
                    sdep::Message::Alert { .. } => return Err(Error::ResponseInvalid),
                    sdep::Message::Error { id } => return Err(Error::ErrorResponse { id }),
                },
                Err(Error::Sdep {
                    source: sdep::Error::DeviceNotReady,
                }) => {
                    continue;
                }
                Err(e) => return Err(e),
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
