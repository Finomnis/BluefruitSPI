use crate::{delays, sdep, Error, SpiBus};

struct SdepRawDriver<SPI, CS> {
    spi: SPI,
    cs: CS,
}

pub struct SdepDriver<SPI, CS> {
    raw: SdepRawDriver<SPI, CS>,
    buffer: [u8; sdep::SDEP_MAX_MESSAGE_SIZE],
}

impl<SPI, CS> SdepDriver<SPI, CS>
where
    SPI: SpiBus,
    CS: embedded_hal::digital::OutputPin,
{
    pub fn new(spi: SPI, cs: CS) -> Self {
        Self {
            raw: SdepRawDriver { spi, cs },
            buffer: [0u8; sdep::SDEP_MAX_MESSAGE_SIZE],
        }
    }

    /// Write a raw SDEP message to the device.
    pub async fn write<'a, DELAY: embedded_hal_async::delay::DelayNs>(
        &mut self,
        msg: sdep::Message<'a>,
        delay: &mut DELAY,
    ) -> Result<(), Error<SPI::Error>> {
        let mut iteration = 0;
        loop {
            let result = self.raw.write(&msg, &mut self.buffer, delay).await;
            if !matches!(
                result,
                Err(Error::Sdep {
                    source: sdep::Error::DeviceNotReady,
                })
            ) || iteration >= delays::WRITE_RETRY_COUNT
            {
                break result;
            }

            iteration += 1;
        }
    }

    /// Read a raw SDEP message from the device.
    pub async fn read<DELAY: embedded_hal_async::delay::DelayNs>(
        &mut self,
        delay: &mut DELAY,
    ) -> Result<sdep::Message, Error<SPI::Error>> {
        let mut iteration = 0;

        let buffer = &mut self.buffer;

        loop {
            // Reborrow to allow `buffer` to be re-used.
            //
            // For more info, see https://docs.rs/polonius-the-crab.
            //
            // Soundness: `result` gets dropped after the `if` clause, which
            // causes `buffer` to no longer be borrowed. The borrow checker doesn't see this, however,
            // because the lifetime is tied to `result`.
            let tentative_borrowed_buffer = unsafe { &mut *(buffer as *mut _) };

            let result = self.raw.read(tentative_borrowed_buffer, delay).await;
            if !matches!(
                result,
                Err(Error::Sdep {
                    source: sdep::Error::DeviceNotReady,
                })
            ) || iteration >= delays::READ_RETRY_COUNT
            {
                break result;
            }
            drop(result);

            iteration += 1;
        }
    }
}

impl<SPI, CS> SdepRawDriver<SPI, CS>
where
    SPI: SpiBus,
    CS: embedded_hal::digital::OutputPin,
{
    /// Read a raw SDEP message from the device.
    ///
    /// Do not retry if the device signals `DeviceNotReady`.
    async fn read<'a, DELAY: embedded_hal_async::delay::DelayNs>(
        &mut self,
        buffer: &'a mut [u8; sdep::SDEP_MAX_MESSAGE_SIZE],
        delay: &mut DELAY,
    ) -> Result<sdep::Message<'a>, Error<SPI::Error>> {
        self.cs.set_low().unwrap();
        delay.delay_us(delays::CS_TO_SCK_US).await;
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
                } => delays::READ_RETRY_DELAY_US,
                _ => delays::AFTER_SDEP_READ_US,
            },
        };

        delay.delay_us(delay_us).await;

        result
    }

    /// Write a raw SDEP message to the device.
    ///
    /// Do not retry if the device signals `DeviceNotReady`.
    async fn write<'a, DELAY: embedded_hal_async::delay::DelayNs>(
        &mut self,
        msg: &sdep::Message<'a>,
        buffer: &mut [u8; sdep::SDEP_MAX_MESSAGE_SIZE],
        delay: &mut DELAY,
    ) -> Result<(), Error<SPI::Error>> {
        let data = msg.to_bytes(buffer);

        self.cs.set_low().unwrap();
        delay.delay_us(delays::CS_TO_SCK_US).await;
        let result = self.spi.transmit(data);
        self.cs.set_high().unwrap();

        let write_response = result.map_err(|source| Error::SpiBus { source })?;
        let (result, delay_us) = match sdep::MessageType::from_repr(write_response) {
            Some(sdep::MessageType::DeviceReadOverflow) => {
                if msg.more_data() {
                    (Ok(()), delays::BETWEEN_SDEP_WRITE_US)
                } else {
                    (Ok(()), delays::AFTER_SDEP_WRITE_US)
                }
            }
            Some(sdep::MessageType::DeviceNotReady) => (
                Err(Error::Sdep {
                    source: sdep::Error::DeviceNotReady,
                }),
                delays::WRITE_RETRY_DELAY_US,
            ),
            _ => (
                Err(Error::WriteResponseInvalid {
                    response: write_response,
                }),
                delays::AFTER_SDEP_WRITE_US,
            ),
        };

        delay.delay_us(delay_us).await;

        result
    }
}
