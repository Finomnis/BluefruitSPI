use crate::{delays, sdep, Error, SpiBus};

pub struct SdepDriver<SPI, CS> {
    spi: SPI,
    cs: CS,
    buffer: [u8; sdep::SDEP_MAX_MESSAGE_SIZE],
}

impl<SPI, CS> SdepDriver<SPI, CS>
where
    SPI: SpiBus,
    CS: embedded_hal::digital::OutputPin,
{
    pub fn new(spi: SPI, cs: CS) -> Self {
        Self {
            spi,
            cs,
            buffer: [0u8; sdep::SDEP_MAX_MESSAGE_SIZE],
        }
    }

    /// Write a raw SDEP message to the device
    async fn write<'a, DELAY: embedded_hal_async::delay::DelayNs>(
        &mut self,
        msg: sdep::Message<'a>,
        delay: &mut DELAY,
    ) -> Result<(), Error<SPI::Error>> {
        let data = msg.to_bytes(&mut self.buffer);

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

    /// Write a raw SDEP message to the device
    async fn read<DELAY: embedded_hal_async::delay::DelayNs>(
        &mut self,
        delay: &mut DELAY,
    ) -> Result<sdep::Message, Error<SPI::Error>> {
        let buffer = &mut self.buffer;

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
}
