use crate::{delays, sdep, Error, SpiBus};

struct SdepRawDriver<SPI, CS> {
    spi: SPI,
    cs: CS,
}

pub struct SdepDriver<IRQ, SPI, CS> {
    raw: SdepRawDriver<SPI, CS>,
    irq: IRQ,
    buffer: [u8; sdep::SDEP_MAX_MESSAGE_SIZE],
}

impl<IRQ, SPI, CS> SdepDriver<IRQ, SPI, CS>
where
    IRQ: embedded_hal::digital::InputPin, //TODO: use interrupts + embedded_hal_async::digital::Wait,
    SPI: SpiBus,
    CS: embedded_hal::digital::OutputPin,
{
    pub fn new(irq: IRQ, spi: SPI, cs: CS) -> Self {
        Self {
            raw: SdepRawDriver { spi, cs },
            irq,
            buffer: [0u8; sdep::SDEP_MAX_MESSAGE_SIZE],
        }
    }

    pub async fn execute_command<'a, DELAY: embedded_hal_async::delay::DelayNs>(
        &mut self,
        command_type: sdep::CommandType,
        data_iter: &mut dyn Iterator<Item = u8>,
        result_buffer: &'a mut [u8],
        delay: &mut DELAY,
    ) -> Result<&'a [u8], Error<SPI::Error>> {
        // Send Command
        let mut payload = [0u8; sdep::SDEP_MAX_PAYLOAD_SIZE];
        let mut data = data_iter.peekable();
        let mut send_finished = false;
        while !send_finished {
            let mut payload_size = 0;
            for el in payload.iter_mut() {
                if let Some(d) = data.next() {
                    *el = d;
                    payload_size += 1;
                } else {
                    send_finished = true;
                    break;
                }
            }

            send_finished = send_finished || data.peek().is_none();

            self.write(
                sdep::Message::Command {
                    id: command_type.into(),
                    payload: &payload[..payload_size],
                    more_data: !send_finished,
                },
                delay,
            )
            .await?;
        }

        // Wait for response
        let mut timeout_left = delays::RESPONSE_TIMEOUT_MS;
        while !self.irq.is_high().unwrap() {
            if timeout_left == 0 {
                return Err(Error::Timeout);
            }

            delay.delay_ms(delays::IRQ_POLL_PERIOD_MS).await;
            timeout_left = timeout_left.saturating_sub(delays::IRQ_POLL_PERIOD_MS);
        }

        // Read response
        let data = result_buffer;
        let mut data_size = 0;

        loop {
            if !self.irq.is_high().unwrap() {
                return Err(Error::ResponsePayloadIncomplete);
            }

            match self.read(delay).await? {
                sdep::Message::Command { .. } => return Err(Error::ResponseInvalid),
                sdep::Message::Response {
                    id,
                    payload,
                    more_data,
                } => {
                    if id != command_type.into() {
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
            }
        }

        Ok(&data[..data_size])
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
    async fn read<DELAY: embedded_hal_async::delay::DelayNs>(
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
