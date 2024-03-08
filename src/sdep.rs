//! Implementation of the SDEP protocol

use snafu::prelude::*;
use strum::FromRepr;

/// The `id` field of a [`Message::Command`] or [`Message::Response`].
///
/// Note that this list is not exhaustive,
/// future protocol changes could add new IDs.
#[derive(FromRepr, Debug, Eq, PartialEq)]
#[repr(u16)]
pub enum CommandType {
    /// Initializes/resets the device.
    Initialize = 0xbeef,
    /// An AT command.
    AtWrapper = 0x0a00,
    /// A Ble Uart TX command.
    ///
    /// This one is not used in the CircuitPython library,
    /// so I'm not sure if it even works.
    BleUartTx = 0x0a01,
    /// A Ble Uart RX command.
    ///
    /// This one is not used in the CircuitPython library,
    /// so I'm not sure if it even works.
    BleUartRx = 0x0a02,
}

impl From<CommandType> for u16 {
    fn from(value: CommandType) -> Self {
        value as u16
    }
}

/// The `id` field of a [`Message::Alert`].
///
/// Note that this list is not exhaustive,
/// future protocol changes could add new IDs.
#[derive(FromRepr, Debug, Eq, PartialEq)]
#[repr(u16)]
pub enum AlertType {
    /// Reserved for future use
    Reserved = 0x0000,
    /// The system is about to reset
    SystemReset = 0x0001,
    /// The battery level is low
    BatteryLow = 0x0002,
    /// The battery level is critically low
    BatteryCritical = 0x0003,
}

impl From<AlertType> for u16 {
    fn from(value: AlertType) -> Self {
        value as u16
    }
}

/// The `id` field of a [`Message::Error`].
///
/// Note that this list is not exhaustive,
/// future protocol changes could add new IDs.
#[derive(FromRepr, Debug, Eq, PartialEq)]
#[repr(u16)]
pub enum ErrorType {
    /// Reserved for future use
    Reserved = 0x0000,
    /// CMD ID wasn't found in the lookup table
    InvalidCmdId = 0x0001,
    /// The message payload was invalid
    InvalidPayload = 0x0003,
}

impl From<ErrorType> for u16 {
    fn from(value: ErrorType) -> Self {
        value as u16
    }
}

/// Message Type Indicator
#[derive(FromRepr, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum MessageType {
    /// Command message
    Command = 0x10,
    /// Response message
    Response = 0x20,
    /// Alert message
    Alert = 0x40,
    /// Error message
    Error = 0x80,
    /// Secondary device not ready (wait a bit and try again)
    DeviceNotReady = 0xFE,
    /// Secondary device read overflow indicator (you've read more data than is available)
    DeviceReadOverflow = 0xFF,
}

impl From<MessageType> for u8 {
    fn from(value: MessageType) -> Self {
        value as u8
    }
}

/// An SDEP Message.
#[derive(Debug, Eq, PartialEq)]
pub enum Message<'a> {
    /// Message of type 'Command'.
    Command {
        /// The command ID. See [`CommandType`].
        id: u16,
        /// The command payload.
        payload: &'a [u8],
        /// Whether the next package still belongs to the same message.
        more_data: bool,
    },
    /// Message of type 'Response'.
    Response {
        /// The response ID. See [`CommandType`].
        id: u16,
        /// The response payload.
        payload: &'a [u8],
        /// Whether the next package still belongs to the same message.
        more_data: bool,
    },
    /// Message of type 'Alert'.
    Alert {
        /// The alert ID. See [`AlertType`].
        id: u16,
        /// The alert payload.
        payload: &'a [u8],
    },
    /// Message of type 'Error'.
    Error {
        /// The error ID. See [`ErrorType`].
        id: u16,
    },
}

impl<'a> Message<'a> {
    /// Serializes a message into a byte buffer.
    ///
    /// # Arguments
    ///
    /// * `buffer` - A buffer that provides storage for the message.
    ///
    /// # Returns
    ///
    /// A slice to the part of the buffer that contains the serialized message.
    pub fn to_bytes<'b>(&self, buffer: &'b mut [u8; 20]) -> &'b [u8] {
        fn fill_data<'c>(
            buffer: &'c mut [u8; 20],
            message_type: MessageType,
            id: u16,
            payload: &[u8],
            more_data: bool,
        ) -> &'c [u8] {
            let payload_size = payload.len();
            assert!(payload_size <= 16);

            let id_bytes: [u8; 2] = id.to_le_bytes();

            buffer[0] = message_type.into();
            buffer[1] = id_bytes[0];
            buffer[2] = id_bytes[1];
            buffer[3] = payload_size as u8;
            if more_data {
                buffer[3] |= 0b1000_0000;
            }

            buffer[4..(4 + payload_size)].clone_from_slice(payload);

            &buffer[..(4 + payload_size)]
        }

        match self {
            Message::Command {
                id,
                more_data,
                payload,
            } => fill_data(buffer, MessageType::Command, *id, payload, *more_data),
            Message::Response {
                id,
                more_data,
                payload,
            } => fill_data(buffer, MessageType::Response, *id, payload, *more_data),
            Message::Alert { id, payload } => {
                fill_data(buffer, MessageType::Alert, *id, payload, false)
            }
            Message::Error { id } => fill_data(buffer, MessageType::Error, *id, &[], false),
        }
    }

    /// Deserializes a message from a byte buffer.
    pub fn from_bytes(data: &'a [u8; 20]) -> Result<Self, Error> {
        let header_type = data[0];
        let header_id = u16::from_le_bytes([data[1], data[2]]);
        let header_payload_info = data[3];

        fn read_payload<'b>(data: &'b [u8]) -> Result<&'b [u8], Error> {
            let header_payload_info = data[3];
            let payload_len = header_payload_info & 0b0001_1111;
            data.get(4..usize::from(4 + payload_len))
                .ok_or(Error::PayloadTooLong)
        }

        if let Some(message_type) = MessageType::from_repr(header_type) {
            match message_type {
                MessageType::Command => {
                    let id = header_id;
                    let more_data = (header_payload_info & 0b1000_0000) != 0;
                    let payload = read_payload(data)?;
                    Ok(Message::Command {
                        id,
                        more_data,
                        payload,
                    })
                }
                MessageType::Response => {
                    let id = header_id;
                    let more_data = (header_payload_info & 0b1000_0000) != 0;
                    let payload = read_payload(data)?;
                    Ok(Message::Response {
                        id,
                        more_data,
                        payload,
                    })
                }
                MessageType::Alert => {
                    let id = header_id;
                    let payload = read_payload(data)?;
                    Ok(Message::Alert { id, payload })
                }
                MessageType::Error => {
                    let id = header_id;
                    Ok(Message::Error { id })
                }
                MessageType::DeviceNotReady => Err(Error::DeviceNotReady),
                MessageType::DeviceReadOverflow => Err(Error::DeviceReadOverflow),
            }
        } else {
            Err(Error::UnknownMessageType)
        }
    }
}

/// All the errors that can happen in the SDEP protocol.
#[derive(Debug, Snafu)]
pub enum Error {
    /// The message contains an unknown type.
    UnknownMessageType,
    /// The device signalled a read overflow.
    DeviceReadOverflow,
    /// The device signalled that it was not ready yet.
    DeviceNotReady,
    /// Payload length is longer than allowed in the SDEP protocol.
    PayloadTooLong,
}
