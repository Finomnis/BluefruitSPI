pub use _delays::*;

mod _delays {
    pub const RESET_PULSE_LENGTH_MS: u32 = 10;
    pub const AFTER_RESET_MS: u32 = 500;
    pub const AFTER_INIT_MS: u32 = 1000;
    pub const AFTER_SDEP_READ_US: u32 = 300;
    pub const AFTER_SDEP_WRITE_US: u32 = 300;
    pub const BETWEEN_SDEP_READ_US: u32 = 300;
    pub const BETWEEN_SDEP_WRITE_US: u32 = 300;
    pub const WRITE_RETRY_DELAY_US: u32 = 300;
    pub const WRITE_RETRY_COUNT: u32 = 5000;
    pub const READ_RETRY_DELAY_US: u32 = 300;
    pub const READ_RETRY_COUNT: u32 = 5000;
    pub const CS_TO_SCK_US: u32 = 100;
    pub const RESPONSE_TIMEOUT_MS: u32 = 200;
    pub const IRQ_POLL_PERIOD_MS: u32 = 1;
}
