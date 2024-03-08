pub use _delays::*;

mod _delays {
    pub const RESET_PULSE_LENGTH_MS: u32 = 10;
    pub const AFTER_RESET_MS: u32 = 500;
    pub const AFTER_INIT_MS: u32 = 1000;
    pub const AFTER_SDEP_READ_US: u32 = 300;
    pub const AFTER_SDEP_WRITE_US: u32 = 300;
    pub const CS_TO_SCK_US: u32 = 100;
}
