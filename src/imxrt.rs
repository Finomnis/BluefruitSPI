//! SDEP SPI driver for i.MX RT chips
//!
//! Example:
//!
//! ```no_run
//! let ble_cs = gpio2.output(pins.p10);
//! let ble_irq = gpio3.input(pins.p39);
//! let ble_rst = gpio1.output(pins.p14);
//! let ble_spi = ImxrtSdepSpi::new(
//!     lpspi4,
//!     pins.p11,
//!     pins.p12,
//!     pins.p13,
//!     board::LPSPI_FREQUENCY,
//! );
//! let ble = BluefruitSPI::new(ble_spi, ble_cs, ble_rst, ble_irq, Mono);
//! ```

use imxrt_iomuxc as iomuxc;
use imxrt_ral as ral;

use crate::sdep::SDEP_MAX_MESSAGE_SIZE;

/// An SDEP SPI driver for i.MX RT.
#[cfg_attr(docsrs, doc(cfg(feature = "imxrt")))]
pub struct ImxrtSdepSpi<const N: u8> {
    lpspi: ral::lpspi::Instance<N>,
}

impl<const N: u8> ImxrtSdepSpi<N> {
    /// Create a new driver instance.
    pub fn new<SDO, SDI, SCK>(
        lpspi: ral::lpspi::Instance<N>,
        mut sdo: SDO,
        mut sdi: SDI,
        mut sck: SCK,
        source_clock_hz: u32,
    ) -> Self
    where
        SDO: iomuxc::lpspi::Pin<Module = iomuxc::consts::Const<N>, Signal = iomuxc::lpspi::Sdo>,
        SDI: iomuxc::lpspi::Pin<Module = iomuxc::consts::Const<N>, Signal = iomuxc::lpspi::Sdi>,
        SCK: iomuxc::lpspi::Pin<Module = iomuxc::consts::Const<N>, Signal = iomuxc::lpspi::Sck>,
    {
        // Reset and disable
        ral::modify_reg!(ral::lpspi, lpspi, CR, MEN: MEN_0, RST: RST_1);
        while ral::read_reg!(ral::lpspi, lpspi, CR, MEN == MEN_1) {}
        ral::modify_reg!(ral::lpspi, lpspi, CR, RST: RST_0, RTF: RTF_1, RRF: RRF_1);

        // Configure master mode
        ral::write_reg!(
            ral::lpspi,
            lpspi,
            CFGR1,
            MASTER: MASTER_1,
            SAMPLE: SAMPLE_1
        );

        // Set clock frequency
        //
        // Round up, so we always get a resulting SPI clock that is
        // equal or less than the requested frequency.
        let sckdiv = source_clock_hz.div_ceil(crate::RECOMMENDED_SPI_BITRATE_HZ);
        ral::write_reg!(ral::lpspi, lpspi, CCR, SCKDIV: sckdiv);

        // Configure pins
        iomuxc::lpspi::prepare(&mut sdo);
        iomuxc::lpspi::prepare(&mut sdi);
        iomuxc::lpspi::prepare(&mut sck);

        // Enable
        ral::write_reg!(ral::lpspi, lpspi, CR, MEN: MEN_1);

        Self { lpspi }
    }

    fn busy(&mut self) -> bool {
        ral::read_reg!(ral::lpspi, self.lpspi, SR, MBF == MBF_1)
    }
}

impl<const N: u8> crate::SpiBus for ImxrtSdepSpi<N> {
    type Error = crate::Infallible;

    fn transmit(&mut self, mut data: &[u8]) -> Result<u8, Self::Error> {
        while self.busy() {}

        // Reset FIFOS
        ral::modify_reg!(ral::lpspi, self.lpspi, CR, RRF: RRF_1, RTF: RTF_1);

        // log::debug!("Sdep data: {:02x?}", data);

        ral::write_reg!(ral::lpspi, self.lpspi, SR, TCF: TCF_1, FCF: FCF_1);

        let framesz = (data.len() * 8 - 1) as u32;
        ral::write_reg!(ral::lpspi, self.lpspi, TCR, FRAMESZ: framesz);

        while !data.is_empty() {
            let mut word = 0;
            for pos in 0..4 {
                if let Some(val) = data.get(pos) {
                    word = (word << 8) | u32::from(*val);
                }
            }

            ral::write_reg!(ral::lpspi, self.lpspi, TDR, word);

            data = data.get(4..).unwrap_or_default();
        }

        while ral::read_reg!(ral::lpspi, self.lpspi, SR, TCF != TCF_1) {}
        while ral::read_reg!(ral::lpspi, self.lpspi, SR, FCF != FCF_1) {}
        let response = ral::read_reg!(ral::lpspi, self.lpspi, RDR).to_be_bytes()[0];

        Ok(response)
    }

    fn receive(&mut self, buffer: &mut [u8; SDEP_MAX_MESSAGE_SIZE]) -> Result<(), Self::Error> {
        let mut buffer = buffer.as_mut_slice();

        while self.busy() {}

        // Reset FIFOS
        ral::modify_reg!(ral::lpspi, self.lpspi, CR, RRF: RRF_1, RTF: RTF_1);

        ral::write_reg!(ral::lpspi, self.lpspi, SR, FCF: FCF_1);

        let framesz = (buffer.len() * 8 - 1) as u32;
        ral::write_reg!(ral::lpspi, self.lpspi, TCR, TXMSK: TXMSK_1, FRAMESZ: framesz);

        while ral::read_reg!(ral::lpspi, self.lpspi, SR, FCF != FCF_1) {}

        while !buffer.is_empty() {
            let mut word = ral::read_reg!(ral::lpspi, self.lpspi, RDR);

            for pos in (0..4).rev() {
                if let Some(val) = buffer.get_mut(pos) {
                    *val = word as u8;
                    word = word >> 8;
                }
            }

            buffer = buffer.get_mut(4..).unwrap_or_default();
        }

        Ok(())
    }
}
