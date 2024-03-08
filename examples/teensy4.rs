// This example demonstrates a program that also acts as a rebootor.
// When being programmed with the `-r` flag, it will reboot itself into
// the bootloader.

#![no_std]
#![no_main]

use teensy4_bsp as bsp;

use bsp::hal;

mod spi {
    use snafu::prelude::*;

    use super::bsp::hal::iomuxc;
    use super::bsp::ral;

    pub struct ImxrtSdepSpi<const N: u8> {
        lpspi: ral::lpspi::Instance<N>,
    }

    impl<const N: u8> ImxrtSdepSpi<N> {
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
            let sckdiv = source_clock_hz.div_ceil(bluefruitspi::RECOMMENDED_SPI_BITRATE_HZ);
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

    /// An impossible error; this error can never happen.
    #[derive(Debug, Snafu)]
    pub enum Infallible {}

    impl<const N: u8> bluefruitspi::SpiBus for ImxrtSdepSpi<N> {
        type Error = Infallible;

        fn transmit(&mut self, mut data: &[u8]) -> Result<(), Self::Error> {
            while self.busy() {}

            log::debug!("Sdep data: {:02x?}", data);

            let framesz = (data.len() * 8 - 1) as u32;
            ral::write_reg!(ral::lpspi, self.lpspi, TCR, RXMSK: RXMSK_1, FRAMESZ: framesz);

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

            while self.busy() {}

            Ok(())
        }

        fn receive(&mut self, buffer: &mut [u8; 20]) -> Result<(), Self::Error> {
            let mut buffer = buffer.as_mut_slice();

            while self.busy() {}

            let framesz = (buffer.len() * 8 - 1) as u32;
            ral::write_reg!(ral::lpspi, self.lpspi, TCR, TXMSK: TXMSK_1, FRAMESZ: framesz);

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
}

use bsp::pins::common::{P0, P1};
imxrt_uart_panic::register!(LPUART6, P1, P0, 115200, teensy4_panic::sos);

#[rtic::app(device = teensy4_bsp)]
mod app {
    use super::bsp;

    use bsp::board;
    use bsp::hal;
    use bsp::logging;

    use embedded_io::Write;

    use hal::usbd::{BusAdapter, EndpointMemory, EndpointState, Speed};
    use usb_device::bus::UsbBusAllocator;

    use teensy4_selfrebootor::Rebootor;

    /// This allocation is shared across all USB endpoints. It needs to be large
    /// enough to hold the maximum packet size for *all* endpoints. If you start
    /// noticing panics, check to make sure that this is large enough for all endpoints.
    static EP_MEMORY: EndpointMemory<1024> = EndpointMemory::new();
    /// This manages the endpoints. It's large enough to hold the maximum number
    /// of endpoints; we're not using all the endpoints in this example.
    static EP_STATE: EndpointState = EndpointState::max_endpoints();

    const LOG_POLL_INTERVAL: u32 = board::PERCLK_FREQUENCY / 100;
    const LOG_DMA_CHANNEL: usize = 0;

    #[local]
    struct Local {
        poll_log: hal::pit::Pit<3>,
        log_poller: logging::Poller,
        rebootor: Rebootor<'static>,
    }

    #[shared]
    struct Shared {}

    #[init(local = [bus: Option<UsbBusAllocator<BusAdapter>> = None])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let board::Resources {
            mut dma,
            pit: (_, _, _, mut poll_log),
            pins,
            usb,
            lpuart6,
            ..
        } = board::t40(cx.device);

        // Logging
        let log_dma = dma[LOG_DMA_CHANNEL].take().unwrap();
        let mut log_uart = board::lpuart(lpuart6, pins.p1, pins.p0, 115200);
        log_uart.disable(|uart| {
            uart.enable_fifo(hal::lpuart::Watermark::tx(4));
        });
        log_uart
            .write_all("\r\n===== Bluefruit LE SPI Friend example =====\r\n\r\n".as_bytes())
            .unwrap();
        log_uart.flush().unwrap();
        let log_poller =
            logging::log::lpuart(log_uart, log_dma, logging::Interrupts::Enabled).unwrap();
        poll_log.set_interrupt_enable(true);
        poll_log.set_load_timer_value(LOG_POLL_INTERVAL);
        poll_log.enable();

        // USB
        let bus = BusAdapter::with_speed(usb, &EP_MEMORY, &EP_STATE, Speed::LowFull);
        let bus = cx.local.bus.insert(UsbBusAllocator::new(bus));
        let rebootor = teensy4_selfrebootor::Rebootor::new(bus);

        (
            Shared {},
            Local {
                log_poller,
                poll_log,
                rebootor,
            },
        )
    }

    #[task(binds = USB_OTG1, priority = 5, local = [rebootor])]
    fn usb1(ctx: usb1::Context) {
        ctx.local.rebootor.poll();
    }

    #[task(binds = PIT, priority = 1, local = [poll_log, log_poller])]
    fn logger(cx: logger::Context) {
        let logger::LocalResources {
            poll_log,
            log_poller,
            ..
        } = cx.local;

        if poll_log.is_elapsed() {
            poll_log.clear_elapsed();

            log_poller.poll();
        }
    }
}
