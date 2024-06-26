// This example demonstrates the usage of this crate on a Teensy MicroMod board.
//
// It implements a simple `echo` UART service.
// To connect to it, use the "Bluefruit Connect" Smartphone App.

#![no_std]
#![no_main]

use teensy4_bsp as bsp;

use bsp::hal;

use bsp::pins::common::{P0, P1};
imxrt_uart_panic::register!(LPUART6, P1, P0, 115200, teensy4_panic::sos);

#[rtic::app(device = teensy4_bsp, dispatchers = [CAN1, CAN2, CAN3])]
mod app {
    use super::bsp;

    use bsp::board;
    use bsp::hal;
    use bsp::pins;

    use embedded_io::Write;

    use hal::usbd::{BusAdapter, EndpointMemory, EndpointState, Speed};
    use usb_device::bus::UsbBusAllocator;

    use teensy4_selfrebootor::Rebootor;

    use bluefruitspi::BluefruitSPI;

    use rtic_monotonics::imxrt::prelude::*;
    imxrt_gpt1_monotonic!(Mono, board::PERCLK_FREQUENCY);

    /// This allocation is shared across all USB endpoints. It needs to be large
    /// enough to hold the maximum packet size for *all* endpoints. If you start
    /// noticing panics, check to make sure that this is large enough for all endpoints.
    static EP_MEMORY: EndpointMemory<1024> = EndpointMemory::new();
    /// This manages the endpoints. It's large enough to hold the maximum number
    /// of endpoints; we're not using all the endpoints in this example.
    static EP_STATE: EndpointState = EndpointState::max_endpoints();

    const LOG_POLL_INTERVAL: u32 = board::PERCLK_FREQUENCY / 100;
    const LOG_DMA_CHANNEL: usize = 0;

    type BlePeripheral = BluefruitSPI<
        bluefruitspi::imxrt::ImxrtSdepSpi<4>,
        hal::gpio::Output<pins::common::P10>,
        hal::gpio::Output<pins::common::P14>,
        hal::gpio::Input<pins::tmm::P39>,
        Mono,
    >;

    #[local]
    struct Local {
        poll_log: hal::pit::Pit<3>,
        log_poller: imxrt_log::Poller,
        rebootor: Rebootor<'static>,
        ble: BlePeripheral,
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
            mut gpt1,
            lpspi4,
            mut gpio1,
            mut gpio2,
            mut gpio3,
            ..
        } = board::tmm(cx.device);

        // Logging
        let log_dma = dma[LOG_DMA_CHANNEL].take().unwrap();
        let mut log_uart = board::lpuart(lpuart6, pins.p1, pins.p0, 115200);
        log_uart.disable(|uart| {
            uart.enable_fifo(hal::lpuart::Watermark::tx(4));
        });
        log_uart
            .write_all("\r\n===== Bluefruit LE SPI Friend echo example =====\r\n\r\n".as_bytes())
            .unwrap();
        log_uart.flush().unwrap();
        let log_poller =
            imxrt_log::log::lpuart(log_uart, log_dma, imxrt_log::Interrupts::Enabled).unwrap();
        poll_log.set_interrupt_enable(true);
        poll_log.set_load_timer_value(LOG_POLL_INTERVAL);
        poll_log.enable();

        // Monotonic
        gpt1.set_clock_source(hal::gpt::ClockSource::PeripheralClock);
        Mono::start(gpt1.release());

        // SPI
        let ble_cs = gpio2.output(pins.p10);
        let ble_irq = gpio3.input(pins.p39);
        let ble_rst = gpio1.output(pins.p14);
        let ble_spi = bluefruitspi::imxrt::ImxrtSdepSpi::new(
            lpspi4,
            pins.p11,
            pins.p12,
            pins.p13,
            board::LPSPI_FREQUENCY,
        );
        let ble = BluefruitSPI::new(ble_spi, ble_cs, ble_rst, ble_irq, Mono);

        // USB
        let bus = BusAdapter::with_speed(usb, &EP_MEMORY, &EP_STATE, Speed::LowFull);
        let bus = cx.local.bus.insert(UsbBusAllocator::new(bus));
        let rebootor = teensy4_selfrebootor::Rebootor::new(bus);

        // Spawn tasks
        ble::spawn().unwrap();

        (
            Shared {},
            Local {
                log_poller,
                poll_log,
                rebootor,
                ble,
            },
        )
    }

    async fn ble_controller(
        ble: &mut BlePeripheral,
    ) -> Result<bluefruitspi::Infallible, bluefruitspi::Error<bluefruitspi::Infallible>> {
        let mut uart_buffer = [0u8; bluefruitspi::UART_RX_MIN_BUFFER_SIZE];

        // Initialize the device and perform a factory reset
        ble.init().await?;
        ble.command(b"AT+FACTORYRESET").await?;
        Mono::delay(1.secs_at_least()).await;

        // Print the response to 'ATI' (info request)
        log::info!(
            "ATI Command:\r\n{}",
            core::str::from_utf8(ble.command(b"ATI").await?)
                .map_err(|_| bluefruitspi::Error::ResponseInvalid)?
        );

        // Change advertised name
        ble.command(b"AT+GAPDEVNAME=Rust meets BLE").await?;

        // Set LED to blink on BLE UART traffic
        ble.command(b"AT+HWMODELED=BLEUART").await?;

        loop {
            log::info!("Waiting for connection ...");
            while !ble.connected().await? {
                Mono::delay(5.millis_at_least()).await;
            }

            log::info!("Connected!");

            while ble.connected().await? {
                loop {
                    let received = ble.uart_rx(&mut uart_buffer).await?;
                    if received.is_empty() {
                        break;
                    }

                    if let Ok(s) = core::str::from_utf8(received) {
                        log::info!("Echoing {} bytes: {:?}", received.len(), s);
                    } else {
                        log::info!("Echoing {} bytes: {:?}", received.len(), received);
                    }

                    ble.uart_tx(received.iter().cloned()).await?;
                }

                Mono::delay(10.millis()).await;
            }

            log::info!("Disconnected!");
        }
    }

    #[task(priority = 3, local = [ble])]
    async fn ble(ctx: ble::Context) {
        let ble::LocalResources { ble, .. } = ctx.local;

        loop {
            let e = ble_controller(ble).await.unwrap_err();
            log::error!("BLE seems to have crashed: {}", e);
            log::info!("Restarting BLE ...");
            Mono::delay(100.millis()).await;
        }
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
