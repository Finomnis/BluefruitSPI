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
    use bsp::logging;
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

    #[local]
    struct Local {
        poll_log: hal::pit::Pit<3>,
        log_poller: logging::Poller,
        rebootor: Rebootor<'static>,
        ble: BluefruitSPI<
            bluefruitspi::imxrt::ImxrtSdepSpi<4>,
            hal::gpio::Output<pins::common::P10>,
            hal::gpio::Output<pins::common::P14>,
            hal::gpio::Input<pins::tmm::P39>,
            Mono,
        >,
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
            .write_all("\r\n===== Bluefruit LE SPI Friend example =====\r\n\r\n".as_bytes())
            .unwrap();
        log_uart.flush().unwrap();
        let log_poller =
            logging::log::lpuart(log_uart, log_dma, logging::Interrupts::Enabled).unwrap();
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

    #[task(priority = 3, local = [ble])]
    async fn ble(ctx: ble::Context) {
        let ble::LocalResources { ble, .. } = ctx.local;

        // Initialize the device and perform a factory reset
        ble.init().await.unwrap();
        ble.command(b"AT+FACTORYRESET").await.unwrap();
        Mono::delay(1.secs_at_least()).await;

        // Print the response to 'ATI' (info request)
        log::info!(
            "ATI Command:\r\n{}",
            core::str::from_utf8(ble.command(b"ATI").await.unwrap()).unwrap()
        );

        // Change advertised name
        ble.command(b"AT+GAPDEVNAME=Rust meets BLE").await.unwrap();

        // Set LED to blink on BLE UART traffic
        ble.command(b"AT+HWMODELED=BLEUART").await.unwrap();

        loop {
            while !ble.connected().await.unwrap() {
                log::info!("Waiting for connection ...");
                Mono::delay(500.millis_at_least()).await;
            }

            log::info!("Connected!");

            let mut counter = 250;
            let mut t_last = Mono::now();
            let mut num_received = 0;
            while ble.connected().await.unwrap() {
                // if let Err(e) = ble
                //     .uart_tx(
                //         core::iter::once(counter)
                //             .chain((0u16..2048).map(|i| i as u8))
                //             .chain(core::iter::once(counter)),
                //     )
                //     .await
                // {
                //     log::error!("Uart TX failed: {}", e);
                // }
                //counter = counter.wrapping_add(1);
                loop {
                    let received = ble.uart_rx().await.unwrap();
                    num_received += received.len();
                    if received.is_empty() {
                        break;
                    }
                    log::info!(" <- {}", received.len());
                }
                let t_now = Mono::now();
                let t_diff = t_now - t_last;
                if t_diff.to_millis() > 1000 {
                    t_last = t_now;
                    let bytes_per_second =
                        num_received as f32 / (t_diff.to_millis() as f32 / 1000.0);
                    log::info!("Throughput RX: {} bytes/sec", bytes_per_second);
                    num_received = 0;
                }
            }
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
