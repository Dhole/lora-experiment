//! Uses the timer interrupt to blink a led with different frequencies.
//!
//! This assumes that a LED is connected to pc13 as is the case on the blue pill board.
//!
//! Note: Without additional hardware, PC13 should not be used to drive an LED, see page 5.1.2 of
//! the reference manaual for an explanation. This is not an issue on the blue pill.

#![no_std]
#![no_main]

#[macro_use]
extern crate urpc;

// extern crate cortex_m_semihosting;
// extern crate panic_semihosting;

mod e32_lora;
mod serial;

use as_slice::{AsMutSlice, AsSlice};

use cortex_m::asm;
// use cortex_m_rt::{entry, exception};
use panic_semihosting as _;

// use cortex_m_semihosting::hprintln;

use urpc::{
    consts,
    server::{self, Request as _},
    OptBufNo, OptBufYes,
};

// you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_halt as _;

// use cortex_m::asm::wfi;
use rtfm::app;
use rtfm::cyccnt::{Duration, Instant};

use ssd1306;

// use embedded_graphics::fonts::Font6x8;
// use embedded_graphics::prelude::*;
// use embedded_graphics::Drawing;
use embedded_graphics::{
    fonts::{Font6x8, Text},
    pixelcolor::BinaryColor,
    prelude::*,
    style::TextStyle,
};
use embedded_hal::digital::v2::{InputPin, OutputPin};
use ssd1306::{interface::I2cInterface, prelude::*};
use stm32f1xx_hal::{
    self,
    // pac,
    delay::Delay,
    device::I2C1,
    dma,
    gpio::{
        gpioa::{PA10, PA9},
        gpiob::{PB8, PB9},
        gpioc::PC13,
    },
    gpio::{Alternate, Floating, Input, OpenDrain, Output, PushPull, State},
    i2c::{BlockingI2c, DutyCycle, Mode},
    prelude::*,
    serial::{self as _serial, Config, Serial},
    stm32,
    timer::{self, CountDownTimer, Timer},
};

use stm32f1xx_hal::time::Hertz;

use arrayvec::{ArrayString, ArrayVec};
use core::convert::TryInto;
use core::fmt::Write;
use core::str;
use cortex_m::{peripheral::DWT, singleton};
use stm32f1::stm32f103::USART1;

// use crate::rpc_packet;

type OledDisplay = ssd1306::mode::graphics::GraphicsMode<
    I2cInterface<BlockingI2c<I2C1, (PB8<Alternate<OpenDrain>>, PB9<Alternate<OpenDrain>>)>>,
>;

type SerialPCPins = (PA9<Alternate<PushPull>>, PA10<Input<Floating>>);
type SerialPCTx = _serial::Tx<USART1, SerialPCPins>;
type SerialPCRx = _serial::Rx<USART1, SerialPCPins>;
type SerialPCRxDma = dma::RxDma<SerialPCRx, dma::dma1::C5>;
type SerialPCTxDma = dma::TxDma<SerialPCTx, dma::dma1::C4>;
// type SerialPCRxDmaTransfer =
//     dma::Transfer<dma::W, &'static mut [u8; 4], dma::RxDma<serial::Rx<USART1>, dma::dma1::C5>>;
type SerialPCRxDmaTransfer =
    dma::Transfer<dma::W, &'static mut ArrayVec<[u8; 32]>, dma::RxDma<SerialPCRx, dma::dma1::C5>>;
const SYSCLK: Hertz = Hertz(72_000_000);
const REFRESH_FREQ: Hertz = Hertz(6);

const RX_PC_BUF_LEN: usize = 32;
const TX_PC_BUF_LEN: usize = 32;

server_requests! {
    ServerRequest;
    (0, ping, Ping([u8; 4], OptBufNo, [u8; 4], OptBufNo)),
    (1, send_bytes, SendBytes((), OptBufYes, (), OptBufNo)),
    (2, add, Add((u8, u8), OptBufNo, u8, OptBufNo))
}

#[app(device = stm32, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        led: PC13<Output<PushPull>>,
        timer_handler: CountDownTimer<stm32::TIM1>,
        display: OledDisplay,
        tx_pc: Option<SerialPCTxDma>,
        rx_pc: Option<SerialPCRxDma>,
        #[init(None)]
        rx_pc_transfer: Option<SerialPCRxDmaTransfer>,
        // rx_pc: SerialPCRx,
        rx_pc_str: ArrayString<[u8; 32]>,
        #[init(0)]
        idle_dur: u32,
        #[init(false)]
        led_state: bool,
    }

    #[init]
    fn init(mut cx: init::Context) -> init::LateResources {
        // Initialize (enable) the monotonic timer (CYCCNT)
        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();
        let dp = stm32::Peripherals::take().unwrap();
        let cp = cortex_m::peripheral::Peripherals::take().unwrap();
        // let p = &mut c.core; //cortex_m::peripheral::Peripherals::take().unwrap();
        //                      // Take ownership over the raw flash and rcc devices and convert them into the corresponding
        //                      // HAL structs
        let mut flash = dp.FLASH.constrain();
        let mut rcc = dp.RCC.constrain();
        let mut afio = dp.AFIO.constrain(&mut rcc.apb2);

        // Freeze the configuration of all the clocks in the system and store the frozen frequencies
        // in `clocks`
        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(SYSCLK)
            .pclk1(36.mhz())
            .freeze(&mut flash.acr);

        assert!(clocks.usbclk_valid());

        // Acquire the GPIOC peripheral
        let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
        let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

        // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the
        // function in order to configure the port. For pins 0-7, crl should be passed instead
        let led = gpioc
            .pc13
            .into_push_pull_output_with_state(&mut gpioc.crh, State::High);

        let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
        let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);

        let i2c = BlockingI2c::i2c1(
            dp.I2C1,
            (scl, sda),
            &mut afio.mapr,
            Mode::Fast {
                frequency: 800_000.hz(),
                duty_cycle: DutyCycle::Ratio2to1,
            },
            clocks,
            &mut rcc.apb1,
            1000,
            10,
            1000,
            1000,
        );

        let mut display: GraphicsMode<_> = ssd1306::Builder::new().connect_i2c(i2c).into();
        display.init().unwrap();
        display.flush().unwrap();

        let mut dma1_channels = dp.DMA1.split(&mut rcc.ahb);

        // USART1
        let tx_pc = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let rx_pc = gpioa.pa10;
        let mut serial_pc = Serial::usart1(
            dp.USART1,
            (tx_pc, rx_pc),
            &mut afio.mapr,
            Config::default().baudrate(9600.bps()),
            clocks,
            &mut rcc.apb2,
        );
        let (mut tx_pc, mut rx_pc) = serial_pc.split();
        // rx_pc.listen();
        dma1_channels.5.listen(dma::Event::TransferComplete);
        let mut rx_pc = rx_pc.with_dma(dma1_channels.5);
        // dma1_channels.4.listen(dma::Event::TransferComplete);
        let mut tx_pc = tx_pc.with_dma(dma1_channels.4);

        let aux = gpioa.pa7;

        let mut m0 = gpiob.pb0.into_push_pull_output(&mut gpiob.crl);
        let mut m1 = gpiob.pb1.into_push_pull_output(&mut gpiob.crl);

        let mut delay = Delay::new(cp.SYST, clocks);

        // USART2
        let tx_lora = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
        let rx_lora = gpioa.pa3;
        let mut serial_lora = Serial::usart2(
            dp.USART2,
            (tx_lora, rx_lora),
            &mut afio.mapr,
            Config::default().baudrate(9600.bps()),
            clocks,
            &mut rcc.apb1,
        );

        // BEGIN Full circle
        let (mut tx_lora, mut rx_lora) = serial_lora.split();

        let mut tx_dma_lora = tx_lora.with_dma(dma1_channels.7);
        let mut rx_dma_lora = rx_lora.with_dma(dma1_channels.6);

        m0.set_low();
        m1.set_low();
        delay.delay_ms(2u16);
        delay.delay_ms(10u16);
        while aux.is_low().ok().unwrap() {}
        delay.delay_ms(2u16);

        // while aux.is_low().ok().unwrap() {}
        let (_, tx_dma_lora) = tx_dma_lora.write(b"XXX").wait();

        while aux.is_low().ok().unwrap() {}
        delay.delay_ms(2u16);

        m0.set_high();
        m1.set_high();
        delay.delay_ms(2u16);
        delay.delay_ms(10u16);
        while aux.is_low().ok().unwrap() {}
        delay.delay_ms(2u16);

        let (_, tx_dma_lora) = tx_dma_lora.write(b"\xc1\xc1\xc1").wait();
        // let (_, tx_dma_lora) = tx_dma_lora.write(b"\xc3\xc3\xc3").wait();
        // let (_, tx_dma_lora) = tx_dma_lora.write(b"XXX").wait();

        let buf = singleton!(: ArrayVec<[u8; 32]> = ArrayVec::<[u8; 32]>::new()).unwrap();
        unsafe {
            buf.set_len(6);
        }
        // let buf = singleton!(: [u8; 6] = [0; 6]).unwrap();

        let mut rx_pc_str = ArrayString::<[u8; 32]>::new();
        // write!(rx_pc_str, "{}", buf.as_slice().len());
        // BEGIN READ DMA
        let (buf, rx_dma_lora) = rx_dma_lora.read(buf).wait();

        for b in buf.iter() {
            write!(rx_pc_str, "{:02x}", b).unwrap();
        }
        // let b = nb::block!(rx_lora.read()).unwrap();
        // write!(rx_pc_str, "{:02x}", b).unwrap();
        // END READ DMA

        let (tx_lora, tx_dma_chan) = tx_dma_lora.split();
        let (rx_lora, rx_dma_chan) = rx_dma_lora.split();
        // Again
        // let mut tx_lora = tx_lora.with_dma(dma_chan);
        // let (_, tx_lora) = tx_lora.write(b"\nGOODBYE\n").wait();
        // let (tx_lora, dma_chan) = tx_lora.split();

        let mut serial_lora = Serial::join(tx_lora, rx_lora);
        // END Full circle

        // TODO: DMA
        // let (mut tx_lora, mut rx_lora) = serial_lora.split();
        // // rx_pc.listen();
        // // dma2_channels.5.listen(dma::Event::TransferComplete);
        // let mut rx_lora = rx_lora.with_dma(dma1_channels.6);
        // // dma1_channels.4.listen(dma::Event::TransferComplete);
        // let mut tx_lora = tx_lora.with_dma(dma1_channels.7);

        // nb::block!(serial_lora.write('A' as u8));
        // nb::block!(serial_lora.write('-' as u8));
        // nb::block!(serial_lora.flush());
        let mut serial_lora = serial::Serial::new(serial_lora);
        // nb::block!(serial_lora.write('B' as u8));
        // nb::block!(serial_lora.flush());

        let mut ctx = serial::Context::new(afio.mapr, rcc.apb1);

        let lora = e32_lora::LoRa::new(
            e32_lora::ModePins::new(m0, m1),
            aux,
            serial_lora,
            clocks,
            &mut ctx,
        );

        // Configure the syst timer to trigger an update every second and enables interrupt
        let mut timer_handler =
            Timer::tim1(dp.TIM1, &clocks, &mut rcc.apb2).start_count_down(REFRESH_FREQ);
        timer_handler.listen(timer::Event::Update);

        rtfm::pend(stm32::Interrupt::DMA1_CHANNEL5);

        // hprintln!("+ Init");
        // Init the static resources to use them later through RTFM
        init::LateResources {
            led,
            timer_handler,
            display,
            tx_pc: Some(tx_pc),
            rx_pc: Some(rx_pc),
            rx_pc_str: rx_pc_str,
        }
    }

    #[idle(resources = [idle_dur])]
    fn idle(mut cx: idle::Context) -> ! {
        let mut resumed = DWT::get_cycle_count();
        loop {
            cx.resources.idle_dur.lock(|idle_dur| {
                if *idle_dur == 0 {
                    resumed = DWT::get_cycle_count();
                }
                *idle_dur = {
                    let (idle_dur, _) = DWT::get_cycle_count().overflowing_sub(resumed);
                    idle_dur
                };
            });
        }
    }

    #[task(binds = TIM1_UP, resources = [display, timer_handler, idle_dur, rx_pc_str], priority = 1)]
    fn tim1_up(mut cx: tim1_up::Context) {
        static mut count: u8 = 0;
        static mut idle: f32 = 0.0;

        // hprintln!("a0");
        *idle = *idle * 0.8
            + 0.2 * (*cx.resources.idle_dur as f32 / (SYSCLK.0 / REFRESH_FREQ.0) as f32);
        *cx.resources.idle_dur = 0;

        let mut line1 = ArrayString::<[u8; 32]>::new();
        write!(&mut line1, "{:.3}% {}", 1.0 - *idle, count).unwrap();
        let mut line2 = ArrayString::<[u8; 32]>::new();
        cx.resources.rx_pc_str.lock(|rx_pc_str| {
            write!(&mut line2, "{}", rx_pc_str).unwrap();
        });
        cx.resources.display.clear();
        // cx.resources.display.draw(
        //     Font6x8::render_str(&line1).into_iter().chain(
        //         Font6x8::render_str(&line2)
        //             .translate(Point::new(0, 16))
        //             .into_iter(),
        //     ),
        // );
        match cx.resources.display.draw_iter(
            Text::new(&line1, Point::new(0, 0))
                .into_styled(TextStyle::new(Font6x8, BinaryColor::On))
                .into_iter()
                .chain(
                    Text::new(&line2, Point::new(0, 16))
                        .into_styled(TextStyle::new(Font6x8, BinaryColor::On))
                        .into_iter(),
                ),
        ) {
            Ok(_) => {}
            Err(_) => {}
        }
        // .draw(cx.resources.display)
        // .unwrap();

        match cx.resources.display.flush() {
            Ok(()) => {}
            Err(e) => {
                // hprintln!("{:?}", e);
            }
        }

        *count = {
            let (n, _) = count.overflowing_add(1);
            n
        };
        // Clears the update flag
        cx.resources.timer_handler.clear_update_interrupt_flag();
        // hprintln!("a1");
    }

    // #[task(binds = DMA1_CHANNEL4, priority = 3)]
    // fn tx_pc_dma(cx: tx_pc_dma::Context) {}

    #[task(binds = DMA1_CHANNEL5, resources = [rx_pc, rx_pc_str, rx_pc_transfer, tx_pc], priority = 2)]
    fn rx_pc_dma(cx: rx_pc_dma::Context) {
        static mut s_init: bool = false;
        static mut s_rx_pc: Option<SerialPCRxDma> = None;
        static mut s_rx_pc_transfer: Option<SerialPCRxDmaTransfer> = None;
        static mut s_rpc_server: Option<server::RpcServer> = None;
        static mut s_tx_buf: Option<&'static mut ArrayVec<[u8; 32]>> = None;

        static mut s_dbg: u8 = 0;

        // hprintln!("b0");
        if !*s_init {
            *s_rpc_server = Some(server::RpcServer::new(RX_PC_BUF_LEN as u16));
            let rx_pc = cx.resources.rx_pc.take().unwrap();

            let rx_buf = singleton!(: ArrayVec<[u8; RX_PC_BUF_LEN]> =
                ArrayVec::<[u8; RX_PC_BUF_LEN]>::new())
            .unwrap();
            *s_tx_buf = Some(
                singleton!(: ArrayVec<[u8; TX_PC_BUF_LEN]> =
                        ArrayVec::<[u8; TX_PC_BUF_LEN]>::new())
                .unwrap(),
            );

            unsafe {
                rx_buf.set_len(consts::REQ_HEADER_LEN);
            }
            *s_rx_pc_transfer = Some(rx_pc.read(rx_buf));
            *s_init = true;
            // hprintln!("rx_pc_dma init").unwrap();
            return;
        }

        let mut rpc_server = s_rpc_server.take().unwrap();
        let rx_pc_transfer = s_rx_pc_transfer.take().unwrap();
        let (mut rx_buf, rx_pc) = rx_pc_transfer.wait();

        // DEBUG
        // let parse_res = match rpc_server.parse(&rx_buf) {
        let parse_res = match ServerRequest::from_rpc(&mut rpc_server, &rx_buf) {
            Ok(r) => r,
            Err(e) => {
                cx.resources.rx_pc_str.clear();
                write!(cx.resources.rx_pc_str, "XXX").unwrap();
                return;
            }
        };
        let read_len = match parse_res {
            server::ParseResult::NeedBytes(n) => n,
            server::ParseResult::Request(req) => {
                let mut tx_buf = s_tx_buf.take().unwrap();
                unsafe {
                    tx_buf.set_len(32);
                }
                let write_len = match req {
                    ServerRequest::Ping(ping) => {
                        let ping_body = ping.body;
                        cx.resources.rx_pc_str.clear();
                        write!(
                            cx.resources.rx_pc_str,
                            "PING: {}",
                            str::from_utf8(&ping_body).unwrap()
                        )
                        .unwrap();
                        ping.reply(ping_body, &mut tx_buf).unwrap()
                    }
                    ServerRequest::SendBytes((send_bytes, buf)) => {
                        cx.resources.rx_pc_str.clear();
                        write!(
                            cx.resources.rx_pc_str,
                            "BYTES: {}",
                            str::from_utf8(buf).unwrap()
                        )
                        .unwrap();
                        send_bytes.reply((), &mut tx_buf).unwrap()
                    }
                    ServerRequest::Add(add) => {
                        let (x, y) = add.body;
                        cx.resources.rx_pc_str.clear();
                        write!(cx.resources.rx_pc_str, "{} + {}", x, y).unwrap();
                        add.reply(x + y, &mut tx_buf).unwrap()
                    }
                };
                unsafe {
                    tx_buf.set_len(write_len);
                }
                let mut tx_buf = if write_len != 0 {
                    let tx_pc = cx.resources.tx_pc.take().unwrap();
                    let tx_pc_transfer = tx_pc.write(tx_buf);
                    let (mut tx_buf, tx_pc) = tx_pc_transfer.wait();
                    *cx.resources.tx_pc = Some(tx_pc);
                    tx_buf
                } else {
                    tx_buf
                };
                *s_tx_buf = Some(tx_buf);
                consts::REQ_HEADER_LEN
            }
        };

        unsafe {
            rx_buf.set_len(read_len);
        }
        *s_rx_pc_transfer = Some(rx_pc.read(rx_buf));
        *s_rpc_server = Some(rpc_server);
        // hprintln!("b1");
    }

    // extern "C" {
    //     fn EXTI0();
    // }
};

// #[exception]
// fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
//     // prints the exception frame using semihosting
//     let _ = hprintln!("{:#?}", ef);
//     asm::bkpt();
//     loop {
//         asm::bkpt();
//     }
// }
