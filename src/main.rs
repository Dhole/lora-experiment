//! Uses the timer interrupt to blink a led with different frequencies.
//!
//! This assumes that a LED is connected to pc13 as is the case on the blue pill board.
//!
//! Note: Without additional hardware, PC13 should not be used to drive an LED, see page 5.1.2 of
//! the reference manaual for an explanation. This is not an issue on the blue pill.

#![no_std]
#![no_main]

// you can put a breakpoint on `rust_begin_unwind` to catch panics
use panic_halt as _;

// use cortex_m::asm::wfi;
use rtfm::app;
use rtfm::cyccnt::{Duration, Instant};

use ssd1306;

use embedded_graphics::fonts::Font6x8;
use embedded_graphics::prelude::*;
use embedded_graphics::Drawing;
use embedded_hal::digital::v2::OutputPin;
use ssd1306::{interface::I2cInterface, prelude::*};
use stm32f1xx_hal::{
    self,
    device::I2C1,
    dma,
    gpio::{
        gpioa::{PA10, PA9},
        gpiob::{PB8, PB9},
        gpioc::PC13,
    },
    gpio::{Alternate, Floating, Input, OpenDrain, Output, PushPull, State},
    i2c::{BlockingI2c, DutyCycle, Mode},
    // pac,
    prelude::*,
    serial::{self, Config, Serial},
    stm32,
    timer::{self, CountDownTimer, Timer},
};

use stm32f1xx_hal::time::Hertz;

use arrayvec::{ArrayString, ArrayVec};
use core::convert::TryInto;
use core::fmt::Write;
use cortex_m::singleton;
use stm32f1::stm32f103::USART1;

type OledDisplay = ssd1306::mode::graphics::GraphicsMode<
    I2cInterface<BlockingI2c<I2C1, (PB8<Alternate<OpenDrain>>, PB9<Alternate<OpenDrain>>)>>,
>;

type SerialPCTx = serial::Tx<USART1>;
type SerialPCRx = serial::Rx<USART1>;
type SerialPCRxDma = dma::RxDma<SerialPCRx, dma::dma1::C5>;
// type SerialPCRxDmaTransfer =
//     dma::Transfer<dma::W, &'static mut [u8; 4], dma::RxDma<serial::Rx<USART1>, dma::dma1::C5>>;
type SerialPCRxDmaTransfer = dma::Transfer<
    dma::W,
    &'static mut ArrayVec<[u8; 32]>,
    dma::RxDma<serial::Rx<USART1>, dma::dma1::C5>,
>;
const SYSCLK: Hertz = Hertz(72_000_000);
const REFRESH_FREQ: Hertz = Hertz(8);

#[app(device = stm32, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        led: PC13<Output<PushPull>>,
        timer_handler: CountDownTimer<stm32::TIM1>,
        display: OledDisplay,
        tx_pc: SerialPCTx,
        rx_pc: Option<SerialPCRxDma>,
        #[init(None)]
        transfer: Option<SerialPCRxDmaTransfer>,
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
                frequency: 400_000,
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

        let channels = dp.DMA1.split(&mut rcc.ahb);

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
        let mut rx_pc = rx_pc.with_dma(channels.5);

        // Configure the syst timer to trigger an update every second and enables interrupt
        let mut timer_handler =
            Timer::tim1(dp.TIM1, &clocks, &mut rcc.apb2).start_count_down(REFRESH_FREQ);
        timer_handler.listen(timer::Event::Update);

        rtfm::pend(stm32::Interrupt::USART1);

        // Init the static resources to use them later through RTFM
        init::LateResources {
            led,
            timer_handler,
            display,
            tx_pc,
            rx_pc: Some(rx_pc),
            rx_pc_str: ArrayString::<[u8; 32]>::new(),
        }
    }

    #[idle(resources = [idle_dur])]
    fn idle(mut cx: idle::Context) -> ! {
        let mut resumed = Instant::now();
        loop {
            cx.resources.idle_dur.lock(|idle_dur| {
                if *idle_dur == 0 {
                    resumed = Instant::now();
                }
                *idle_dur = (Instant::now() - resumed).as_cycles();
            });
        }
    }

    #[task(binds = TIM1_UP, resources = [display, timer_handler, idle_dur, rx_pc_str], priority = 1)]
    fn tim1_up(mut cx: tim1_up::Context) {
        // Depending on the application, you could want to delegate some of the work done here to
        // the idle task if you want to minimize the latency of interrupts with same priority (if
        // you have any). That could be done with some kind of machine state, etc.

        // Count used to change the timer update frequency
        static mut count: u8 = 0;
        static mut idle: f32 = 0.0;

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
        cx.resources.display.draw(
            Font6x8::render_str(&line1).into_iter().chain(
                Font6x8::render_str(&line2)
                    .translate(Point::new(0, 16))
                    .into_iter(),
            ),
        );
        cx.resources.display.flush().unwrap();

        *count += 1;
        // Clears the update flag
        cx.resources.timer_handler.clear_update_interrupt_flag();
    }

    #[task(binds = USART1, resources = [rx_pc, rx_pc_str, transfer], priority = 2)]
    fn usart0_rx(cx: usart0_rx::Context) {
        static mut state: u8 = 0;
        static mut rx_pc: Option<SerialPCRxDma> = None;
        static mut transfer: Option<SerialPCRxDmaTransfer> = None;

        // static mut buf: Option<[u8; 4]> = Some([0; 4]);
        // static mut buf: [u8; 4] = [0; 4];
        // static mut buf: &'static mut [u8; 8] = singleton!(: [u8; 8] = [0; 8]).unwrap();
        // let mut buf = [0; 4];

        match state {
            0 => {
                *rx_pc = Some(cx.resources.rx_pc.take().unwrap());
                let rx_pc = rx_pc.take().unwrap();
                // let rx_pc = cx.resources.rx_pc.take().unwrap();

                // let buf = singleton!(: [u8; 4] = [0; 4]).unwrap();
                let buf = singleton!(: ArrayVec<[u8; 32]> = ArrayVec::<[u8; 32]>::new()).unwrap();

                // *transfer = Some(rx_pc.read(buf));
                // let transfer = rx_pc.read(&mut buf);
                // let mut buf = buf.take().unwrap();
                *transfer = Some(rx_pc.read(buf));
                // *cx.resources.transfer = Some(rx_pc.read(buf));
                // let transfer = rx_pc.read(&mut buf);
                *state = 1;
            }
            1 => {
                *state = 2;
            }
            2 => {
                *state = 1;
            }
            _ => unreachable!(),
        }
        // let b = cx.resources.rx_pc.read().unwrap();
        // if cx.resources.rx_pc_str.len() == 32 {
        //     cx.resources.rx_pc_str.clear();
        // }
        // cx.resources.rx_pc_str.push(char::from(b));
    }

    // extern "C" {
    //     fn EXTI0();
    // }
};
