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
use embedded_hal::digital::v2::OutputPin;
use ssd1306::{interface::I2cInterface, prelude::*};
use stm32f1xx_hal::{
    self,
    device::I2C1,
    gpio::{gpiob::PB8, gpiob::PB9, gpioc::PC13},
    gpio::{Alternate, OpenDrain, Output, PushPull, State},
    i2c::{BlockingI2c, DutyCycle, Mode},
    // pac,
    prelude::*,
    stm32,
    timer::{CountDownTimer, Event, Timer},
};

use stm32f1xx_hal::time::Hertz;

use arrayvec::ArrayString;
use core::convert::TryInto;
use core::fmt::Write;

type OledDisplay = ssd1306::mode::graphics::GraphicsMode<
    I2cInterface<BlockingI2c<I2C1, (PB8<Alternate<OpenDrain>>, PB9<Alternate<OpenDrain>>)>>,
>;

const SYSCLK: Hertz = Hertz(72_000_000);
const REFRESH_FREQ: Hertz = Hertz(30);

#[app(device = stm32, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        led: PC13<Output<PushPull>>,
        timer_handler: CountDownTimer<stm32::TIM1>,
        display: OledDisplay,
        #[init(0)]
        idle_cnt: u32,
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
        let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
        let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);

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

        // Configure the syst timer to trigger an update every second and enables interrupt
        let mut timer = Timer::tim1(dp.TIM1, &clocks, &mut rcc.apb2).start_count_down(REFRESH_FREQ);
        timer.listen(Event::Update);

        // rtfm::pend(stm32::Interrupt::TIM1_UP);

        // Init the static resources to use them later through RTFM
        init::LateResources {
            led: led,
            timer_handler: timer,
            display: display,
        }
    }

    #[idle(resources = [idle_cnt])]
    fn idle(mut cx: idle::Context) -> ! {
        loop {
            cx.resources.idle_cnt.lock(|idle_cnt| {
                *idle_cnt = *idle_cnt + 1;
            });
        }
    }

    #[task(binds = TIM1_UP, resources = [display, timer_handler, idle_cnt], priority = 1)]
    fn tim1_up(cx: tim1_up::Context) {
        // Depending on the application, you could want to delegate some of the work done here to
        // the idle task if you want to minimize the latency of interrupts with same priority (if
        // you have any). That could be done with some kind of machine state, etc.

        // Count used to change the timer update frequency
        static mut count: u8 = 0;
        static mut started: bool = false;
        static mut max_idle: f32 = 1.0;

        let mut buf = ArrayString::<[u8; 32]>::new();
        let load = 1.0 - (*cx.resources.idle_cnt as f32 / *max_idle);
        write!(&mut buf, "{:.3}% {}", load, count).unwrap();
        cx.resources.display.clear();
        cx.resources.display.draw(
            Font6x8::render_str(&buf)
                .with_stroke(Some(1u8.into()))
                .into_iter(),
        );
        cx.resources.display.flush().unwrap();

        if !*started {
            *max_idle = *cx.resources.idle_cnt as f32;
            *started = true;
        }
        *cx.resources.idle_cnt = 0;

        *count += 1;
        // Clears the update flag
        cx.resources.timer_handler.clear_update_interrupt_flag();
    }

    // extern "C" {
    //     fn EXTI0();
    // }
};
