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

use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::{
    self,
    gpio::{gpioc::PC13, Output, PushPull, State},
    // pac,
    prelude::*,
    stm32,
    timer::{CountDownTimer, Event, Timer},
};

use stm32f1xx_hal::time::Hertz;
const SYSCLK: Hertz = Hertz(72_000_000);

#[app(device = stm32)]
const APP: () = {
    struct Resources {
        led: PC13<Output<PushPull>>,
        timer_handler: CountDownTimer<stm32::TIM1>,
        #[init(false)]
        led_state: bool,
    }

    #[init]
    fn init(_: init::Context) -> init::LateResources {
        let dp = stm32::Peripherals::take().unwrap();
        // let p = &mut c.core; //cortex_m::peripheral::Peripherals::take().unwrap();
        //                      // Take ownership over the raw flash and rcc devices and convert them into the corresponding
        //                      // HAL structs
        let mut flash = dp.FLASH.constrain();
        let mut rcc = dp.RCC.constrain();

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

        // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the
        // function in order to configure the port. For pins 0-7, crl should be passed instead
        let led = gpioc
            .pc13
            .into_push_pull_output_with_state(&mut gpioc.crh, State::High);

        // Configure the syst timer to trigger an update every second and enables interrupt
        let mut timer = Timer::tim1(dp.TIM1, &clocks, &mut rcc.apb2).start_count_down(1.hz());
        timer.listen(Event::Update);

        // rtfm::pend(stm32::Interrupt::TIM1_UP);

        // Init the static resources to use them later through RTFM
        init::LateResources {
            led: led,
            timer_handler: timer,
        }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        static mut i: u32 = 0;
        loop {
            *i += 1;
        }
    }

    #[task(binds = TIM1_UP, resources = [led, timer_handler, led_state], priority = 1)]
    fn tim1_up(c: tim1_up::Context) {
        // Depending on the application, you could want to delegate some of the work done here to
        // the idle task if you want to minimize the latency of interrupts with same priority (if
        // you have any). That could be done with some kind of machine state, etc.

        // Count used to change the timer update frequency
        static mut COUNT: u8 = 0;

        if *c.resources.led_state {
            // Uses resourcers managed by rtfm to turn led off (on bluepill)
            c.resources.led.set_high().unwrap();
            *c.resources.led_state = false;
        } else {
            c.resources.led.set_low().unwrap();
            *c.resources.led_state = true;
        }
        *COUNT += 1;

        if *COUNT == 4 {
            // Changes timer update frequency
            c.resources.timer_handler.start(2.hz());
        } else if *COUNT == 12 {
            c.resources.timer_handler.start(1.hz());
            *COUNT = 0;
        }

        // Clears the update flag
        c.resources.timer_handler.clear_update_interrupt_flag();
    }

    // extern "C" {
    //     fn EXTI0();
    // }
};
