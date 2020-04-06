pub trait Iterator {
    type Item;

    fn next(&mut self) -> Option<Self::Item>;
}
pub trait SerialCfg {
    type Context;
    type Clocks;

    fn set_baudrate(&mut self, ctx: &mut Self::Context, baudrate: u32, clocks: Self::Clocks);
}

use core::convert::Infallible;
use embedded_hal::serial::{Read, Write};
use stm32f1::stm32f103::{self, USART1, USART2, USART3};
use stm32f1xx_hal::{
    afio, rcc,
    serial::{self, Pins},
    time,
};

pub struct Serial<USART, PINS>(Option<serial::Serial<USART, PINS>>);

pub struct Context<RCCBUS: rcc::RccBus> {
    mapr: afio::MAPR,
    apb: RCCBUS::Bus,
}

macro_rules! serial_cfg_impl {
    ($USARTX:ident, $usartx:ident) => {
        impl<PINS> Read<u8> for Serial<$USARTX, PINS> {
            type Error = serial::Error;

            fn read(&mut self) -> nb::Result<u8, serial::Error> {
                self.0.as_mut().unwrap().read()
            }
        }

        impl<PINS> Write<u8> for Serial<$USARTX, PINS> {
            type Error = Infallible;

            fn flush(&mut self) -> nb::Result<(), Self::Error> {
                self.0.as_mut().unwrap().flush()
            }

            fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
                self.0.as_mut().unwrap().write(byte)
            }
        }

        impl<PINS: Pins<$USARTX>> SerialCfg for Serial<$USARTX, PINS> {
            type Context = Context<$USARTX>;
            type Clocks = rcc::Clocks;

            fn set_baudrate(
                &mut self,
                ctx: &mut Self::Context,
                baudrate: u32,
                clocks: Self::Clocks,
            ) {
                let (usart, pins) = self.0.take().unwrap().release();
                let s = serial::Serial::$usartx(
                    usart,
                    pins,
                    &mut ctx.mapr,
                    serial::Config::default().baudrate(time::Bps(baudrate)),
                    clocks,
                    &mut ctx.apb,
                );
                self.0 = Some(s)
            }
        }
    };
}

serial_cfg_impl!(USART1, usart1);
serial_cfg_impl!(USART2, usart2);
serial_cfg_impl!(USART3, usart3);
