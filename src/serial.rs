use nb;

pub trait Read {
    /// Read error
    type Error;

    /// Reads a single word from the serial interface
    fn read(&mut self) -> nb::Result<u8, Self::Error>;

    fn read_exact(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error> {
        for i in 0..buffer.len() {
            buffer[i] = nb::block!(self.read())?;
        }

        Ok(())
    }
}

pub trait Write {
    /// Write error
    type Error;

    /// Writes a single byte to the serial interface
    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error>;

    /// Ensures that none of the previously written words are still buffered
    fn flush(&mut self) -> nb::Result<(), Self::Error>;

    fn write_all(&mut self, buffer: &[u8]) -> Result<(), Self::Error> {
        for byte in buffer {
            nb::block!(self.write(byte.clone()))?;
        }

        Ok(())
    }
}

pub trait SerialCfg {
    type Context;
    type Clocks: Copy + Clone;

    fn set_baudrate(&mut self, ctx: &mut Self::Context, baudrate: u32, clocks: Self::Clocks);
}

use core::convert::Infallible;
use embedded_hal::serial::{Read as _, Write as _};
use stm32f1::stm32f103::{self, USART1, USART2, USART3};
use stm32f1xx_hal::{
    afio, rcc,
    serial::{self, Pins},
    time,
};

// pub trait SerialRead = embedded_hal::serial::Read<u8, Error = hal::serial::Error>;
// pub trait SerialWrite = embedded_hal::serial::Write<u8, Error = hal::serial::Error>;

pub struct Serial<USART, PINS>(Option<serial::Serial<USART, PINS>>);

pub struct Context<RCCBUS: rcc::RccBus> {
    mapr: afio::MAPR,
    apb: RCCBUS::Bus,
}

impl<RCCBUS: rcc::RccBus> Context<RCCBUS> {
    pub fn new(mapr: afio::MAPR, apb: RCCBUS::Bus) -> Self {
        Self { mapr, apb }
    }
}

impl<USART, PINS> Serial<USART, PINS> {
    pub fn new(s: serial::Serial<USART, PINS>) -> Self {
        Self(Some(s))
    }
}

// impl<USART, PINS> Serial<USART, PINS> {
//     pub fn new(
//         rx: dma::RxDma<serial::Rx<USART1>, dma::dma1::C5>,
//         tx: dma::TxDma<serial::Tx<USART1>, dma::dma1::C4>,
//     ) -> Self {
//         Self(Some(s))
//     }
// }

macro_rules! serial_cfg_impl {
    ($USARTX:ident, $usartx:ident) => {
        impl<PINS> Read for Serial<$USARTX, PINS> {
            type Error = serial::Error;

            fn read(&mut self) -> nb::Result<u8, serial::Error> {
                self.0.as_mut().unwrap().read()
            }
        }

        impl<PINS> Write for Serial<$USARTX, PINS> {
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
