#![allow(dead_code)]

use crate::serial::{Read, Serial, SerialCfg, Write};
// use embedded_hal::serial::{Read, Write};

use core::convert;

#[derive(Clone, Copy)]
pub enum UartMode {
    Parity8N1 = 0b00,
    Parity8O1 = 0b01,
    Parity8E1 = 0b10,
}

#[derive(Clone, Copy)]
pub enum UartRate {
    Bps1200 = 0b000,
    Bps2400 = 0b001,
    Bps4800 = 0b010,
    Bps9600 = 0b011,
    Bps19200 = 0b100,
    Bps38400 = 0b101,
    Bps57600 = 0b110,
    Bps115200 = 0b111,
}

#[derive(Clone, Copy)]
pub enum AirRate {
    Bps300 = 0b000,
    Bps1200 = 0b001,
    Bps2400 = 0b010,
    Bps4800 = 0b011,
    Bps9600 = 0b100,
    Bps19200 = 0b101,
}

#[derive(Clone, Copy)]
pub enum Channel {
    Mhz410 = 0b00000,
    Mhz411 = 0b00001,
    Mhz412 = 0b00010,
    Mhz413 = 0b00011,
    Mhz414 = 0b00100,
    Mhz415 = 0b00101,
    Mhz416 = 0b00110,
    Mhz417 = 0b00111,
    Mhz418 = 0b01000,
    Mhz419 = 0b01001,
    Mhz420 = 0b01010,
    Mhz421 = 0b01011,
    Mhz422 = 0b01100,
    Mhz423 = 0b01101,
    Mhz424 = 0b01110,
    Mhz425 = 0b01111,
    Mhz426 = 0b10000,
    Mhz427 = 0b10001,
    Mhz428 = 0b10010,
    Mhz429 = 0b10011,
    Mhz430 = 0b10100,
    Mhz431 = 0b10101,
    Mhz432 = 0b10110,
    Mhz433 = 0b10111,
    Mhz434 = 0b11000,
    Mhz435 = 0b11001,
    Mhz436 = 0b11010,
    Mhz437 = 0b11011,
    Mhz438 = 0b11100,
    Mhz439 = 0b11101,
    Mhz440 = 0b11110,
    Mhz441 = 0b11111,
}

#[derive(Clone, Copy)]
pub enum TransMode {
    Transparent = 0b0,
    Fixed = 0b1,
}

#[derive(Clone, Copy)]
pub enum IoMode {
    PushPull = 0b0,
    OpenCollector = 0b1,
}

#[derive(Clone, Copy)]
pub enum WakeUpTime {
    Ms250,
    Ms500,
    Ms750,
    Ms1000,
    Ms1250,
    Ms1500,
    Ms1750,
    Ms2000,
}

#[derive(Clone, Copy)]
pub enum Fec {
    Off = 0b0,
    On = 0b1,
}

#[derive(Clone, Copy)]
pub enum TxPower {
    P0,
    P1,
    P2,
    P3,
}

pub enum TxPower100 {
    Dbm20,
    Dbm17,
    Dbm14,
    Dbm10,
}

impl convert::From<TxPower100> for TxPower {
    fn from(txp: TxPower100) -> Self {
        match txp {
            TxPower100::Dbm20 => Self::P0,
            TxPower100::Dbm17 => Self::P1,
            TxPower100::Dbm14 => Self::P2,
            TxPower100::Dbm10 => Self::P3,
        }
    }
}

pub enum TxPower500 {
    Dbm27,
    Dbm24,
    Dbm21,
    Dbm18,
}

impl convert::From<TxPower500> for TxPower {
    fn from(txp: TxPower500) -> Self {
        match txp {
            TxPower500::Dbm27 => Self::P0,
            TxPower500::Dbm24 => Self::P1,
            TxPower500::Dbm21 => Self::P2,
            TxPower500::Dbm18 => Self::P3,
        }
    }
}

pub enum TxPower1W {
    Dbm30,
    Dbm27,
    Dbm24,
    Dbm21,
}

impl convert::From<TxPower1W> for TxPower {
    fn from(txp: TxPower1W) -> Self {
        match txp {
            TxPower1W::Dbm30 => Self::P0,
            TxPower1W::Dbm27 => Self::P1,
            TxPower1W::Dbm24 => Self::P2,
            TxPower1W::Dbm21 => Self::P3,
        }
    }
}

pub struct Config {
    addr: u16,
    uart_mode: UartMode,
    uart_rate: UartRate,
    air_rate: AirRate,
    channel: Channel,
    trans_mode: TransMode,
    io_mode: IoMode,
    wakeup_time: WakeUpTime,
    fec: bool,
    tx_power: TxPower,
}

impl Config {
    fn param_array(&self, save: bool) -> [u8; 6] {
        let mut params = [0; 6];
        if save {
            params[0] = 0xc0;
        } else {
            params[0] = 0xc2;
        }
        let addr_be = self.addr.to_be_bytes();
        params[1] = addr_be[0];
        params[2] = addr_be[1];

        params[3] = (self.uart_mode as u8) << 6;
        params[3] |= (self.uart_rate as u8) << 3;
        params[3] |= self.air_rate as u8;

        params[4] = self.channel as u8;

        params[5] = (self.trans_mode as u8) << 7;
        params[5] |= (self.io_mode as u8) << 6;
        params[5] |= (self.wakeup_time as u8) << 3;
        params[5] |= (self.fec as u8) << 2;
        params[5] |= self.tx_power as u8;
        params
    }
}

use embedded_hal::digital::v2::{InputPin, OutputPin};

// pub enum Error<MODEPIN0, MODEPIN1, AUXPIN>
// where
//     MODEPIN0: OutputPin,
//     MODEPIN1: OutputPin,
//     AUXPIN: InputPin,
// {
//     OutputPin(MODEPIN::Error),
//     InputPin(AUXPIN::Error),
// }

pub struct ModePins<PIN0, PIN1>
where
    PIN0: OutputPin,
    PIN1: OutputPin,
{
    m0: PIN0,
    m1: PIN1,
}

impl<PIN0, PIN1> ModePins<PIN0, PIN1>
where
    PIN0: OutputPin,
    PIN1: OutputPin,
{
    pub fn new(m0: PIN0, m1: PIN1) -> Self {
        Self { m0, m1 }
    }
}

enum Mode {
    Normal = 0b00,
    WakeUp = 0b01,
    PowerSaving = 0b10,
    Sleep = 0b11,
}

impl<PIN0, PIN1> ModePins<PIN0, PIN1>
where
    PIN0: OutputPin,
    PIN1: OutputPin,
{
    fn _set(&mut self, mode: Mode) {
        let (m0_high, m1_high) = match mode {
            Mode::Normal => (false, false),
            Mode::WakeUp => (false, false),
            Mode::PowerSaving => (false, false),
            Mode::Sleep => (false, false),
        };
        if m0_high {
            self.m0.set_high();
        } else {
            self.m0.set_low();
        }
        if m1_high {
            self.m1.set_high();
        } else {
            self.m1.set_low();
        }
    }
    fn set(&mut self, mode: Mode) {
        self._set(mode);
    }
}

pub struct LoRa<SERIAL, MODEPIN0, MODEPIN1, AUXPIN>
where
    SERIAL: SerialCfg + Read + Write,
    MODEPIN0: OutputPin,
    MODEPIN1: OutputPin,
    AUXPIN: InputPin,
{
    cfg: Config,
    clocks: SERIAL::Clocks,
    serial: SERIAL,
    mode_pins: ModePins<MODEPIN0, MODEPIN1>,
    aux: AUXPIN,
}

impl<SERIAL, MODEPIN0, MODEPIN1, AUXPIN> LoRa<SERIAL, MODEPIN0, MODEPIN1, AUXPIN>
where
    SERIAL: SerialCfg + Read + Write,
    MODEPIN0: OutputPin,
    MODEPIN1: OutputPin,
    AUXPIN: InputPin,
{
    pub fn new(
        mode_pins: ModePins<MODEPIN0, MODEPIN1>,
        aux: AUXPIN,
        mut serial: SERIAL,
        clocks: SERIAL::Clocks,
        mut ctx: &mut SERIAL::Context,
    ) -> Self {
        // serial.write('B' as u8);
        // serial.flush();
        // for b in [0, 0, 0x17, 0x41, 0x41, 0x41].iter() {
        // for b in [0, 0, 0x17, 0x41, 0x41, 0x41].iter() {
        //     nb::block!(serial.write(*b));
        // }
        serial.write_all(b"\nHELLO\n").ok();
        serial.flush();

        serial.set_baudrate(&mut ctx, 9600, clocks);
        let cfg = Config {
            addr: 0x0000,
            uart_mode: UartMode::Parity8N1,
            uart_rate: UartRate::Bps9600,
            air_rate: AirRate::Bps2400,
            channel: Channel::Mhz433,
            trans_mode: TransMode::Transparent,
            io_mode: IoMode::PushPull,
            wakeup_time: WakeUpTime::Ms250,
            fec: true,
            tx_power: TxPower::P0,
        };
        Self {
            cfg,
            clocks,
            serial,
            mode_pins,
            aux,
        }
    }

    pub fn ready(&self) {
        while self.aux.is_low().ok().unwrap() {}
    }

    pub fn set_cfg(&mut self, cfg: Config) {
        self.cfg = cfg;
        self.mode_pins.set(Mode::Sleep);
        self.ready();
        // self.serial.
    }
}
