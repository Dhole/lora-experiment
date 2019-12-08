use core::convert;

#[derive(Clone, Copy)]
enum UartMode {
    Parity8N1 = 0b00,
    Parity8O1 = 0b01,
    Parity8E1 = 0b10,
}

#[derive(Clone, Copy)]
enum UartRate {
    Bps1200 = 0b000,
    Bps2400 = 0b001,
    Bps4800 = 0b010,
    Bps9600 = 0b011,
    Bps19200 = 0b100,
    Bps38400 = 0b101,
    Bps57600 = 0b110,
    Bps11520 = 0b111,
}

#[derive(Clone, Copy)]
enum AirRate {
    Bps300 = 0b000,
    Bps1200 = 0b001,
    Bps2400 = 0b010,
    Bps4800 = 0b011,
    Bps9600 = 0b100,
    Bps19200 = 0b101,
}

#[derive(Clone, Copy)]
enum Channel {
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
enum TransMode {
    Transparent = 0b0,
    Fixed = 0b1,
}

#[derive(Clone, Copy)]
enum IoMode {
    PushPull = 0b0,
    OpenCollector = 0b1,
}

#[derive(Clone, Copy)]
enum WakeUpTime {
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
enum Fec {
    Off = 0b0,
    On = 0b1,
}

#[derive(Clone, Copy)]
enum TxPower {
    P0,
    P1,
    P2,
    P3,
}

enum TxPower100 {
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

enum TxPower500 {
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

enum TxPower1W {
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

struct Config {
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
