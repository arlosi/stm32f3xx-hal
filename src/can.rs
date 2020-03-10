use crate::stm32::{CAN};
use crate::rcc::{Clocks, APB1};
use crate::gpio::AF9;
use crate::time::Hertz;

#[derive(Debug)]
pub enum Error {

    #[doc(hidden)]
    _Extensible,
}

#[derive(Debug)]
pub struct CanFrame
{
    id: u32,
    rtr: bool,
    ext: bool,
    dlc: u8,
    data: [u8; 8],
}

impl CanFrame
{
    const STD_MASK: u32 = 0x7FF;
    const EXT_MASK: u32 = 0x1FFFFFFF;

    pub fn new (id: u32, data: &[u8], rtr: bool, ext: bool) -> Self {
        assert!(id < if ext {Self::EXT_MASK} else {Self::STD_MASK});
        assert!(data.len() <= 8);

        let mut data_copy: [u8; 8] = Default::default();
        data_copy[0..data.len()].copy_from_slice(data);

        CanFrame {
            id : id,
            rtr: rtr,
            ext: ext,
            dlc: data.len() as u8,
            data: data_copy,
        }
    }

    pub fn id(&self) -> u32 {
        self.id
    }

    pub fn is_extended(&self) -> bool {
        self.ext
    }

    pub fn is_rtr(&self) -> bool {
        self.rtr
    }

    pub fn data(&self) -> &[u8]
    {
        &self.data[..self.dlc as usize]
    }
}

pub unsafe trait TxPin<CAN> {}
pub unsafe trait RxPin<CAN> {}

use crate::gpio::gpioa::{PA11, PA12};
unsafe impl RxPin<CAN> for PA11<AF9> {}
unsafe impl TxPin<CAN> for PA12<AF9> {}

use crate::gpio::gpiob::{PB8, PB9};
unsafe impl RxPin<CAN> for PB8<AF9> {}
unsafe impl TxPin<CAN> for PB9<AF9> {}

pub struct Can<CAN, PINS> {
    can: CAN,
    pins: PINS,
}

#[cfg(any(
    feature = "stm32f334",
))]
impl<TX, RX> Can<CAN, (TX, RX)> {
    pub fn can<F>(
        can: CAN,
        pins: (TX, RX),
        baudrate: F,
        clocks: Clocks,
        apb1: &mut APB1,
    ) -> Self
    where
        F: Into<Hertz>,
        TX: TxPin<CAN>,
        RX: RxPin<CAN>,
    {
        apb1.enr().modify(|_, w| w.canen().enabled());
        apb1.rstr().modify(|_, w| w.canrst().reset());
        apb1.rstr().modify(|_, w| w.canrst().clear_bit());

        // Wake up
        can.mcr.modify(|_, w| w.sleep().clear_bit());
        while can.msr.read().slak().bits() {};
        
        // Enter initialization mode
        can.mcr.modify(|_, w| w.inrq().set_bit());
        while !can.msr.read().inak().bits() {};

        {
            let ts1: u8 = 13;
            let ts2: u8 =  2;
            let sjw: u8 = 1;
            let prescaler = clocks.pclk1().0 / (baudrate.into().0 * (1 + ts1 + ts2) as u32);
            assert!(prescaler < 512);

            can.btr.write(|w| {
                w.silm().clear_bit(); // Normal
                w.lbkm().clear_bit(); // No loopback
                unsafe { 
                    w.ts1().bits(ts1 - 1);
                    w.ts2().bits(ts2 - 1);
                    w.sjw().bits(sjw - 1);
                    w.brp().bits((prescaler - 1) as u16);
                } 
                w
            });
        }

        // Leave initialization mode
        can.mcr.modify(|_, w| w.inrq().clear_bit());
        while can.msr.read().inak().bits() {};

        Can {can, pins}
    }

    pub fn tx(&mut self, frame: &CanFrame) {
        let tsr = self.can.tsr.read();
        let tx = if tsr.tme0().bit() {
            Some(&self.can.tx[0])
        } else if tsr.tme1().bit() {
            Some(&self.can.tx[1])
        } else if tsr.tme2().bit() {
            Some(&self.can.tx[2])
        } else {
            None
        };

        if let Some(tx) = tx {
            tx.tdtr.modify(|_, w| {
                unsafe { w.dlc().bits(frame.data().len() as u8); }
                w.tgt().clear_bit();
                w
            });

            tx.tir.modify(|_, w| {
                if frame.is_extended() {
                    w.ide().extended();
                    unsafe { w.exid().bits(frame.id()) };
                } else {
                    w.ide().standard();
                    unsafe { w.stid().bits(frame.id() as u16) };
                }
                
                if frame.is_rtr() {
                    w.rtr().remote();
                } else {
                    w.rtr().data();
                }
                w
            });

            tx.tdlr.write(|w| unsafe { 
                w.data0().bits(frame.data[0]);
                w.data1().bits(frame.data[1]);
                w.data2().bits(frame.data[2]);
                w.data3().bits(frame.data[3]);
                w
            });

            tx.tdhr.write(|w| unsafe { 
                w.data4().bits(frame.data[4]);
                w.data5().bits(frame.data[5]);
                w.data6().bits(frame.data[6]);
                w.data7().bits(frame.data[7]);
                w
            });

            tx.tir.modify(|_, w| w.txrq().set_bit());
        }
    }
}
