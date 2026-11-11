//! The time base for step generation: a free-running 32-bit counter with an
//! alarm interrupt.
//!
//! TC2 and TC3 are chained into one 32-bit counter clocked at the CPU
//! frequency, which gives 8.3 ns resolution and wraps every 36 seconds.
//! Because it never stops or restarts, steps scheduled against it do not
//! accumulate timing errors.

use atsamd_hal::{
    clock::Tc2Tc3Clock,
    pac::{self, interrupt, tc2::Count32},
};

use crate::board::CPU_HZ;

/// Rate of the counter.
pub const TICKS_PER_SECOND: u32 = CPU_HZ;

/// An alarm this close to the present may be missed by the compare logic,
/// so it is raised by hand instead.
const MIN_ALARM_LEAD_TICKS: u32 = TICKS_PER_SECOND / 1_000_000 * 4;

pub struct StepTimer {
    tc: pac::Tc2,
    /// Upper half of the counter; held so that nothing else can use it.
    _tc_high: pac::Tc3,
}

impl StepTimer {
    /// Starts the counter. The alarm interrupt is `TC2`; it still has to be
    /// unmasked.
    pub fn new(
        tc: pac::Tc2,
        tc_high: pac::Tc3,
        mclk: &mut pac::Mclk,
        _clock: &Tc2Tc3Clock,
    ) -> Self {
        mclk.apbbmask()
            .modify(|_, w| w.tc2_().set_bit().tc3_().set_bit());

        let timer = Self {
            tc,
            _tc_high: tc_high,
        };
        let regs = timer.regs();
        regs.ctrla().write(|w| w.swrst().set_bit());
        while regs.syncbusy().read().swrst().bit_is_set() {}

        regs.ctrla()
            .write(|w| w.mode().count32().prescaler().div1());
        regs.wave().write(|w| w.wavegen().nfrq());
        regs.intenset().write(|w| w.mc0().set_bit());
        regs.ctrla().modify(|_, w| w.enable().set_bit());
        while regs.syncbusy().read().enable().bit_is_set() {}
        timer
    }

    fn regs(&self) -> &Count32 {
        self.tc.count32()
    }

    /// The current counter value.
    pub fn now(&self) -> u32 {
        let regs = self.regs();
        // The counter has to be copied into its register on request.
        regs.ctrlbset().write(|w| w.cmd().readsync());
        while regs.syncbusy().read().ctrlb().bit_is_set() {}
        while !regs.ctrlbset().read().cmd().is_none() {}
        while regs.syncbusy().read().count().bit_is_set() {}
        regs.count().read().bits()
    }

    /// Requests the interrupt for when the counter reaches `at`, or straight
    /// away if that time has already passed.
    pub fn set_alarm(&mut self, at: u32) {
        let regs = self.regs();
        regs.cc(0).write(|w| unsafe { w.bits(at) });
        while regs.syncbusy().read().cc0().bit_is_set() {}

        let lead = at.wrapping_sub(self.now()) as i32;
        if lead < MIN_ALARM_LEAD_TICKS as i32 {
            pac::NVIC::pend(interrupt::TC2);
        }
    }

    /// Acknowledges the alarm. Call at the start of the interrupt handler.
    pub fn clear_alarm(&mut self) {
        self.regs().intflag().write(|w| w.mc0().set_bit());
    }
}
