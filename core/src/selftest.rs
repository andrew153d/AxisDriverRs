//! Bookkeeping for the power-on self test.
//!
//! The firmware checks each peripheral it can verify without a host or an
//! operator and records the outcome here. The checks themselves need the
//! hardware and live in the firmware crate.

/// One peripheral check. The order is the order they are reported in.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Check {
    /// The step timer interrupt fires at the expected rate.
    StepTimer,
    /// The TMC2209 answers, accepted its configuration and reports no fault.
    MotorDriver,
    /// The shaft encoder answers and sees its magnet.
    Encoder,
    /// The settings and the MAC address can be read from the EEPROM.
    Eeprom,
}

impl Check {
    pub const ALL: [Self; 4] = [
        Self::StepTimer,
        Self::MotorDriver,
        Self::Encoder,
        Self::Eeprom,
    ];
}

/// Outcome of the self test. Checks that were never recorded count as
/// failed.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Report {
    passed: [bool; Check::ALL.len()],
}

impl Report {
    pub const fn new() -> Self {
        Self {
            passed: [false; Check::ALL.len()],
        }
    }

    pub fn record(&mut self, check: Check, passed: bool) {
        self.passed[check as usize] = passed;
    }

    pub fn passed(&self, check: Check) -> bool {
        self.passed[check as usize]
    }

    pub fn all_passed(&self) -> bool {
        self.passed.iter().all(|&passed| passed)
    }

    /// Every check with its outcome, in reporting order.
    pub fn results(&self) -> impl Iterator<Item = (Check, bool)> + '_ {
        Check::ALL
            .into_iter()
            .map(|check| (check, self.passed(check)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unrecorded_checks_count_as_failed() {
        let mut report = Report::new();
        assert!(!report.all_passed());
        for check in Check::ALL {
            assert!(!report.passed(check));
            report.record(check, true);
        }
        assert!(report.all_passed());
    }

    #[test]
    fn reports_results_in_order() {
        let mut report = Report::new();
        report.record(Check::Encoder, true);
        let results: Vec<_> = report.results().collect();
        assert_eq!(
            results,
            [
                (Check::StepTimer, false),
                (Check::MotorDriver, false),
                (Check::Encoder, true),
                (Check::Eeprom, false),
            ]
        );
    }
}
