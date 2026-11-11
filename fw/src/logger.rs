//! Where `log` messages go: out over RTT for an attached debugger, and into
//! a buffer that the host can fetch over the protocol ("Get Log").

use core::cell::RefCell;
use core::fmt::{self, Write};

use axis_core::logbuf::LogBuffer;
use cortex_m::interrupt::Mutex;
use log::{LevelFilter, Log, Metadata, Record};
use rtt_target::{rprint, rtt_init_print, ChannelMode};

use crate::clock;

/// Enough for the start-up self test report, with room to spare.
const BUFFER_SIZE: usize = 8192;
/// Longer messages are cut short.
const MAX_LINE_LENGTH: usize = 256;

static BUFFER: Mutex<RefCell<LogBuffer<BUFFER_SIZE>>> = Mutex::new(RefCell::new(LogBuffer::new()));
static LOGGER: Logger = Logger;

/// Starts logging. Call once, before anything logs.
pub fn init() {
    // Never block on RTT: without a debugger attached nothing reads it.
    rtt_init_print!(ChannelMode::NoBlockSkip, BUFFER_SIZE);
    // Fails only if a logger is already set, which leaves logging working.
    let _ = log::set_logger(&LOGGER);
    log::set_max_level(LevelFilter::Debug);
}

/// Moves the oldest buffered log text into `out`, returning its length.
/// Zero means the host has read everything.
pub fn read(out: &mut [u8]) -> usize {
    cortex_m::interrupt::free(|cs| BUFFER.borrow(cs).borrow_mut().read(out))
}

struct Logger;

impl Log for Logger {
    fn enabled(&self, _metadata: &Metadata) -> bool {
        true
    }

    fn log(&self, record: &Record) {
        // Format outside the critical section: it is slow, and the step
        // timer interrupt must not be held off for long.
        let mut line = Line::new();
        let now = clock::millis();
        let _ = write!(
            line,
            "{:>6}.{:03} {:<5} {}: {}",
            now / 1_000,
            now % 1_000,
            record.level(),
            record.module_path().unwrap_or("?"),
            record.args()
        );
        let line = line.finish();

        rprint!("{}", line);
        cortex_m::interrupt::free(|cs| BUFFER.borrow(cs).borrow_mut().push_line(line.as_bytes()));
    }

    fn flush(&self) {}
}

/// One line of log text; anything that does not fit is dropped.
struct Line {
    bytes: [u8; MAX_LINE_LENGTH],
    len: usize,
}

impl Line {
    fn new() -> Self {
        Self {
            bytes: [0; MAX_LINE_LENGTH],
            len: 0,
        }
    }

    /// Ends the line and returns it.
    fn finish(&mut self) -> &str {
        self.bytes[self.len] = b'\n';
        // Only whole characters are ever copied in, so this cannot fail.
        core::str::from_utf8(&self.bytes[..=self.len]).unwrap_or("?\n")
    }
}

impl Write for Line {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        // Keep one byte free for the newline, and never split a character.
        let room = MAX_LINE_LENGTH - 1 - self.len;
        let mut count = text.len().min(room);
        while !text.is_char_boundary(count) {
            count -= 1;
        }
        self.bytes[self.len..self.len + count].copy_from_slice(&text.as_bytes()[..count]);
        self.len += count;
        Ok(())
    }
}
