//! A bounded buffer of log text waiting to be fetched by the host.

use heapless::Deque;

/// Shown in place of text that was dropped to make room.
const GAP_MARKER: &[u8] = b"[... older log lines lost ...]\n";

/// Keeps the most recent log lines, up to `N` bytes.
///
/// When full, the oldest lines are dropped whole, and the gap is marked in
/// the text read back.
pub struct LogBuffer<const N: usize> {
    bytes: Deque<u8, N>,
    /// Lines were dropped since the last read.
    gap: bool,
    /// How much of the gap marker the reader has been given so far.
    marker_sent: usize,
}

impl<const N: usize> Default for LogBuffer<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> LogBuffer<N> {
    pub const fn new() -> Self {
        Self {
            bytes: Deque::new(),
            gap: false,
            marker_sent: 0,
        }
    }

    /// Appends one line, which should end in a newline. A line longer than
    /// the buffer is cut short.
    pub fn push_line(&mut self, line: &[u8]) {
        let line = &line[..line.len().min(N)];
        while N - self.bytes.len() < line.len() {
            self.drop_oldest_line();
        }
        for &byte in line {
            // Cannot fail: room was made above.
            let _ = self.bytes.push_back(byte);
        }
    }

    fn drop_oldest_line(&mut self) {
        self.gap = true;
        self.marker_sent = 0;
        while let Some(byte) = self.bytes.pop_front() {
            if byte == b'\n' {
                break;
            }
        }
    }

    /// Moves the oldest text into `out`, returning how many bytes were
    /// written. Zero means there is nothing left to read.
    pub fn read(&mut self, out: &mut [u8]) -> usize {
        let mut written = 0;
        if self.gap {
            let marker = &GAP_MARKER[self.marker_sent..];
            let count = marker.len().min(out.len());
            out[..count].copy_from_slice(&marker[..count]);
            written = count;
            self.marker_sent += count;
            if self.marker_sent == GAP_MARKER.len() {
                self.gap = false;
                self.marker_sent = 0;
            }
        }
        while written < out.len() {
            let Some(byte) = self.bytes.pop_front() else {
                break;
            };
            out[written] = byte;
            written += 1;
        }
        written
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn read_all<const N: usize>(buffer: &mut LogBuffer<N>, chunk: usize) -> String {
        let mut text = Vec::new();
        let mut out = vec![0; chunk];
        loop {
            let count = buffer.read(&mut out);
            if count == 0 {
                return String::from_utf8(text).unwrap();
            }
            text.extend_from_slice(&out[..count]);
        }
    }

    #[test]
    fn reads_back_lines_in_order_and_only_once() {
        let mut buffer = LogBuffer::<64>::new();
        buffer.push_line(b"first\n");
        buffer.push_line(b"second\n");
        assert_eq!(read_all(&mut buffer, 5), "first\nsecond\n");
        assert_eq!(read_all(&mut buffer, 5), "");

        buffer.push_line(b"third\n");
        assert_eq!(read_all(&mut buffer, 64), "third\n");
    }

    #[test]
    fn drops_whole_old_lines_when_full_and_marks_the_gap() {
        let mut buffer = LogBuffer::<16>::new();
        buffer.push_line(b"aaaaaa\n");
        buffer.push_line(b"bbbbbb\n");
        buffer.push_line(b"cccccc\n");
        assert_eq!(
            read_all(&mut buffer, 7),
            "[... older log lines lost ...]\nbbbbbb\ncccccc\n"
        );
        // The gap is only reported once.
        buffer.push_line(b"dddddd\n");
        assert_eq!(read_all(&mut buffer, 7), "dddddd\n");
    }

    #[test]
    fn a_line_longer_than_the_buffer_is_cut_short() {
        let mut buffer = LogBuffer::<8>::new();
        buffer.push_line(b"keep\n");
        buffer.push_line(b"0123456789\n");
        assert_eq!(
            read_all(&mut buffer, 64),
            "[... older log lines lost ...]\n01234567"
        );
    }
}
