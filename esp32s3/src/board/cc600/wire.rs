//! A device's transmit line: bytes leave at the baud rate, at most [`BURST`] per poll so the
//! chip's 128-byte RX FIFO never overflows between polls.
use std::collections::VecDeque;
use crate::periph::CPU_HZ;

pub const BURST: usize = 64;

pub struct Wire { out: VecDeque<u8>, at: u64, byte: u64 }

impl Wire {
    pub fn new(baud: u32) -> Self { Wire { out: VecDeque::new(), at: 0, byte: CPU_HZ * 10 / u64::from(baud) } }

    /// Queues `data` behind what is still on the line.
    pub fn send(&mut self, cycle: u64, data: &[u8]) {
        if self.out.is_empty() { self.at = self.at.max(cycle + self.byte); }
        self.out.extend(data);
    }

    /// Bytes on the wire by `cycle`, at most [`BURST`].
    pub fn transmit(&mut self, cycle: u64) -> Vec<u8> {
        let mut data = Vec::new();
        while self.at <= cycle && data.len() < BURST {
            let Some(b) = self.out.pop_front() else { break };
            data.push(b);
            self.at += self.byte;
        }
        data
    }

    pub fn next_deadline(&self) -> Option<u64> { (!self.out.is_empty()).then_some(self.at) }

    pub fn clear(&mut self) { self.out.clear(); }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bytes_leave_at_the_baud_rate_in_bursts() {
        let mut w = Wire::new(115_200);
        let byte = CPU_HZ * 10 / 115_200;
        w.send(0, &[0; 100]);
        assert!(w.transmit(0).is_empty());
        assert_eq!(w.transmit(byte * 3).len(), 3);
        assert_eq!(w.transmit(u64::MAX).len(), BURST);
        assert_eq!(w.next_deadline(), Some(byte * (4 + BURST as u64)));
    }
}
