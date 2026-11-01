//! Virtual ANT+ sensors behind the nRF UART protocol (opencc600 crates/nrf-link and sensors::ant).
use crate::periph::CPU_HZ;
use super::nrf::frame;

const TYPES: [u8; 3] = [120, 121, 11];

#[derive(Default, Clone, Copy)]
struct Sensor {
    enabled: bool,
    number: u16,
    transmission: u8,
    channel: Option<(u16, u8, u64)>,
    last_page: Option<u64>,
    connected: bool,
}
impl Sensor {
    fn matches(&self) -> bool {
        self.enabled && self.channel.is_some_and(|(n, t, _)| (n == 0 || n == self.number) && (t == 0 || t == self.transmission))
    }
    fn fresh(&self, cycle: u64) -> bool { self.matches() && self.last_page.is_some_and(|at| cycle.saturating_sub(at) < 5 * CPU_HZ) }
}

#[derive(Default)]
pub struct Ant {
    sensors: [Sensor; 3],
    scan_end: Option<u64>,
    next: Option<u64>,
}
impl Ant {
    /// Host command: `ant TYPE NUMBER TRANSMISSION HEX_PAGE|idle|off`. An idle sensor is discoverable,
    /// but sends no measurement; off simulates a flat battery. Pages only enter an explicitly opened channel.
    pub fn command(&mut self, cycle: u64, args: &[&str]) -> Result<Vec<Vec<u8>>, String> {
        let [kind, number, transmission, value] = args else { return Err("ant TYPE NUMBER TRANSMISSION HEX_PAGE|idle|off".into()) };
        let index = kind.parse::<u8>().ok().and_then(|t| TYPES.iter().position(|&k| k == t)).ok_or("ANT type must be 120, 121 or 11")?;
        let number = number.parse::<u16>().ok().filter(|&n| n != 0).ok_or("ANT number must be 1..65535")?;
        let transmission = transmission.parse::<u8>().map_err(|_| "invalid ANT transmission type")?;
        let page = if matches!(*value, "idle" | "off") { None } else {
            if value.len() != 16 || !value.bytes().all(|b| b.is_ascii_hexdigit()) { return Err("ANT page must contain 8 hexadecimal bytes".into()) }
            let mut page = [0; 8];
            for (i, byte) in page.iter_mut().enumerate() { *byte = u8::from_str_radix(&value[i * 2..i * 2 + 2], 16).map_err(|_| "invalid ANT page")?; }
            Some(page)
        };
        let s = &mut self.sensors[index];
        s.enabled = *value != "off"; s.number = number; s.transmission = transmission;
        s.last_page = page.map(|_| cycle);
        let mut out = Vec::new();
        if s.matches() {
            if let Some(page) = page { out.push(frame(4, TYPES[index], &page)); }
            if !s.connected && page.is_some() { out.push(Self::state(index, s, 3)); }
        }
        s.connected = s.fresh(cycle);
        if self.next.is_none() && (s.enabled || s.channel.is_some()) { self.next = Some(cycle); }
        Ok(out)
    }
    fn state(index: usize, s: &Sensor, state: u8) -> Vec<u8> {
        let (n, t, _) = s.channel.unwrap_or((s.number, s.transmission, 0));
        frame(4, 1, &[0x17, TYPES[index], n as u8, (n >> 8) as u8, t, state, 0, 0])
    }
    pub fn receive(&mut self, cycle: u64, kind: u8, cmd: u8, p: &[u8]) -> Option<Vec<Vec<u8>>> {
        if kind != 2 || p.len() != 8 { return None }
        let mut out = Vec::new();
        if cmd == 0x10 && p[..3] == [0xe1, 2, 0] {
            let seconds = u16::from_le_bytes([p[3], p[4]]);
            self.scan_end = Some(cycle + u64::from(seconds) * CPU_HZ);
            out.push(frame(5, cmd, &[0xe1, 2, 1, 0, 0, 0, 0, 0]));
        } else if cmd == 1 && p[0] == 0x17 {
            let i = TYPES.iter().position(|&t| t == p[1])?;
            let s = &mut self.sensors[i];
            s.channel = if p[5] == 1 { None } else { Some((u16::from_le_bytes([p[2], p[3]]), p[4], cycle + u64::from(p[6]) * CPU_HZ)) };
            s.connected = false;
            out.push(frame(5, cmd, &[0; 8]));
            if s.channel.is_some() { out.push(Self::state(i, s, 4)); }
        } else if cmd == 11 && p[..2] == [1, 0xaa] {
            out.push(frame(5, cmd, &[0; 8]));
            if self.sensors[2].fresh(cycle) { out.push(frame(4, 11, &[1, 0xac, 0, 0, 0, 0, 0, 0])); }
        } else { return None }
        self.next = Some(cycle);
        Some(out)
    }
    pub fn advance(&mut self, cycle: u64) -> Vec<Vec<u8>> {
        if self.next.is_none_or(|at| at > cycle) { return Vec::new() }
        let mut out = Vec::new();
        if let Some(end) = self.scan_end {
            if cycle >= end {
                self.scan_end = None;
                out.push(frame(4, 0x10, &[0xf3, 3, 0, 0, 0, 0, 0, 0]));
            } else {
                for (i, s) in self.sensors.iter().enumerate().filter(|(_, s)| s.enabled) {
                    let [lo, hi] = s.number.to_le_bytes();
                    out.push(frame(4, 0x10, &[0xf3, 3, TYPES[i], lo, hi, s.transmission, (-42i8) as u8, 1]));
                }
            }
        }
        for (i, s) in self.sensors.iter_mut().enumerate().filter(|(_, s)| s.channel.is_some()) {
            let state = if s.fresh(cycle) { 3 } else if !s.connected && s.channel.is_some_and(|(_, _, end)| cycle >= end) { 5 } else { 4 };
            out.push(Self::state(i, s, state));
            s.connected = state == 3;
            if state == 5 { s.channel = None; }
        }
        self.next = (self.scan_end.is_some() || self.sensors.iter().any(|s| s.channel.is_some())).then_some(cycle + CPU_HZ);
        out
    }
    pub fn next_deadline(&self) -> Option<u64> { self.next }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn discover_pair_cut_and_restore_without_bypassing_the_radio() {
        let mut a = Ant::default();
        assert!(a.command(0, &["120", "6001", "1", "0000000000000096"]).unwrap().is_empty());
        a.receive(0, 2, 0x10, &[0xe1, 2, 0, 5, 0, 0xff, 0, 0]).unwrap();
        let f = a.advance(0);
        assert_eq!(&f[0][6..14], &[0xf3, 3, 120, 0x71, 0x17, 1, 214, 1]);
        // Wrong identity: no measurement, even though this device type is open.
        a.receive(0, 2, 1, &[0x17, 120, 1, 0, 1, 0, 25, 0]).unwrap();
        assert!(a.command(CPU_HZ, &["120", "6001", "1", "0000000000000096"]).unwrap().is_empty());
        a.receive(CPU_HZ, 2, 1, &[0x17, 120, 0x71, 0x17, 1, 0, 25, 0]).unwrap();
        let f = a.command(CPU_HZ, &["120", "6001", "1", "0000000000000096"]).unwrap();
        assert_eq!((f[0][5], f[0][13]), (120, 150));
        a.command(2 * CPU_HZ, &["120", "6001", "1", "off"]).unwrap();
        assert!(a.advance(2 * CPU_HZ).iter().all(|f| f[5] != 120 && f[6] != 0xf3));
        assert!(!a.sensors[0].connected);
        let f = a.command(3 * CPU_HZ, &["120", "6001", "1", "0000000000000096"]).unwrap();
        assert_eq!(f[0][5], 120);
        assert_eq!(f[1][11], 3);
    }

    #[test]
    fn missing_measurements_expire_and_invalid_host_input_is_rejected() {
        let mut a = Ant::default();
        assert!(a.command(0, &["120", "0", "1", "idle"]).is_err());
        assert!(a.command(0, &["120", "1", "1", "éééééééé"]).is_err());
        a.command(0, &["120", "6001", "1", "idle"]).unwrap();
        a.receive(0, 2, 1, &[0x17, 120, 0, 0, 0, 0, 25, 0]).unwrap();
        a.command(0, &["120", "6001", "1", "0000000000000096"]).unwrap();
        assert_eq!(a.advance(5 * CPU_HZ)[0][11], 4);
        assert_eq!(a.advance(26 * CPU_HZ)[0][11], 5);
        assert!(a.next_deadline().is_none());
    }
}
