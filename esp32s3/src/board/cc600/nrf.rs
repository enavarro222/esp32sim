//! The CC600's nRF52 co-processor, as the ESP32-S3 sees it on UART2 (TX GPIO42, RX GPIO41,
//! 115200 8N1): the keys, the battery, the RTC, the barometer and the ambient light.
//!
//! Frame: `A5 N 6F F1 TYPE CMD payload[N-4] CRClo CRChi`, CRC-16/XMODEM over everything before
//! the CRC; type 1 query, 2 set (ESP32 → nRF), 4 report, 5 acknowledge (nRF → ESP32). Layouts and
//! rates from the open firmware's `crates/nrf-link` (github.com/enavarro222/opencc600), read on
//! a CC600: battery and power source every 300 ms, RTC every 5 s, barometer at 5 Hz, light at
//! 3.3 Hz. The stream runs from reset, as after the power key's power-on gesture. ANT+ answers
//! the commands but finds no sensor.
use crate::periph::CPU_HZ;
use super::wire::Wire;

pub const TX_PIN: u8 = 42;
pub const RX_PIN: u8 = 41;
pub const BAUD: u32 = 115_200;
const MS: u64 = CPU_HZ / 1000;
/// The first RTC report, once the firmware listens: at 0 it is lost and the clock waits 5 s more.
const RTC_FIRST: u64 = 1000 * MS;

pub const KEY_HELD: u64 = 500 * MS;
pub const KEY_REPEAT: u64 = 250 * MS;
pub const KEY_CLICK: u8 = 1;
pub const KEY_LONG: u8 = 4;
pub const KEY_LONG_RELEASE: u8 = 5;

pub fn crc16(data: &[u8]) -> u16 {
    data.iter().fold(0u16, |mut crc, &b| {
        crc ^= u16::from(b) << 8;
        for _ in 0..8 { crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x1021 } else { crc << 1 }; }
        crc
    })
}

pub fn frame(kind: u8, cmd: u8, payload: &[u8]) -> Vec<u8> {
    let mut f = vec![0xA5, payload.len() as u8 + 4, 0x6F, 0xF1, kind, cmd];
    f.extend_from_slice(payload);
    let crc = crc16(&f);
    f.extend_from_slice(&crc.to_le_bytes());
    f
}

/// Days since 1970-01-01 to (year, month 1..=12, day) (H. Hinnant's `civil_from_days`).
pub(super) fn civil(days: i64) -> (i64, u8, u8) {
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    (yoe + era * 400 + i64::from(m <= 2), m as u8, (doy - (153 * mp + 2) / 5 + 1) as u8)
}

fn days(y: i64, m: u8, d: u8) -> i64 {
    let (y, m) = if m <= 2 { (y - 1, i64::from(m) + 9) } else { (y, i64::from(m) - 3) };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    era * 146_097 + yoe * 365 + yoe / 4 - yoe / 100 + (153 * m + 2) / 5 + i64::from(d) - 1 - 719_468
}

#[derive(Default, Clone, Copy)]
struct Key { down: Option<u64>, next_long: Option<u64>, long: bool }

pub struct Nrf {
    /// RTC in Unix seconds at cycle 0; 0 until the host's clock is known (the browser build gives
    /// it with the first run slice, after the board exists).
    rtc_at_zero: i64,
    pub battery_mv: u16,
    pub battery_percent: u8,
    /// 0 USB powered, 1 on battery.
    pub power_source: u8,
    /// Barometer: pressure from this altitude in the standard atmosphere.
    pub altitude_m: f64,
    pub centi_celsius: i16,
    pub light: u16,
    /// Last `gps_power` value: 0 off, 1 on (a restart of a running receiver), and how many came.
    pub gps_power: u8,
    pub gps_commands: u32,
    pub powered_off: bool,
    rx: Vec<u8>,
    wire: Wire,
    keys: [Key; 3],
    /// Next report of each periodic stream: battery + source, RTC, barometer, light.
    next: [u64; 4],
    light_step: usize,
    ant_scan_end: Option<u64>,
    pub frames_in: u64,
    pub frames_out: u64,
    pub key_events: u64,
}

impl Nrf {
    pub fn new(unix_now: i64) -> Self {
        let mut n = Nrf {
            rtc_at_zero: unix_now, battery_mv: 3950, battery_percent: 80, power_source: 1, altitude_m: 150.0,
            centi_celsius: 2000, light: 2500, gps_power: 0, gps_commands: 0, powered_off: false, rx: Vec::new(), wire: Wire::new(BAUD),
            keys: [Key::default(); 3], next: [0, RTC_FIRST, 0, 0], light_step: 0, ant_scan_end: None,
            frames_in: 0, frames_out: 0, key_events: 0,
        };
        n.send(0, frame(4, 0x01, &[0x01, 0x01, 0, 0, 0, 0, 0, 0, 0x02, 0x13]));
        n
    }

    pub fn unix(&self, cycle: u64) -> i64 { self.rtc_at_zero + (cycle / CPU_HZ) as i64 }

    fn send(&mut self, cycle: u64, f: Vec<u8>) {
        self.wire.send(cycle, &f);
        self.frames_out += 1;
    }

    fn report(&mut self, cycle: u64, cmd: u8, payload: [u8; 8]) { self.send(cycle, frame(4, cmd, &payload)); }

    fn rtc_frame(&self, cycle: u64) -> [u8; 8] {
        let t = self.unix(cycle);
        let (d, s) = (t.div_euclid(86_400), t.rem_euclid(86_400));
        let (y, m, day) = civil(d);
        let weekday = (d + 4).rem_euclid(7) as u8;
        // the nRF's own frames carry month - 1
        [0x53, 0xFF, (s % 60) as u8, (s / 60 % 60) as u8, (s / 3600) as u8, day | weekday << 5, m - 1, (y - 1900) as u8]
    }

    fn pressure_centi_pa(&self) -> u32 {
        (101_325.0 * (1.0 - 2.255_77e-5 * self.altitude_m).powf(5.255_88) * 100.0) as u32
    }

    /// Bytes from the ESP32 on [`TX_PIN`].
    pub fn receive(&mut self, cycle: u64, byte: u8) {
        self.rx.push(byte);
        loop {
            let b = &self.rx;
            let header = !b.is_empty() && b[0] == 0xA5 && b.get(1).is_none_or(|&n| (4..=0x84).contains(&n))
                && b.get(2).is_none_or(|&m| m == 0x6F) && b.get(4).is_none_or(|&t| (1..=5).contains(&t));
            if !header { if b.is_empty() { return; } self.rx.remove(0); continue; }
            if b.len() < 2 || b.len() < b[1] as usize + 4 { return; }
            let total = b[1] as usize + 4;
            if crc16(&b[..total - 2]) != u16::from_le_bytes([b[total - 2], b[total - 1]]) { self.rx.remove(0); continue; }
            let f: Vec<u8> = self.rx.drain(..total).collect();
            self.frames_in += 1;
            self.command(cycle, f[4], f[5], &f[6..total - 2]);
        }
    }

    fn command(&mut self, cycle: u64, kind: u8, cmd: u8, p: &[u8]) {
        let at = |i: usize| p.get(i).copied().unwrap_or(0);
        let ack = |grp| [at(0), grp, 0x01, 0, 0, 0, 0, 0];
        match (kind, cmd, at(0), at(1)) {
            // power-on reason query: 4, the power key
            (1, 0x10, 0xE2, 0x02) => self.report(cycle, 0x10, [0xE2, 0x02, 0, 0, 0xFF, 0x04, 0, 0xFF]),
            (2, 0x10, 0xE2, grp) => {
                match (grp, at(2), at(5)) {
                    (0x02, 0x00, 0) => self.powered_off = true,
                    (0x02, 0x07, v) => { self.gps_power = v; self.gps_commands += 1; }
                    _ => {},
                }
                self.send(cycle, frame(5, 0x10, &ack(grp)));
            }
            (2, 0x00, 0x53, _) => {
                let t = days(1900 + i64::from(at(7)), at(6), at(5) & 0x1F) * 86_400
                    + i64::from(at(4)) * 3600 + i64::from(at(3)) * 60 + i64::from(at(2));
                self.rtc_at_zero = t - (cycle / CPU_HZ) as i64;
                self.send(cycle, frame(5, 0x00, &ack(at(1))));
            }
            (2, 0x10, 0xE1, _) => {
                let seconds = u64::from(u16::from_le_bytes([at(3), at(4)]));
                self.ant_scan_end = Some(cycle + seconds * CPU_HZ);
                self.send(cycle, frame(5, 0x10, &ack(0x02)));
            }
            (2, c, _, _) => self.send(cycle, frame(5, c, &[0; 8])),
            _ => {},
        }
    }

    /// A key press (`down`) or release, 0 top left, 1 bottom left, 2 bottom right.
    pub fn key(&mut self, cycle: u64, key: usize, down: bool) {
        let Some(k) = self.keys.get_mut(key) else { return };
        if down {
            if k.down.is_none() { *k = Key { down: Some(cycle), next_long: Some(cycle + KEY_HELD), long: false }; }
            return;
        }
        if k.down.is_none() { return; }
        self.keys_due(cycle);
        let k = &mut self.keys[key];
        (k.down, k.next_long) = (None, None);
        let event = if k.long { KEY_LONG_RELEASE } else { KEY_CLICK };
        self.key_frame(cycle, key, event);
    }

    fn key_frame(&mut self, cycle: u64, key: usize, event: u8) {
        self.key_events += 1;
        self.report(cycle, 0x10, [0x49, key as u8, 0, 0, 0, 0, event, 0x80]);
    }

    fn keys_due(&mut self, cycle: u64) {
        for key in 0..3 {
            while let Some(at) = self.keys[key].next_long.filter(|&at| at <= cycle) {
                self.keys[key].long = true;
                self.keys[key].next_long = Some(at + KEY_REPEAT);
                self.key_frame(at, key, KEY_LONG);
            }
        }
    }

    /// Reports due by `cycle`.
    pub fn advance(&mut self, cycle: u64) {
        if self.rtc_at_zero == 0 {
            self.rtc_at_zero = (esp_soc::host::unix_time_ms() / 1000) as i64 - (cycle / CPU_HZ) as i64;
        }
        self.keys_due(cycle);
        if self.ant_scan_end.is_some_and(|at| at <= cycle) {
            self.ant_scan_end = None;
            self.report(cycle, 0x10, [0xF3, 0x03, 0, 0, 0, 0, 0, 0]);
        }
        if self.powered_off { return; }
        const PERIOD: [u64; 4] = [300 * MS, 5000 * MS, 200 * MS, 300 * MS];
        for (i, period) in PERIOD.iter().enumerate() {
            while self.next[i] <= cycle {
                let at = self.next[i];
                self.next[i] += period;
                match i {
                    0 => {
                        let [lo, hi] = self.battery_mv.to_le_bytes();
                        self.report(at, 0x00, [0x52, 0xFF, 0xFF, 0xFF, lo, hi, self.battery_percent, 0xFF]);
                        self.report(at, 0x10, [0xF0, 0x02, self.power_source, 0, 0, 0, 0, 0]);
                    }
                    1 => { let f = self.rtc_frame(at); self.report(at, 0x00, f); }
                    2 => {
                        let [t0, t1] = self.centi_celsius.to_le_bytes();
                        let [p0, p1, p2, p3] = self.pressure_centi_pa().to_le_bytes();
                        self.report(at, 0x10, [0xF1, 0x03, t0, t1, p0, p1, p2, p3]);
                    }
                    _ => {
                        // the ambient light (600), then two channels that stay dark indoors
                        let param: u32 = [600, 2000, 3000][self.light_step];
                        let level = if param == 600 { self.light } else { 5 };
                        self.light_step = (self.light_step + 1) % 3;
                        let [a, b, c, d] = param.to_le_bytes();
                        let [l0, l1] = level.to_le_bytes();
                        self.report(at, 0x10, [0xF1, 0x04, a, b, c, d, l0, l1]);
                    }
                }
            }
        }
    }

    /// Bytes on the wire by `cycle`.
    pub fn transmit(&mut self, cycle: u64) -> Vec<u8> { self.wire.transmit(cycle) }

    pub fn next_deadline(&self) -> u64 {
        let mut at = self.keys.iter().filter_map(|k| k.next_long).chain(self.ant_scan_end).min().unwrap_or(u64::MAX);
        if !self.powered_off { at = at.min(*self.next.iter().min().unwrap()); }
        if let Some(wire) = self.wire.next_deadline() { at = at.min(wire); }
        at
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frames(mut bytes: &[u8]) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        while bytes.len() >= 2 {
            let total = bytes[1] as usize + 4;
            assert_eq!(crc16(&bytes[..total - 2]).to_le_bytes(), [bytes[total - 2], bytes[total - 1]]);
            out.push(bytes[..total].to_vec());
            bytes = &bytes[total..];
        }
        out
    }

    fn drain(n: &mut Nrf, cycle: u64) -> Vec<Vec<u8>> {
        n.advance(cycle);
        let mut all = Vec::new();
        loop { let d = n.transmit(u64::MAX); if d.is_empty() { break } all.extend(d); }
        frames(&all)
    }

    /// The frames of the CC600 captures in opencc600's `crates/nrf-link/tests/frames.rs`.
    #[test]
    fn reports_match_the_captured_frames() {
        // 2026-09-17 12:58:05 UTC, a Thursday
        let mut n = Nrf::new(1_789_649_884);
        n.battery_mv = 4343;
        n.battery_percent = 100;
        n.power_source = 0;
        let f = drain(&mut n, RTC_FIRST);
        assert!(f.iter().any(|f| f[..14] == [0xA5, 0x0C, 0x6F, 0xF1, 0x04, 0x00, 0x52, 0xFF, 0xFF, 0xFF, 0xF7, 0x10, 0x64, 0xFF]));
        assert!(f.iter().any(|f| f[..14] == [0xA5, 0x0C, 0x6F, 0xF1, 0x04, 0x00, 0x53, 0xFF, 0x05, 0x3A, 0x0C, 0x91, 0x08, 0x7E]));
        assert!(f.iter().any(|f| f[..14] == [0xA5, 0x0C, 0x6F, 0xF1, 0x04, 0x10, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]));
        assert_eq!(f[0][..16], [0xA5, 0x0E, 0x6F, 0xF1, 0x04, 0x01, 0x01, 0x01, 0, 0, 0, 0, 0, 0, 0x02, 0x13]);
    }

    #[test]
    fn answers_the_power_on_handshake_and_takes_the_rtc() {
        let mut n = Nrf::new(0);
        drain(&mut n, 0);
        // power_on, check_power_on_reason, set_rtc 2026-09-17 12:58:05, as the firmware builds them
        for f in [frame(2, 0x10, &[0xE2, 0x02, 0, 0, 0, 1, 0, 0]), frame(1, 0x10, &[0xE2, 0x02, 0, 0, 0, 1, 0, 0]),
                  frame(2, 0x00, &[0x53, 0xFF, 5, 58, 12, 17 | 4 << 5, 9, 126])] {
            for b in f { n.receive(10, b); }
        }
        let f = drain(&mut n, 10);
        assert_eq!(f[0][4..9], [0x05, 0x10, 0xE2, 0x02, 0x01]);
        assert_eq!(f[1][4..14], [0x04, 0x10, 0xE2, 0x02, 0x00, 0x00, 0xFF, 0x04, 0x00, 0xFF]);
        assert_eq!(f[2][4], 0x05);
        assert_eq!(n.unix(10 + 2 * CPU_HZ), 1_789_649_887);
        assert_eq!(n.frames_in, 3);
    }

    #[test]
    fn keys_click_or_repeat_while_held() {
        let mut n = Nrf::new(0);
        n.powered_off = true;
        drain(&mut n, 0);
        let key = |f: &Vec<u8>| (f[7], f[12]);
        n.key(0, 2, true);
        n.key(100 * MS, 2, false);
        assert_eq!(drain(&mut n, 100 * MS).iter().map(key).collect::<Vec<_>>(), [(2, KEY_CLICK)]);
        n.key(MS * 1000, 0, true);
        assert!(drain(&mut n, MS * 1499).is_empty());
        assert_eq!(n.next_deadline(), MS * 1500);
        n.key(MS * 2100, 0, false);
        let got: Vec<_> = drain(&mut n, MS * 2100).iter().map(key).collect();
        assert_eq!(got, [(0, KEY_LONG), (0, KEY_LONG), (0, KEY_LONG), (0, KEY_LONG_RELEASE)]);
    }

    #[test]
    fn the_rtc_starts_from_the_host_clock_once_known() {
        let mut n = Nrf::new(0);
        n.advance(2 * CPU_HZ);
        let host = (esp_soc::host::unix_time_ms() / 1000) as i64;
        assert!((n.unix(2 * CPU_HZ) - host).abs() <= 1);
    }

    #[test]
    fn pressure_follows_altitude() {
        let mut n = Nrf::new(0);
        n.altitude_m = 0.0;
        assert_eq!(n.pressure_centi_pa(), 10_132_500);
        n.altitude_m = 1000.0;
        assert!((8_987_000..8_988_000).contains(&n.pressure_centi_pa()));
    }
}
