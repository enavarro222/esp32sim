//! The CC600's Airoha AG3352Q GNSS receiver on UART0 (the ESP32's TX GPIO1 to it, its output to
//! RX GPIO0), 921600 8N1, NMEA; its power comes from the nRF (`gps_power`).
//!
//! What the open firmware reads (opencc600 `crates/gnss`): GGA then RMC of the same second (one
//! epoch), GSV for the satellites in view, `$PAIR001,<command>,<result>` acknowledges and the
//! `$PAIR011,001` start message. Powered, it sends a sentence set every second, without a fix
//! until [`Gnss::ttff_s`] has passed and a position is set ([`Gnss::command`]).
use crate::periph::CPU_HZ;
use super::wire::Wire;

pub const TX_PIN: u8 = 1;
pub const RX_PIN: u8 = 0;
pub const BAUD: u32 = 921_600;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Position { pub lat: f64, pub lon: f64, pub alt_m: f64, pub speed_kmh: f64, pub course_deg: f64 }

pub struct Gnss {
    /// Power-on cycle, while powered.
    on_since: Option<u64>,
    /// Seconds from power-on to the first fix.
    pub ttff_s: u64,
    /// Where the receiver is; `None`: no fix.
    pub position: Option<Position>,
    /// UTC in Unix seconds at cycle 0; 0 until the host's clock is known.
    clock_at_zero: i64,
    next_epoch: u64,
    line: Vec<u8>,
    wire: Wire,
    pub epochs: u64,
    pub commands: u64,
}

impl Default for Gnss { fn default() -> Self { Self::new() } }

/// `$body*CS\r\n`.
pub fn sentence(body: &str) -> String {
    format!("${body}*{:02X}\r\n", body.bytes().fold(0u8, |x, b| x ^ b))
}

fn coord(value: f64, deg_digits: usize, hemis: [char; 2]) -> String {
    let a = value.abs();
    let deg = a.trunc();
    format!("{:0w$}{:08.5},{}", deg as u32, (a - deg) * 60.0, hemis[usize::from(value < 0.0)], w = deg_digits)
}

impl Gnss {
    pub fn new() -> Self {
        Gnss { on_since: None, ttff_s: 3, position: None, clock_at_zero: 0, next_epoch: 0, line: Vec::new(),
               wire: Wire::new(BAUD), epochs: 0, commands: 0 }
    }

    pub fn powered(&self) -> bool { self.on_since.is_some() }

    /// Power from the nRF: on, the receiver announces its start and its first epoch comes a second later.
    pub fn set_power(&mut self, cycle: u64, on: bool) {
        match (on, self.on_since) {
            (true, None) => {
                self.on_since = Some(cycle);
                self.next_epoch = cycle + CPU_HZ;
                self.wire.send(cycle, sentence("PAIR011,001").as_bytes());
            }
            (false, Some(_)) => { self.on_since = None; self.wire.clear(); self.line.clear(); }
            _ => {}
        }
    }

    /// `gnss LAT LON [ALT_M [SPEED_KMH [COURSE_DEG]]]`, `gnss nofix`, `gnss ttff SECONDS`.
    pub fn command(&mut self, args: &[&str]) -> Result<(), String> {
        let num = |s: &str| s.parse::<f64>().map_err(|_| format!("gnss: not a number: {s}"));
        match args {
            ["nofix"] => self.position = None,
            ["ttff", s] => self.ttff_s = s.parse().map_err(|_| format!("gnss: not a number of seconds: {s}"))?,
            [lat, lon, rest @ ..] if rest.len() <= 3 => {
                let (lat, lon) = (num(lat)?, num(lon)?);
                if !(-90.0..=90.0).contains(&lat) || !(-180.0..=180.0).contains(&lon) { return Err("gnss: position out of range".into()); }
                let mut more = [0.0; 3];
                for (m, s) in more.iter_mut().zip(rest) { *m = num(s)?; }
                self.position = Some(Position { lat, lon, alt_m: more[0], speed_kmh: more[1].max(0.0), course_deg: more[2].rem_euclid(360.0) });
            }
            _ => return Err("gnss LAT LON [ALT_M [SPEED_KMH [COURSE_DEG]]] | gnss nofix | gnss ttff SECONDS".into()),
        }
        Ok(())
    }

    /// Bytes from the ESP32: `$PAIRnnn,...` commands, each acknowledged as done.
    pub fn receive(&mut self, cycle: u64, byte: u8) {
        if !self.powered() { return; }
        if byte != b'\n' { if self.line.len() < 128 { self.line.push(byte); } return; }
        let line = String::from_utf8_lossy(&self.line).into_owned();
        self.line.clear();
        let Some(number) = line.strip_prefix("$PAIR").and_then(|r| r.get(..3)).filter(|n| n.bytes().all(|b| b.is_ascii_digit())) else { return };
        self.commands += 1;
        self.wire.send(cycle, sentence(&format!("PAIR001,{number},0")).as_bytes());
    }

    fn utc(&mut self, cycle: u64) -> i64 {
        if self.clock_at_zero == 0 {
            self.clock_at_zero = (esp_soc::host::unix_time_ms() / 1000) as i64 - (cycle / CPU_HZ) as i64;
        }
        self.clock_at_zero + (cycle / CPU_HZ) as i64
    }

    /// The epochs due by `cycle`.
    pub fn advance(&mut self, cycle: u64) {
        let Some(on) = self.on_since else { return };
        while self.next_epoch <= cycle {
            let at = self.next_epoch;
            self.next_epoch += CPU_HZ;
            let fix = self.position.filter(|_| at - on >= self.ttff_s * CPU_HZ);
            let t = self.utc(at);
            let (days, s) = (t.div_euclid(86_400), t.rem_euclid(86_400));
            let (y, m, d) = super::nrf::civil(days);
            let time = format!("{:02}{:02}{:02}.000", s / 3600, s / 60 % 60, s % 60);
            let date = format!("{d:02}{m:02}{:02}", y % 100);
            let mut out = String::new();
            match fix {
                Some(p) => {
                    let (lat, lon) = (coord(p.lat, 2, ['N', 'S']), coord(p.lon, 3, ['E', 'W']));
                    out += &sentence(&format!("GNGGA,{time},{lat},{lon},1,12,0.80,{:.1},M,48.0,M,,", p.alt_m));
                    out += &sentence(&format!("GNRMC,{time},A,{lat},{lon},{:.3},{:.2},{date},,,A", p.speed_kmh / 1.852, p.course_deg));
                }
                None => {
                    out += &sentence(&format!("GNGGA,{time},,,,,0,00,99.99,,,,,,"));
                    out += &sentence(&format!("GNRMC,{time},V,,,,,,,{date},,,N"));
                }
            }
            if self.epochs.is_multiple_of(5) {
                let n = if fix.is_some() { 12 } else { 4 };
                out += &sentence(&format!("GPGSV,1,1,{n:02}"));
            }
            self.epochs += 1;
            self.wire.send(at, out.as_bytes());
        }
    }

    pub fn transmit(&mut self, cycle: u64) -> Vec<u8> { self.wire.transmit(cycle) }

    pub fn next_deadline(&self) -> Option<u64> {
        let epoch = self.on_since.map(|_| self.next_epoch);
        match (epoch, self.wire.next_deadline()) { (Some(a), Some(b)) => Some(a.min(b)), (a, b) => a.or(b) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lines(g: &mut Gnss) -> Vec<String> {
        let bytes = std::iter::from_fn(|| Some(g.transmit(u64::MAX)).filter(|d| !d.is_empty())).flatten().collect::<Vec<u8>>();
        String::from_utf8(bytes).unwrap().lines().map(str::to_string).collect()
    }

    #[test]
    fn checksums_and_coordinates_are_nmea() {
        assert_eq!(sentence("PAIR011,001"), "$PAIR011,001*27\r\n");
        assert_eq!(coord(45.0641, 2, ['N', 'S']), "4503.84600,N");
        assert_eq!(coord(-6.4078, 3, ['E', 'W']), "00624.46800,W");
    }

    #[test]
    fn powered_it_starts_then_fixes_and_acknowledges_commands() {
        let mut g = Gnss::new();
        g.command(&["45.0641", "6.4078", "2642", "18", "90"]).unwrap();
        g.set_power(0, true);
        assert_eq!(lines(&mut g), ["$PAIR011,001*27"]);
        g.advance(CPU_HZ);
        let first = lines(&mut g);
        assert!(first[0].starts_with("$GNGGA,") && first[0].contains(",0,00,"), "{first:?}");
        assert!(first[1].contains(",V,"));
        assert!(first[2].starts_with("$GPGSV,1,1,04"));
        g.advance(3 * CPU_HZ);
        let fixed = lines(&mut g);
        let rmc = fixed.iter().rfind(|l| l.starts_with("$GNRMC")).unwrap();
        assert!(rmc.contains(",A,4503.84600,N,00624.46800,E,9.719,90.00,"), "{rmc}");
        assert!(fixed.iter().any(|l| l.contains(",1,12,0.80,2642.0,M,")));
        for b in b"$PAIR062,0,1*3F\r\n" { g.receive(0, *b); }
        assert_eq!(lines(&mut g), [sentence("PAIR001,062,0").trim_end()]);
        g.set_power(10, false);
        assert_eq!(g.next_deadline(), None);
    }

    #[test]
    fn commands_are_checked() {
        let mut g = Gnss::new();
        assert!(g.command(&["91", "0"]).is_err());
        assert!(g.command(&["x"]).is_err());
        g.command(&["ttff", "30"]).unwrap();
        assert_eq!(g.ttff_s, 30);
        g.command(&["1", "2"]).unwrap();
        g.command(&["nofix"]).unwrap();
        assert_eq!(g.position, None);
    }
}
