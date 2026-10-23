//! GEOID CC600 bike computer (a Magene C606 board without eMMC or touch):
//!   - ST7789 240×320 on the LCD_CAM i8080 16-bit bus: CS 2, DC 40, WR 3, D0–D15 on
//!     4 38 5 37 6 36 7 35 8 34 9 33 10 47 11 48
//!   - backlight LEDC on GPIO 45 (unmodelled, harmless)
//!   - an Airoha AG3352Q GNSS receiver on UART0 (TX 1, RX 0, through UHCI): [`gnss::Gnss`], powered
//!     from reset, switched off and restarted by the nRF; `board gnss LAT LON [ALT [KMH [COURSE]]]`
//!     sets where it is
//!   - the keys, RTC, battery, barometer and light sensor sit behind an nRF52 on UART2
//!     (TX 42, RX 41): [`nrf::Nrf`]. Its keys are the pseudo pins `key0` (top left), `key1`
//!     (bottom left) and `key2` (bottom right), 64..=66, for `press` and the UI's buttons.
//!     `board usb on|off` plugs or unplugs the USB power, `board battery PERCENT [MV]` sets the
//!     battery; the nRF power-cycles the ESP32 (see [`nrf::Nrf`]).
//!
//! Pins from the open firmware (github.com/enavarro222/opencc600, `firmware/src/bin/main.rs`).
use esp_soc::board::{BoardModel, VirtualCycle};
use esp_soc::devices::DcsPanel;
use esp_soc::uart::{UartInput, UartRoute};

pub mod gnss;
pub mod nrf;
pub mod wire;

/// Pseudo pin of key 0; keys 1 and 2 follow.
pub const PIN_KEY0: u8 = 64;

pub struct Cc600 { pub panel: DcsPanel, pub nrf: nrf::Nrf, pub gnss: gnss::Gnss,
                   /// The nRF's GNSS power commands already applied; `None` before the first poll.
                   gps_seen: Option<u32> }
impl Default for Cc600 { fn default() -> Self { Self::new() } }

impl Cc600 {
    pub fn new() -> Self {
        Cc600 { panel: DcsPanel::st7789(), nrf: nrf::Nrf::new((esp_soc::host::unix_time_ms() / 1000) as i64), gnss: gnss::Gnss::new(), gps_seen: None }
    }
}
impl BoardModel for Cc600 {
    fn name(&self) -> &'static str { "cc600" }
    fn uses_gpio_edges(&self) -> bool { false }
    fn uses_uart_pins(&self) -> bool { true }
    fn uart_tx(&mut self, cycle: VirtualCycle, route: UartRoute, byte: u8) {
        if route.transmits_on(nrf::TX_PIN) && route.matches_baud(nrf::BAUD) { self.nrf.receive(cycle, byte); }
        if route.transmits_on(gnss::TX_PIN) && route.matches_baud(gnss::BAUD) { self.gnss.receive(cycle, byte); }
    }
    fn uart_rx(&mut self, cycle: VirtualCycle) -> Vec<UartInput> {
        self.nrf.advance(cycle);
        // powered from reset (opencc600 docs/REVERSE_NOTES.md, GNSS); gps_power 0 cuts it, 1 restarts it
        match self.gps_seen {
            None => self.gnss.set_power(cycle, true),
            Some(seen) if seen != self.nrf.gps_commands => {
                self.gnss.set_power(cycle, false);
                if self.nrf.gps_power != 0 { self.gnss.set_power(cycle, true); }
            }
            _ => {}
        }
        self.gps_seen = Some(self.nrf.gps_commands);
        self.gnss.advance(cycle);
        let mut inputs = Vec::new();
        for (pin, baud, data) in [(nrf::RX_PIN, nrf::BAUD, self.nrf.transmit(cycle)), (gnss::RX_PIN, gnss::BAUD, self.gnss.transmit(cycle))] {
            if !data.is_empty() { inputs.push(UartInput::new(pin, baud, data)); }
        }
        inputs
    }
    fn next_deadline(&self) -> Option<VirtualCycle> {
        Some(self.gnss.next_deadline().map_or(self.nrf.next_deadline(), |g| g.min(self.nrf.next_deadline())))
    }
    fn command(&mut self, _cycle: VirtualCycle, line: &str) -> Result<(), String> {
        let words: Vec<&str> = line.split_whitespace().collect();
        fn number<T: std::str::FromStr>(w: &str) -> Result<T, String> { w.parse().map_err(|_| format!("cc600: bad number {w:?}")) }
        match words.as_slice() {
            ["usb", on @ ("on" | "off")] => { self.nrf.power_source = u8::from(*on == "off"); return Ok(()); }
            ["battery", percent, mv @ ..] => {
                self.nrf.battery_percent = number(percent)?;
                if let [mv] = mv { self.nrf.battery_mv = number(mv)?; }
                return Ok(());
            }
            ["gnss", args @ ..] => self.gnss.command(args)?,
            _ => return Err(format!("cc600: unknown command {line:?} (gnss ..., usb on|off, battery PERCENT [MV])")),
        }
        // the barometer follows the receiver's altitude
        if let Some(p) = self.gnss.position { self.nrf.altitude_m = p.alt_m; }
        Ok(())
    }
    fn take_power_cycle(&mut self) -> bool { std::mem::take(&mut self.nrf.power_cycle) }
    fn host_input(&mut self, cycle: VirtualCycle, pin: u8, level: bool) -> bool {
        let Some(key) = pin.checked_sub(PIN_KEY0).filter(|&k| k < 3) else { return false };
        self.nrf.key(cycle, key as usize, !level);
        true
    }
    fn named_pin(&self, name: &str) -> Option<u8> {
        match name { "key0" | "top-left" => Some(PIN_KEY0), "key1" | "bottom-left" => Some(PIN_KEY0 + 1),
                     "key2" | "bottom-right" => Some(PIN_KEY0 + 2), _ => None }
    }
    fn lcd_i80(&mut self, cmd: Option<u32>, data: &[u8], wide: bool) {
        let p = &mut self.panel;
        if let Some(c) = cmd { p.dc = false; p.byte(c as u8); }
        if wide { for w in data.as_chunks::<2>().0 { p.word(u16::from_le_bytes(*w)); } }
        else { p.dc = true; for &b in data { p.byte(b); } }
    }
    fn display(&self) -> Option<(u32, u32, Vec<u16>, u64)> { Some((240, 320, self.panel.gram.clone(), self.panel.pixels_written)) }
    fn display_version(&self) -> u64 { self.panel.pixels_written }
    fn display_quiet_push(&self) -> bool { true }
    fn display_frames(&self) -> u64 { self.panel.frames }
    fn gram(&self) -> Option<(Vec<u16>, usize, usize)> { Some((self.panel.gram.clone(), self.panel.cols, self.panel.rows)) }
    fn report(&self) -> String {
        let p = &self.panel;
        let n = &self.nrf;
        let g = &self.gnss;
        format!("[emu] panel: {} RAMWR, {} pixels, madctl={:#x} inverted={} on={} bbox={:?}\n[emu] nrf: {} frames in, {} out, {} key events, gps power {}{}\n[emu] gnss: {} epochs, {} $PAIR commands, {}",
            p.frames, p.pixels_written, p.madctl, p.inverted, p.on, p.bbox(), n.frames_in, n.frames_out, n.key_events, n.gps_power,
            if n.powered_off { ", powered off" } else { "" }, g.epochs, g.commands,
            g.position.map_or("no position".into(), |p| format!("at {:.5} {:.5}", p.lat, p.lon)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// What esp-hal's `I8080::send` hands over for the firmware's window + RAMWR: 8-bit
    /// parameters, then the 16-bit little-endian frame buffer.
    #[test]
    fn the_receiver_runs_from_reset_and_the_nrf_restarts_or_cuts_it() {
        let mut b = Cc600::new();
        // a poll applies the nRF's commands, the next one gets the bytes on the wire by then
        let starts = |b: &mut Cc600, cycle: u64| [cycle, cycle + 2_400_000].iter().flat_map(|&c| b.uart_rx(c))
            .filter(|i| i.pin == gnss::RX_PIN).map(|i| String::from_utf8_lossy(&i.data).matches("PAIR011,001").count()).sum::<usize>();
        assert_eq!(starts(&mut b, 100_000_000), 1, "started at reset");
        for b2 in nrf::frame(2, 0x10, &[0xE2, 0x02, 0x07, 0, 0, 1, 0, 0]) { b.nrf.receive(0, b2); }
        assert_eq!(starts(&mut b, 200_000_000), 1, "gps_power(1) restarts it");
        for b2 in nrf::frame(2, 0x10, &[0xE2, 0x02, 0x07, 0, 0, 0, 0, 0]) { b.nrf.receive(0, b2); }
        b.uart_rx(300_000_000);
        assert!(!b.gnss.powered());
    }

    #[test]
    fn i8080_transfers_draw_on_the_panel() {
        let mut b = Cc600::new();
        b.lcd_i80(Some(0x29), &[], false);
        b.lcd_i80(Some(0x2a), &[0, 0, 0, 239], false);
        b.lcd_i80(Some(0x2b), &[0, 10, 0, 10], false);
        b.lcd_i80(Some(0x2c), &[0x00, 0xf8, 0xe0, 0x07], true);
        let (w, h, px, _) = b.display().unwrap();
        assert_eq!((w, h, px[10 * 240], px[10 * 240 + 1]), (240, 320, 0xf800, 0x07e0));
        assert!(b.panel.on);
    }
}
