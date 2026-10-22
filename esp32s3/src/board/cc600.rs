//! GEOID CC600 bike computer (a Magene C606 board without eMMC or touch):
//!   - ST7789 240×320 on the LCD_CAM i8080 16-bit bus: CS 2, DC 40, WR 3, D0–D15 on
//!     4 38 5 37 6 36 7 35 8 34 9 33 10 47 11 48
//!   - backlight LEDC on GPIO 45 (unmodelled, harmless)
//!   - the keys, RTC, battery, barometer and light sensor sit behind an nRF52 on UART2
//!     (TX 42, RX 41): [`nrf::Nrf`]. Its keys are the pseudo pins `key0` (top left), `key1`
//!     (bottom left) and `key2` (bottom right), 64..=66, for `press` and the UI's buttons.
//!
//! Pins from the open firmware (github.com/enavarro222/opencc600, `firmware/src/bin/main.rs`).
use esp_soc::board::{BoardModel, VirtualCycle};
use esp_soc::devices::DcsPanel;
use esp_soc::uart::{UartInput, UartRoute};

pub mod nrf;

/// Pseudo pin of key 0; keys 1 and 2 follow.
pub const PIN_KEY0: u8 = 64;

pub struct Cc600 { pub panel: DcsPanel, pub nrf: nrf::Nrf }
impl Default for Cc600 { fn default() -> Self { Self::new() } }

impl Cc600 {
    pub fn new() -> Self {
        Cc600 { panel: DcsPanel::st7789(), nrf: nrf::Nrf::new((esp_soc::host::unix_time_ms() / 1000) as i64) }
    }
}
impl BoardModel for Cc600 {
    fn name(&self) -> &'static str { "cc600" }
    fn uses_gpio_edges(&self) -> bool { false }
    fn uses_uart_pins(&self) -> bool { true }
    fn uart_tx(&mut self, cycle: VirtualCycle, route: UartRoute, byte: u8) {
        if route.transmits_on(nrf::TX_PIN) && route.matches_baud(nrf::BAUD) { self.nrf.receive(cycle, byte); }
    }
    fn uart_rx(&mut self, cycle: VirtualCycle) -> Vec<UartInput> {
        self.nrf.advance(cycle);
        let data = self.nrf.transmit(cycle);
        if data.is_empty() { Vec::new() } else { vec![UartInput::new(nrf::RX_PIN, nrf::BAUD, data)] }
    }
    fn next_deadline(&self) -> Option<VirtualCycle> { Some(self.nrf.next_deadline()) }
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
        format!("[emu] panel: {} RAMWR, {} pixels, madctl={:#x} inverted={} on={} bbox={:?}\n[emu] nrf: {} frames in, {} out, {} key events, gps power {}{}",
            p.frames, p.pixels_written, p.madctl, p.inverted, p.on, p.bbox(), n.frames_in, n.frames_out, n.key_events, n.gps_power,
            if n.powered_off { ", powered off" } else { "" })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// What esp-hal's `I8080::send` hands over for the firmware's window + RAMWR: 8-bit
    /// parameters, then the 16-bit little-endian frame buffer.
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
