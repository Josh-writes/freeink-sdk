#pragma once

// UC8179 panel driver — Seeed reTerminal E1001 (7.5" 800x480 B/W).
//
// Deliberately separate from Uc8179Driver (the Xteink X4 family): the E1001's glass is a
// plain 7.5" UC8179 module driven the GxEPD2 way — host programs PWR (0x01) and BTST,
// the panel is addressed 800x480 (no 600-gate padding), OTP waveforms, 1-bpp KW mode
// (DTM1 = OLD plane, DTM2 = NEW plane). The register sequence and refresh flow are the
// ones proven on a real unit by the e1001-probe bring-up sketch (full refresh 1.6 s).
//
// Single-buffer only: after every refresh the OLD plane is re-synced with the frame that
// was just shown, so the next fast refresh diffs against it. No grayscale.
//
// BUSY_N: low while busy, high when idle (BusyPolarity::UcIdleHigh).

#include "PanelDriver.h"

namespace freeink {

PanelDriver& uc8179E1001Driver();

// Runtime tuning from the app's Settings.
//   framesPerPhase: 0 = the panel's built-in partial waveform; otherwise a register-loaded waveform with
//                   that many frames per drive phase (about 20 ms each; shorter is faster but greyer).
//   vcom:           VCOM DC register value used with the register waveform, or -1 to leave it alone.
//   fullEvery:      partial updates between clean-up refreshes.
//   softClean:      true = clean up with the fast waveform on every pixel (no black flash);
//                   false = the full flashing refresh.
// Timing probe: the app stamps these (millis()) when a key arrives and when it starts drawing.
extern volatile uint32_t inputMs, updateMs;

void uc8179E1001Tune(uint8_t framesPerPhase, int vcom, uint16_t fullEvery, bool softClean);

}  // namespace freeink
