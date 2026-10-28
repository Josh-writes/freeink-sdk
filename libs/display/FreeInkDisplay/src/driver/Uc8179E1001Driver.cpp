#include "Uc8179E1001Driver.h"

#include <BoardConfig.h>
#if FREEINK_DRIVER_UC8179_E1001

#include <Arduino.h>

#include <esp_heap_caps.h>
#include <stdlib.h>
#include <string.h>

namespace freeink {
namespace {

constexpr uint8_t CMD_PSR = 0x00;
constexpr uint8_t CMD_PWR = 0x01;
constexpr uint8_t CMD_POF = 0x02;
constexpr uint8_t CMD_PON = 0x04;
constexpr uint8_t CMD_BTST = 0x06;
constexpr uint8_t CMD_DSLP = 0x07;
constexpr uint8_t CMD_DTM1 = 0x10;  // OLD plane (KW mode)
constexpr uint8_t CMD_DRF = 0x12;
constexpr uint8_t CMD_DTM2 = 0x13;  // NEW plane (KW mode)
constexpr uint8_t CMD_DUSPI = 0x15;
constexpr uint8_t CMD_CDI = 0x50;
constexpr uint8_t CMD_TCON = 0x60;
constexpr uint8_t CMD_TRES = 0x61;
constexpr uint8_t CMD_PTL = 0x90;
constexpr uint8_t CMD_PTIN = 0x91;
constexpr uint8_t CMD_PTOUT = 0x92;
constexpr uint8_t CMD_CCSET = 0xE0;
constexpr uint8_t CMD_PWS = 0xE3;
constexpr uint8_t CMD_TSSET = 0xE5;

constexpr uint16_t PANEL_W = 800;
constexpr uint16_t PANEL_H = 480;
constexpr uint16_t PANEL_WB = PANEL_W / 8;
constexpr uint32_t PANEL_BYTES = static_cast<uint32_t>(PANEL_WB) * PANEL_H;

constexpr uint8_t TSSET_FULL = 0x5A;
constexpr uint8_t TSSET_PARTIAL = 0x6E;
constexpr uint8_t CMD_VDCS = 0x82;
constexpr uint8_t CMD_LUTC = 0x20;  // VCOM, then 0x21 WW, 0x22 KW, 0x23 WK, 0x24 KK, 0x25 border

// Tuning set from Settings (see uc8179E1001Tune).
uint8_t g_frames = 12;   // register waveform, frames per phase (16 ~ 770 ms, 12 ~ 570 ms); 0 = the built-in 870 ms one
int g_vcom = -1;
uint16_t g_fullEvery = 0xFFFF;   // never clean up: the built-in waveform does not ghost
bool g_cfgChanged = false;
bool g_softClean = true;

class Uc8179E1001Driver : public PanelDriver {
 public:
  uint32_t spiHz() const override {
    return BoardConfig::ACTIVE.displaySpiHz != 0 ? BoardConfig::ACTIVE.displaySpiHz : 4000000;
  }
  BusyPolarity busyPolarity() const override { return BusyPolarity::UcIdleHigh; }
  PanelGeometry geometry() const override { return {PANEL_W, PANEL_H, PANEL_WB, PANEL_BYTES}; }

  void begin(EpdBus& bus) override {
    bus.reset(100);
    bus.waitBusy(" e1001_reset");
    _powered = false;
    _partialMode = false;
    _oldValid = false;
  }

  void deepSleep(EpdBus& bus) override {
    powerOff(bus);
    bus.cmd(CMD_DSLP);
    bus.data(0xA5);
    _oldValid = false;  // RAM contents are lost in deep sleep
  }

  void requestResync(uint8_t) override { _oldValid = false; }
  void skipInitialResync() override {}

  void display(EpdBus& bus, const uint8_t* fb, const uint8_t*, RefreshMode mode, bool turnOff) override {
    if (!fb) return;
    const bool due = _sinceFull >= g_fullEvery;
    if (mode == RefreshMode::Fast && _oldValid && _shown && (!due || g_softClean)) {
      if (due) {
        partial(bus, fb, 0, 0, PANEL_W, PANEL_H, true);
      } else {
        // Send only the part of the screen that changed; the panel stays powered between updates
        // (deepSleep and the full refresh still power it down).
        uint16_t x, y, w, h;
        if (!dirtyBox(fb, x, y, w, h)) return;
        partial(bus, fb, x, y, w, h);
      }
      remember(fb);
      return;
    }
    full(bus, fb);
    remember(fb);
    if (turnOff) powerOff(bus);
  }

  void displayWindow(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, uint16_t x, uint16_t y, uint16_t w,
                     uint16_t h, bool turnOff) override {
    if (!fb) return;
    if (!_oldValid) {
      display(bus, fb, prev, RefreshMode::Full, turnOff);
      return;
    }
    if (_sinceFull >= g_fullEvery) {  // clean-up due: soft or full, whole screen
      display(bus, fb, prev, RefreshMode::Fast, turnOff);
      return;
    }
    partial(bus, fb, x, y, w, h);
    remember(fb);
  }

 private:
  bool _powered = false;
  bool _partialMode = false;
  bool _oldValid = false;
  bool _shown = false;          // _shadow holds the frame the panel is showing
  uint8_t* _shadow = nullptr;   // copy of the last frame sent, to find what changed
  uint16_t _sinceFull = 0;

  void remember(const uint8_t* fb) {
    if (!_shadow) {
      _shadow = static_cast<uint8_t*>(heap_caps_malloc(PANEL_BYTES, MALLOC_CAP_SPIRAM));
      if (!_shadow) _shadow = static_cast<uint8_t*>(malloc(PANEL_BYTES));
    }
    if (_shadow) {
      memcpy(_shadow, fb, PANEL_BYTES);
      _shown = true;
    }
  }

  // Bounding box (byte-aligned columns) of what differs from the last frame; false if nothing did.
  bool dirtyBox(const uint8_t* fb, uint16_t& x, uint16_t& y, uint16_t& w, uint16_t& h) {
    int y0 = -1, y1 = -1, b0 = PANEL_WB, b1 = -1;
    for (uint16_t r = 0; r < PANEL_H; r++) {
      const uint8_t* a = fb + static_cast<uint32_t>(r) * PANEL_WB;
      const uint8_t* b = _shadow + static_cast<uint32_t>(r) * PANEL_WB;
      if (memcmp(a, b, PANEL_WB) == 0) continue;
      if (y0 < 0) y0 = r;
      y1 = r;
      int i = 0;
      while (a[i] == b[i]) i++;
      int j = PANEL_WB - 1;
      while (a[j] == b[j]) j--;
      if (i < b0) b0 = i;
      if (j > b1) b1 = j;
    }
    if (y0 < 0) return false;
    x = static_cast<uint16_t>(b0 * 8);
    w = static_cast<uint16_t>((b1 - b0 + 1) * 8);
    y = static_cast<uint16_t>(y0);
    h = static_cast<uint16_t>(y1 - y0 + 1);
    return true;
  }

  // Register waveform (GxEPD2 GDEY075T7 recipe, shortened): charge-balancing pre-phase, then the drive.
  void writeLut(EpdBus& bus, uint8_t reg, uint8_t levels) {
    bus.cmd(reg);
    for (int i = 0; i < 42; i++) {
      uint8_t b = 0;
      if (i == 0) b = levels;
      else if (i == 1 || i == 3) b = g_frames;
      else if (i == 5) b = 1;
      bus.data(b);
    }
  }
  void loadRegisterWaveform(EpdBus& bus) {
    bus.cmd(CMD_PSR);
    bus.data(0x3F);  // KW mode, LUT from registers
    if (g_vcom >= 0) {
      bus.cmd(CMD_VDCS);
      bus.data(static_cast<uint8_t>(g_vcom));
    }
    bus.cmd(CMD_CDI);
    bus.data(0x39);
    bus.data(0x07);
    writeLut(bus, CMD_LUTC, 0x00);
    writeLut(bus, 0x21, 0x00);
    writeLut(bus, 0x22, 0x5A);
    writeLut(bus, 0x23, 0x84);
    writeLut(bus, 0x24, 0x00);
    writeLut(bus, 0x25, 0x00);
  }

  void initCommon(EpdBus& bus) {
    bus.cmd(CMD_PSR);
    bus.data(0x1F);  // KW mode, OTP LUT, scan up, shift right
    bus.cmd(CMD_PWR);
    bus.data(0x07);
    bus.data(0x07);
    bus.data(0x3F);
    bus.data(0x3F);
    bus.data(0x09);
    bus.cmd(CMD_BTST);
    bus.data(0x17);
    bus.data(0x17);
    bus.data(0x28);
    bus.data(0x17);
    bus.cmd(CMD_TRES);
    bus.data(PANEL_W >> 8);
    bus.data(PANEL_W & 0xFF);
    bus.data(PANEL_H >> 8);
    bus.data(PANEL_H & 0xFF);
    bus.cmd(CMD_DUSPI);
    bus.data(0x00);
    bus.cmd(CMD_CDI);
    bus.data(0x29);
    bus.data(0x07);
    bus.cmd(CMD_TCON);
    bus.data(0x22);
    bus.cmd(CMD_PWS);
    bus.data(0x22);
  }

  void powerOn(EpdBus& bus) {
    if (_powered) return;
    bus.cmd(CMD_PON);
    bus.waitBusy(" e1001_PON");
    _powered = true;
  }

  void powerOff(EpdBus& bus) {
    if (!_powered) return;
    bus.cmd(CMD_POF);
    bus.waitBusy(" e1001_POF");
    _powered = false;
    _partialMode = false;
  }

  // Stream rows [y, y+h) x bytes [xb, xb+cols) of `fb` into plane `ramCmd`, optionally complemented.
  void streamRows(EpdBus& bus, uint8_t ramCmd, const uint8_t* fb, uint16_t xb, uint16_t cols, uint16_t y,
                  uint16_t h, bool invert) {
    uint8_t row[PANEL_WB];
    bus.cmd(ramCmd);
    bus.beginTxn();
    for (uint16_t r = 0; r < h; r++) {
      const uint8_t* src = fb + static_cast<uint32_t>(y + r) * PANEL_WB + xb;
      if (invert) {
        for (uint16_t i = 0; i < cols; i++) row[i] = static_cast<uint8_t>(~src[i]);
        bus.rawWriteBytes(row, cols);
      } else {
        bus.rawWriteBytes(src, cols);
      }
    }
    bus.endTxn();
  }

  void setWindow(EpdBus& bus, uint16_t xs, uint16_t y, uint16_t cols8, uint16_t h) {
    const uint16_t xe = static_cast<uint16_t>((xs + cols8 - 1) | 7);
    const uint16_t ye = static_cast<uint16_t>(y + h - 1);
    bus.cmd(CMD_PTL);
    bus.data(xs >> 8);
    bus.data(xs & 0xFF);
    bus.data(xe >> 8);
    bus.data(xe & 0xFF);
    bus.data(y >> 8);
    bus.data(y & 0xFF);
    bus.data(ye >> 8);
    bus.data(ye & 0xFF);
    bus.data(0x01);
  }

  // Flashing refresh: OLD plane = complement of the new frame, so every pixel is driven.
  void full(EpdBus& bus, const uint8_t* fb) {
    const uint32_t t0 = millis();
    powerOff(bus);  // re-init needs the booster off; also drops partial mode
    initCommon(bus);
    powerOn(bus);
    streamRows(bus, CMD_DTM2, fb, 0, PANEL_WB, 0, PANEL_H, false);
    streamRows(bus, CMD_DTM1, fb, 0, PANEL_WB, 0, PANEL_H, true);
    bus.cmd(CMD_CCSET);
    bus.data(0x02);
    bus.cmd(CMD_TSSET);
    bus.data(TSSET_FULL);
    const uint32_t tStream = millis();
    bus.cmd(CMD_DRF);
    bus.waitRefreshComplete(" e1001_DRF_full");
    streamRows(bus, CMD_DTM1, fb, 0, PANEL_WB, 0, PANEL_H, false);  // OLD := shown frame
    _oldValid = true;
    _sinceFull = 0;
#if defined(MS_EDITOR_PROF)
    Serial.printf("[epd] full %lu ms\n", (unsigned long)(millis() - t0));
#endif
  }

  void partial(EpdBus& bus, const uint8_t* fb, uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool clean = false) {
    const uint32_t t0 = millis();
    if (inputMs) {
#if defined(MS_EDITOR_PROF)
      Serial.printf("[epd] key->draw start %lu ms, draw->driver %lu ms\n", (unsigned long)(updateMs - inputMs), (unsigned long)(t0 - updateMs));
#endif
      inputMs = 0;
    }
    if (g_cfgChanged) {  // a Settings change: re-initialise with the new waveform
      powerOff(bus);
      _partialMode = false;
      g_cfgChanged = false;
    }
    const bool init = !_partialMode;
    if (!_partialMode) {
      initCommon(bus);
      if (g_frames > 0) {
        loadRegisterWaveform(bus);
      } else {
        bus.cmd(CMD_CCSET);
        bus.data(0x02);
        bus.cmd(CMD_TSSET);
        bus.data(TSSET_PARTIAL);
      }
      powerOn(bus);
      _partialMode = true;
    }
    _sinceFull = clean ? 0 : _sinceFull + 1;
    const uint32_t tInit = millis();
    const uint16_t xs = x & ~7u;
    const uint16_t xe = static_cast<uint16_t>((x + w - 1) | 7);
    const uint16_t cols = static_cast<uint16_t>((xe - xs + 1) / 8);

    bus.cmd(CMD_PTIN);
    setWindow(bus, xs, y, static_cast<uint16_t>(cols * 8), h);
    // Soft clean-up: tell the panel every pixel is changing (OLD = complement of the new frame), so each one
    // is driven again to its own colour with the fast waveform, with no full black flash.
    if (clean) streamRows(bus, CMD_DTM1, fb, xs / 8, cols, y, h, true);
    streamRows(bus, CMD_DTM2, fb, xs / 8, cols, y, h, false);
    const uint32_t tStream = millis();
    bus.cmd(CMD_DRF);
    bus.waitRefreshComplete(" e1001_DRF_part");
    const uint32_t tDrf = millis();
    bus.cmd(CMD_PTOUT);

    bus.cmd(CMD_PTIN);
    setWindow(bus, xs, y, static_cast<uint16_t>(cols * 8), h);
    streamRows(bus, CMD_DTM1, fb, xs / 8, cols, y, h, false);  // OLD := shown window
    bus.cmd(CMD_PTOUT);
#if defined(MS_EDITOR_PROF)
    Serial.printf("[epd] part %ux%u@%u,%u total %lu (init %lu stream %lu drf %lu old %lu)%s\n", (unsigned)w, (unsigned)h, (unsigned)x, (unsigned)y,
                  (unsigned long)(millis() - t0), (unsigned long)(tInit - t0), (unsigned long)(tStream - tInit), (unsigned long)(tDrf - tStream), (unsigned long)(millis() - tDrf), init ? " +init" : "");
#endif
  }
};

}  // namespace

volatile uint32_t inputMs = 0, updateMs = 0;

PanelDriver& uc8179E1001Driver() {
  static Uc8179E1001Driver instance;
  return instance;
}

void uc8179E1001Tune(uint8_t framesPerPhase, int vcom, uint16_t fullEvery, bool softClean) {
  g_softClean = softClean;
  g_frames = framesPerPhase;
  g_vcom = vcom;
  g_fullEvery = fullEvery > 0 ? fullEvery : 0xFFFF;   // 0 = never clean up
  g_cfgChanged = true;
}

}  // namespace freeink

#endif  // FREEINK_DRIVER_UC8179_E1001
