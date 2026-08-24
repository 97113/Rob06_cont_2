#include "ui.h"
#include "controller.h"
#include "logger.h"
#include "types.h"
#include "param_table.h"

#include <M5Unified.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

namespace {

constexpr float DEG2RAD = 0.017453293f;
constexpr float RAD2DEG = 57.29578f;

constexpr int SB_H   = 22;    // status bar height
constexpr int BODY_Y = SB_H;
constexpr int BODY_H = 240 - SB_H - 40;
constexpr int BTN_Y  = 240 - 40;

enum Page  : uint8_t { PG_MONITOR = 0, PG_CONTROL, PG_CONFIG, PG_COUNT };
enum Layer : uint8_t { LY_NONE = 0, LY_KEYPAD, LY_CALIB, LY_RESULT, LY_CANID };

struct Rect {
  int16_t x, y, w, h;
  bool hit(int16_t px, int16_t py) const {
    return px >= x && px < x + w && py >= y && py < y + h;
  }
};

constexpr int ROWS_CFG = 4;   // parameter rows visible per CONFIG page


// --- UI state --------------------------------------------------------------
UiTorqueSource* g_src = nullptr;
Page      g_page      = PG_MONITOR;
Layer     g_layer     = LY_NONE;
bool      g_redraw    = true;

int       g_cfg_top   = 0;
int       g_kp_target = -1;        // index into PDEF, or -2 for target angle,
                                   // -3 for torque command
char      g_kp_buf[16] = {0};

Params    g_par;
Telemetry g_tm;

uint8_t   g_id_edit     = DEFAULT_MOTOR_ID;   // CAN-ID editor working value
float     g_target_deg  = 200.0f;  // position set-point shown on CONTROL
float     g_torque_cmd  = 0.0f;    // N.m shown on CONTROL

// trend ring for the MONITOR strip chart
constexpr int TREND_N = 120;
float     g_trend[TREND_N] = {0};
int       g_trend_i = 0;

// Per-field cache to suppress redraw flicker. The strings must fit whole: a
// key that gets truncated never compares equal to the next one, so the field
// redraws at the full 20 Hz and flickers - which is what the 24 byte version
// did to the CAN diagnostics row and now to the status bar.
char      g_last[24][80] = {{0}};
char      g_toast[40] = {0};
uint32_t  g_toast_t = 0;

// --- hit boxes rebuilt on every draw --------------------------------------
Rect g_box[32];
int  g_nbox = 0;
int  addBox(Rect r) { if (g_nbox < 32) { g_box[g_nbox] = r; return g_nbox++; } return -1; }
// Box ids are only valid after the owning page has been drawn at least once,
// so every hit test is guarded - a touch that arrives first must not index
// into an uninitialised slot.
bool hitBox(int id, int16_t x, int16_t y) {
  return id >= 0 && id < g_nbox && g_box[id].hit(x, y);
}

// ---------------------------------------------------------------------------
void toast(const char* s) { snprintf(g_toast, sizeof(g_toast), "%s", s); g_toast_t = millis(); }

void req(uint8_t t, float f = 0.0f, uint8_t u = 0) {
  CtrlRequest r; r.type = t; r.arg_f = f; r.arg_u8 = u;
  ctrl::request(r);
}

float* pval(int i) { return pfield(g_par, i); }

bool cached(int slot, const char* s) {
  if (slot < 0 || slot >= (int)(sizeof(g_last) / sizeof(g_last[0]))) return false;
  if (strcmp(g_last[slot], s) == 0) return true;
  snprintf(g_last[slot], sizeof(g_last[slot]), "%s", s);
  return false;
}
void invalidateCache() { memset(g_last, 0, sizeof(g_last)); }

// ---------------------------------------------------------------------------
void drawButton(const Rect& r, const char* label, uint16_t bg, uint16_t fg) {
  M5.Display.fillRoundRect(r.x, r.y, r.w, r.h, 4, bg);
  M5.Display.drawRoundRect(r.x, r.y, r.w, r.h, 4, TFT_DARKGREY);
  M5.Display.setTextColor(fg, bg);
  M5.Display.setTextDatum(middle_center);
  M5.Display.setFont(&fonts::Font2);
  M5.Display.drawString(label, r.x + r.w / 2, r.y + r.h / 2);
}

// Most significant reason first - the status bar has room for one.
const char* eventName(uint32_t ev) {
  if (!ev) return "OK";
  if (ev & EV_OVERCURRENT)  return "OVERCURRENT";
  if (ev & EV_DRIVER_CHIP)  return "DRIVER-IC";
  if (ev & EV_TEMP_TRIP)    return "OVERTEMP";
  if (ev & EV_ENCODER)      return "ENCODER";
  if (ev & EV_STALL)        return "STALL";
  if (ev & EV_POS_INIT)     return "POS-INIT";
  if (ev & EV_HW_ID)        return "HW-ID";
  if (ev & EV_OVERVOLT)     return "OVERVOLT";
  if (ev & EV_UNDERVOLT)    return "UNDERVOLT";
  if (ev & EV_CAN_BUS_OFF)  return "BUS-OFF";
  if (ev & EV_CAN_MISS)     return "CAN-MISS";
  if (ev & EV_CMDSRC_STALE) return "CMD-STALE";
  if (ev & EV_POS_LIMIT)    return "POS-LIMIT";
  if (ev & EV_AUX_STALE)    return "AUX-QUIET";
  if (ev & EV_TEMP_DERATE)  return "DERATE";
  if (ev & EV_REGEN_LIMIT)  return "REGEN-LIM";
  if (ev & EV_UNCALIBRATED) return "UNCAL";
  return "?";
}

// What the status bar should say on the right. A latched fault always wins;
// with none outstanding it shows what is limiting the axis at this instant,
// prefixed with "~" so a live cap can never be mistaken for an incident.
const char* statusWord(char* buf, size_t n) {
  if (g_tm.events) return eventName(g_tm.events);
  if (g_tm.active) { snprintf(buf, n, "~%s", eventName(g_tm.active)); return buf; }
  return "OK";
}

// ---------------------------------------------------------------------------
void drawStatusBar() {
  const bool flt = (g_tm.state == ST_FAULT_HARD);
  const bool sft = (g_tm.state == ST_FAULT_SOFT);
  uint16_t bg = flt ? TFT_RED : (sft ? TFT_ORANGE : (g_tm.enabled ? TFT_DARKGREEN : TFT_NAVY));

  char w[24];
  const char* word = statusWord(w, sizeof(w));

  char s[64];
  snprintf(s, sizeof(s), "%s|%.1f|%s|%c|%.1f",
           ctrl::stateName(g_tm.state), g_tm.temp, word,
           (g_tm.miss * 50 < g_tm.mtx + 1) ? 'K' : '!', g_tm.vbus);
  if (cached(0, s) && !g_redraw) return;

  M5.Display.fillRect(0, 0, 320, SB_H, bg);
  M5.Display.setTextColor(TFT_WHITE, bg);
  M5.Display.setFont(&fonts::Font2);
  M5.Display.setTextDatum(middle_left);
  M5.Display.drawString(ctrl::stateName(g_tm.state), 4, SB_H / 2);

  char b[40];
  snprintf(b, sizeof(b), "%.1fC  %.1fV%s", g_tm.temp, g_tm.vbus,
           g_tm.aux_ok ? "" : "?");   // "?" = reading is stale, not measured now
  M5.Display.setTextDatum(middle_center);
  M5.Display.drawString(b, 160, SB_H / 2);

  snprintf(b, sizeof(b), "%s %s",
           (g_tm.miss * 50 < g_tm.mtx + 1) ? "CAN:OK" : "CAN:!!", word);
  M5.Display.setTextDatum(middle_right);
  M5.Display.drawString(b, 316, SB_H / 2);
}

// ---------------------------------------------------------------------------
void drawBottomBar() {
  // Deliberately does NOT reset g_nbox: it is drawn after the page body, and
  // clearing the box list here would invalidate every hit box on the page.
  const char* a = "PAGE >";
  const char* b = (g_tm.state == ST_RUN_POS || g_tm.state == ST_RUN_TRQ) ? "MOVE" : "START";
  const char* c = (g_layer == LY_CALIB) ? "ABORT" : "STOP";
  M5.Display.fillRect(0, BTN_Y, 320, 40, TFT_BLACK);
  drawButton({   4, BTN_Y + 4, 100, 32 }, a, TFT_DARKGREY, TFT_WHITE);
  drawButton({ 110, BTN_Y + 4, 100, 32 }, b, TFT_DARKGREEN, TFT_WHITE);
  drawButton({ 216, BTN_Y + 4, 100, 32 }, c, TFT_MAROON,   TFT_WHITE);
}

// ---------------------------------------------------------------------------
void row(int slot, int y, const char* label, const char* value, uint16_t col) {
  char key[80];
  snprintf(key, sizeof(key), "%s", value);
  if (cached(slot, key) && !g_redraw) return;
  M5.Display.fillRect(0, y, 320, 30, TFT_BLACK);
  M5.Display.setFont(&fonts::Font2);
  M5.Display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  M5.Display.setTextDatum(middle_left);
  M5.Display.drawString(label, 6, y + 15);
  M5.Display.setFont(&fonts::Font4);
  M5.Display.setTextColor(col, TFT_BLACK);
  M5.Display.setTextDatum(middle_right);
  M5.Display.drawString(value, 314, y + 15);
}

void drawMonitor() {
  if (g_redraw) M5.Display.fillRect(0, BODY_Y, 320, BODY_H, TFT_BLACK);
  char v[24];
  int y = BODY_Y + 2;

  snprintf(v, sizeof(v), "%.2f Nm", g_tm.torque);           row(1, y, "TORQUE",  v, TFT_CYAN);      y += 28;
  snprintf(v, sizeof(v), "%.2f A",  g_tm.iq);               row(2, y, "CURRENT", v, TFT_CYAN);      y += 28;
  snprintf(v, sizeof(v), "%.1f d",  g_tm.p_cmd * RAD2DEG);  row(3, y, "CMD ANG", v, TFT_YELLOW);    y += 28;
  snprintf(v, sizeof(v), "%.1f d",  g_tm.pos   * RAD2DEG);  row(4, y, "ANGLE",   v, TFT_GREENYELLOW); y += 28;
  snprintf(v, sizeof(v), "%.1f C",  g_tm.temp);             row(5, y, "MOT TEMP",v,
           g_tm.temp > g_par.temp_derate ? TFT_ORANGE : TFT_WHITE);
  y += 28;

  // CAN link diagnostics. Reading order matters when bringing the bus up:
  //   Q climbing with TEC 0  -> never transmitted; RX line stuck dominant
  //                             (DIP off, PwrCAN unpowered, wrong RX pin)
  //   TEC/BE climbing        -> transmitting but nothing ACKs
  //                             (CANH/CANL swap, no termination, motor off)
  //   st:OFF                 -> controller went bus-off
  {
    static const char* ST[5] = { "STOP", "RUN", "OFF", "REC", "?" };
    const uint8_t si = (g_tm.twai_state < 4) ? g_tm.twai_state : 4;
    char d[80];
    snprintf(d, sizeof(d), "st:%s TX%lu RX%lu MS%lu TEC%u BE%lu Q%lu",
             ST[si], (unsigned long)g_tm.tx, (unsigned long)g_tm.rx,
             (unsigned long)g_tm.miss, (unsigned)g_tm.twai_tec,
             (unsigned long)g_tm.twai_buserr, (unsigned long)g_tm.twai_queued);
    if (!cached(6, d) || g_redraw) {
      M5.Display.fillRect(0, y, 320, 16, TFT_BLACK);
      M5.Display.setFont(&fonts::Font2);
      M5.Display.setTextColor(g_tm.rx > 0 ? TFT_GREEN : TFT_ORANGE, TFT_BLACK);
      M5.Display.setTextDatum(top_left);
      M5.Display.drawString(d, 4, y);
    }
  }
  y += 17;

  // strip chart of the angle
  const int gx = 4, gw = 312, gh = BODY_Y + BODY_H - y - 2;
  if (gh > 12) {
    M5.Display.fillRect(gx, y, gw, gh, 0x1082);
    float mn = 1e9f, mx = -1e9f;
    for (int i = 0; i < TREND_N; i++) { mn = fminf(mn, g_trend[i]); mx = fmaxf(mx, g_trend[i]); }
    if (mx - mn < 1.0f) { const float c = 0.5f * (mx + mn); mn = c - 0.5f; mx = c + 0.5f; }
    int prev = -1;
    for (int i = 0; i < TREND_N; i++) {
      const float val = g_trend[(g_trend_i + i) % TREND_N];
      const int py = y + gh - 1 - (int)((val - mn) / (mx - mn) * (gh - 2));
      const int px = gx + i * gw / TREND_N;
      if (prev >= 0) M5.Display.drawLine(px - gw / TREND_N, prev, px, py, TFT_GREEN);
      prev = py;
    }
  }
}

// ---------------------------------------------------------------------------
void stepperRow(int y, const char* label, const char* value,
                int& b_mm, int& b_m, int& b_p, int& b_pp, int& b_val) {
  M5.Display.fillRect(0, y, 320, 34, TFT_BLACK);
  M5.Display.setFont(&fonts::Font2);
  M5.Display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  M5.Display.setTextDatum(middle_left);
  M5.Display.drawString(label, 4, y + 16);

  b_mm = addBox({   4, (int16_t)(y + 2), 30, 30 });
  b_m  = addBox({  36, (int16_t)(y + 2), 30, 30 });
  b_p  = addBox({ 254, (int16_t)(y + 2), 30, 30 });
  b_pp = addBox({ 286, (int16_t)(y + 2), 30, 30 });
  b_val= addBox({  70, (int16_t)(y + 2), 180, 30 });

  drawButton(g_box[b_mm], "--", TFT_DARKGREY, TFT_WHITE);
  drawButton(g_box[b_m],  "-",  TFT_DARKGREY, TFT_WHITE);
  drawButton(g_box[b_p],  "+",  TFT_DARKGREY, TFT_WHITE);
  drawButton(g_box[b_pp], "++", TFT_DARKGREY, TFT_WHITE);

  M5.Display.fillRoundRect(70, y + 2, 180, 30, 4, 0x18E3);
  M5.Display.setFont(&fonts::Font4);
  M5.Display.setTextColor(TFT_WHITE, 0x18E3);
  M5.Display.setTextDatum(middle_center);
  M5.Display.drawString(value, 160, y + 17);
}

int g_cb[16] = { -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 };  // CONTROL page

void drawControl() {
  M5.Display.fillRect(0, BODY_Y, 320, BODY_H, TFT_BLACK);
  g_nbox = 0;

  // mode selector
  const bool pos = (g_tm.mode == CM_POSITION);
  g_cb[0] = addBox({   4, BODY_Y + 2, 154, 26 });
  g_cb[1] = addBox({ 162, BODY_Y + 2, 154, 26 });
  drawButton(g_box[g_cb[0]], "POSITION", pos ? TFT_BLUE : TFT_DARKGREY, TFT_WHITE);
  drawButton(g_box[g_cb[1]], "TORQUE", !pos ? TFT_BLUE : TFT_DARKGREY, TFT_WHITE);

  char v[24];
  if (pos) {
    snprintf(v, sizeof(v), "%.1f deg", g_target_deg);
    stepperRow(BODY_Y + 32, "TARGET ANGLE", v, g_cb[2], g_cb[3], g_cb[4], g_cb[5], g_cb[6]);
    // result line
    M5.Display.setFont(&fonts::Font2);
    M5.Display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    M5.Display.setTextDatum(middle_left);
    snprintf(v, sizeof(v), "reach %lu ms  peak %.1f rad/s",
             (unsigned long)g_tm.reach_ms, g_tm.peak_vel);
    M5.Display.drawString(v, 6, BODY_Y + 76);
  } else {
    snprintf(v, sizeof(v), "%.2f Nm", g_torque_cmd);
    stepperRow(BODY_Y + 32, "TORQUE CMD", v, g_cb[2], g_cb[3], g_cb[4], g_cb[5], g_cb[6]);
    M5.Display.setFont(&fonts::Font2);
    M5.Display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    M5.Display.setTextDatum(middle_left);
    snprintf(v, sizeof(v), "= %.2f A  (Kt %.3f Nm/Arms)",
             (g_par.kt > 1e-3f) ? g_torque_cmd / g_par.kt : 0.0f, g_par.kt);
    M5.Display.drawString(v, 6, BODY_Y + 76);
  }

  // zero controls
  g_cb[7] = addBox({   4, BODY_Y + 92, 154, 30 });
  g_cb[8] = addBox({ 162, BODY_Y + 92, 154, 30 });
  drawButton(g_box[g_cb[7]], "ZERO (motor)", TFT_PURPLE, TFT_WHITE);
  drawButton(g_box[g_cb[8]], "ZERO (soft)",  TFT_PURPLE, TFT_WHITE);

  g_cb[9]  = addBox({   4, BODY_Y + 126, 154, 28 });
  g_cb[10] = addBox({ 162, BODY_Y + 126, 154, 28 });
  drawButton(g_box[g_cb[9]],  "CLEAR FAULT", TFT_OLIVE, TFT_WHITE);
  drawButton(g_box[g_cb[10]], "GO TO TARGET", TFT_DARKGREEN, TFT_WHITE);
}

// ---------------------------------------------------------------------------
int g_gb[16] = { -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 };  // CONFIG page

void drawConfig() {
  M5.Display.fillRect(0, BODY_Y, 320, BODY_H, TFT_BLACK);
  g_nbox = 0;
  M5.Display.setFont(&fonts::Font2);

  for (int r = 0; r < ROWS_CFG; r++) {
    const int i = g_cfg_top + r;
    if (i >= N_PDEF) break;
    const int y = BODY_Y + 2 + r * 30;

    M5.Display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    M5.Display.setTextDatum(middle_left);
    M5.Display.drawString(PDEF[i].name, 4, y + 14);

    char v[24];
    snprintf(v, sizeof(v), "%.*f%s", PDEF[i].dec, *pval(i), PDEF[i].unit);
    const int bv = addBox({ 108, (int16_t)y, 90, 28 });
    M5.Display.fillRoundRect(108, y, 90, 28, 3, 0x18E3);
    M5.Display.setTextColor(TFT_WHITE, 0x18E3);
    M5.Display.setTextDatum(middle_center);
    M5.Display.drawString(v, 153, y + 14);

    const int bm = addBox({ 202, (int16_t)y, 26, 28 });
    const int bp = addBox({ 230, (int16_t)y, 26, 28 });
    drawButton(g_box[bm], "-", TFT_DARKGREY, TFT_WHITE);
    drawButton(g_box[bp], "+", TFT_DARKGREY, TFT_WHITE);
    (void)bv;
  }

  const int y2 = BODY_Y + 2 + ROWS_CFG * 30;
  g_gb[0] = addBox({   4, (int16_t)y2, 50, 28 });
  g_gb[1] = addBox({  58, (int16_t)y2, 50, 28 });
  g_gb[2] = addBox({ 114, (int16_t)y2, 96, 28 });
  g_gb[3] = addBox({ 214, (int16_t)y2, 102, 28 });
  drawButton(g_box[g_gb[0]], "UP",     TFT_DARKGREY, TFT_WHITE);
  drawButton(g_box[g_gb[1]], "DOWN",   TFT_DARKGREY, TFT_WHITE);
  drawButton(g_box[g_gb[2]], "CALIB",  TFT_ORANGE,   TFT_BLACK);
  drawButton(g_box[g_gb[3]], "SD DUMP",TFT_DARKCYAN, TFT_WHITE);

  // Live CAN wiring, so the DIP setting can be checked without a rebuild.
  // Tapping it opens the CAN-ID editor.
  char s[56];
  snprintf(s, sizeof(s), "CAN TX G%u / RX G%u / %luk / id 0x%02X  >",
           g_par.can_tx, g_par.can_rx,
           (unsigned long)(g_par.can_baud / 1000), g_par.motor_id);
  g_gb[5] = addBox({ 0, (int16_t)(y2 + 30), 320, 22 });
  M5.Display.setFont(&fonts::Font2);
  M5.Display.setTextColor(TFT_DARKCYAN, TFT_BLACK);
  M5.Display.setTextDatum(middle_left);
  M5.Display.drawString(s, 6, y2 + 41);
}

// ---------------------------------------------------------------------------
// The four TX and four RX pins the PwrCAN "CAN Select" DIP can route to a
// Core2. Beware: the board silkscreen prints Core / Core2 / CoreS3 side by
// side, and the CoreS3 column contains pins (G10, G6, G7 ...) that do not
// exist on the Core2 M5-Bus at all. Only this Core2 column is valid here.
const uint8_t TX_OPT[4] = { 14, 2, 27, 0 };    // DIP CH1 CH2 CH3 CH4
const uint8_t RX_OPT[4] = { 13, 19, 34, 35 };  // DIP CH5 CH6 CH7 CH8
const uint32_t BAUD_OPT[4] = { 1000000, 500000, 250000, 125000 };

int g_ib[16] = { -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 };
uint8_t  g_tx_edit = 0, g_rx_edit = 0, g_bd_edit = 0;

int optIndex(const uint8_t* opt, uint8_t v) {
  for (int i = 0; i < 4; i++) if (opt[i] == v) return i;
  return 0;
}
int baudIndex(uint32_t v) {
  for (int i = 0; i < 4; i++) if (BAUD_OPT[i] == v) return i;
  return 0;
}

void cycleRow(int y, const char* label, const char* value, int& bm, int& bp) {
  M5.Display.fillRect(0, y, 320, 24, TFT_BLACK);
  M5.Display.setFont(&fonts::Font2);
  M5.Display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  M5.Display.setTextDatum(middle_left);
  M5.Display.drawString(label, 4, y + 12);

  bm = addBox({ 150, (int16_t)y, 26, 22 });
  bp = addBox({ 290, (int16_t)y, 26, 22 });
  drawButton(g_box[bm], "<", TFT_DARKGREY, TFT_WHITE);
  drawButton(g_box[bp], ">", TFT_DARKGREY, TFT_WHITE);

  M5.Display.fillRoundRect(180, y, 106, 22, 3, 0x18E3);
  M5.Display.setTextColor(TFT_WHITE, 0x18E3);
  M5.Display.setTextDatum(middle_center);
  M5.Display.drawString(value, 233, y + 11);
}

void drawCanId() {
  M5.Display.fillRect(0, BODY_Y, 320, BODY_H, TFT_BLACK);
  g_nbox = 0;
  char v[24];

  M5.Display.setFont(&fonts::Font2);
  M5.Display.setTextColor(TFT_CYAN, TFT_BLACK);
  M5.Display.setTextDatum(middle_center);
  M5.Display.drawString("CAN SETUP", 160, BODY_Y + 8);

  snprintf(v, sizeof(v), "G%u  (CH%d)", TX_OPT[g_tx_edit], g_tx_edit + 1);
  cycleRow(BODY_Y + 20, "TX pin", v, g_ib[0], g_ib[1]);

  snprintf(v, sizeof(v), "G%u  (CH%d)", RX_OPT[g_rx_edit], g_rx_edit + 5);
  cycleRow(BODY_Y + 46, "RX pin", v, g_ib[2], g_ib[3]);

  snprintf(v, sizeof(v), "%lu k", (unsigned long)(BAUD_OPT[g_bd_edit] / 1000));
  cycleRow(BODY_Y + 72, "bit rate", v, g_ib[4], g_ib[5]);

  // motor id stepper
  M5.Display.fillRect(0, BODY_Y + 98, 320, 24, TFT_BLACK);
  M5.Display.setFont(&fonts::Font2);
  M5.Display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  M5.Display.setTextDatum(middle_left);
  M5.Display.drawString("motor id", 4, BODY_Y + 110);
  g_ib[6] = addBox({ 122, (int16_t)(BODY_Y + 98), 26, 22 });
  g_ib[7] = addBox({ 150, (int16_t)(BODY_Y + 98), 26, 22 });
  g_ib[8] = addBox({ 262, (int16_t)(BODY_Y + 98), 26, 22 });
  g_ib[9] = addBox({ 290, (int16_t)(BODY_Y + 98), 26, 22 });
  drawButton(g_box[g_ib[6]], "-16", TFT_DARKGREY, TFT_WHITE);
  drawButton(g_box[g_ib[7]], "-1",  TFT_DARKGREY, TFT_WHITE);
  drawButton(g_box[g_ib[8]], "+1",  TFT_DARKGREY, TFT_WHITE);
  drawButton(g_box[g_ib[9]], "+16", TFT_DARKGREY, TFT_WHITE);
  snprintf(v, sizeof(v), "0x%02X", g_id_edit);
  M5.Display.fillRoundRect(180, BODY_Y + 98, 78, 22, 3, 0x18E3);
  M5.Display.setTextColor(TFT_WHITE, 0x18E3);
  M5.Display.setTextDatum(middle_center);
  M5.Display.drawString(v, 219, BODY_Y + 109);

  g_ib[10] = addBox({   4, (int16_t)(BODY_Y + 126), 154, 24 });
  g_ib[11] = addBox({ 162, (int16_t)(BODY_Y + 126), 154, 24 });
  drawButton(g_box[g_ib[10]], "APPLY (restart CAN)", TFT_BLUE,   TFT_WHITE);
  drawButton(g_box[g_ib[11]], "WRITE ID TO MOTOR",  TFT_ORANGE, TFT_BLACK);

  g_ib[12] = addBox({ 110, (int16_t)(BODY_Y + 152), 100, 22 });
  drawButton(g_box[g_ib[12]], "CLOSE", TFT_DARKGREY, TFT_WHITE);
}

// ---------------------------------------------------------------------------
const char* KEYS[16] = { "7","8","9","CLR", "4","5","6","+/-",
                         "1","2","3",".",   "0","<",  "OK","ESC" };
int g_kb[16] = { -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 };  // keypad

void drawKeypad() {
  M5.Display.fillRect(0, 0, 320, 240, TFT_BLACK);
  g_nbox = 0;
  M5.Display.setFont(&fonts::Font4);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextDatum(middle_center);
  M5.Display.fillRoundRect(10, 6, 300, 34, 4, 0x18E3);
  M5.Display.drawString(g_kp_buf[0] ? g_kp_buf : "0", 160, 23);

  for (int i = 0; i < 16; i++) {
    const int cx = i % 4, cy = i / 4;
    Rect r{ (int16_t)(10 + cx * 76), (int16_t)(48 + cy * 46), 72, 42 };
    g_kb[i] = addBox(r);
    const bool ok = (strcmp(KEYS[i], "OK") == 0);
    const bool es = (strcmp(KEYS[i], "ESC") == 0);
    drawButton(r, KEYS[i], ok ? TFT_DARKGREEN : (es ? TFT_MAROON : TFT_DARKGREY), TFT_WHITE);
  }
}

void drawCalib() {
  M5.Display.fillRect(0, BODY_Y, 320, BODY_H, TFT_BLACK);
  g_nbox = 0;
  M5.Display.setFont(&fonts::Font4);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextDatum(middle_center);
  M5.Display.drawString("CALIBRATING", 160, BODY_Y + 24);
  M5.Display.setFont(&fonts::Font2);
  M5.Display.drawString(g_tm.calib_msg, 160, BODY_Y + 54);

  M5.Display.drawRect(20, BODY_Y + 74, 280, 22, TFT_WHITE);
  const int w = (int)(276.0f * g_tm.calib_pct / 100.0f);
  M5.Display.fillRect(22, BODY_Y + 76, w, 18, TFT_GREEN);
  M5.Display.fillRect(22 + w, BODY_Y + 76, 276 - w, 18, TFT_BLACK);

  char s[48];
  snprintf(s, sizeof(s), "%u%%   keep the axis clear", g_tm.calib_pct);
  M5.Display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  M5.Display.drawString(s, 160, BODY_Y + 112);
}

void drawResult() {
  M5.Display.fillRect(0, BODY_Y, 320, BODY_H, TFT_BLACK);
  g_nbox = 0;
  M5.Display.setFont(&fonts::Font2);
  M5.Display.setTextDatum(middle_left);

  char s[64];
  int y = BODY_Y + 6;
  const bool met = (g_par.meas_reach_ms > 0 && g_par.meas_reach_ms <= 100);

  M5.Display.setTextColor(met ? TFT_GREEN : TFT_ORANGE, TFT_BLACK);
  snprintf(s, sizeof(s), "200 deg reached in %lu ms  [%s]",
           (unsigned long)g_par.meas_reach_ms, met ? "PASS" : "OVER 100ms");
  M5.Display.drawString(s, 6, y); y += 20;

  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  snprintf(s, sizeof(s), "peak speed  %.1f rad/s (%.0f rpm)",
           g_par.meas_vmax, g_par.meas_vmax * 9.5493f);
  M5.Display.drawString(s, 6, y); y += 18;
  snprintf(s, sizeof(s), "peak accel  %.0f rad/s2", g_par.meas_amax);
  M5.Display.drawString(s, 6, y); y += 18;
  snprintf(s, sizeof(s), "J %.4f kgm2  Fc %.3f  Fv %.4f",
           g_par.j_hat, g_par.fric_c, g_par.fric_v);
  M5.Display.drawString(s, 6, y); y += 18;
  snprintf(s, sizeof(s), "Kp %.1f  Kd %.2f", g_par.kp, g_par.kd);
  M5.Display.drawString(s, 6, y); y += 18;
  snprintf(s, sizeof(s), "VBUS %.1f V  sag %.1f V", g_par.meas_vbus_ref, g_par.meas_vbus_sag);
  M5.Display.drawString(s, 6, y); y += 18;
  snprintf(s, sizeof(s), "CAN rtt %.0f us  drop %.2f %%",
           g_par.meas_rtt_us, g_par.meas_drop_pct);
  M5.Display.drawString(s, 6, y); y += 22;

  g_gb[4] = addBox({ 110, (int16_t)y, 100, 28 });
  drawButton(g_box[g_gb[4]], "CLOSE", TFT_DARKGREY, TFT_WHITE);
}

// ---------------------------------------------------------------------------
void openKeypad(int target, float current) {
  g_kp_target = target;
  snprintf(g_kp_buf, sizeof(g_kp_buf), "%g", (double)current);
  g_layer = LY_KEYPAD;
  g_redraw = true;
}

void commitKeypad() {
  const float v = atof(g_kp_buf);
  if (g_kp_target >= 0) {
    *pval(g_kp_target) = fminf(fmaxf(v, PDEF[g_kp_target].mn), PDEF[g_kp_target].mx);
    ctrl::setParams(g_par);
  } else if (g_kp_target == -2) {
    g_target_deg = v;
  } else if (g_kp_target == -3) {
    g_torque_cmd = fminf(fmaxf(v, -g_par.lim_torque), g_par.lim_torque);
    if (g_src) g_src->set(g_torque_cmd, millis());
  }
  g_layer  = LY_NONE;
  g_redraw = true;
}

// ---------------------------------------------------------------------------
void onTouch(int16_t x, int16_t y) {
  if (g_layer == LY_KEYPAD) {
    for (int i = 0; i < 16; i++) {
      if (!hitBox(g_kb[i], x, y)) continue;
      const char* k = KEYS[i];
      const size_t n = strlen(g_kp_buf);
      if      (!strcmp(k, "OK"))  { commitKeypad(); return; }
      else if (!strcmp(k, "ESC")) { g_layer = LY_NONE; g_redraw = true; return; }
      else if (!strcmp(k, "CLR")) { g_kp_buf[0] = 0; }
      else if (!strcmp(k, "<"))   { if (n) g_kp_buf[n - 1] = 0; }
      else if (!strcmp(k, "+/-")) {
        if (g_kp_buf[0] == '-') memmove(g_kp_buf, g_kp_buf + 1, n);
        else if (n + 1 < sizeof(g_kp_buf)) { memmove(g_kp_buf + 1, g_kp_buf, n + 1); g_kp_buf[0] = '-'; }
      } else if (n + 1 < sizeof(g_kp_buf)) {
        g_kp_buf[n] = k[0]; g_kp_buf[n + 1] = 0;
      }
      g_redraw = true;
      return;
    }
    return;
  }

  if (g_layer == LY_CANID) {
    if (hitBox(g_ib[0], x, y)) { g_tx_edit = (g_tx_edit + 3) & 3; g_redraw = true; return; }
    if (hitBox(g_ib[1], x, y)) { g_tx_edit = (g_tx_edit + 1) & 3; g_redraw = true; return; }
    if (hitBox(g_ib[2], x, y)) { g_rx_edit = (g_rx_edit + 3) & 3; g_redraw = true; return; }
    if (hitBox(g_ib[3], x, y)) { g_rx_edit = (g_rx_edit + 1) & 3; g_redraw = true; return; }
    if (hitBox(g_ib[4], x, y)) { g_bd_edit = (g_bd_edit + 3) & 3; g_redraw = true; return; }
    if (hitBox(g_ib[5], x, y)) { g_bd_edit = (g_bd_edit + 1) & 3; g_redraw = true; return; }
    if (hitBox(g_ib[6], x, y)) { g_id_edit = (uint8_t)((g_id_edit - 16) & 0x7F); g_redraw = true; return; }
    if (hitBox(g_ib[7], x, y)) { g_id_edit = (uint8_t)((g_id_edit -  1) & 0x7F); g_redraw = true; return; }
    if (hitBox(g_ib[8], x, y)) { g_id_edit = (uint8_t)((g_id_edit +  1) & 0x7F); g_redraw = true; return; }
    if (hitBox(g_ib[9], x, y)) { g_id_edit = (uint8_t)((g_id_edit + 16) & 0x7F); g_redraw = true; return; }

    if (hitBox(g_ib[10], x, y)) {
      if (ctrl::busy()) { toast("stop first"); return; }
      g_par.can_tx   = TX_OPT[g_tx_edit];
      g_par.can_rx   = RX_OPT[g_rx_edit];
      g_par.can_baud = BAUD_OPT[g_bd_edit];
      g_par.motor_id = g_id_edit;
      ctrl::setParams(g_par);            // persists to NVS
      req(REQ_CAN_REINIT);               // reinstall TWAI with the new wiring
      toast("CAN restarted");
      g_redraw = true; return;
    }
    if (hitBox(g_ib[11], x, y)) {
      if (ctrl::busy()) { toast("stop first"); return; }
      req(REQ_WRITE_MOTOR_CANID, 0, g_id_edit);   // Type7 + Type22
      g_par.motor_id = g_id_edit;
      ctrl::setParams(g_par);
      toast("motor id rewritten");
      g_redraw = true; return;
    }
    if (hitBox(g_ib[12], x, y)) { g_layer = LY_NONE; g_redraw = true; invalidateCache(); }
    return;
  }

  if (g_layer == LY_RESULT) {
    if (hitBox(g_gb[4], x, y)) { g_layer = LY_NONE; g_redraw = true; }
    return;
  }
  if (g_layer == LY_CALIB) return;   // only BtnC aborts

  if (g_page == PG_CONTROL) {
    const bool pos = (g_tm.mode == CM_POSITION);
    if (hitBox(g_cb[0], x, y)) { req(REQ_SET_MODE, 0, CM_POSITION); g_redraw = true; return; }
    if (hitBox(g_cb[1], x, y)) { req(REQ_SET_MODE, 0, CM_TORQUE);   g_redraw = true; return; }

    const float step = pos ? 1.0f  : 0.1f;
    const float big  = pos ? 10.0f : 1.0f;
    float* v = pos ? &g_target_deg : &g_torque_cmd;
    bool  changed = false;
    if (hitBox(g_cb[2], x, y)) { *v -= big;  changed = true; }
    if (hitBox(g_cb[3], x, y)) { *v -= step; changed = true; }
    if (hitBox(g_cb[4], x, y)) { *v += step; changed = true; }
    if (hitBox(g_cb[5], x, y)) { *v += big;  changed = true; }
    if (hitBox(g_cb[6], x, y)) { openKeypad(pos ? -2 : -3, *v); return; }

    if (changed) {
      if (pos) {
        g_target_deg = fminf(fmaxf(g_target_deg, g_par.pos_min * RAD2DEG),
                                                  g_par.pos_max * RAD2DEG);
        req(REQ_SET_TARGET_POS, g_target_deg * DEG2RAD);
      } else {
        g_torque_cmd = fminf(fmaxf(g_torque_cmd, -g_par.lim_torque), g_par.lim_torque);
        if (g_src) g_src->set(g_torque_cmd, millis());
      }
      g_redraw = true;
      return;
    }

    if (hitBox(g_cb[7], x, y)) {
      if (ctrl::busy()) toast("stop first");
      else { req(REQ_ZERO_MOTOR); toast("mechanical zero set"); }
      return;
    }
    if (hitBox(g_cb[8], x, y))  { req(REQ_ZERO_SOFT); toast("soft zero set"); return; }
    if (hitBox(g_cb[9], x, y))  { req(REQ_CLEAR_FAULT); toast("fault cleared"); return; }
    if (hitBox(g_cb[10], x, y)) { req(REQ_SET_TARGET_POS, g_target_deg * DEG2RAD); return; }
    return;
  }

  if (g_page == PG_CONFIG) {
    for (int r = 0; r < ROWS_CFG; r++) {
      const int i = g_cfg_top + r;
      if (i >= N_PDEF) break;
      const int base = r * 3;
      if (hitBox(base + 0, x, y)) { openKeypad(i, *pval(i)); return; }
      if (hitBox(base + 1, x, y)) {
        *pval(i) = fmaxf(PDEF[i].mn, *pval(i) - PDEF[i].step);
        ctrl::setParams(g_par); g_redraw = true; return;
      }
      if (hitBox(base + 2, x, y)) {
        *pval(i) = fminf(PDEF[i].mx, *pval(i) + PDEF[i].step);
        ctrl::setParams(g_par); g_redraw = true; return;
      }
    }
    if (hitBox(g_gb[0], x, y)) { g_cfg_top = (g_cfg_top >= ROWS_CFG) ? g_cfg_top - ROWS_CFG : 0; g_redraw = true; return; }
    if (hitBox(g_gb[1], x, y)) { if (g_cfg_top + ROWS_CFG < N_PDEF) g_cfg_top += ROWS_CFG; g_redraw = true; return; }
    if (hitBox(g_gb[2], x, y)) {
      req(REQ_CALIB_START); g_layer = LY_CALIB; g_redraw = true; return;
    }
    if (hitBox(g_gb[5], x, y)) {
      g_id_edit = g_par.motor_id;
      g_tx_edit = (uint8_t)optIndex(TX_OPT, g_par.can_tx);
      g_rx_edit = (uint8_t)optIndex(RX_OPT, g_par.can_rx);
      g_bd_edit = (uint8_t)baudIndex(g_par.can_baud);
      g_layer   = LY_CANID;
      g_redraw  = true;
      return;
    }
    if (hitBox(g_gb[3], x, y)) {
      if (ctrl::busy()) { toast("stop first"); return; }
      char path[48];
      const int n = logger::dumpToSd(path, sizeof(path));
      if (n < 0) toast("SD dump failed");
      else { char m[40]; snprintf(m, sizeof(m), "%d rows -> %s", n, path); toast(m); }
      return;
    }
  }
}

// ---------------------------------------------------------------------------
void drawToast() {
  if (!g_toast[0]) return;
  if (millis() - g_toast_t > 2500) { g_toast[0] = 0; g_redraw = true; return; }
  M5.Display.setFont(&fonts::Font2);
  M5.Display.setTextDatum(middle_center);
  M5.Display.setTextColor(TFT_BLACK, TFT_YELLOW);
  M5.Display.fillRoundRect(20, BTN_Y - 26, 280, 22, 3, TFT_YELLOW);
  M5.Display.drawString(g_toast, 160, BTN_Y - 15);
}

} // namespace

// ===========================================================================
namespace ui {

void begin(UiTorqueSource* torque_src) {
  g_src = torque_src;
  ctrl::getParams(g_par);
  M5.Display.setRotation(1);
  M5.Display.fillScreen(TFT_BLACK);
  g_redraw = true;
}

void tick() {
  static uint32_t last_draw = 0;
  static bool     was_calib = false;

  M5.update();

  // --- physical buttons ---------------------------------------------------
  if (M5.BtnA.wasPressed() && g_layer == LY_NONE) {
    g_page   = (Page)((g_page + 1) % PG_COUNT);
    g_redraw = true;
    invalidateCache();
  }
  if (M5.BtnB.wasPressed() && g_layer == LY_NONE) {
    if (g_tm.state == ST_RUN_POS)      req(REQ_SET_TARGET_POS, g_target_deg * DEG2RAD);
    else if (g_tm.state == ST_RUN_TRQ) { if (g_src) g_src->set(g_torque_cmd, millis()); }
    else if (g_tm.state == ST_FAULT_HARD) {
      // START is refused outright while latched. Say so instead of doing
      // nothing - a silent button is the worst possible feedback here.
      char m[40];
      snprintf(m, sizeof(m), "%s - CLEAR FAULT on CONTROL", eventName(g_tm.events));
      toast(m);
    }
    else                                req(REQ_START);
    g_redraw = true;
  }
  if (M5.BtnC.wasPressed()) {
    if (g_layer == LY_CALIB) { req(REQ_CALIB_ABORT); g_layer = LY_NONE; }
    else                      req(REQ_STOP);
    g_redraw = true;
    invalidateCache();
  }

  // --- touch --------------------------------------------------------------
  // Act on release, but only if the finger stayed put: a drag or flick that
  // happens to end on a control must not trigger it. y >= 240 belongs to the
  // BtnA/B/C strip, which M5Unified already handled above.
  const auto t = M5.Touch.getDetail();
  if (t.wasReleased() && t.y < 240 &&
      abs(t.distanceX()) < 12 && abs(t.distanceY()) < 12) {
    onTouch(t.x, t.y);
  }

  // --- redraw at ~20 Hz ---------------------------------------------------
  const uint32_t now = millis();
  if (now - last_draw < 50 && !g_redraw) return;
  last_draw = now;

  ctrl::snapshot(g_tm);

  // trend sampling
  g_trend[g_trend_i] = g_tm.pos * RAD2DEG;
  g_trend_i = (g_trend_i + 1) % TREND_N;

  // calibration overlay follows the controller, not the button press
  const bool calib = (g_tm.state == ST_CALIB);
  if (calib) g_layer = LY_CALIB;
  if (was_calib && !calib) {
    ctrl::getParams(g_par);
    g_layer  = g_par.calibrated ? LY_RESULT : LY_NONE;
    g_redraw = true;
    invalidateCache();
  }
  was_calib = calib;

  if (g_layer != LY_KEYPAD) drawStatusBar();

  switch (g_layer) {
    case LY_KEYPAD: if (g_redraw) drawKeypad();  g_redraw = false; return;
    case LY_CALIB:  drawCalib();  drawBottomBar(); g_redraw = false; return;
    case LY_RESULT: if (g_redraw) { drawResult(); drawBottomBar(); } g_redraw = false; return;
    case LY_CANID:  if (g_redraw) { drawCanId(); drawBottomBar(); } g_redraw = false; return;
    default: break;
  }

  // CONTROL carries live values (reach time, peak speed), so it is refreshed
  // periodically as well as on interaction; CONFIG only changes when touched.
  static uint32_t last_ctrl = 0;
  switch (g_page) {
    case PG_MONITOR: drawMonitor(); break;
    case PG_CONTROL:
      if (g_redraw || now - last_ctrl > 500) { drawControl(); last_ctrl = now; }
      break;
    case PG_CONFIG:  if (g_redraw) drawConfig(); break;
    default: break;
  }

  if (g_redraw) drawBottomBar();
  drawToast();
  g_redraw = false;
}

} // namespace ui
