#include "serial_link.h"
#include "controller.h"
#include "param_table.h"
#include "logger.h"
#include "types.h"

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

namespace {

constexpr float DEG2RAD = 0.017453293f;
constexpr float RAD2DEG = 57.29578f;
constexpr uint32_t STREAM_HZ_MAX = 200;

UiTorqueSource* g_src = nullptr;
char      g_buf[96];
uint8_t   g_len       = 0;
uint32_t  g_stream_hz = 0;
uint32_t  g_next_us   = 0;

const char* TELEM_HEADER =
  "#H t_ms,state,events,enabled,p_cmd_deg,p_act_deg,v_cmd,v_act,"
  "t_cmd,t_act,iq,vbus,temp,reach_ms,peak_vel,tx,rx,miss";

void req(uint8_t t, float f = 0.0f, uint8_t u = 0) {
  CtrlRequest r; r.type = t; r.arg_f = f; r.arg_u8 = u;
  ctrl::request(r);
}

void banner() {
  Serial.println("#RS06 brake actuator controller, serial link v1");
  Serial.println("#commands: START STOP CLEARFAULT MODE MOVE TRQ ZERO CALIB "
                 "PARAMS SET STREAM DUMP SDDUMP");
}

void sendParams() {
  Params p; ctrl::getParams(p);
  char line[96];
  for (int i = 0; i < N_PDEF; i++) {
    snprintf(line, sizeof(line), "P %s=%.*f %s",
             PDEF[i].key, PDEF[i].dec, *pfield(p, i), PDEF[i].unit);
    Serial.println(line);
  }
  snprintf(line, sizeof(line), "P motor_id=0x%02X can_tx=G%u can_rx=G%u baud=%lu",
           p.motor_id, p.can_tx, p.can_rx, (unsigned long)p.can_baud);
  Serial.println(line);
  snprintf(line, sizeof(line),
           "P calibrated=%d reach_ms=%lu vmax=%.2f amax=%.0f",
           p.calibrated ? 1 : 0, (unsigned long)p.meas_reach_ms,
           p.meas_vmax, p.meas_amax);
  Serial.println(line);
  Serial.println("OK PARAMS");
}

void sendTelemetry() {
  Telemetry t; ctrl::snapshot(t);
  char line[224];
  snprintf(line, sizeof(line),
    "T,%lu,%s,%u,%d,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.3f,%.2f,%.1f,%lu,%.3f,%lu,%lu,%lu",
    (unsigned long)millis(), ctrl::stateName(t.state), (unsigned)t.events,
    t.enabled ? 1 : 0,
    t.p_cmd * RAD2DEG, t.pos * RAD2DEG,
    t.v_cmd, t.vel, t.t_cmd, t.torque,
    t.iq, t.vbus, t.temp,
    (unsigned long)t.reach_ms, t.peak_vel,
    (unsigned long)t.tx, (unsigned long)t.rx, (unsigned long)t.miss);
  Serial.println(line);
}

// --- tiny tokenizer --------------------------------------------------------
char* nextTok(char*& p) {
  while (*p == ' ' || *p == '\t') p++;
  if (!*p) return nullptr;
  char* start = p;
  while (*p && *p != ' ' && *p != '\t') p++;
  if (*p) { *p = 0; p++; }
  return start;
}

void upper(char* s) { for (; *s; s++) *s = (char)toupper((unsigned char)*s); }

void handle(char* line) {
  char* p = line;
  char* cmd = nextTok(p);
  if (!cmd) return;
  upper(cmd);

  if (!strcmp(cmd, "PING")) { Serial.println("OK PONG"); return; }
  if (!strcmp(cmd, "ID"))   { banner(); return; }

  if (!strcmp(cmd, "START"))      { req(REQ_START);       Serial.println("OK START");  return; }
  if (!strcmp(cmd, "STOP"))       { req(REQ_STOP);        Serial.println("OK STOP");   return; }
  if (!strcmp(cmd, "CLEARFAULT")) { req(REQ_CLEAR_FAULT); Serial.println("OK CLEARFAULT"); return; }

  if (!strcmp(cmd, "MODE")) {
    char* a = nextTok(p);
    if (!a) { Serial.println("ERR MODE needs POS or TRQ"); return; }
    upper(a);
    if (!strcmp(a, "POS"))      { req(REQ_SET_MODE, 0, CM_POSITION); Serial.println("OK MODE POS"); }
    else if (!strcmp(a, "TRQ")) { req(REQ_SET_MODE, 0, CM_TORQUE);   Serial.println("OK MODE TRQ"); }
    else                          Serial.println("ERR MODE needs POS or TRQ");
    return;
  }

  if (!strcmp(cmd, "MOVE")) {
    char* a = nextTok(p);
    if (!a) { Serial.println("ERR MOVE needs degrees"); return; }
    const float deg = atof(a);
    req(REQ_SET_TARGET_POS, deg * DEG2RAD);
    Serial.print("OK MOVE "); Serial.println(deg, 2);
    return;
  }

  if (!strcmp(cmd, "TRQ")) {
    char* a = nextTok(p);
    if (!a) { Serial.println("ERR TRQ needs Nm"); return; }
    Params par; ctrl::getParams(par);
    float nm = atof(a);
    if (nm >  par.lim_torque) nm =  par.lim_torque;
    if (nm < -par.lim_torque) nm = -par.lim_torque;
    if (g_src) g_src->set(nm, millis());
    Serial.print("OK TRQ "); Serial.println(nm, 3);
    return;
  }

  if (!strcmp(cmd, "ZERO")) {
    char* a = nextTok(p);
    if (!a) { Serial.println("ERR ZERO needs MOTOR or SOFT"); return; }
    upper(a);
    if (!strcmp(a, "MOTOR")) {
      if (ctrl::busy()) { Serial.println("ERR stop first"); return; }
      req(REQ_ZERO_MOTOR); Serial.println("OK ZERO MOTOR");
    } else if (!strcmp(a, "SOFT")) {
      req(REQ_ZERO_SOFT);  Serial.println("OK ZERO SOFT");
    } else Serial.println("ERR ZERO needs MOTOR or SOFT");
    return;
  }

  if (!strcmp(cmd, "CALIB")) {
    char* a = nextTok(p);
    if (!a) { Serial.println("ERR CALIB needs START or ABORT"); return; }
    upper(a);
    if (!strcmp(a, "START"))      { req(REQ_CALIB_START); Serial.println("OK CALIB START"); }
    else if (!strcmp(a, "ABORT")) { req(REQ_CALIB_ABORT); Serial.println("OK CALIB ABORT"); }
    else                            Serial.println("ERR CALIB needs START or ABORT");
    return;
  }

  if (!strcmp(cmd, "PARAMS")) { sendParams(); return; }

  if (!strcmp(cmd, "SET")) {
    char* k = nextTok(p);
    char* v = nextTok(p);
    if (!k || !v) { Serial.println("ERR SET needs <key> <value>"); return; }
    const int i = paramIndexByKey(k);
    if (i < 0) { Serial.print("ERR unknown key "); Serial.println(k); return; }
    float val = atof(v);
    if (val < PDEF[i].mn) val = PDEF[i].mn;
    if (val > PDEF[i].mx) val = PDEF[i].mx;
    Params par; ctrl::getParams(par);
    *pfield(par, i) = val;
    ctrl::setParams(par);                 // persists and re-pushes to the motor
    Serial.print("OK SET "); Serial.print(PDEF[i].key);
    Serial.print("="); Serial.println(val, PDEF[i].dec);
    return;
  }

  if (!strcmp(cmd, "STREAM")) {
    char* a = nextTok(p);
    uint32_t hz = a ? (uint32_t)atoi(a) : 0;
    if (hz > STREAM_HZ_MAX) hz = STREAM_HZ_MAX;
    g_stream_hz = hz;
    g_next_us   = micros();
    if (hz) Serial.println(TELEM_HEADER);
    Serial.print("OK STREAM "); Serial.println((unsigned long)hz);
    return;
  }

  if (!strcmp(cmd, "DUMP")) {
    char* a = nextTok(p);
    const float sec = a ? atof(a) : 3.0f;
    size_t n = (size_t)(sec * 1000.0f);      // the ring runs at 1 kHz
    if (n == 0) n = 1;
    const int rows = logger::dumpToStream(Serial, n);
    if (rows < 0) Serial.println("ERR DUMP failed");
    else { Serial.print("OK DUMP "); Serial.println(rows); }
    return;
  }

  if (!strcmp(cmd, "SDDUMP")) {
    if (ctrl::busy()) { Serial.println("ERR stop first"); return; }
    char path[48];
    const int rows = logger::dumpToSd(path, sizeof(path));
    if (rows < 0) Serial.println("ERR SDDUMP failed");
    else { Serial.print("OK SDDUMP "); Serial.print(rows); Serial.print(" "); Serial.println(path); }
    return;
  }

  Serial.print("ERR unknown command "); Serial.println(cmd);
}

} // namespace

namespace serial_link {

void begin(UiTorqueSource* torque_src) {
  g_src = torque_src;
  g_len = 0;
  banner();
}

void tick() {
  // --- inbound ------------------------------------------------------------
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      g_buf[g_len] = 0;
      if (g_len) handle(g_buf);
      g_len = 0;
      continue;
    }
    if (g_len < sizeof(g_buf) - 1) g_buf[g_len++] = c;
    else { g_len = 0; Serial.println("ERR line too long"); }
  }

  // --- outbound stream ----------------------------------------------------
  if (!g_stream_hz) return;
  const uint32_t now = micros();
  const uint32_t period = 1000000UL / g_stream_hz;
  if ((int32_t)(now - g_next_us) < 0) return;
  g_next_us += period;
  if ((int32_t)(now - g_next_us) > (int32_t)period) g_next_us = now + period;  // catch up
  sendTelemetry();
}

} // namespace serial_link
