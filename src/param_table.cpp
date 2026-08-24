#include "param_table.h"
#include <string.h>

const ParamDef PDEF[] = {
  // name           key       offset                        min   max    step    big     unit     dec
  { "Kp",          "kp",    offsetof(Params, kp),          0,   500,  1,      10,     "",       1 },
  { "Kd",          "kd",    offsetof(Params, kd),          0,   100,  0.1f,   1,      "",       2 },
  { "Kd (torque)", "kdt",   offsetof(Params, kd_torque),   0,   100,  0.05f,  0.5f,   "",       2 },
  { "Torque lim",  "tlim",  offsetof(Params, lim_torque),  0,    36,  0.5f,   5,      "Nm",     1 },
  { "Hold torque", "thold", offsetof(Params, lim_torque_hold), 0, 36,  0.5f,   2,      "Nm",     1 },
  { "Deadband",    "db",    offsetof(Params, pos_deadband),   0,    5,  0.05f,  0.5f,   "deg",    2 },
  { "Speed lim",   "vlim",  offsetof(Params, lim_speed),   0,    50,  1,      5,      "rad/s",  1 },
  { "Accel lim",   "alim",  offsetof(Params, lim_acc),     10, 3000,  25,     250,    "r/s2",   0 },
  { "Decel lim",   "dlim",  offsetof(Params, lim_dec),     10, 3000,  25,     250,    "r/s2",   0 },
  { "Pos min",     "pmin",  offsetof(Params, pos_min),    -12,    0,  0.1f,   1,      "rad",    2 },
  { "Pos max",     "pmax",  offsetof(Params, pos_max),      0,   12,  0.1f,   1,      "rad",    2 },
  { "J hat",       "jhat",  offsetof(Params, j_hat),   0.0001f,   1,  0.0005f,0.005f, "kgm2",   4 },
  { "Friction Fc", "fc",    offsetof(Params, fric_c),       0,    5,  0.01f,  0.1f,   "Nm",     3 },
  { "Friction Fv", "fv",    offsetof(Params, fric_v),       0,    1,  0.002f, 0.02f,  "Nms",    4 },
  { "Kt",          "kt",    offsetof(Params, kt),        0.1f,    5,  0.01f,  0.1f,   "Nm/A",   3 },
  { "Temp derate", "tder",  offsetof(Params, temp_derate), 40,  140,  1,      5,      "C",      0 },
  { "Temp trip",   "ttrip", offsetof(Params, temp_trip),   50,  145,  1,      5,      "C",      0 },
  { "VBUS margin", "vmarg", offsetof(Params, vbus_margin),  1,   10,  0.5f,   1,      "V",      1 },
  { "VBUS min",    "vmin",  offsetof(Params, vbus_min),    10,   55,  0.5f,   2,      "V",      1 },
  { "Regen limit", "regen", offsetof(Params, regen_w_max), 50, 3000,  25,     200,    "W",      0 },
};

const int N_PDEF = (int)(sizeof(PDEF) / sizeof(PDEF[0]));

int paramIndexByKey(const char* key) {
  if (!key) return -1;
  for (int i = 0; i < N_PDEF; i++) {
    if (strcmp(PDEF[i].key, key) == 0) return i;
  }
  return -1;
}
