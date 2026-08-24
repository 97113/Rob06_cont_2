// ---------------------------------------------------------------------------
// param_table.h : the one description of every editable tuning value.
//
// Shared by the on-device CONFIG page and the USB serial link, so a parameter
// added here shows up in both without further work.
//   name = human label for the 320x240 screen
//   key  = short token used by the serial protocol (SET <key> <value>)
// ---------------------------------------------------------------------------
#pragma once
#include "types.h"
#include <stddef.h>
#include <stdint.h>

struct ParamDef {
  const char* name;
  const char* key;
  size_t      off;
  float       mn, mx, step, big;
  const char* unit;
  uint8_t     dec;
};

extern const ParamDef PDEF[];
extern const int      N_PDEF;

inline float* pfield(Params& p, int i) {
  return (float*)((uint8_t*)&p + PDEF[i].off);
}
inline const float* pfield(const Params& p, int i) {
  return (const float*)((const uint8_t*)&p + PDEF[i].off);
}
int paramIndexByKey(const char* key);   // -1 when unknown
