#include "nvs_store.h"
#include <Preferences.h>

static const char* NS = "rs06";

namespace store {

void load(Params& p) {
  Preferences pref;
  if (!pref.begin(NS, true)) return;
  if (pref.getBytesLength("blob") == sizeof(Params)) {
    Params tmp;
    pref.getBytes("blob", &tmp, sizeof(Params));
    // Same size but an older revision: keep the compiled-in defaults rather
    // than resurrecting stale wiring.
    if (tmp.magic == PARAMS_MAGIC) p = tmp;
  }
  pref.end();
}

void save(const Params& p) {
  Preferences pref;
  if (!pref.begin(NS, false)) return;
  pref.putBytes("blob", &p, sizeof(Params));
  pref.end();
}

void clear() {
  Preferences pref;
  if (!pref.begin(NS, false)) return;
  pref.clear();
  pref.end();
}

} // namespace store
