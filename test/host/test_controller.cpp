// ---------------------------------------------------------------------------
// Host tests for the controller's safety layer.
//
// controller.cpp keeps its state in an anonymous namespace, so these drive it
// the way the rest of the firmware does - through ctrl::request() and
// ctrl::snapshot() - and read the results out of the published telemetry. The
// control task is not spawned on the host (see stubs_impl.cpp); ctrl::begin()
// runs the same setup and the test steps time forward itself.
//
// What is pinned down here is the part the captured test history showed was
// broken: the difference between a fault that happened and a limit that is
// biting right now, and the refusal to act on a supply voltage nobody has
// measured recently.
// ---------------------------------------------------------------------------
#include "../../src/controller.h"
#include "../../src/rs06_proto.h"
#include "fake_bus.h"

#include <cstdio>
#include <cstring>
#include <cmath>

using namespace rs06;

static int g_fail = 0;
static int g_run  = 0;

#define CHECK(cond, ...) do {                                        \
    g_run++;                                                         \
    if (!(cond)) { g_fail++;                                         \
      std::printf("  FAIL %s:%d  ", __FILE__, __LINE__);             \
      std::printf(__VA_ARGS__); std::printf("\n"); }                 \
  } while (0)

static void section(const char* s) { std::printf("\n== %s\n", s); }

static constexpr uint8_t MOTOR = 0x7F;

// --- a motor that answers, with settable readings --------------------------
static float g_vbus   = 32.3f;
static float g_iqf    = -4.25f;
static float g_pos    = 0.0f;
static float g_vel    = 0.0f;
static float g_temp   = 40.0f;
static bool  g_answer_aux = true;
static int64_t g_aux_latency_us = 900;   // deliberately later than one slot

static twai_message_t feedbackFrame() {
  twai_message_t m{};
  m.identifier = make_id(CT_FEEDBACK, (uint16_t)MOTOR, HOST_ID);
  m.extd = 1; m.data_length_code = 8;
  be_put16(&m.data[0], f2u16(g_pos, -P_LIM, P_LIM));
  be_put16(&m.data[2], f2u16(g_vel, -V_LIM, V_LIM));
  be_put16(&m.data[4], f2u16(0.0f, -T_LIM, T_LIM));
  be_put16(&m.data[6], (uint16_t)(g_temp * 10.0f));
  return m;
}

static void motorReplies(const twai_message_t& sent) {
  switch (id_type(sent.identifier)) {
  case CT_MOTION_CTRL: case CT_ENABLE: case CT_STOP:
  case CT_SET_ZERO:    case CT_PARAM_WRITE:
    fake::schedule(feedbackFrame(), 200);
    break;
  case CT_PARAM_READ: {
    if (!g_answer_aux) break;
    const uint16_t idx = (uint16_t)sent.data[0] | ((uint16_t)sent.data[1] << 8);
    twai_message_t m{};
    m.identifier = make_id(CT_PARAM_READ, (uint16_t)MOTOR, HOST_ID);
    m.extd = 1; m.data_length_code = 8;
    le_put16(&m.data[0], idx);
    le_put_f32(&m.data[4], (idx == P_VBUS) ? g_vbus : (idx == P_IQF) ? g_iqf : 0.0f);
    fake::schedule(m, g_aux_latency_us);
    break;
  }
  case CT_FAULT_FB: {
    if (!g_answer_aux) break;
    twai_message_t m{};
    m.identifier = make_id(CT_FAULT_FB, (uint16_t)MOTOR, HOST_ID);
    m.extd = 1; m.data_length_code = 8;
    le_put32(&m.data[0], 0);
    le_put32(&m.data[4], 0);
    fake::schedule(m, g_aux_latency_us);
    break;
  }
  case CT_ACTIVE_REP: {
    twai_message_t m{};
    m.identifier = make_id(CT_ACTIVE_REP, (uint16_t)MOTOR, HOST_ID);
    m.extd = 1; m.data_length_code = 8;
    fake::schedule(m, 200);
    break;
  }
  default: break;
  }
}

// controller.cpp compiles its private tick() behind this hook when built with
// CTRL_HOST_TEST, so the loop can be stepped without an RTOS.
extern "C" void ctrl_host_tick();

// One control slot is 1 ms of wall clock whether or not the work filled it.
// Topping the virtual clock up here is what makes ages, timeouts and the
// aux schedule mean the same thing they do on the hardware.
static void ticks(int n) {
  for (int i = 0; i < n; i++) {
    const int64_t t0 = fake::nowUs();
    ctrl_host_tick();
    const int64_t spent = fake::nowUs() - t0;
    if (spent < (int64_t)CTRL_PERIOD_US) fake::advance(CTRL_PERIOD_US - spent);
  }
}

static Telemetry snap() { Telemetry t; ctrl::snapshot(t); return t; }

static void bootController() {
  fake::reset();
  fake::auto_reply = motorReplies;
  g_answer_aux = true;
  g_vbus = 32.3f; g_iqf = -4.25f; g_pos = 0.0f; g_vel = 0.0f; g_temp = 40.0f;
  ctrl::begin(nullptr);
}

// ===========================================================================
int main() {
  std::printf("controller host tests");

  // -----------------------------------------------------------------------
  section("auxiliary readings arrive despite being slower than a slot");
  {
    bootController();
    ticks(60);                       // 60 ms of control loop
    Telemetry t = snap();
    CHECK(t.aux_ok, "VBUS must be fresh after 60 slots (age %u ms)", t.aux_age_ms);
    CHECK(std::fabs(t.vbus - 32.3f) < 0.01f, "VBUS: got %.3f", t.vbus);
    CHECK(std::fabs(t.iq + 4.25f) < 0.01f,
          "phase current must actually update - it never did in the captures: got %.3f",
          t.iq);
  }

  // -----------------------------------------------------------------------
  section("a silent auxiliary channel is reported, not papered over");
  {
    bootController();
    ticks(60);
    CHECK(snap().aux_ok, "fresh to begin with");

    g_answer_aux = false;            // the motor stops answering reads
    ticks(300);                      // 300 ms
    Telemetry t = snap();
    CHECK(!t.aux_ok, "stale VBUS must be flagged, age %u ms", t.aux_age_ms);
    CHECK(t.aux_age_ms > AUX_STALE_MS, "age must grow: %u ms", t.aux_age_ms);
  }

  // -----------------------------------------------------------------------
  section("the supply reference is a median of rest samples");
  {
    bootController();
    // A single outlier arrives first - the reading the old one-sample seed
    // would have built every threshold on for the rest of the session.
    g_vbus = 45.0f;  ticks(8);
    g_vbus = 32.3f;  ticks(200);
    Telemetry t = snap();
    // vbus_margin is 4 V, so a 32.3 V reference puts the trip at 36.3.
    // Latching the outlier instead would have given 49.
    CHECK(std::fabs(t.vbus_trip - 36.3f) < 0.2f,
          "trip must follow the resting supply, not one stray sample: got %.2f",
          t.vbus_trip);
  }

  // -----------------------------------------------------------------------
  section("regen limiting is instantaneous, faults latch");
  {
    bootController();
    ticks(40);

    CtrlRequest r{}; r.type = REQ_START; ctrl::request(r);
    ticks(5);

    // Put the axis where the motor says it is BEFORE commanding the move, or
    // the profile plans from a stale position, finishes instantly and nothing
    // ever brakes.
    g_pos = 1.0f; g_vel = 31.5f;
    ticks(3);

    // Command the opposite direction at the measured 31.5 rad/s, so the
    // profile brakes hard against real motion: 600 W / 31.5 rad/s = 19.05 Nm,
    // just under the 20 Nm torque limit, which is exactly the everyday
    // condition that used to light REGEN-LIM permanently.
    r.type = REQ_SET_TARGET_POS; r.arg_f = -1.0f; ctrl::request(r);
    ticks(2);

    bool saw_active_regen = false;
    for (int i = 0; i < 60; i++) {
      ticks(1);
      if (snap().active & EV_REGEN_LIMIT) saw_active_regen = true;
    }
    CHECK(saw_active_regen, "the regen cap should show up in the active word");

    // Now stop moving. The instantaneous word must clear itself.
    g_vel = 0.0f;
    r.type = REQ_STOP; ctrl::request(r);
    ticks(20);
    Telemetry t = snap();
    CHECK(!(t.active & EV_REGEN_LIMIT),
          "regen must clear once it stops applying, active=0x%08X", t.active);
    CHECK(!(t.events & EV_REGEN_LIMIT),
          "and must never have latched, events=0x%08X", t.events);
  }

  // -----------------------------------------------------------------------
  section("Type21 causes map to distinct events");
  {
    struct Case { uint32_t flt; uint32_t ev; const char* name; };
    const Case cases[] = {
      { FLT_DRIVER_CHIP, EV_DRIVER_CHIP, "driver chip" },
      { FLT_POS_INIT,    EV_POS_INIT,    "position init" },
      { FLT_HW_ID,       EV_HW_ID,       "hardware id" },
      { FLT_OVERTEMP,    EV_TEMP_TRIP,   "overtemp" },
      { FLT_IB_OVERCUR,  EV_OVERCURRENT, "phase overcurrent" },
    };
    for (const Case& c : cases) {
      bootController();
      ticks(20);
      // Answer the fault poll with this cause.
      fake::auto_reply = nullptr;
      twai_message_t m{};
      m.identifier = make_id(CT_FAULT_FB, (uint16_t)MOTOR, HOST_ID);
      m.extd = 1; m.data_length_code = 8;
      le_put32(&m.data[0], c.flt);
      le_put32(&m.data[4], 0);
      fake::queueNow(m);
      ticks(3);

      Telemetry t = snap();
      CHECK((t.events & c.ev) != 0,
            "%s must set its own event bit, events=0x%08X", c.name, t.events);
      CHECK(t.state == ST_FAULT_HARD, "%s must be a hard fault", c.name);
      // The old code reported all five of these as OVERCURRENT.
      if (c.ev != EV_OVERCURRENT) {
        CHECK(!(t.events & EV_OVERCURRENT),
              "%s must NOT be reported as overcurrent, events=0x%08X", c.name, t.events);
      }
      fake::auto_reply = motorReplies;
    }
  }

  // -----------------------------------------------------------------------
  section("the supply floor scales with the supply actually fitted");
  {
    // A 13S pack and a 32 V bench supply need different undervoltage floors,
    // and the fixed 14 V default protects neither. The floor is derived from
    // the measured rest voltage instead, so the same build guards both.
    bootController();
    g_vbus = 32.3f;
    ticks(200);                       // reference settles at 32.3 -> floor 22.6
    CHECK(snap().state != ST_FAULT_HARD, "healthy supply must not trip");

    g_vbus = 20.0f;                   // 62 % of rest: a collapse, not a sag
    ticks(40);
    CHECK(snap().state == ST_FAULT_HARD,
          "a rail at 62 %% of its rest voltage must trip even though 20 V is "
          "above the fixed 14 V floor, state=%s", ctrl::stateName(snap().state));
    CHECK((snap().events & EV_UNDERVOLT) != 0,
          "and must say why: events=0x%08X", snap().events);
  }

  // -----------------------------------------------------------------------
  section("a worst-case measured sag does not trip");
  {
    // The deepest sag in the captured history is 32.3 -> 28.6 V on a
    // full-effort move, about 11 %. The guard must have room for that.
    bootController();
    g_vbus = 32.3f;
    ticks(200);
    g_vbus = 28.6f;
    ticks(60);
    CHECK(snap().state != ST_FAULT_HARD,
          "an 11 %% sag is normal and must not trip, state=%s",
          ctrl::stateName(snap().state));
  }

  std::printf("\n%d checks, %d failed\n", g_run, g_fail);
  return g_fail ? 1 : 0;
}
