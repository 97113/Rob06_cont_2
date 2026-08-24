// ---------------------------------------------------------------------------
// Host tests for the RS06 CAN transport.
//
// These exist because of a specific field failure: across the twelve 1 kHz
// captures in tools/, VBUS updated 0.7 times a second against a design rate of
// 200 Hz and the phase current never updated at all, which left every
// protection built on VBUS running blind. The cause was that an auxiliary
// reply arriving after its slot was swallowed and discarded by the next
// slot's feedback wait. The tests below pin that behaviour down so it cannot
// come back.
// ---------------------------------------------------------------------------
#include "../../src/rs06_driver.h"
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

// --- reply builders --------------------------------------------------------
static constexpr uint8_t MOTOR = 0x7F;

static twai_message_t feedbackFrame(float pos, float vel, float trq, float temp) {
  twai_message_t m{};
  m.identifier = make_id(CT_FEEDBACK, (uint16_t)MOTOR, HOST_ID);
  m.extd = 1;
  m.data_length_code = 8;
  be_put16(&m.data[0], f2u16(pos, -P_LIM, P_LIM));
  be_put16(&m.data[2], f2u16(vel, -V_LIM, V_LIM));
  be_put16(&m.data[4], f2u16(trq, -T_LIM, T_LIM));
  be_put16(&m.data[6], (uint16_t)(temp * 10.0f));
  return m;
}

static twai_message_t paramFrame(uint16_t idx, float value) {
  twai_message_t m{};
  m.identifier = make_id(CT_PARAM_READ, (uint16_t)MOTOR, HOST_ID);
  m.extd = 1;
  m.data_length_code = 8;
  le_put16(&m.data[0], idx);
  le_put_f32(&m.data[4], value);
  return m;
}

static twai_message_t faultFrame(uint32_t fault, uint32_t warn) {
  twai_message_t m{};
  m.identifier = make_id(CT_FAULT_FB, (uint16_t)MOTOR, HOST_ID);
  m.extd = 1;
  m.data_length_code = 8;
  le_put32(&m.data[0], fault);
  le_put32(&m.data[4], warn);
  return m;
}

static bool isType(const twai_message_t& m, uint8_t ty) { return id_type(m.identifier) == ty; }

// ---------------------------------------------------------------------------
// A motor that answers Type1 promptly and auxiliary reads slowly - the real
// timing relationship, and the one the old code could not survive.
// ---------------------------------------------------------------------------
static int64_t g_motion_latency_us = 200;
static int64_t g_aux_latency_us    = 800;
static float   g_vbus_value        = 32.3f;
static float   g_iqf_value         = -4.25f;
static float   g_pos_value         = 0.0f;

static void motorReplies(const twai_message_t& sent) {
  switch (id_type(sent.identifier)) {
  case CT_MOTION_CTRL:
  case CT_ENABLE:
  case CT_STOP:
  case CT_SET_ZERO:
    fake::schedule(feedbackFrame(g_pos_value, 1.5f, 0.25f, 41.0f), g_motion_latency_us);
    break;
  case CT_PARAM_READ: {
    const uint16_t idx = (uint16_t)sent.data[0] | ((uint16_t)sent.data[1] << 8);
    const float v = (idx == P_VBUS) ? g_vbus_value
                  : (idx == P_IQF)  ? g_iqf_value
                                    : 0.0f;
    fake::schedule(paramFrame(idx, v), g_aux_latency_us);
    break;
  }
  case CT_FAULT_FB:
    fake::schedule(faultFrame(FLT_OVERVOLTAGE, 0), g_aux_latency_us);
    break;
  default:
    break;
  }
}

static void startDriver(Rs06& can) {
  fake::reset();
  fake::auto_reply = motorReplies;
  can.begin(MOTOR, 27, 19, 1000000);
}

// One control tick: the motion exchange, then the scheduled aux request.
static void runTick(Rs06& can, int n, uint16_t aux_idx = 0, bool want_fault = false) {
  for (int i = 0; i < n; i++) {
    can.pump();
    Feedback fb;
    can.sendMotion(MotionCmd{ 0, 0, 0, 0, 0 }, fb);
    if (aux_idx)    can.requestParam(aux_idx);
    if (want_fault) can.requestFault();
    // The slot ends here whether or not the aux answer has arrived, exactly
    // as the control task's 1 ms budget forces.
  }
}

// ===========================================================================
int main() {
  std::printf("rs06_driver host tests");

  // -----------------------------------------------------------------------
  section("motion exchange completes inside the slot");
  {
    Rs06 can;
    g_motion_latency_us = 200;
    g_pos_value = 1.234f;
    startDriver(can);

    Feedback fb{};
    const bool ok = can.sendMotion(MotionCmd{ 0.5f, 1.0f, 30.0f, 1.5f, 2.0f }, fb);
    CHECK(ok, "sendMotion should succeed with a 200 us reply");
    CHECK(std::fabs(fb.pos - 1.234f) < 0.01f, "position round trip: got %.4f", fb.pos);
    CHECK(can.motionTxCount() == 1, "one Type1 counted, got %u", can.motionTxCount());
    CHECK(can.missCount() == 0, "no miss expected, got %u", can.missCount());

    // The torque feed-forward rides in the ID, not the payload.
    CHECK(!fake::tx_log.empty() && isType(fake::tx_log[0], CT_MOTION_CTRL), "Type1 sent");
    const float t_ff = u162f(id_area2(fake::tx_log[0].identifier), -T_LIM, T_LIM);
    CHECK(std::fabs(t_ff - 2.0f) < 0.01f, "t_ff in the ID: got %.4f", t_ff);
  }

  // -----------------------------------------------------------------------
  section("a stale queued feedback does not satisfy the next exchange");
  {
    Rs06 can;
    g_motion_latency_us = 200;
    g_pos_value = 2.0f;
    startDriver(can);

    // Something from the previous slot is already sitting in the queue.
    fake::queueNow(feedbackFrame(9.9f, 0, 0, 0));

    Feedback fb{};
    const bool ok = can.sendMotion(MotionCmd{ 0, 0, 0, 0, 0 }, fb);
    CHECK(ok, "exchange should still complete");
    CHECK(std::fabs(fb.pos - 2.0f) < 0.02f,
          "must report THIS slot's position, not the stale 9.9: got %.4f", fb.pos);
  }

  // -----------------------------------------------------------------------
  section("a reply slower than the slot is still harvested");
  {
    Rs06 can;
    g_motion_latency_us = 200;
    g_aux_latency_us    = 800;   // lands two slots after the request
    g_vbus_value        = 32.3f;
    startDriver(can);

    // Ask once, then keep running normal slots. The answer arrives during a
    // later slot's feedback wait - the exact case the old code threw away.
    can.requestParam(P_VBUS);
    runTick(can, 5);

    float v = 0; uint32_t age = 0;
    const bool got = can.paramValue(P_VBUS, v, age);
    CHECK(got, "a late VBUS reply must still reach the cache");
    CHECK(std::fabs(v - 32.3f) < 1e-3f, "VBUS value: got %.4f", v);
    CHECK(age < AUX_STALE_MS, "and must be fresh: age %u ms", age);
  }

  // -----------------------------------------------------------------------
  section("sustained polling keeps the reading fresh");
  {
    Rs06 can;
    g_motion_latency_us = 200;
    g_aux_latency_us    = 800;
    startDriver(can);

    // 50 slots, requesting VBUS on every fifth - the 200 Hz schedule.
    for (int i = 0; i < 50; i++) runTick(can, 1, (i % 5 == 0) ? P_VBUS : 0);

    float v = 0; uint32_t age = 0;
    CHECK(can.paramValue(P_VBUS, v, age), "VBUS must be present after 50 slots");
    CHECK(age <= AUX_STALE_MS, "VBUS must stay fresh under the real schedule: age %u ms", age);
    CHECK(can.missCount() == 0,
          "aux polling must not count as a missed exchange, got %u", can.missCount());
  }

  // -----------------------------------------------------------------------
  section("replies are filed per index, not first-come");
  {
    Rs06 can;
    startDriver(can);
    fake::auto_reply = nullptr;         // hand-place the replies out of order

    can.requestParam(P_VBUS);
    can.requestParam(P_IQF);
    fake::queueNow(paramFrame(P_IQF,  -4.25f));   // answers arrive reversed
    fake::queueNow(paramFrame(P_VBUS, 48.6f));
    can.pump();

    float v = 0, iq = 0; uint32_t age = 0;
    CHECK(can.paramValue(P_VBUS, v, age) && std::fabs(v - 48.6f) < 1e-3f,
          "VBUS slot: got %.4f", v);
    CHECK(can.paramValue(P_IQF, iq, age) && std::fabs(iq + 4.25f) < 1e-3f,
          "IQF slot: got %.4f", iq);

    // An index nobody asked about must not masquerade as data.
    float other = 0;
    CHECK(!can.paramValue(P_MECH_POS, other, age), "unrequested index must read as absent");
  }

  // -----------------------------------------------------------------------
  section("a quiet auxiliary channel shows up as age, not as a stale value");
  {
    Rs06 can;
    g_aux_latency_us = 800;
    startDriver(can);

    can.requestParam(P_VBUS);
    runTick(can, 5);
    float v = 0; uint32_t age0 = 0;
    CHECK(can.paramValue(P_VBUS, v, age0), "value present");

    // The motor goes quiet for a while.
    fake::auto_reply = nullptr;
    fake::advance(300000);              // 300 ms of silence
    uint32_t age1 = 0;
    can.paramValue(P_VBUS, v, age1);
    CHECK(age1 > AUX_STALE_MS,
          "age must expose the silence: %u ms (was %u)", age1, age0);
    CHECK(std::fabs(v - 32.3f) < 1e-3f,
          "the last value is still readable - it is the AGE that says do not trust it");
  }

  // -----------------------------------------------------------------------
  section("Type21 fault frame decodes with an age");
  {
    Rs06 can;
    g_aux_latency_us = 400;
    startDriver(can);

    uint32_t f = 0, w = 0, age = 0;
    CHECK(!can.faultWords(f, w, age), "nothing reported before the first reply");

    can.requestFault();
    runTick(can, 3);
    CHECK(can.faultWords(f, w, age), "fault frame must be filed");
    CHECK(f == FLT_OVERVOLTAGE, "fault word: got 0x%08X", f);
    CHECK(age <= AUX_STALE_MS, "fault age: %u ms", age);
  }

  // -----------------------------------------------------------------------
  section("blocking readParamF waits for ITS index");
  {
    Rs06 can;
    startDriver(can);
    fake::auto_reply = nullptr;

    // A leftover answer for a different index is in the queue first. The old
    // implementation accepted the first Type17 frame it saw, failed the index
    // check and returned false - killing a read that would have succeeded.
    fake::queueNow(paramFrame(P_IQF, -1.0f));
    fake::schedule(paramFrame(P_VBUS, 51.2f), 250);

    float v = 0;
    CHECK(can.readParamF(P_VBUS, v), "read must survive an interleaved reply");
    CHECK(std::fabs(v - 51.2f) < 1e-3f, "value: got %.4f", v);
  }

  // -----------------------------------------------------------------------
  section("a dead motor is reported as a miss, not as data");
  {
    Rs06 can;
    startDriver(can);
    fake::auto_reply = nullptr;         // nothing ever answers

    Feedback fb{};
    CHECK(!can.sendMotion(MotionCmd{ 0, 0, 0, 0, 0 }, fb), "must fail");
    CHECK(can.missCount() == 1, "one miss, got %u", can.missCount());

    float v = 0; uint32_t age = 0;
    CHECK(!can.paramValue(P_VBUS, v, age), "no VBUS reading should exist");
  }

  // -----------------------------------------------------------------------
  section("resetCache forgets readings from the previous wiring");
  {
    Rs06 can;
    g_aux_latency_us = 200;
    startDriver(can);
    can.requestParam(P_VBUS);
    runTick(can, 3);

    float v = 0; uint32_t age = 0;
    CHECK(can.paramValue(P_VBUS, v, age), "value present before reset");
    can.resetCache();
    CHECK(!can.paramValue(P_VBUS, v, age),
          "after a CAN re-init the old supply reading must be gone");
  }

  // -----------------------------------------------------------------------
  section("fixed point round trips");
  {
    // Type1 quantises position to 8*pi/65535 rad = 0.022 deg.
    for (float p : { -12.0f, -3.4907f, 0.0f, 0.5f, 3.4907f, 12.0f }) {
      const float back = u162f(f2u16(p, -P_LIM, P_LIM), -P_LIM, P_LIM);
      CHECK(std::fabs(back - p) < 4e-4f, "position %.4f -> %.4f", p, back);
    }
    // The manual's worked example: loc_kp reads back as 30.0.
    const uint8_t resp[8] = { 0x1E, 0x70, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x41 };
    CHECK(std::fabs(le_get_f32(&resp[4]) - 30.0f) < 1e-6f,
          "manual loc_kp example: got %.4f", le_get_f32(&resp[4]));
  }

  std::printf("\n%d checks, %d failed\n", g_run, g_fail);
  return g_fail ? 1 : 0;
}
