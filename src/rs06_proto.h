// ---------------------------------------------------------------------------
// rs06_proto.h : RobStride RS06 private CAN protocol (CAN2.0B, 1 Mbps)
//
// 29-bit ID layout:  bit28..24 = communication type
//                    bit23..8  = data area 2
//                    bit7..0   = destination address
//
// ENDIANNESS WARNING - the protocol is deliberately inconsistent:
//   * Type 1 / Type 2   8-byte payload fields are BIG endian
//     ("the high byte is in front and the low byte is in the rear")
//   * Type 17 / Type 18 index (Byte0..1) and data (Byte4..7) are LITTLE endian
//     ("the low byte is first and the high byte is second")
//   Verified against the manual's loc_kp read example:
//     response 1E 70 00 00 00 00 F0 41 -> 0x41F00000 -> 30.0f
// ---------------------------------------------------------------------------
#pragma once
#include <stdint.h>
#include <string.h>
#include <math.h>

namespace rs06 {

// --- Full-scale ranges of the Type1/Type2 packed fields --------------------
constexpr float P_LIM  = 12.566370614f;   // +-4*pi rad (output shaft)
constexpr float V_LIM  = 50.0f;           // +-50 rad/s
constexpr float T_LIM  = 36.0f;           // +-36 N.m
constexpr float KP_LIM = 5000.0f;
constexpr float KD_LIM = 100.0f;

enum CommType : uint8_t {
  CT_GET_ID      = 0x00,
  CT_MOTION_CTRL = 0x01,
  CT_FEEDBACK    = 0x02,
  CT_ENABLE      = 0x03,
  CT_STOP        = 0x04,
  CT_SET_ZERO    = 0x06,
  CT_SET_CANID   = 0x07,
  CT_PARAM_READ  = 0x11,
  CT_PARAM_WRITE = 0x12,
  CT_FAULT_FB    = 0x15,
  CT_SAVE        = 0x16,
  CT_BAUD        = 0x17,
  CT_ACTIVE_REP  = 0x18,
  CT_PROTOCOL    = 0x19,
};

enum ParamIdx : uint16_t {
  P_RUN_MODE     = 0x7005, P_IQ_REF   = 0x7006, P_SPD_REF   = 0x700A,
  P_LIMIT_TORQUE = 0x700B, P_CUR_KP   = 0x7010, P_CUR_KI    = 0x7011,
  P_CUR_FILT     = 0x7014, P_LOC_REF  = 0x7016, P_LIMIT_SPD = 0x7017,
  P_LIMIT_CUR    = 0x7018, P_MECH_POS = 0x7019, P_IQF       = 0x701A,
  P_MECH_VEL     = 0x701B, P_VBUS     = 0x701C, P_LOC_KP    = 0x701E,
  P_SPD_KP       = 0x701F, P_SPD_KI   = 0x7020, P_SPD_FILT  = 0x7021,
  P_ACC_RAD      = 0x7022, P_VEL_MAX  = 0x7024, P_ACC_SET   = 0x7025,
  P_EPSCAN       = 0x7026, P_CAN_TMO  = 0x7028, P_ZERO_STA  = 0x7029,
  P_DAMPER       = 0x702A, P_ADD_OFF  = 0x702B, P_ALVEOLOUS = 0x702C,
  P_IQ_TEST      = 0x702D, P_DCC_SET  = 0x702E,
};

enum RunMode : uint8_t {
  MODE_MOTION = 0, MODE_POS_PP = 1, MODE_VELOCITY = 2,
  MODE_CURRENT = 3, MODE_POS_CSP = 5,
};

// --- Type2 status word (ID bit16..23) --------------------------------------
// NOTE: there is no overvoltage bit here. Overvoltage lives only in the
// Type21 fault frame (bit3) - see FaultBits below.
enum Fb2Fault : uint8_t {
  FB2_UNDERVOLTAGE = 1 << 0,   // ID bit16
  FB2_OVERCURRENT  = 1 << 1,   // ID bit17  (three phase)
  FB2_OVERTEMP     = 1 << 2,   // ID bit18
  FB2_ENCODER      = 1 << 3,   // ID bit19  (magnetic encoding fault)
  FB2_STALL        = 1 << 4,   // ID bit20  (stall / overload)
  FB2_UNCALIBRATED = 1 << 5,   // ID bit21
};

enum ModeState : uint8_t { MS_RESET = 0, MS_CALI = 1, MS_MOTOR = 2 };

// --- Type21 fault frame, Byte0..3 ------------------------------------------
enum FaultBits : uint32_t {
  FLT_OVERTEMP      = 1u << 0,
  FLT_DRIVER_CHIP   = 1u << 1,
  FLT_UNDERVOLTAGE  = 1u << 2,
  FLT_OVERVOLTAGE   = 1u << 3,   // <-- the regen-braking failure mode
  FLT_IB_OVERCUR    = 1u << 4,
  FLT_IC_OVERCUR    = 1u << 5,
  FLT_ENC_UNCAL     = 1u << 7,
  FLT_HW_ID         = 1u << 8,
  FLT_POS_INIT      = 1u << 9,
  FLT_STALL_ALGO    = 1u << 14,
  FLT_IA_OVERCUR    = 1u << 16,
};
enum WarnBits : uint32_t { WRN_OVERTEMP = 1u << 0 };

// --- Fixed point helpers ---------------------------------------------------
static inline uint16_t f2u16(float x, float xmin, float xmax) {
  if (x < xmin) x = xmin;
  if (x > xmax) x = xmax;
  return (uint16_t)((x - xmin) * 65535.0f / (xmax - xmin) + 0.5f);
}
static inline float u162f(uint16_t v, float xmin, float xmax) {
  return (float)v * (xmax - xmin) / 65535.0f + xmin;
}

// --- ID helpers ------------------------------------------------------------
static inline uint32_t make_id(uint8_t type, uint16_t area2, uint8_t dest) {
  return ((uint32_t)type << 24) | ((uint32_t)area2 << 8) | dest;
}
static inline uint8_t  id_type(uint32_t id)  { return (uint8_t)((id >> 24) & 0x1F); }
static inline uint16_t id_area2(uint32_t id) { return (uint16_t)((id >> 8) & 0xFFFF); }
static inline uint8_t  id_dest(uint32_t id)  { return (uint8_t)(id & 0xFF); }

// --- Big endian pack/unpack (Type1 / Type2 payloads) -----------------------
static inline void be_put16(uint8_t* p, uint16_t v) { p[0] = v >> 8; p[1] = v & 0xFF; }
static inline uint16_t be_get16(const uint8_t* p)   { return ((uint16_t)p[0] << 8) | p[1]; }

// --- Little endian pack/unpack (Type17 / Type18 payloads) ------------------
static inline void le_put16(uint8_t* p, uint16_t v) { p[0] = v & 0xFF; p[1] = v >> 8; }
static inline void le_put32(uint8_t* p, uint32_t v) {
  p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
}
static inline uint32_t le_get32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline void le_put_f32(uint8_t* p, float f) {
  uint32_t u; memcpy(&u, &f, 4); le_put32(p, u);
}
static inline float le_get_f32(const uint8_t* p) {
  uint32_t u = le_get32(p); float f; memcpy(&f, &u, 4); return f;
}

// --- Frame builders --------------------------------------------------------
// Type 1: torque feed-forward rides in the ID (bit23..8), not in the payload.
struct MotionCmd { float p_set, v_set, kp, kd, t_ff; };

static inline void build_motion(const MotionCmd& c, uint8_t motor_id,
                                uint32_t& id, uint8_t d[8]) {
  id = make_id(CT_MOTION_CTRL, f2u16(c.t_ff, -T_LIM, T_LIM), motor_id);
  be_put16(&d[0], f2u16(c.p_set, -P_LIM, P_LIM));
  be_put16(&d[2], f2u16(c.v_set, -V_LIM, V_LIM));
  be_put16(&d[4], f2u16(c.kp,    0.0f,   KP_LIM));
  be_put16(&d[6], f2u16(c.kd,    0.0f,   KD_LIM));
}

struct Feedback {
  float    pos;        // rad, output shaft, wraps at +-4*pi
  float    vel;        // rad/s
  float    torque;     // N.m
  float    temp;       // degC
  uint8_t  motor_id;
  uint8_t  fault;      // Fb2Fault bitfield
  uint8_t  mode_state; // ModeState
};

static inline bool parse_feedback(uint32_t id, const uint8_t d[8], Feedback& fb) {
  if (id_type(id) != CT_FEEDBACK) return false;
  uint16_t a2  = id_area2(id);
  fb.motor_id  = (uint8_t)(a2 & 0xFF);          // ID bit8..15
  fb.fault     = (uint8_t)((a2 >> 8) & 0x3F);   // ID bit16..21
  fb.mode_state= (uint8_t)((a2 >> 14) & 0x03);  // ID bit22..23
  fb.pos    = u162f(be_get16(&d[0]), -P_LIM, P_LIM);
  fb.vel    = u162f(be_get16(&d[2]), -V_LIM, V_LIM);
  fb.torque = u162f(be_get16(&d[4]), -T_LIM, T_LIM);
  fb.temp   = (float)be_get16(&d[6]) * 0.1f;
  return true;
}

} // namespace rs06
