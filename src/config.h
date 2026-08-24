// ---------------------------------------------------------------------------
// config.h : build-time wiring and default tuning values
// ---------------------------------------------------------------------------
#pragma once
#include <stdint.h>

// --- CAN wiring (Module13.2 PwrCAN "CAN Select" DIP -> Core2 M5-Bus) ------
// Only CANH/CANL are wired to the RS06; the XT30 power pins of the PwrCAN are
// tied straight to its 9-24V DC jack and must never see the 48V motor rail.
//
//   DIP   dir  Core2 GPIO   M5-Bus pin
//   CH1   TX   G14          16
//   CH2   TX   G2           23
//   CH3   TX   G27          21
//   CH4   TX   G0           24     (strapping pin - avoid)
//   CH5   RX   G13          15
//   CH6   RX   G19          22
//   CH7   RX   G34          26     (input only)
//   CH8   RX   G35           2     (input only)
//
// Exactly one CH1..CH4 and one CH5..CH8 must be ON, and the RS485 Select DIP
// must not claim the same pins.
//
// This build is wired CH3 (TX G27) + CH6 (RX G19), measured on the actual
// hardware.
//
// These are only the power-on defaults: the live wiring lives in Params
// (can_tx / can_rx / can_baud) so the boot-time CAN scanner can correct it
// without a rebuild. Hold BtnA at power-up to run the scanner.
#define PIN_CAN_RX   GPIO_NUM_19   // DIP CH6
#define PIN_CAN_TX   GPIO_NUM_27   // DIP CH3

// --- CAN identities --------------------------------------------------------
static constexpr uint8_t  DEFAULT_MOTOR_ID = 0x7F;
static constexpr uint8_t  HOST_ID          = 0xFD;

// Bumped whenever the Params layout or its defaults change in a way that must
// override whatever is already sitting in NVS. A stored blob whose magic does
// not match is discarded and the defaults below take over.
static constexpr uint32_t PARAMS_MAGIC = 0x52533604;   // 'RS6' + revision

// --- Task layout -----------------------------------------------------------
static constexpr int      CTRL_CORE       = 1;
static constexpr int      UI_CORE         = 0;
static constexpr uint32_t CTRL_PERIOD_US  = 1000;   // 1 kHz
static constexpr int      CTRL_PRIO       = 20;
static constexpr int      UI_PRIO         = 2;

// Response wait budget for the Type1 exchange, which must complete inside the
// slot because the control law needs this millisecond's position.
static constexpr uint32_t RESP_TIMEOUT_US      = 600;
// Budget for the blocking helpers used during setup (parameter writes, the
// synchronous readParamF). Never runs inside a moving control loop.
static constexpr uint32_t SYNC_RESP_TIMEOUT_US = 600;

// Aux request rates. These now schedule a fire-and-forget REQUEST; the reply
// is filed by Rs06::pump() whenever it arrives, so a slow answer no longer
// costs the slot - or gets thrown away, which is what used to happen.
static constexpr uint32_t AUX_IQF_DIV        = 20;  // 1kHz/20  =  50 Hz
static constexpr uint32_t AUX_VBUS_DIV       = 5;   // 1kHz/5   = 200 Hz
static constexpr uint32_t AUX_VBUS_DIV_DECEL = 2;   // 1kHz/2   = 500 Hz (braking)
static constexpr uint32_t AUX_FAULT_DIV      = 20;  // 1kHz/20  =  50 Hz

// How old a cached auxiliary reading may be before the safety layer stops
// trusting it. Generous next to the 5 ms request interval, tight enough that
// a dead link is noticed within one control decision.
static constexpr uint32_t AUX_STALE_MS       = 100;

// --- Motor / mechanics defaults -------------------------------------------
// Kt referred to the OUTPUT shaft (9:1 already included).
// Cross-checked against the datasheet: 11 N.m / 14.3 Apk (=10.1 Arms) = 1.09.
static constexpr float DEF_KT_NM_PER_ARMS = 1.09f;

static constexpr float DEF_J_HAT      = 0.012f;   // kg.m^2 at output (reflected rotor)
static constexpr float DEF_FRIC_C     = 0.05f;    // N.m  coulomb
static constexpr float DEF_FRIC_V     = 0.010f;   // N.m/(rad/s) viscous

// --- Limits (user editable from the CONFIG page) --------------------------
static constexpr float DEF_LIM_TORQUE   = 20.0f;  // N.m  (peak 36, rated 11)
// Torque ceiling while the axis is parked on a position set-point. Holding a
// brake actuator does not need move-level torque, and capping it here is what
// keeps the standstill current - and therefore the audible buzz - down.
static constexpr float DEF_LIM_TORQUE_HOLD = 6.0f;   // N.m, 0 disables the cap
// Position error inside which the holding P gain fades to zero. Stops encoder
// ripple and cogging from being amplified into vibration at standstill.
static constexpr float DEF_POS_DEADBAND    = 0.20f;  // deg
static constexpr float DEF_LIM_SPEED    = 50.0f;  // rad/s (protocol ceiling)
static constexpr float DEF_LIM_ACC      = 900.0f; // rad/s^2
static constexpr float DEF_LIM_DEC      = 900.0f; // rad/s^2
static constexpr float DEF_POS_MIN      = -12.0f; // rad (soft, mechanism TBD)
static constexpr float DEF_POS_MAX      =  12.0f;

// --- Gains -----------------------------------------------------------------
static constexpr float DEF_KP           = 30.0f;
static constexpr float DEF_KD           = 1.5f;
static constexpr float DEF_KD_TORQUE    = 0.20f;  // residual damping in torque mode
static constexpr float DEF_KD_DAMP_STOP = 3.0f;   // damped stop on a soft fault

// --- Safety ----------------------------------------------------------------
static constexpr float    DEF_TEMP_DERATE_C = 100.0f;  // start linear derate
static constexpr float    DEF_TEMP_TRIP_C   = 130.0f;  // hard stop
static constexpr float    DEF_VBUS_MARGIN_V = 4.0f;    // trip = boot VBUS + margin
static constexpr float    DEF_VBUS_TRIP_MAX = 58.0f;   // never above this
// Number of good readings averaged (median) into the rest-voltage reference
// the overvoltage trip is built on. One sample taken at an arbitrary moment
// is not a baseline: if it happens to land just after a braking pulse has
// already lifted the rail, every later threshold is skewed with it.
static constexpr int      VBUS_REF_SAMPLES  = 9;
// Absolute floor, deliberately low so the machine still comes up on a bench
// supply (the RS06 itself is rated from 15 V). This alone is NOT the pack
// guard - see VBUS_SAG_TRIP_FRAC, which scales with whatever supply is
// actually fitted. Raise this to ~38 V if you want a fixed 13S floor as well.
static constexpr float    DEF_VBUS_MIN_V    = 14.0f;
// Supply-relative undervoltage guard: trip if the rail collapses to this
// fraction of its measured rest voltage. Works on a 32 V bench supply and on
// a 13S pack without reconfiguration (0.70 x 54.6 V = 38 V, the pack floor;
// 0.70 x 32 V = 22 V on the bench). The worst sag measured on a full-effort
// 200 deg move is about 11 %, so this has ample margin.
static constexpr float    VBUS_SAG_TRIP_FRAC = 0.70f;
static constexpr float    DEF_REGEN_W_MAX   = 600.0f;  // W, |tau*omega| while braking
static constexpr uint32_t DEF_CAN_MISS_SOFT = 5;       // consecutive misses -> soft
static constexpr uint32_t DEF_CAN_MISS_HARD = 50;      // consecutive misses -> hard
static constexpr uint32_t MOTOR_CAN_TIMEOUT = 2000;    // 0x7028, 20000 == 1s -> 100ms
static constexpr uint32_t SOFT_RAMP_MS      = 40;      // torque ramp-down time

// --- Logging ---------------------------------------------------------------
static constexpr uint32_t LOG_CAPACITY = 30000;        // 30 s @ 1 kHz (PSRAM)

// --- Command source --------------------------------------------------------
static constexpr uint32_t CMDSRC_STALE_MS = 100;       // -> soft fault
