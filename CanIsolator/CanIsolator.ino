// CanIsolator.ino
// Two-bus CAN isolator / gateway using the Teensy 4.x on-chip FlexCAN controllers.
// Hardware: Teensy 4.0 / 4.1 + two 3.3 V CAN transceivers (e.g. SN65HVD230)
// Library : FlexCAN_T4
//
// ============================================================
// WHAT IT DOES
// ============================================================
// Bus A and Bus B are electrically separate networks. Nothing
// crosses between them unless the ROUTE TABLE says so. For each CAN ID you
// can choose:
//
//   - direction       A->B, B->A, or both
//   - PASS / DROP     forward it or explicitly block it
//   - transform       a function that rewrites the frame (ID, length, data)
//                     or suppresses it
//   - minIntervalMs   rate limit; e.g. a 100 Hz frame forwarded at 10 Hz.
//                     The NEWEST frame is always what gets forwarded, so the
//                     far side never ends up holding stale data.
//   - onChange        forward only when the (transformed) data changes...
//   - heartbeatMs     ...but still resend unchanged data at least this often
//
// The isolator can also BUILD frames itself on a timer from cached data
// (GENERATED table). It does this for the Aston Martin cluster frames, so the
// M1 no longer has to transmit them.
//
// IDs that are not in the table follow DEFAULT_FORWARD_A_TO_B / _B_TO_A.
// This build: noisy -> MoTeC is a whitelist (only routed IDs reach the M1),
// MoTeC -> noisy passes everything (see BUS CONFIG below for why).
//
// ============================================================
// WIRING (Teensy 4.0 / 4.1)
// ============================================================
//   Controller pins:  CAN1 = CTX1 pin 22, CRX1 pin 23
//                     CAN2 = CTX2 pin 1,  CRX2 pin 0
//   Transceiver standby: pins 2 and 3, driven LOW at boot (CAN_STANDBY_PINS).
//
//   Which controller each bus uses is set by SWAP_CAN_CONTROLLERS (BUS CONFIG):
//     defined (this build) : Bus A (noisy) = CAN2 (pins 1/0)
//                            Bus B (M1)    = CAN1 (pins 22/23)
//     not defined          : Bus A (noisy) = CAN1 (pins 22/23)
//                            Bus B (M1)    = CAN2 (pins 1/0)
//   Only the physical pins change. All routing, generated frames, and stats
//   keep the same meaning: A is always the noisy side, B is always the M1.
//   Each side needs its own transceiver. Fit 120 ohm termination only if
//   the Teensy is at a physical end of that bus.
//
// ============================================================
// ARCHITECTURE
// ============================================================
//   RX ISR (per bus) -> lock-free RX ring -> loop(): route / transform /
//   rate limit -> software TX ring (per bus) -> free hardware TX mailbox
//
// The isolator keeps its own TX queue and only writes a frame when a
// hardware mailbox is free. It never uses FlexCAN_T4's internal TX buffer,
// whose events() drain can copy one frame into several mailboxes. Before a
// mailbox is loaded the code also checks that no other mailbox is still
// sending the same ID. Frames with the same ID therefore always go out in
// the order they arrived, so rolling counters and multi-frame sequences
// stay intact.

#include <FlexCAN_T4.h>

// ---------------------------------------------------------------------------
// VEHICLE IDs (edit these)
// ---------------------------------------------------------------------------
// Bus A = noisy side (ABS, keypad / buttons, gauge board, ...)
// Bus B = quiet side (MoTeC M1)
//
static const uint32_t ID_ABS     = 0x2B2;   // 690: ABS frame on Bus A, forwarded to the M1 with buttons merged in
static const uint32_t ID_BUTTONS = 0x178;   // 376: steering wheel on Bus A, forwarded AND extra bytes copied into ABS

// ---------------------------------------------------------------------------
// SERIAL OUTPUT CONTROL
// ---------------------------------------------------------------------------
// Uncomment for debugging. When commented out, all Serial code is compiled out.
//
//#define SERIAL_ENABLE

#ifdef SERIAL_ENABLE
  // Print traffic statistics (frames/s in, out, dropped) every STATS_INTERVAL_MS.
  static const bool PRINT_STATS = true;
  // Print a per-ID rate table for both buses every STATS_INTERVAL_MS.
  // Use this to see where the bloat is before you build the route table.
  static const bool SNIFF_IDS   = false;
  static const unsigned long STATS_INTERVAL_MS = 5000UL;
  // Print a line whenever a frame with one of these IDs is SENT with different
  // data from the last one printed. Use it to watch button bits appear in the
  // merged ABS frame. Uses the OUTPUT ID (after any remap). Empty = off.
  static const uint32_t TRACE_TX_IDS[] = { ID_ABS, ID_BUTTONS };
  // Measure real CPU cost with the Cortex-M7 cycle counter and add it to the
  // stats line: % of CPU spent routing frames, and the slowest single frame.
  static const bool PROFILE_CPU = true;
  // While the M1 is still sending its own copies of the generated frames,
  // compare each one with what the isolator would build at that moment.
  // Constant-bit mismatches are printed immediately (they must never happen).
  // Live-field differences are only counted (timing makes small ones normal).
  static const bool VERIFY_GENERATED = true;
#endif

// ---------------------------------------------------------------------------
// BUS CONFIG
// ---------------------------------------------------------------------------
// NOTE: standard CAN is 500 kbit/s. The FlexCAN timing calculator will try
// to hit whatever number you enter, so if your network really is 512k,
// change these values.
static const uint32_t BUS_A_BAUD = 500000;
static const uint32_t BUS_B_BAUD = 500000;

// What to do with IDs that are NOT in the route table.
//
// A->B (noisy -> MoTeC): whitelist. Only what the route table allows reaches
//   the M1. This is what fixes the M1 transmit overload.
// B->A (MoTeC -> noisy): pass everything. Before the isolator, every M1 frame
//   was already on the noisy bus, so this adds no load there. It also keeps
//   every noisy-side device that listens to the M1 (gauge board, dash,
//   logger...) working without having to list each ID. The gauge's IDs are
//   still listed explicitly in ROUTES, so they keep working if you ever flip
//   this to false.
static const bool DEFAULT_FORWARD_A_TO_B = false;
static const bool DEFAULT_FORWARD_B_TO_A = true;

// ---------------------------------------------------------------------------
// ROBUSTNESS CONFIG
// ---------------------------------------------------------------------------
// Hardware watchdog: if loop() stops running for WATCHDOG_TIMEOUT_HALF_S x 0.5 s,
// the Teensy resets itself and resumes forwarding (boot takes ~0.3 s).
// Valid range 1-256 (0.5 s - 128 s).
#define WATCHDOG_ENABLE
static const uint16_t WATCHDOG_TIMEOUT_HALF_S = 2;   // 1.0 s

// A queued frame that cannot be sent within this time is discarded instead of
// being sent late. This matters when the far bus is unplugged, the ECU is
// off, or the bus is too busy: when it comes back you get fresh data, not a
// burst of old frames. Keep it a little above your slowest important period.
static const uint16_t TX_MAX_AGE_MS = 50;

// Physical controller for each bus. See WIRING at the top of the file.
//   defined     : noisy side on CAN2 (pins 1/0),   M1 on CAN1 (pins 22/23)
//   not defined : noisy side on CAN1 (pins 22/23), M1 on CAN2 (pins 1/0)
#define SWAP_CAN_CONTROLLERS

#ifdef SWAP_CAN_CONTROLLERS
  #define BUS_A_HW CAN2
  #define BUS_B_HW CAN1
#else
  #define BUS_A_HW CAN1
  #define BUS_B_HW CAN2
#endif

FlexCAN_T4<BUS_A_HW, RX_SIZE_16, TX_SIZE_16> canA;  // Bus A (noisy)
FlexCAN_T4<BUS_B_HW, RX_SIZE_16, TX_SIZE_16> canB;  // Bus B (MoTeC M1)

// Transceiver standby pins. The dual CAN adapter board for Teensy 4.0/4.1
// wires each transceiver's standby input to a Teensy pin. HIGH or floating =
// standby: the transceiver only passes wake-up noise and can't transmit.
// They must be driven LOW for normal operation, before the buses start.
static const uint8_t CAN_STANDBY_PINS[] = { 2, 3 };
// RX/TX_SIZE are kept small on purpose. The ISR hands frames straight to the
// isolator's own rings below, and the library's TX buffer is never used.

static const uint8_t BUS_A = 0;
static const uint8_t BUS_B = 1;

// ---------------------------------------------------------------------------
// ROUTE TABLE TYPES
// ---------------------------------------------------------------------------
enum Direction : uint8_t {
  A_TO_B = 0x01,
  B_TO_A = 0x02,
  BOTH   = 0x03,
};

enum Action : uint8_t {
  PASS,   // forward (optionally transformed / rate limited)
  DROP,   // block. Useful when a default is PASS, or to carve a hole in a range.
};

// Transform callback. Edit 'msg' in place: you may change id, flags.extended,
// len and buf[]. Return false to suppress the frame completely.
typedef bool (*TransformFn)(CAN_message_t &msg);

struct Route {
  uint32_t    idFirst;        // CAN ID (or first ID of an inclusive range)
  uint32_t    idLast;         // last ID of range; 0 = single ID (same as idFirst)
  bool        extended;       // false = 11-bit, true = 29-bit
  Direction   dir;            // which way(s) this route applies
  Action      action;         // PASS or DROP
  TransformFn transform;      // nullptr = forward unchanged
  uint16_t    minIntervalMs;  // 0 = no rate limit           (single-ID routes only)
  bool        onChange;       // forward only on data change (single-ID routes only)
  uint16_t    heartbeatMs;    // with onChange: resend unchanged data after this
                              // long (0 = never resend unchanged data)
};

// ---------------------------------------------------------------------------
// BYTE HELPERS FOR TRANSFORMS
// ---------------------------------------------------------------------------
static inline uint16_t getU16BE(const uint8_t *b, uint8_t i) { return (uint16_t)(b[i] << 8) | b[i + 1]; }
static inline uint16_t getU16LE(const uint8_t *b, uint8_t i) { return (uint16_t)(b[i + 1] << 8) | b[i]; }
static inline void     setU16BE(uint8_t *b, uint8_t i, uint16_t v) { b[i] = v >> 8; b[i + 1] = v & 0xFF; }
static inline void     setU16LE(uint8_t *b, uint8_t i, uint16_t v) { b[i] = v & 0xFF; b[i + 1] = v >> 8; }

// ---------------------------------------------------------------------------
// FRAME CACHE (lets a transform use data from OTHER frames)
// ---------------------------------------------------------------------------
// Each ID listed here has its most recent frame stored as it is received,
// whatever the route table says about it. A cached ID can therefore be
// DROPped (never forwarded) and still feed data into another frame.
//
// Inside a transform, call:
//   getCached(BUS_A, id, src)              -> true if a frame has ever arrived
//   getCached(BUS_A, id, src, 500)         -> ...and is no older than 500 ms
//   getCachedLatched(BUS_A, id, src, 250)  -> bit-per-button data, short presses kept
//   getCached(BUS_A, 0x18FEF100, src, 0, true)  for 29-bit IDs
struct CacheDef { uint8_t bus; uint32_t id; bool extended; };
static const CacheDef CACHE_IDS[] = {
  //  bus     id           ext
  { BUS_A,  ID_BUTTONS,  false },   // merged into the ABS frame by xfAbsWithButtons
  { BUS_B,  0x640,       false },   // Engine_Speed            -> generated Aston Dash 2/3
  { BUS_B,  0x649,       false },   // Coolant_Temperature     -> generated Aston Dash 1/3
  { BUS_B,  0x659,       false },   // Vehicle_Speed           -> generated Aston Dash 3/3
};
static const uint8_t NUM_CACHED = sizeof(CACHE_IDS) / sizeof(CACHE_IDS[0]);

struct CachedFrame {
  CAN_message_t msg;
  uint32_t      rxMs;
  bool          valid;
  uint8_t       latch[8];   // OR of every frame since the last getCachedLatched()
};
static CachedFrame frameCache[NUM_CACHED];

// Called for every received frame, before routing.
static void cacheUpdate(uint8_t bus, const CAN_message_t &m, uint32_t now) {
  for (uint8_t i = 0; i < NUM_CACHED; i++) {
    const CacheDef &c = CACHE_IDS[i];
    if (c.bus == bus && c.id == m.id && c.extended == (bool)m.flags.extended) {
      frameCache[i].msg   = m;
      frameCache[i].rxMs  = now;
      frameCache[i].valid = true;
      for (uint8_t b = 0; b < 8; b++) frameCache[i].latch[b] |= m.buf[b];
      return;
    }
  }
}

// Like getCached(), but each bit reads 1 if it was set in ANY frame received
// since the previous call, not only in the latest one. The latch then clears.
//
// Use this for BIT-PER-BUTTON data carried into a host frame. Say a button
// is pressed and released between two ABS frames: getCached() would only see
// the release and the press would be lost. getCachedLatched() reports it as
// pressed in the next ABS frame, so the press always gets through.
//
// Don't use it for value-encoded data (e.g. a byte holding a button number
// or a rotary position): ORing different values together corrupts them.
bool getCachedLatched(uint8_t bus, uint32_t id, CAN_message_t &out, uint32_t maxAgeMs = 0, bool extended = false) {
  for (uint8_t i = 0; i < NUM_CACHED; i++) {
    const CacheDef &c = CACHE_IDS[i];
    if (c.bus != bus || c.id != id || c.extended != extended) continue;
    if (!frameCache[i].valid) return false;
    out = frameCache[i].msg;
    for (uint8_t b = 0; b < 8; b++) { out.buf[b] |= frameCache[i].latch[b]; frameCache[i].latch[b] = 0; }
    if (maxAgeMs && (millis() - frameCache[i].rxMs) > maxAgeMs) return false;
    return true;
  }
  return false;
}

// Copy the latest cached frame for (bus, id) into 'out'.
// maxAgeMs = 0 accepts any age. Returns false if the frame was never seen,
// is too old, or isn't listed in CACHE_IDS.
bool getCached(uint8_t bus, uint32_t id, CAN_message_t &out, uint32_t maxAgeMs = 0, bool extended = false) {
  for (uint8_t i = 0; i < NUM_CACHED; i++) {
    const CacheDef &c = CACHE_IDS[i];
    if (c.bus != bus || c.id != id || c.extended != extended) continue;
    if (!frameCache[i].valid) return false;
    if (maxAgeMs && (millis() - frameCache[i].rxMs) > maxAgeMs) return false;
    out = frameCache[i].msg;
    return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// TRANSFORMS
// ---------------------------------------------------------------------------
// *** EXAMPLES. Replace these with your own vehicle-specific logic. ***

// Re-map an ID. The frame leaves the isolator under a different ID.
bool xfRemapTo0x5A0(CAN_message_t &msg) {
  msg.id = 0x5A0;
  return true;
}

// Rescale a 16-bit big-endian value in bytes 2-3 (e.g. RPM x4 -> RPM x1)
// and clear the bytes the receiver doesn't need.
bool xfRescaleRpm(CAN_message_t &msg) {
  if (msg.len < 4) return false;          // malformed: don't forward
  uint16_t raw = getU16BE(msg.buf, 2);
  setU16BE(msg.buf, 2, raw / 4);
  for (uint8_t i = 4; i < 8; i++) msg.buf[i] = 0;
  return true;
}

// Mask off bits. Here only byte 0 bit 0 and byte 1 are kept, the rest are zeroed.
bool xfMaskBits(CAN_message_t &msg) {
  msg.buf[0] &= 0x01;
  for (uint8_t i = 2; i < 8; i++) msg.buf[i] = 0;
  return true;
}

// Conditional suppression: forward only when byte 0 is non-zero.
bool xfOnlyWhenActive(CAN_message_t &msg) {
  return msg.buf[0] != 0;
}

// ---- Extend the wheel's button inputs through the ABS frame ----
// The ABS frame (ID_ABS, 690) uses data bytes 1-3 only, so bytes 4-8
// (buf[3]..buf[7]) are free. The steering wheel frame (ID_BUTTONS, 376) is
// still forwarded to the M1 unchanged, and its first 16 buttons arrive
// through the M1's existing 2 receive messages. The wheel bytes mapped below
// are ALSO copied into 690, so the M1 can read buttons beyond those 16
// without needing another receive slot. This is an extension, not a
// replacement.
//
// BUTTON_MAP says which wheel byte goes into which ABS byte:
//   srcByte / dstByte : buf[] index, 0-based (CAN "byte 1" = buf[0])
//   mask              : which bits to copy. Other dst bits keep the ABS value.
//   catchTaps         : false = plain copy of the latest wheel value (the
//                         default on every row). Pure pass-through: a
//                         momentary button stays momentary, a toggle stays a
//                         toggle. No state, timers, or toggling are added here.
//                         Safe for any encoding.
//                       true  = also report a press that started AND ended
//                         between two ABS frames, for one ABS frame. This is
//                         NOT a toggle/latch: the bit still drops when the
//                         button is released. Use ONLY for one-bit-per-button
//                         bytes: ORing values together corrupts them.
//
// ABS bytes not listed here (currently 6-8) pass through as the ABS module
// sent them.
struct ButtonByteMap { uint8_t srcByte; uint8_t dstByte; uint8_t mask; bool catchTaps; };
static const ButtonByteMap BUTTON_MAP[] = {
  //  wheel   ABS     mask   catchTaps
  {   6,      3,      0xFF,  false },   // wheel byte 7 -> ABS byte 4
  {   7,      4,      0xFF,  false }    // wheel byte 8 -> ABS byte 5
};
static const uint8_t NUM_BUTTON_MAP = sizeof(BUTTON_MAP) / sizeof(BUTTON_MAP[0]);

// Steering wheel frame older than this = wheel offline: mapped bits read 0 (released).
static const uint32_t BUTTON_STALE_MS = 250;

// Until the wheel frame has been seen at least once since boot, the ABS frame
// passes through UNMODIFIED.
bool xfAbsWithButtons(CAN_message_t &msg) {
  CAN_message_t latest, taps;
  if (!getCached(BUS_A, ID_BUTTONS, latest)) return true;             // wheel never seen: ABS as-is
  bool fresh = getCached(BUS_A, ID_BUTTONS, latest, BUTTON_STALE_MS);
  getCachedLatched(BUS_A, ID_BUTTONS, taps);                          // OR since last ABS frame; resets it

  msg.len = 8;
  for (uint8_t i = 0; i < NUM_BUTTON_MAP; i++) {
    const ButtonByteMap &m = BUTTON_MAP[i];
    uint8_t src = 0;                                                  // stale wheel = released
    if (fresh) src = m.catchTaps ? taps.buf[m.srcByte] : latest.buf[m.srcByte];
    msg.buf[m.dstByte] = (msg.buf[m.dstByte] & ~m.mask) | (src & m.mask);
  }
  return true;
}

// ---------------------------------------------------------------------------
// GENERATED FRAMES (built by the isolator instead of the M1)
// ---------------------------------------------------------------------------
// The Aston Martin cluster frames used to be M1 "CAN Transmit Custom"
// messages 1-3. Each one is a fixed 8-byte pattern plus a single live value,
// and that value is already in the M1's General Ver5 broadcast. The isolator
// therefore builds them itself on Bus A, and the M1 no longer has to.
//
// Every frame is a byte-for-byte copy of the M1's: same ID, ID type, length 8,
// same constant bytes, and the live value in the same bits with the same
// scaling. The constants below were read from the tune's custom transmit setup
// (Source = None signals send their Adder value).
//
// SOURCE RULE: a frame is only sent while its source frame from the M1 is
// fresh (younger than GEN_SOURCE_STALE_MS). M1 off or unplugged -> the cluster
// frames stop, exactly as they did when the M1 sent them itself. No frame is
// ever built from missing or invented data.
//
// The M1's own copies of these IDs are DROPped in ROUTES, so the cluster never
// sees two sources, whether or not they are still enabled in the M1.
//
// A/B SWITCH: false = don't generate. The M1's own Aston frames are passed
// through unchanged instead (the M1 custom messages 1-3 must then be ENABLED
// in the M1). Use this to tell an isolator problem from a cluster problem.
static const bool GENERATE_ASTON_FRAMES = true;
static const Action ASTON_M1_COPY = GENERATE_ASTON_FRAMES ? DROP : PASS;

static const uint32_t GEN_SOURCE_STALE_MS = 500;

static inline uint32_t clampRound(float v, uint32_t maxV) {
  if (v <= 0.0f) return 0;
  uint32_t r = (uint32_t)(v + 0.5f);
  return r > maxV ? maxV : r;
}

// Aston Dash 1/3 - 0x15B00260 (29-bit), 10 Hz
//   bytes 0-6 : 00 00 B5 7F 5F 00 C0        (constant)
//   byte  7   : Coolant Temperature degC + 65
//   Source    : M1 0x649 byte 0 = degC + 40  ->  byte 7 = raw + 25
bool genAstonDash1(CAN_message_t &out) {
  CAN_message_t src;
  if (!getCached(BUS_B, 0x649, src, GEN_SOURCE_STALE_MS)) return false;
  static const uint8_t FIXED[8] = { 0x00, 0x00, 0xB5, 0x7F, 0x5F, 0x00, 0xC0, 0x00 };
  memcpy(out.buf, FIXED, 8);
  uint16_t v = (uint16_t)src.buf[0] + 25;
  out.buf[7] = v > 255 ? 255 : (uint8_t)v;
  return true;
}

// NOTE ON UNITS: the M1 custom transmit scales a channel's INTERNAL value,
// and the M1's internal units are not the Ver5 DBC units:
//   Engine Speed  : deg/s   (1 rpm = 6 deg/s)
//   Vehicle Speed : m/s     (1 m/s = 3.6 km/h)
//   Coolant Temp  : degC
// The formulas below reproduce the M1's output from the Ver5 values.

// Aston Dash 2/3 - 0x00040360 (29-bit), 100 Hz
//   bytes 0-4 : 00 00 63 04 AC              (constant)
//   bits 40-50: 2047 (all ones)             (constant)
//   bits 51-63: Engine Speed                (13 bits, big endian)
//               M1 config: deg/s / 6 = rpm, so the field is plain RPM
//   -> byte 5 = FF, byte 6 = E0 | (rpm >> 8), byte 7 = rpm & FF
//   Source    : M1 0x640 bytes 0-1, rpm, big endian
bool genAstonDash2(CAN_message_t &out) {
  CAN_message_t src;
  if (!getCached(BUS_B, 0x640, src, GEN_SOURCE_STALE_MS)) return false;
  static const uint8_t FIXED[8] = { 0x00, 0x00, 0x63, 0x04, 0xAC, 0xFF, 0xE0, 0x00 };
  memcpy(out.buf, FIXED, 8);
  uint16_t rpm = getU16BE(src.buf, 0);
  if (rpm > 0x1FFF) rpm = 0x1FFF;                       // 13-bit field: max 8191 rpm
  out.buf[6] = 0xE0 | (uint8_t)(rpm >> 8);
  out.buf[7] = (uint8_t)(rpm & 0xFF);
  return true;
}

// Aston Dash 3/3 - 0x091C0170 (29-bit), 10 Hz
//   bytes 0-5 : 00 08 DF 7E C0 00           (constant)
//   byte  6   : Vehicle Speed m/s x 1.42    (M1 internal unit is m/s)
//   byte  7   : 00                          (constant)
//   Source    : M1 0x659 bytes 4-5, 0.1 km/h, big endian
//               m/s = raw / 36  ->  byte 6 = raw x 1.42 / 36
bool genAstonDash3(CAN_message_t &out) {
  CAN_message_t src;
  if (!getCached(BUS_B, 0x659, src, GEN_SOURCE_STALE_MS)) return false;
  static const uint8_t FIXED[8] = { 0x00, 0x08, 0xDF, 0x7E, 0xC0, 0x00, 0x00, 0x00 };
  memcpy(out.buf, FIXED, 8);
  out.buf[6] = (uint8_t)clampRound(getU16BE(src.buf, 4) * (1.42f / 36.0f), 255);
  return true;
}

// Builder: fill out.buf (all 8 bytes). Return false to skip this period.
// id / ext / len are set by the scheduler.
typedef bool (*GenBuildFn)(CAN_message_t &out);

struct GenFrame {
  uint32_t   id;
  bool       extended;
  uint8_t    bus;          // bus the frame is SENT on
  uint16_t   periodMs;
  GenBuildFn build;
  uint8_t    fixedMask[8]; // bits that are constant (used by VERIFY_GENERATED)
};

static const GenFrame GENERATED[] = {
  //  id          ext   bus    period  builder        constant bits (1 = fixed)
  { 0x15B00260, true, BUS_A, 100,  genAstonDash1, { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00 } },  // Aston Dash 1/3
  { 0x00040360, true, BUS_A, 10,   genAstonDash2, { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xE0,0x00 } },  // Aston Dash 2/3
  { 0x091C0170, true, BUS_A, 100,  genAstonDash3, { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0xFF } },  // Aston Dash 3/3
};
static const uint8_t NUM_GENERATED = sizeof(GENERATED) / sizeof(GENERATED[0]);

// ---------------------------------------------------------------------------
// ROUTE TABLE
// ---------------------------------------------------------------------------
// The first matching row wins (it must match ID, ID type, and direction).
// Put specific DROP rows ABOVE any range rows they should override.
//
// Build with SERIAL_ENABLE + SNIFF_IDS to see what's on each bus and how often.
//
// Rate limits on frames the gauge board uses: its data, icon and warning
// timeouts are all 500 ms, so keep minMs at 100 or below on those IDs.
static const Route ROUTES[] = {
  //  first       last  ext    dir     action  transform          minMs  onChg  hbMs

  // ===== Bus A (noisy) -> Bus B (MoTeC M1): whitelist =====
  // ABS (690) with steering wheel buttons merged in. No rate limit: button latency = ABS period.
  { ID_ABS,     0,    false, A_TO_B, PASS,   xfAbsWithButtons,  0,     false, 0    },
  // Steering wheel (376): ALWAYS forwarded unchanged. The M1 reads its first
  // 16 buttons from this frame through 2 receive messages. 690 only carries
  // extra wheel bytes on top of that (extension, not replacement). Keep this
  // as PASS.
  { ID_BUTTONS, 0,    false, A_TO_B, PASS,   nullptr,           0,     false, 0    },
  // Straight pass-through to the M1. Every frame here adds load on the M1
  // bus: check the rates with SNIFF_IDS and add a minMs if one is faster
  // than the M1 needs.
  // 206: ABS wheel speeds -> M1. Forwarded UNMODIFIED: no transform, no rate
  // limit, nothing merged in. Don't add a transform or minMs here.
  { 0x0CE,      0,    false, A_TO_B, PASS,   nullptr,           0,     false, 0    },  // 206  ABS wheel speed (unmodified)

  // ===== Bus B (MoTeC M1) -> Bus A (noisy): M1 output used by the gauge board =====
  // (DEFAULT_FORWARD_B_TO_A already passes these. Listing them records the
  //  dependency, and keeps the gauge working if that default is ever changed.)
  // Byte layouts the gauge decodes: M1 General Ver5 (0x640-0x67x) and M1 Cruise 0x661 v1.
  { 0x640,      0,    false, B_TO_A, PASS,   nullptr,           0,     false, 0    },  // MAP
  { 0x641,      0,    false, B_TO_A, PASS,   nullptr,           0,     false, 0    },  // fuel pressure, injector duty
  { 0x644,      0,    false, B_TO_A, PASS,   nullptr,           0,     false, 0    },  // oil pressure
  { 0x649,      0,    false, B_TO_A, PASS,   nullptr,           0,     false, 0    },  // coolant temp, battery V
  { 0x64A,      0,    false, B_TO_A, PASS,   nullptr,           0,     false, 0    },  // ambient pressure (boost calc)
  { 0x64C,      0,    false, B_TO_A, PASS,   nullptr,           0,     false, 0    },  // warning flags (red screen)
  { 0x64E,      0,    false, B_TO_A, PASS,   nullptr,           0,     false, 0    },  // launch / traction switch icons
  { 0x650,      0,    false, B_TO_A, PASS,   nullptr,           0,     false, 0    },  // Driver_Switch_2 (two-step icon)
  { 0x651,      0,    false, B_TO_A, PASS,   nullptr,           0,     false, 0    },  // lambda bank 1 / 2
  { 0x659,      0,    false, B_TO_A, PASS,   nullptr,           0,     false, 0    },  // vehicle speed (+ odometer)
  { 0x661,      0,    false, B_TO_A, PASS,   nullptr,           0,     false, 0    },  // cruise state / set speed
  { 0x670,      0,    false, B_TO_A, PASS,   nullptr,           0,     false, 0    },  // ethanol %
  { 0x6A8,      0,    false, B_TO_A, PASS,   nullptr,           0,     false, 0    },  // exhaust bypass icon (if the M1 sends it)

  // ===== Aston Martin cluster =====
  // The ext column matters: a 29-bit ID and an 11-bit ID with the same number
  // are different frames on the wire.
  // Dash 1/3 - 3/3 are now BUILT by the isolator (see GENERATED). The M1's own
  // copies are dropped, so the cluster only ever sees one source. That makes
  // it safe to leave them enabled in the M1 while you verify, then disable
  // M1 CAN Transmit Custom messages 1-3 to take the load off the M1.
  // (ASTON_M1_COPY = DROP while GENERATE_ASTON_FRAMES is true, PASS when false.)
  { 0x15B00260, 0,    true,  B_TO_A, ASTON_M1_COPY, nullptr,    0,     false, 0    },  // 363856480 (29-bit) - Aston Dash 1/3 (generated)
  { 0x00040360, 0,    true,  B_TO_A, ASTON_M1_COPY, nullptr,    0,     false, 0    },  // 263008    (29-bit) - Aston Dash 2/3 (generated)
  { 0x091C0170, 0,    true,  B_TO_A, ASTON_M1_COPY, nullptr,    0,     false, 0    },  // 152830320 (29-bit) - Aston Dash 3/3 (generated)
  { 0x168,      0,    false, B_TO_A, PASS,   nullptr,           0,     false, 0    },  // 360       (11-bit) - Legacy Zestek?
};

// ---------------------------------------------------------------------------
// EXAMPLE ROWS (reference only: copy into ROUTES above as needed)
// ---------------------------------------------------------------------------
//  { 0x200,  0,      false, A_TO_B, PASS, nullptr,          100, false, 0    },  // 100 Hz -> 10 Hz
//  { 0x201,  0,      false, A_TO_B, PASS, nullptr,          0,   true,  1000 },  // on change, >= 1 Hz
//  { 0x202,  0,      false, A_TO_B, PASS, nullptr,          50,  true,  500  },  // on change, <= 20 Hz, 500 ms heartbeat
//  { 0x300,  0,      false, A_TO_B, PASS, xfRemapTo0x5A0,   0,   false, 0    },  // change ID
//  { 0x301,  0,      false, A_TO_B, PASS, xfRescaleRpm,     20,  false, 0    },  // rescale + rate limit
//  { 0x302,  0,      false, A_TO_B, PASS, xfMaskBits,       0,   true,  250  },  // mask bits, on change
//  { 0x303,  0,      false, B_TO_A, PASS, xfOnlyWhenActive, 0,   false, 0    },  // conditional
//  { 0x7E5,  0,      false, BOTH,   DROP, nullptr,          0,   false, 0    },  // carve a hole...
//  { 0x7DF,  0x7EF,  false, BOTH,   PASS, nullptr,          0,   false, 0    },  // ...in a range
//  { 0x18FEF100, 0,  true,  A_TO_B, PASS, nullptr,          100, false, 0    },  // 29-bit ID
static const uint16_t NUM_ROUTES = sizeof(ROUTES) / sizeof(ROUTES[0]);

// ===========================================================================
// Nothing below this line normally needs editing.
// ===========================================================================

// ---------------------------------------------------------------------------
// Frame ring buffer (single producer / single consumer, lock-free)
// ---------------------------------------------------------------------------
template <uint16_t SIZE>
struct FrameRing {
  static_assert((SIZE & (SIZE - 1)) == 0, "FrameRing SIZE must be a power of 2");
  CAN_message_t      slot[SIZE];
  volatile uint16_t  head = 0;   // producer writes
  volatile uint16_t  tail = 0;   // consumer reads

  bool push(const CAN_message_t &m) {
    uint16_t h = head, next = (h + 1) & (SIZE - 1);
    if (next == tail) return false;          // full
    slot[h] = m;
    __asm__ volatile("" ::: "memory");       // data before index
    head = next;
    return true;
  }
  bool pop(CAN_message_t &m) {
    uint16_t t = tail;
    if (t == head) return false;             // empty
    m = slot[t];
    __asm__ volatile("" ::: "memory");
    tail = (t + 1) & (SIZE - 1);
    return true;
  }
  const CAN_message_t *peek() const { return (tail == head) ? nullptr : &slot[tail]; }
  void drop() { tail = (tail + 1) & (SIZE - 1); }
  bool empty() const { return tail == head; }
};

// At 500 kbit/s a bus carries at most ~8,000 frames/s (short frames).
// 512 RX slots is ~60 ms of a fully saturated bus.
static FrameRing<512> rxRing[2];
static FrameRing<256> txRing[2];

// ---------------------------------------------------------------------------
// Per-route, per-direction rate-limit state
// ---------------------------------------------------------------------------
struct RouteState {
  CAN_message_t lastSent;      // last frame actually queued for TX
  CAN_message_t pending;       // newest frame held back by rate limit
  uint32_t      lastSentMs;
  bool          everSent;
  bool          hasPending;
};
static RouteState routeState[NUM_ROUTES][2];   // [route][source bus]

// ---------------------------------------------------------------------------
// Statistics
// ---------------------------------------------------------------------------
struct BusStats {
  uint32_t rx;            // frames received on this bus
  uint32_t fwd;           // frames forwarded FROM this bus to the other
  uint32_t filtered;      // dropped: no route / DROP route / transform veto
  uint32_t suppressed;    // dropped: rate limit or unchanged data
  uint32_t tx;            // frames transmitted ON this bus
  uint32_t rxOverflow;    // RX ring full (loop() too slow)
  uint32_t txOverflow;    // TX ring full (bus saturated / not acked)
  uint32_t txStale;       // discarded: waited longer than TX_MAX_AGE_MS
  uint32_t busOffEvents;  // transitions into bus-off
};
static BusStats stats[2];
static volatile uint32_t rxOverflowIsr[2];

// ---------------------------------------------------------------------------
// RX ISR callbacks (run in interrupt context: keep minimal)
// ---------------------------------------------------------------------------
// FlexCAN_T4 calls onReceive handlers straight from the ISR as long as
// events() is never called. The isolator relies on that.
void onRxA(const CAN_message_t &msg) { if (!rxRing[BUS_A].push(msg)) rxOverflowIsr[BUS_A]++; }
void onRxB(const CAN_message_t &msg) { if (!rxRing[BUS_B].push(msg)) rxOverflowIsr[BUS_B]++; }

// ---------------------------------------------------------------------------
// Hardware TX: load a free mailbox directly, preserving per-ID ordering
// ---------------------------------------------------------------------------
static inline uint32_t encodeMbId(const CAN_message_t &m) {
  return m.flags.extended ? (m.id & FLEXCAN_MB_ID_EXT_MASK) : FLEXCAN_MB_ID_IDSTD(m.id);
}

// Returns a free TX mailbox number, or -1 if none is free or if another
// mailbox is still sending the same ID (keeps same-ID frames in order).
static int findTxMailbox(uint32_t base, const CAN_message_t &m) {
  const uint8_t  maxMb = FLEXCANb_MAXMB_SIZE(base);
  const uint8_t  first = 6 + ((((FLEXCANb_CTRL2(base) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2);
  const uint32_t want  = encodeMbId(m);
  int freeMb = -1;
  for (uint8_t i = first; i < maxMb; i++) {
    uint32_t cs   = FLEXCANb_MBn_CS(base, i);
    uint32_t code = FLEXCAN_get_code(cs);
    if (code == FLEXCAN_MB_CODE_TX_INACTIVE) {
      if (freeMb < 0) freeMb = i;
    } else if (code == FLEXCAN_MB_CODE_TX_ONCE) {
      bool ext = (cs >> 21) & 1;
      if (ext == m.flags.extended && (FLEXCANb_MBn_ID(base, i) & FLEXCAN_MB_ID_EXT_MASK) == want) {
        return -1;   // same ID still in flight, so wait
      }
    }
  }
  return freeMb;
}

// (Takes a bus index rather than a template / typedef parameter: the Arduino
// prototype generator hoists declarations above user types in .ino files.)
// Each queued frame carries the low 16 bits of millis() from when it was
// queued, in its (otherwise unused on TX) 'timestamp' field.
static void drainTx(uint8_t bus) {
  const uint32_t base  = (bus == BUS_A) ? (uint32_t)BUS_A_HW : (uint32_t)BUS_B_HW;
  const uint16_t now16 = (uint16_t)millis();
  const CAN_message_t *m;
  while ((m = txRing[bus].peek()) != nullptr) {
    if ((uint16_t)(now16 - m->timestamp) > TX_MAX_AGE_MS) {
      txRing[bus].drop();          // too old to be useful: discard, try the next one
      stats[bus].txStale++;
      continue;
    }
    int mb = findTxMailbox(base, *m);
    if (mb < 0) return;
    if (bus == BUS_A) canA.write((FLEXCAN_MAILBOX)mb, *m);
    else              canB.write((FLEXCAN_MAILBOX)mb, *m);
    txRing[bus].drop();
    stats[bus].tx++;
  }
}

#ifdef SERIAL_ENABLE
static const uint8_t NUM_TRACE = sizeof(TRACE_TX_IDS) / sizeof(TRACE_TX_IDS[0]);
static CAN_message_t traceLast[NUM_TRACE ? NUM_TRACE : 1][2];
static bool          traceSeen[NUM_TRACE ? NUM_TRACE : 1][2];

static void traceTx(uint8_t dstBus, const CAN_message_t &m) {
  for (uint8_t i = 0; i < NUM_TRACE; i++) {
    if (TRACE_TX_IDS[i] != m.id) continue;
    CAN_message_t &last = traceLast[i][dstBus];
    if (traceSeen[i][dstBus] && last.len == m.len && memcmp(last.buf, m.buf, m.len) == 0) return;
    last = m;
    traceSeen[i][dstBus] = true;
    Serial.printf("%10lu  TX ->%c  %03lX [%u]", millis(), dstBus ? 'B' : 'A', (unsigned long)m.id, m.len);
    for (uint8_t d = 0; d < m.len && d < 8; d++) Serial.printf(" %02X", m.buf[d]);
    Serial.println();
    return;
  }
}
#endif

static bool queueTx(uint8_t dstBus, const CAN_message_t &m) {
#ifdef SERIAL_ENABLE
  traceTx(dstBus, m);
#endif
  CAN_message_t q = m;
  q.timestamp = (uint16_t)millis();
  if (txRing[dstBus].push(q)) return true;
  stats[dstBus].txOverflow++;
  return false;
}

// ---------------------------------------------------------------------------
// Routing
// ---------------------------------------------------------------------------
static inline bool isRange(const Route &r) { return r.idLast != 0 && r.idLast != r.idFirst; }

static int findRouteSlow(uint32_t id, bool ext, uint8_t dirBit) {
  for (uint16_t i = 0; i < NUM_ROUTES; i++) {
    const Route &r = ROUTES[i];
    if (!(r.dir & dirBit)) continue;
    if (r.extended != ext) continue;
    uint32_t last = r.idLast ? r.idLast : r.idFirst;
    if (id >= r.idFirst && id <= last) return i;
  }
  return -1;
}

// Constant-time lookup for 11-bit IDs: the result of the table scan for every
// possible standard ID, per direction, computed once at boot (8 KB of RAM).
// Per-frame cost no longer grows with the number of routes.
static int16_t stdRouteIndex[2][2048];   // [0] = A->B, [1] = B->A; -1 = no route

static void buildRouteIndex() {
  for (uint16_t id = 0; id < 2048; id++) {
    stdRouteIndex[0][id] = (int16_t)findRouteSlow(id, false, A_TO_B);
    stdRouteIndex[1][id] = (int16_t)findRouteSlow(id, false, B_TO_A);
  }
}

static inline int findRoute(const CAN_message_t &m, uint8_t dirBit) {
  if (!m.flags.extended && m.id < 2048) return stdRouteIndex[dirBit == A_TO_B ? 0 : 1][m.id];
  return findRouteSlow(m.id, m.flags.extended, dirBit);   // 29-bit: table scan
}

// Clean copy: carries only what belongs on the wire, never mailbox / seq info.
static inline void copyFrame(CAN_message_t &dst, const CAN_message_t &src) {
  dst = CAN_message_t();
  dst.id             = src.id;
  dst.flags.extended = src.flags.extended;
  dst.flags.remote   = src.flags.remote;
  dst.len            = src.len > 8 ? 8 : src.len;
  memcpy(dst.buf, src.buf, 8);
}

static inline bool sameFrame(const CAN_message_t &a, const CAN_message_t &b) {
  return a.id == b.id && a.len == b.len &&
         a.flags.extended == b.flags.extended && a.flags.remote == b.flags.remote &&
         memcmp(a.buf, b.buf, a.len) == 0;
}

// Rate-limit / on-change gate for single-ID routes.
static void offerRateLimited(uint16_t ri, uint8_t src, const CAN_message_t &out, uint32_t now) {
  const Route &r  = ROUTES[ri];
  RouteState  &st = routeState[ri][src];
  const uint8_t dst = src ^ 1;

  if (r.onChange && st.everSent && sameFrame(out, st.lastSent) &&
      (r.heartbeatMs == 0 || (now - st.lastSentMs) < r.heartbeatMs)) {
    st.hasPending = false;          // newest data equals what the far side already has
    stats[src].suppressed++;
    return;
  }

  if (r.minIntervalMs && st.everSent && (now - st.lastSentMs) < r.minIntervalMs) {
    if (st.hasPending) stats[src].suppressed++;   // the older pending frame is superseded
    st.pending    = out;
    st.hasPending = true;
    return;
  }

  if (queueTx(dst, out)) {
    st.lastSent   = out;
    st.lastSentMs = now;
    st.everSent   = true;
    st.hasPending = false;
    stats[src].fwd++;
  } else {
    st.pending    = out;            // TX ring full: retry from flushPending()
    st.hasPending = true;
  }
}

// Release rate-limited frames whose interval has elapsed.
static void flushPending(uint32_t now) {
  for (uint16_t ri = 0; ri < NUM_ROUTES; ri++) {
    for (uint8_t src = 0; src < 2; src++) {
      RouteState &st = routeState[ri][src];
      if (!st.hasPending) continue;
      if (st.everSent && (now - st.lastSentMs) < ROUTES[ri].minIntervalMs) continue;
      if (queueTx(src ^ 1, st.pending)) {
        st.lastSent   = st.pending;
        st.lastSentMs = now;
        st.everSent   = true;
        st.hasPending = false;
        stats[src].fwd++;
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Generated-frame scheduler
// ---------------------------------------------------------------------------
// Fixed cadence: the due time advances by exactly one period each send, so
// rates don't drift. After a long gap (boot, or source missing) it re-syncs
// to now instead of sending a catch-up burst.
static uint32_t genDueMs[NUM_GENERATED];
static bool     genStarted[NUM_GENERATED];
static uint32_t genSent[NUM_GENERATED];      // frames queued
static uint32_t genNoSource[NUM_GENERATED];  // periods skipped: source frame missing / stale
#ifdef SERIAL_ENABLE
static uint8_t  genLastBuf[NUM_GENERATED][8];  // last frame queued (shown in the stats)
#endif

static void runGenerators(uint32_t now) {
  if (!GENERATE_ASTON_FRAMES) return;
  for (uint8_t i = 0; i < NUM_GENERATED; i++) {
    const GenFrame &g = GENERATED[i];
    if (genStarted[i] && (int32_t)(now - genDueMs[i]) < 0) continue;   // not due yet
    if (genStarted[i] && (now - genDueMs[i]) < g.periodMs) genDueMs[i] += g.periodMs;
    else                                                   genDueMs[i]  = now + g.periodMs;
    genStarted[i] = true;

    CAN_message_t out;
    out.id             = g.id;
    out.flags.extended = g.extended;
    out.len            = 8;
    memset(out.buf, 0, 8);
    if (!g.build(out)) { genNoSource[i]++; continue; }
    if (queueTx(g.bus, out)) {
      genSent[i]++;
#ifdef SERIAL_ENABLE
      memcpy(genLastBuf[i], out.buf, 8);
#endif
    }
  }
}

#ifdef SERIAL_ENABLE
// VERIFY_GENERATED: compare each M1-sent copy with what we'd build right now.
static uint32_t verChecked[NUM_GENERATED];
static uint32_t verFixedErr[NUM_GENERATED];   // constant bits differ: MUST stay 0
static uint32_t verLiveDiff[NUM_GENERATED];   // live value differs (timing/rounding)
static uint32_t m1Seen[NUM_GENERATED];        // M1-sent copies seen on Bus B (any time)
static uint8_t  m1LastBuf[NUM_GENERATED][8];  // last M1-sent copy
static uint8_t  m1LastLen[NUM_GENERATED];

static void verifyGenerated(uint8_t src, const CAN_message_t &m1) {
  if (!VERIFY_GENERATED) return;
  for (uint8_t i = 0; i < NUM_GENERATED; i++) {
    const GenFrame &g = GENERATED[i];
    if (g.id != m1.id || g.extended != (bool)m1.flags.extended || src == g.bus) continue;
    m1Seen[i]++;
    memcpy(m1LastBuf[i], m1.buf, 8);
    m1LastLen[i] = m1.len;
    CAN_message_t ours;
    memset(ours.buf, 0, 8);
    if (!g.build(ours)) return;                       // no source data yet: nothing to compare
    verChecked[i]++;
    bool fixedBad = (m1.len != 8), liveBad = false;
    for (uint8_t b = 0; b < 8; b++) {
      uint8_t diff = m1.buf[b] ^ ours.buf[b];
      if (diff & g.fixedMask[b])  fixedBad = true;
      if (diff & ~g.fixedMask[b]) liveBad  = true;
    }
    if (liveBad) verLiveDiff[i]++;
    if (fixedBad) {
      if (verFixedErr[i]++ < 20) {                    // don't flood the console
        Serial.printf("VERIFY %08lX CONSTANT MISMATCH  M1 [%u]", (unsigned long)g.id, m1.len);
        for (uint8_t b = 0; b < 8; b++) Serial.printf(" %02X", m1.buf[b]);
        Serial.print("  ours [8]");
        for (uint8_t b = 0; b < 8; b++) Serial.printf(" %02X", ours.buf[b]);
        Serial.println();
      }
    }
    return;
  }
}

static void sniffRecord(uint8_t bus, const CAN_message_t &m);
#endif

static void handleFrame(uint8_t src, const CAN_message_t &rxRaw, uint32_t now) {
  // The ISR copies all 8 data bytes out of the hardware buffer whatever the
  // frame's length (DLC). Bytes past the DLC aren't guaranteed to be zero:
  // they can be leftovers from an earlier frame. Zero them here, so a short
  // frame can never leak junk into a merged frame (e.g. a 3-byte wheel frame
  // copied into ABS bytes 4-8) or into the on-change comparison.
  CAN_message_t rx = rxRaw;
  if (rx.len > 8) rx.len = 8;
  for (uint8_t i = rx.len; i < 8; i++) rx.buf[i] = 0;

  stats[src].rx++;
  cacheUpdate(src, rx, now);
#ifdef SERIAL_ENABLE
  if (SNIFF_IDS) sniffRecord(src, rx);
  if (rx.flags.extended) verifyGenerated(src, rx);
#endif

  const uint8_t dst    = src ^ 1;
  const uint8_t dirBit = (src == BUS_A) ? A_TO_B : B_TO_A;
  const int     ri     = findRoute(rx, dirBit);

  if (ri < 0) {
    bool def = (src == BUS_A) ? DEFAULT_FORWARD_A_TO_B : DEFAULT_FORWARD_B_TO_A;
    if (def) {
      CAN_message_t out; copyFrame(out, rx);
      if (queueTx(dst, out)) stats[src].fwd++;
    } else {
      stats[src].filtered++;
    }
    return;
  }

  const Route &r = ROUTES[ri];
  if (r.action == DROP) { stats[src].filtered++; return; }

  CAN_message_t out; copyFrame(out, rx);
  if (r.transform && !r.transform(out)) { stats[src].filtered++; return; }
  if (out.len > 8) out.len = 8;

  if (isRange(r) || (r.minIntervalMs == 0 && !r.onChange)) {
    if (queueTx(dst, out)) stats[src].fwd++;
    return;
  }
  offerRateLimited((uint16_t)ri, src, out, now);
}

// ---------------------------------------------------------------------------
// Bus health
// ---------------------------------------------------------------------------
// The i.MX RT FlexCAN recovers from bus-off on its own (CTRL1[BOFFREC] = 0,
// which FlexCAN_T4 leaves at the default), so this only counts the events.
static void checkBusHealth() {
  static bool wasBusOff[2] = {false, false};
  const uint32_t bases[2] = { (uint32_t)BUS_A_HW, (uint32_t)BUS_B_HW };
  for (uint8_t b = 0; b < 2; b++) {
    uint32_t fltconf = (FLEXCANb_ESR1(bases[b]) >> 4) & 0x3;   // 0 active, 1 passive, 2/3 bus-off
    bool busOff = fltconf >= 2;
    if (busOff && !wasBusOff[b]) stats[b].busOffEvents++;
    wasBusOff[b] = busOff;
    stats[b].rxOverflow = rxOverflowIsr[b];
  }
}

// ---------------------------------------------------------------------------
// Hardware watchdog (i.MX RT WDOG1)
// ---------------------------------------------------------------------------
// Once enabled it cannot be turned off until the next reset. If watchdogFeed()
// isn't called within the timeout, the chip resets.
#ifdef WATCHDOG_ENABLE
static void watchdogStart() {
  WDOG1_WMCR = 0;                                          // disable 16 s power-down counter
  WDOG1_WCR  = WDOG_WCR_WT(WATCHDOG_TIMEOUT_HALF_S - 1)    // timeout = (WT + 1) x 0.5 s
             | WDOG_WCR_WDZST                              // pause in low-power modes
             | WDOG_WCR_SRS | WDOG_WCR_WDA                 // don't self-trigger a reset now
             | WDOG_WCR_WDT                                // assert reset on timeout
             | WDOG_WCR_WDE;                               // enable (write-once)
}
static inline void watchdogFeed() { WDOG1_WSR = 0x5555; WDOG1_WSR = 0xAAAA; }
#else
static inline void watchdogStart() {}
static inline void watchdogFeed() {}
#endif

// ---------------------------------------------------------------------------
// Serial diagnostics
// ---------------------------------------------------------------------------
#ifdef SERIAL_ENABLE
// Cycle-count profiling (600 MHz => 600 cycles per microsecond).
static uint64_t profBusyCycles  = 0;   // cycles spent on frame work this interval
static uint32_t profMaxFrameCyc = 0;   // worst single handleFrame() this interval
static uint32_t profFrames      = 0;
static const uint16_t SNIFF_MAX = 128;
struct SniffEntry { uint32_t id; bool ext; uint32_t count; uint8_t len; uint8_t data[8]; };
static SniffEntry sniff[2][SNIFF_MAX];
static uint16_t   sniffCount[2];
static uint32_t   sniffLost[2];

static void sniffRecord(uint8_t bus, const CAN_message_t &m) {
  for (uint16_t i = 0; i < sniffCount[bus]; i++) {
    SniffEntry &e = sniff[bus][i];
    if (e.id == m.id && e.ext == (bool)m.flags.extended) {
      e.count++; e.len = m.len; memcpy(e.data, m.buf, 8);
      return;
    }
  }
  if (sniffCount[bus] >= SNIFF_MAX) { sniffLost[bus]++; return; }
  SniffEntry &e = sniff[bus][sniffCount[bus]++];
  e.id = m.id; e.ext = m.flags.extended; e.count = 1; e.len = m.len; memcpy(e.data, m.buf, 8);
}

static void printSniff(uint8_t bus, float secs) {
  // simple insertion sort by ID for readable output
  for (uint16_t i = 1; i < sniffCount[bus]; i++) {
    SniffEntry k = sniff[bus][i]; int16_t j = i - 1;
    while (j >= 0 && sniff[bus][j].id > k.id) { sniff[bus][j + 1] = sniff[bus][j]; j--; }
    sniff[bus][j + 1] = k;
  }
  Serial.printf("--- Bus %c IDs (%u unique) ---\n", bus ? 'B' : 'A', sniffCount[bus]);
  for (uint16_t i = 0; i < sniffCount[bus]; i++) {
    watchdogFeed();   // a long dump to a slow USB host mustn't trip the watchdog
    SniffEntry &e = sniff[bus][i];
    CAN_message_t probe;
    probe.id = e.id;
    probe.flags.extended = e.ext;
    int ri = findRoute(probe, bus == BUS_A ? A_TO_B : B_TO_A);
    Serial.printf(e.ext ? "  %08lX" : "  %03lX     ", (unsigned long)e.id);
    Serial.printf("  %7.1f Hz  [%u]", e.count / secs, e.len);
    for (uint8_t d = 0; d < e.len && d < 8; d++) Serial.printf(" %02X", e.data[d]);
    Serial.printf("%s\n", ri < 0 ? "" : (ROUTES[ri].action == DROP ? "   <DROP>" : "   <ROUTED>"));
    e.count = 0;
  }
  if (sniffLost[bus]) Serial.printf("  (%lu frames from IDs beyond table limit)\n", (unsigned long)sniffLost[bus]);
  sniffLost[bus] = 0;
}

static void printStats(uint32_t now) {
  static uint32_t lastPrint = 0;
  static BusStats prev[2];
  if (now - lastPrint < STATS_INTERVAL_MS) return;
  float secs = (now - lastPrint) / 1000.0f;
  lastPrint = now;

  if (PRINT_STATS) {
    for (uint8_t b = 0; b < 2; b++) {
      const BusStats &s = stats[b], &p = prev[b];
      Serial.printf("[Bus %c] rx %6.0f/s  fwd->%c %6.0f/s  filtered %6.0f/s  suppressed %6.0f/s  "
                    "tx %6.0f/s | rxOvf %lu txOvf %lu stale %lu busOff %lu\n",
                    b ? 'B' : 'A',
                    (s.rx - p.rx) / secs, b ? 'A' : 'B', (s.fwd - p.fwd) / secs,
                    (s.filtered - p.filtered) / secs, (s.suppressed - p.suppressed) / secs,
                    (s.tx - p.tx) / secs,
                    (unsigned long)s.rxOverflow, (unsigned long)s.txOverflow,
                    (unsigned long)s.txStale, (unsigned long)s.busOffEvents);
      prev[b] = s;
    }
  }
  if (PROFILE_CPU) {
    const float cycPerSec = (float)F_CPU_ACTUAL;
    Serial.printf("[CPU] routing load %.2f%%  frames %lu  avg %.2f us/frame  worst %.2f us/frame\n",
                  100.0f * (float)profBusyCycles / (cycPerSec * secs),
                  (unsigned long)profFrames,
                  profFrames ? (float)profBusyCycles / profFrames / (cycPerSec / 1e6f) : 0.0f,
                  (float)profMaxFrameCyc / (cycPerSec / 1e6f));
    profBusyCycles = 0; profMaxFrameCyc = 0; profFrames = 0;
  }
  if (PRINT_STATS) {
    static uint32_t prevSent[NUM_GENERATED], prevNoSrc[NUM_GENERATED];
    for (uint8_t i = 0; i < NUM_GENERATED; i++) {
      Serial.printf("[GEN %08lX] sent %5.1f/s  no-source %5.1f/s",
                    (unsigned long)GENERATED[i].id,
                    (genSent[i] - prevSent[i]) / secs, (genNoSource[i] - prevNoSrc[i]) / secs);
      if (VERIFY_GENERATED)
        Serial.printf(" | verify: checked %lu  CONSTANT MISMATCH %lu  live diff %lu",
                      (unsigned long)verChecked[i], (unsigned long)verFixedErr[i],
                      (unsigned long)verLiveDiff[i]);
      Serial.println();
      // What actually went to the cluster, and the M1's own copy if it still sends one.
      Serial.print("      ours:");
      if (genSent[i]) for (uint8_t b = 0; b < 8; b++) Serial.printf(" %02X", genLastBuf[i][b]);
      else            Serial.print(GENERATE_ASTON_FRAMES ? " (none sent yet)" : " (generation OFF)");
      Serial.print("   M1:");
      if (m1Seen[i]) { Serial.printf(" [%u]", m1LastLen[i]); for (uint8_t b = 0; b < 8; b++) Serial.printf(" %02X", m1LastBuf[i][b]); }
      else           Serial.print(" (M1 not sending this ID)");
      Serial.println();
      prevSent[i] = genSent[i]; prevNoSrc[i] = genNoSource[i];
    }
  }
  if (SNIFF_IDS) { printSniff(BUS_A, secs); printSniff(BUS_B, secs); }
}
#endif

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
// setMaxMB(16): the RX FIFO area takes 8 mailboxes, leaving 8 for TX. That's
// plenty, since loop() refills them within microseconds. Fewer mailboxes also
// means fewer old frames left in hardware while a bus is down.
#define INIT_BUS(can, baud, handler) do { \
    can.begin();                           \
    can.setBaudRate(baud);                 \
    can.setMaxMB(16);                      \
    can.enableFIFO();                      \
    can.enableFIFOInterrupt();             \
    can.onReceive(handler);                \
  } while (0)

void setup() {
  const uint32_t resetCause = SRC_SRSR;  // why did we (re)boot? (cleared at end of setup)

  // Wake the transceivers first: standby pins LOW = normal mode.
  for (uint8_t i = 0; i < sizeof(CAN_STANDBY_PINS); i++) {
    pinMode(CAN_STANDBY_PINS[i], OUTPUT);
    digitalWriteFast(CAN_STANDBY_PINS[i], LOW);
  }
  (void)resetCause;

  buildRouteIndex();                     // before the buses start delivering frames
  INIT_BUS(canA, BUS_A_BAUD, onRxA);
  INIT_BUS(canB, BUS_B_BAUD, onRxB);
  watchdogStart();

#ifdef SERIAL_ENABLE
  // The isolator is already forwarding while we wait here for a USB host.
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {
    watchdogFeed();
    // keep frames moving during the wait
    CAN_message_t m; uint32_t now = millis();
    while (rxRing[BUS_A].pop(m)) handleFrame(BUS_A, m, now);
    while (rxRing[BUS_B].pop(m)) handleFrame(BUS_B, m, now);
    flushPending(now); runGenerators(now); drainTx(BUS_A); drainTx(BUS_B);
  }
  if (resetCause & SRC_SRSR_WDOG_RST_B)
    Serial.println("*** Last reset was caused by the WATCHDOG (loop stalled) ***");
  if (CrashReport) Serial.print(CrashReport);   // Teensy's record of a previous fault
  Serial.printf("CanIsolator ready. A=%lu bps, B=%lu bps, %u routes, default A->B %s, B->A %s\n",
                (unsigned long)BUS_A_BAUD, (unsigned long)BUS_B_BAUD, NUM_ROUTES,
                DEFAULT_FORWARD_A_TO_B ? "PASS" : "DROP", DEFAULT_FORWARD_B_TO_A ? "PASS" : "DROP");
  for (uint16_t i = 0; i < NUM_ROUTES; i++) {
    const Route &r = ROUTES[i];
    if (isRange(r) && (r.minIntervalMs || r.onChange))
      Serial.printf("WARNING: route %u is a range; minIntervalMs/onChange are ignored.\n", i);
    if (r.heartbeatMs && !r.onChange)
      Serial.printf("WARNING: route %u has heartbeatMs without onChange; it has no effect.\n", i);
  }
#endif
  SRC_SRSR = resetCause;   // write-1-to-clear, so the next boot reports only its own cause
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
// Do NOT call canA.events() / canB.events(): that would move the RX callbacks
// out of the ISR and switch on the library's TX buffer.
void loop() {
  watchdogFeed();
  const uint32_t now = millis();
  CAN_message_t m;

#ifdef SERIAL_ENABLE
  const uint32_t passStart = ARM_DWT_CYCCNT;
  uint32_t passFrames = 0;
#endif

  // Alternate between the buses so a flood on one side can't starve the other.
  for (uint16_t n = 0; n < 64; n++) {
#ifdef SERIAL_ENABLE
    uint32_t f0 = ARM_DWT_CYCCNT;
#endif
    bool gotA = rxRing[BUS_A].pop(m); if (gotA) handleFrame(BUS_A, m, now);
    bool gotB = rxRing[BUS_B].pop(m); if (gotB) handleFrame(BUS_B, m, now);
    if (!gotA && !gotB) break;
#ifdef SERIAL_ENABLE
    uint8_t k = (uint8_t)gotA + (uint8_t)gotB;
    uint32_t per = (ARM_DWT_CYCCNT - f0) / k;
    if (per > profMaxFrameCyc) profMaxFrameCyc = per;
    passFrames += k;
#endif
  }

  flushPending(now);
  runGenerators(now);
  drainTx(BUS_A);
  drainTx(BUS_B);
  checkBusHealth();

#ifdef SERIAL_ENABLE
  // Count the whole pass (routing + TX drain + health) only when it did real
  // work. Idle spinning isn't load. ISR time (~1 us/frame) isn't included.
  if (passFrames) { profBusyCycles += ARM_DWT_CYCCNT - passStart; profFrames += passFrames; }
  printStats(now);
#endif
}
