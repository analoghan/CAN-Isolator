// CanIsolatorC6.ino
// Two-bus CAN isolator / gateway for the TechOverflow CANipulator
// (ESP32-C6-WROOM-1U, two on-chip TWAI controllers, two CAN transceivers).
// Originally ported from the Teensy 4.x / FlexCAN_T4 version (deprecated;
// see the git tag "teensy-final").
// Toolchain: Arduino-ESP32 4.x (ESP-IDF 6.x) - uses the IDF "esp_twai" driver.
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
//   - transform       a built-in function that rewrites the frame
//   - minIntervalMs   rate limit; e.g. a 100 Hz frame forwarded at 10 Hz.
//                     The NEWEST frame is always what gets forwarded, so the
//                     far side never ends up holding stale data.
//   - onChange        forward only when the (transformed) data changes...
//   - heartbeatMs     ...but still resend unchanged data at least this often
//
// The isolator can also BUILD frames itself on a timer: constant bytes plus
// fields copied (and rescaled) from other frames, or rolling counters. It
// does this for the Aston Martin cluster frames, so the M1 no longer has to
// transmit them. These are settings too (see GENERATED FRAMES).
//
// IDs that are not in the table follow the default forward setting for
// their direction.
//
// ============================================================
// CONFIGURATION (new in 2.0)
// ============================================================
// The settings (bit rates, connector swap, defaults, route table, button
// map, which generated frames are on) are DATA, not code:
//
//   - The DEFAULT_* tables below are the factory defaults.
//   - The browser tool (ConfigTool/index.html, Chrome or Edge) reads and
//     edits the settings over the USB-C cable. "Send to board" applies them
//     straight away, in RAM only: a power cycle goes back to the saved
//     settings, so you can try a change safely. "Save to board" stores them
//     in flash.
//   - Saved settings are checked (version + checksum + every field) at boot.
//     If they are missing or damaged, the factory defaults are used.
//   - FACTORY RESET without a computer: hold the BOOT button for 3 seconds
//     while the isolator is running. The LED flashes white, the saved
//     settings are erased, and the board restarts on the factory defaults.
//     (Don't hold BOOT while powering up or resetting: that starts the
//     chip's USB flashing mode instead of the firmware.)
//   - Bit rate and connector swap changes take effect after a restart; all
//     other settings take effect immediately.
//   - Saving writes to flash, which pauses the CAN interrupts for a moment:
//     a few frames can be missed. Save with the engine off.
//   - Settings live in their own 64 KB flash area ('isocfg', partitions.csv).
//     Settings saved by older firmware in the default area are moved there
//     at the first boot.
//
// FIRMWARE UPDATES (2.4+): the tool's "Update firmware" sends a .bin over the
// USB link. It is written to the other app slot while the isolator keeps
// running, verified (image checksum), and only then made the boot slot. After
// the restart the new firmware must run for FW_CONFIRM_MS with both CAN
// controllers up before it confirms itself; a crash or watchdog reset before
// that and the bootloader goes back to the previous firmware.
//
// Route transforms: "Merge wheel buttons" is built in (code); custom
// transforms (2.5+) are data: lists of steps (set ID / length / field, copy a
// field from this or another frame with scaling, forward only if a field
// matches, rolling counter) edited in the tool. Generated frames are data too.
//
// The USB link is a plain text protocol (one command per line, one JSON
// object per reply line), described at handleCommand(). The isolator never
// waits for the computer: if the USB output can't keep up, replies are
// dropped, never the CAN traffic.
//
// ============================================================
// WIRING (CANipulator)
// ============================================================
//   Connector CAN0: TX GPIO17, RX GPIO16, RS GPIO0, shutdown GPIO3
//   Connector CAN1: TX GPIO19, RX GPIO18, RS GPIO1, shutdown GPIO2
//   RS and shutdown are driven LOW at boot (transceiver active).
//
//   Which connector each bus uses is the "swap" setting:
//     off (default) : Bus A (noisy) = CAN0,  Bus B (M1) = CAN1
//     on            : Bus A (noisy) = CAN1,  Bus B (M1) = CAN0
//   Only the physical connector changes. All routing, generated frames, and
//   stats keep the same meaning: A is always the noisy side, B is the M1.
//   120 ohm termination is a solder jumper per channel (open from the
//   factory). Bridge it only if the isolator is at a physical end of that
//   bus. On the M1 side the isolator is normally one of only two nodes, so
//   that side usually needs it.
//
//   Status LED (RGB, GPIO8), updated twice a second:
//     green   = frames received on both buses
//     blue    = frames received on one bus only
//     red     = a bus went error-passive or bus-off in the last half second
//     off     = no traffic on either bus
//     magenta = CAN driver failed to start
//     white   = BOOT held: factory reset in progress
//
// ============================================================
// ARCHITECTURE
// ============================================================
//   TWAI RX ISR (per bus) -> lock-free RX ring -> loop(): route / transform /
//   rate limit -> software TX ring (per bus) -> TWAI controller
//
// The ESP32-C6 TWAI controller has ONE transmit buffer, so frames leave in
// exactly the order they were queued: rolling counters and multi-frame
// sequences stay intact without any extra ordering logic. The isolator keeps
// its own TX ring and hands the driver one frame at a time (from a fixed
// per-bus buffer), only after the previous one is done. Frames that wait in
// the ring longer than the TX max age are discarded instead of sent late.
//
// loop() sleeps when there is nothing to do and is woken by the CAN
// interrupts (or after 1 ms for timers and the USB link), so forwarding
// latency stays in the microseconds.

#include <Arduino.h>
#include <Preferences.h>
#include <stdarg.h>
#include <stddef.h>
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_task_wdt.h"
#include "esp_cpu.h"
#include "esp_system.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "GenEngine.h"   // generated-frame bit math (unit-tested on a PC)
#include "esp_ota_ops.h"
#include "nvs_flash.h"
#include "mbedtls/base64.h"

#define FW_VERSION_STR "2.5.5"
static const char    *FW_NAME       = "CanIsolatorC6";
static const char    *FW_VERSION    = FW_VERSION_STR;
// Marker the config tool looks for in a .bin before updating, so it can show
// the version and refuse files that aren't this firmware. Keep the format.
extern "C" __attribute__((used)) const char FW_TAG[] = "CANISO-FW:CanIsolatorC6:" FW_VERSION_STR ":";
static const uint8_t  PROTO_VERSION = 1;

// ---------------------------------------------------------------------------
// FRAME TYPE
// ---------------------------------------------------------------------------
// Field names follow FlexCAN_T4's CAN_message_t (kept from the original
// Teensy port) so the transforms and generators read the same.
struct CAN_message_t {
  uint32_t id = 0;
  uint16_t timestamp = 0;        // isolator use: millis() when queued for TX
  struct {
    bool extended = false;
    bool remote = false;
  } flags;
  uint8_t len = 8;
  uint8_t buf[8] = { 0 };
  uint32_t t100us = 0;           // isolator use: receive time, 0.1 ms units (frame monitor)
};

// ---------------------------------------------------------------------------
// FIXED HARDWARE / ROBUSTNESS CONFIG (not editable from the tool)
// ---------------------------------------------------------------------------
// Watchdog: if loop() stops running for WATCHDOG_TIMEOUT_MS, the ESP32 resets
// itself and resumes forwarding (boot takes roughly 0.3 s).
#define WATCHDOG_ENABLE
static const uint32_t WATCHDOG_TIMEOUT_MS = 1000;
// Priority of the CAN task (Arduino's loop task is 1). See canTask().
static const UBaseType_t CAN_TASK_PRIORITY = 5;

// CANipulator pin map, indexed by connector (CAN0, CAN1).
static const gpio_num_t CH_TX[2]   = { GPIO_NUM_17, GPIO_NUM_19 };
static const gpio_num_t CH_RX[2]   = { GPIO_NUM_16, GPIO_NUM_18 };
static const uint8_t    CH_RS[2]   = { 0, 1 };   // HIGH = standby, LOW = normal
static const uint8_t    CH_SHDN[2] = { 3, 2 };   // HIGH = shutdown, LOW = normal
static const uint8_t    PIN_LED    = 8;          // addressable RGB status LED
static const uint8_t    PIN_BOOT   = 9;          // BOOT button, LOW = pressed
static const uint32_t   FACTORY_RESET_HOLD_MS = 3000;

static const uint8_t BUS_A = 0;   // noisy side
static const uint8_t BUS_B = 1;   // MoTeC M1

// Bit rates the tool may choose.
static const uint32_t VALID_BAUDS[] = { 125000, 250000, 500000, 1000000 };

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

// Built-in transforms a route can use (by number, so they can be stored).
// To add one: write the function in TRANSFORMS below, then add it to this
// enum, XF_NAMES and XF_FNS (same position in all three).
enum TransformId : uint8_t {
  XF_NONE          = 0,
  XF_MERGE_BUTTONS = 1,   // copy steering wheel bytes into this frame (BUTTON MAP)
  NUM_XF
};
static const char *const XF_NAMES[NUM_XF] = { "none", "mergeButtons" };
static const uint8_t XF_CUSTOM = NUM_XF;   // route xf values from here on are custom transforms (settings)

// Transform callback. Edit 'msg' in place: you may change id, flags.extended,
// len and buf[]. Return false to suppress the frame completely.
typedef bool (*TransformFn)(CAN_message_t &msg);

// ---------------------------------------------------------------------------
// STORED CONFIGURATION
// ---------------------------------------------------------------------------
static const uint16_t MAX_ROUTES = 128;
static const uint8_t  MAX_BTN    = 8;
static const uint8_t  NOTE_LEN   = 24;   // including the terminating 0

// One route. Column order matches the old Route table:
//   first  last  ext  dir  action  transform  minMs  onChange  hbMs  note
struct RouteCfg {
  uint32_t idFirst;       // CAN ID (or first ID of an inclusive range)
  uint32_t idLast;        // last ID of range; 0 = single ID (same as idFirst)
  uint8_t  ext;           // 0 = 11-bit, 1 = 29-bit
  uint8_t  dir;           // Direction
  uint8_t  action;        // Action
  uint8_t  xf;            // TransformId
  uint16_t minMs;         // 0 = no rate limit           (single-ID routes only)
  uint8_t  onChange;      // forward only on data change (single-ID routes only)
  uint16_t hbMs;          // with onChange: resend unchanged data after this long
  char     note[NOTE_LEN];// free text, shown in the tool
};

// One BUTTON MAP row: which wheel byte goes into which byte of the frame
// that uses the mergeButtons transform (the ABS frame).
//   src / dst : buf[] index, 0-based (CAN "byte 1" = buf[0])
//   mask      : which bits to copy. Other dst bits keep the ABS value.
//   catchTaps : 0 = plain copy of the latest wheel value. Safe for any
//                   encoding: a momentary button stays momentary, a toggle
//                   stays a toggle.
//               1 = also report a press that started AND ended between two
//                   ABS frames, for one ABS frame. NOT a latch: the bit still
//                   drops when the button is released. Use ONLY for
//                   one-bit-per-button bytes: ORing values corrupts them.
struct ButtonMapCfg { uint8_t src; uint8_t dst; uint8_t mask; uint8_t catchTaps; };

static const uint32_t CFG_MAGIC   = 0x4F534943;   // "CISO"
static const uint16_t CFG_VERSION = 4;            // bump when Config's layout changes (2: generated frames, 3: hardware filter, 4: custom transforms)

struct Config {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint32_t baud[2];          // [BUS_A], [BUS_B]                    (restart)
  uint8_t  swapChannels;     // 0: A=CAN0 B=CAN1, 1: A=CAN1 B=CAN0  (restart)
  uint8_t  defFwd[2];        // unrouted IDs: [BUS_A] = A->B, [BUS_B] = B->A
  uint8_t  numGen;           // generated frames in use
  uint16_t txMaxAgeMs;       // queued frame older than this is discarded
  uint16_t buttonStaleMs;    // wheel frame older than this = buttons released
  uint32_t buttonsId;        // steering wheel frame (Bus A), source for mergeButtons
  uint8_t  buttonsExt;
  uint8_t  numButtonMap;
  uint16_t numRoutes;
  ButtonMapCfg buttonMap[MAX_BTN];
  RouteCfg     routes[MAX_ROUTES];
  GenCfg       gens[MAX_GEN];
  uint8_t  hwFilterA;        // 1 = hardware acceptance filter on Bus A (computed from the routes)
  uint8_t  reserved[31];     // room for future settings without a layout change (keep 0)
  uint8_t  numXf;            // custom transforms in use (routes use them as xf = XF_CUSTOM + index)
  uint8_t  xfPad[3];
  XfCfg    xforms[MAX_XF];
  uint32_t crc;              // CRC32 of everything above
};

// Where the running settings came from (reported to the tool).
enum CfgSource : uint8_t { SRC_DEFAULTS, SRC_SAVED, SRC_SAVED_BAD };

// ---------------------------------------------------------------------------
// FACTORY DEFAULTS (edit these to change what a fresh / reset board runs)
// ---------------------------------------------------------------------------
// Bus A = noisy side (ABS, keypad / buttons, gauge board, ...)
// Bus B = quiet side (MoTeC M1)
static const uint32_t ID_ABS     = 0x2B2;   // 690: ABS frame on Bus A, forwarded to the M1 with buttons merged in
static const uint32_t ID_BUTTONS = 0x178;   // 376: steering wheel on Bus A, forwarded AND extra bytes copied into ABS

static const uint32_t DEFAULT_BAUD_A = 500000;
static const uint32_t DEFAULT_BAUD_B = 500000;
static const bool     DEFAULT_SWAP_CAN_CHANNELS = false;

// What to do with IDs that are NOT in the route table.
// A->B (noisy -> MoTeC): whitelist. Only what the route table allows reaches
//   the M1. This is what fixes the M1 transmit overload.
// B->A (MoTeC -> noisy): pass everything. Before the isolator, every M1 frame
//   was already on the noisy bus, so this adds no load there, and every
//   noisy-side device that listens to the M1 keeps working.
static const bool DEFAULT_FORWARD_A_TO_B = false;
static const bool DEFAULT_FORWARD_B_TO_A = true;

// A queued frame that cannot be sent within this time is discarded instead of
// being sent late (far bus unplugged, ECU off, bus too busy): when it comes
// back you get fresh data, not a burst of old frames.
static const uint16_t DEFAULT_TX_MAX_AGE_MS = 50;

// Steering wheel frame older than this = wheel offline: mapped bits read 0.
static const uint16_t DEFAULT_BUTTON_STALE_MS = 250;

// Hardware acceptance filter on Bus A (noisy side): frames the isolator has
// no use for are rejected by the CAN controller itself, before they cost an
// interrupt. See HARDWARE FILTER. Only active while unrouted A->B IDs are
// dropped (whitelist).
static const bool DEFAULT_HW_FILTER_A = true;

// The ABS frame (690) uses data bytes 1-3 only, so bytes 4-8 are free. The
// steering wheel frame (376) is still forwarded to the M1 unchanged; the
// bytes mapped here are ALSO copied into 690, so the M1 can read buttons
// beyond the first 16 without another receive slot.
static const ButtonMapCfg DEFAULT_BUTTON_MAP[] = {
  //  wheel  ABS   mask   catchTaps
  {   6,     3,    0xFF,  0 },   // wheel byte 7 -> ABS byte 4
  {   7,     4,    0xFF,  0 },   // wheel byte 8 -> ABS byte 5
};

// The first matching row wins (it must match ID, ID type, and direction).
// Put specific DROP rows ABOVE any range rows they should override.
// Rate limits on frames the gauge board uses: its timeouts are 500 ms, so
// keep minMs at 100 or below on those IDs.
static const RouteCfg DEFAULT_ROUTES[] = {
  //  first       last ext dir     action xf                minMs onChg hbMs  note (max 23 chars)
  // ===== Bus A (noisy) -> Bus B (MoTeC M1): whitelist =====
  { ID_ABS,       0,   0,  A_TO_B, PASS,  XF_MERGE_BUTTONS, 0,    0,    0,    "ABS + wheel buttons"     },
  { ID_BUTTONS,   0,   0,  A_TO_B, PASS,  XF_NONE,          0,    0,    0,    "Steering wheel buttons"  },
  { 0x0CE,        0,   0,  A_TO_B, PASS,  XF_NONE,          0,    0,    0,    "ABS wheel speeds"        },
  // ===== Bus B (MoTeC M1) -> Bus A (noisy): M1 output used by the gauge board =====
  { 0x640,        0,   0,  B_TO_A, PASS,  XF_NONE,          0,    0,    0,    "Gauge: MAP"              },
  { 0x641,        0,   0,  B_TO_A, PASS,  XF_NONE,          0,    0,    0,    "Gauge: fuel press, inj"  },
  { 0x644,        0,   0,  B_TO_A, PASS,  XF_NONE,          0,    0,    0,    "Gauge: oil pressure"     },
  { 0x649,        0,   0,  B_TO_A, PASS,  XF_NONE,          0,    0,    0,    "Gauge: coolant, battery" },
  { 0x64A,        0,   0,  B_TO_A, PASS,  XF_NONE,          0,    0,    0,    "Gauge: ambient press"    },
  { 0x64C,        0,   0,  B_TO_A, PASS,  XF_NONE,          0,    0,    0,    "Gauge: warning flags"    },
  { 0x64E,        0,   0,  B_TO_A, PASS,  XF_NONE,          0,    0,    0,    "Gauge: launch/TC icons"  },
  { 0x650,        0,   0,  B_TO_A, PASS,  XF_NONE,          0,    0,    0,    "Gauge: two-step icon"    },
  { 0x651,        0,   0,  B_TO_A, PASS,  XF_NONE,          0,    0,    0,    "Gauge: lambda 1/2"       },
  { 0x659,        0,   0,  B_TO_A, PASS,  XF_NONE,          0,    0,    0,    "Gauge: vehicle speed"    },
  { 0x661,        0,   0,  B_TO_A, PASS,  XF_NONE,          0,    0,    0,    "Gauge: cruise"           },
  { 0x670,        0,   0,  B_TO_A, PASS,  XF_NONE,          0,    0,    0,    "Gauge: ethanol %"        },
  { 0x6A8,        0,   0,  B_TO_A, PASS,  XF_NONE,          0,    0,    0,    "Gauge: exhaust bypass"   },
  // ===== Aston Martin cluster =====
  // While the isolator generates one of these frames (GENERATED), the M1's own
  // copy is dropped automatically, whatever its row says. With generation
  // off, these rows pass the M1's copy through.
  { 0x15B00260,   0,   1,  B_TO_A, PASS,  XF_NONE,          0,    0,    0,    "Aston Dash 1/3 (M1)"     },
  { 0x00040360,   0,   1,  B_TO_A, PASS,  XF_NONE,          0,    0,    0,    "Aston Dash 2/3 (M1)"     },
  { 0x091C0170,   0,   1,  B_TO_A, PASS,  XF_NONE,          0,    0,    0,    "Aston Dash 3/3 (M1)"     },
  { 0x168,        0,   0,  B_TO_A, PASS,  XF_NONE,          0,    0,    0,    "Legacy Zestek?"          },
};

// ---------------------------------------------------------------------------
// Active configuration
// ---------------------------------------------------------------------------
static Config cfg;        // what the isolator is running
static Config staging;    // being received from the tool
static uint8_t  busChannel[2] = { 0, 1 };   // connector used by each bus (set at boot)
static uint32_t hwBaud[2];                  // bit rates the controllers run at
static uint8_t  hwSwap;

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
// The list is built from the settings: entry 0 is the steering wheel frame
// (mergeButtons), then every distinct source frame of the generated frames.
struct CacheDef { uint8_t bus; uint32_t id; bool extended; };
static const uint8_t CACHE_BUTTONS = 0;
static const uint8_t MAX_CACHED = 1 + MAX_GEN * MAX_GEN_SIG + MAX_XF * MAX_XF_OPS;
static CacheDef cacheIds[MAX_CACHED] = { { BUS_A, ID_BUTTONS, false } };
static uint8_t  numCached = 1;

struct CachedFrame {
  CAN_message_t msg;
  uint32_t      rxMs;
  bool          valid;
  uint8_t       latch[8];   // OR of every frame since the last getCachedLatched()
};
static CachedFrame frameCache[MAX_CACHED];

// Called for every received frame, before routing.
static void cacheUpdate(uint8_t bus, const CAN_message_t &m, uint32_t now) {
  for (uint8_t i = 0; i < numCached; i++) {
    const CacheDef &c = cacheIds[i];
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
// For BIT-PER-BUTTON data only (ORing value-encoded bytes corrupts them).
bool getCachedLatched(uint8_t bus, uint32_t id, CAN_message_t &out, uint32_t maxAgeMs = 0, bool extended = false) {
  for (uint8_t i = 0; i < numCached; i++) {
    const CacheDef &c = cacheIds[i];
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
// is too old, or isn't cached.
bool getCached(uint8_t bus, uint32_t id, CAN_message_t &out, uint32_t maxAgeMs = 0, bool extended = false) {
  for (uint8_t i = 0; i < numCached; i++) {
    const CacheDef &c = cacheIds[i];
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
// mergeButtons: copy the BUTTON MAP bytes of the steering wheel frame into
// this frame. Until the wheel frame has been seen at least once since boot,
// the frame passes through UNMODIFIED. Wheel stale = mapped bits read 0.
bool xfMergeButtons(CAN_message_t &msg) {
  CAN_message_t latest, taps;
  const uint32_t id  = cfg.buttonsId;
  const bool     ext = cfg.buttonsExt;
  if (!getCached(BUS_A, id, latest, 0, ext)) return true;              // wheel never seen: as-is
  bool fresh = getCached(BUS_A, id, latest, cfg.buttonStaleMs, ext);
  getCachedLatched(BUS_A, id, taps, 0, ext);                           // OR since last frame; resets it

  msg.len = 8;
  for (uint8_t i = 0; i < cfg.numButtonMap; i++) {
    const ButtonMapCfg &m = cfg.buttonMap[i];
    uint8_t src = 0;                                                   // stale wheel = released
    if (fresh) src = m.catchTaps ? taps.buf[m.src] : latest.buf[m.src];
    msg.buf[m.dst] = (msg.buf[m.dst] & ~m.mask) | (src & m.mask);
  }
  return true;
}

static const TransformFn XF_FNS[NUM_XF] = { nullptr, xfMergeButtons };

// ---------------------------------------------------------------------------
// GENERATED FRAMES (built by the isolator instead of the M1)
// ---------------------------------------------------------------------------
// The Aston Martin cluster frames used to be M1 "CAN Transmit Custom"
// messages 1-3. Each one is a fixed 8-byte pattern plus a single live value
// that is already in the M1's General Ver5 broadcast, so the isolator builds
// them itself on Bus A, byte-for-byte the same as the M1's.
//
// Each generated frame is DATA (see GenEngine.h): ID, bus, period, constant
// bytes, and up to 4 fields that are either copied from a cached source
// frame (out = round(raw x factor + offset), clamped to the field) or a
// rolling counter. The tool edits them; the defaults are below.
//
// SOURCE RULE: a frame is only sent while every source frame is fresh
// (younger than the field's stale time). M1 off -> the cluster frames stop,
// exactly as they did when the M1 sent them itself.
//
// While a frame is enabled, any copy of the same ID arriving on the OTHER bus
// (the M1's own) is dropped, so the receiver never sees two sources. Turn the
// frame off to pass that copy through instead.
static const uint16_t DEFAULT_GEN_STALE_MS = 500;

// NOTE ON UNITS: the M1 custom transmit scales a channel's INTERNAL value
// (Engine Speed deg/s, Vehicle Speed m/s, Coolant degC), not the Ver5 DBC
// units. The factors below reproduce the M1's output from the Ver5 values.
static GenCfg &defaultGen(Config &c, const char *name, uint32_t id, uint8_t ext, uint8_t bus,
                          uint16_t periodMs, const uint8_t base[8]) {
  GenCfg &g = c.gens[c.numGen++];
  memset(&g, 0, sizeof(g));
  strncpy(g.name, name, GEN_NAME_LEN - 1);
  g.id = id; g.ext = ext; g.bus = bus; g.periodMs = periodMs; g.len = 8; g.enabled = 1;
  memcpy(g.base, base, 8);
  return g;
}
static void defaultSourceField(GenCfg &g, uint8_t srcBus, uint32_t srcId, uint8_t srcStart, uint8_t srcLen, uint8_t srcOrder,
                               uint8_t dstStart, uint8_t dstLen, uint8_t dstOrder, float factor, float offset) {
  GenSigCfg &f = g.sig[g.numSig++];
  memset(&f, 0, sizeof(f));
  f.kind = GS_SOURCE; f.srcBus = srcBus; f.srcId = srcId; f.srcExt = 0;
  f.srcStart = srcStart; f.srcLen = srcLen; f.srcOrder = srcOrder;
  f.dstStart = dstStart; f.dstLen = dstLen; f.dstOrder = dstOrder;
  f.factor = factor; f.offset = offset; f.staleMs = DEFAULT_GEN_STALE_MS;
}

static void loadDefaultGens(Config &c) {
  c.numGen = 0;
  // Aston Dash 1/3 - 0x15B00260 (29-bit), 10 Hz
  //   bytes 0-6 : 00 00 B5 7F 5F 00 C0  constant
  //   byte  7   : Coolant degC + 65  <-  M1 0x649 byte 0 (degC + 40) + 25
  static const uint8_t B1[8] = { 0x00, 0x00, 0xB5, 0x7F, 0x5F, 0x00, 0xC0, 0x00 };
  defaultSourceField(defaultGen(c, "Aston Dash 1/3", 0x15B00260, 1, BUS_A, 100, B1),
                     BUS_B, 0x649, 0, 8, BO_INTEL,      56, 8, BO_INTEL,     1.0f, 25.0f);
  // Aston Dash 2/3 - 0x00040360 (29-bit), 100 Hz
  //   bytes 0-4 : 00 00 63 04 AC, bits 40-50 all ones   constant
  //   bits 51-63: Engine Speed rpm, 13 bits big-endian  <-  M1 0x640 bytes 0-1 (rpm, big-endian)
  static const uint8_t B2[8] = { 0x00, 0x00, 0x63, 0x04, 0xAC, 0xFF, 0xE0, 0x00 };
  defaultSourceField(defaultGen(c, "Aston Dash 2/3", 0x00040360, 1, BUS_A, 10, B2),
                     BUS_B, 0x640, 7, 16, BO_MOTOROLA,  52, 13, BO_MOTOROLA, 1.0f, 0.0f);
  // Aston Dash 3/3 - 0x091C0170 (29-bit), 10 Hz
  //   bytes 0-5 : 00 08 DF 7E C0 00, byte 7: 00          constant
  //   byte  6   : Vehicle Speed m/s x 1.42  <-  M1 0x659 bytes 4-5 (0.1 km/h, big-endian) x 1.42 / 36
  static const uint8_t B3[8] = { 0x00, 0x08, 0xDF, 0x7E, 0xC0, 0x00, 0x00, 0x00 };
  defaultSourceField(defaultGen(c, "Aston Dash 3/3", 0x091C0170, 1, BUS_A, 100, B3),
                     BUS_B, 0x659, 39, 16, BO_MOTOROLA, 48, 8, BO_INTEL,     1.42f / 36.0f, 0.0f);
}

static inline bool genEnabled(uint8_t i) { return i < cfg.numGen && cfg.gens[i].enabled; }

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
static RouteState routeState[MAX_ROUTES][2];   // [route][source bus]

// ---------------------------------------------------------------------------
// Statistics
// ---------------------------------------------------------------------------
struct BusStats {
  uint32_t rx;            // frames received on this bus
  uint32_t fwd;           // frames forwarded FROM this bus to the other
  uint32_t filtered;      // dropped: no route / DROP route / transform veto / generated copy
  uint32_t suppressed;    // dropped: rate limit or unchanged data
  uint32_t tx;            // frames transmitted ON this bus
  uint32_t rxOverflow;    // RX ring full (loop() too slow)
  uint32_t txOverflow;    // TX ring full (bus saturated / not acked)
  uint32_t txStale;       // discarded: waited longer than the TX max age
  uint32_t busOffEvents;  // transitions into bus-off
};
static BusStats stats[2];
static volatile uint32_t rxOverflowIsr[2];

// Timing health (reset each time the tool reads 'stat'): the longest loop
// pass and which step took longest in it, the longest a received frame waited
// before routing, and the longest a frame waited in the TX ring.
static const char *const STEP_NAMES[] = { "route", "pending", "generators", "testframes", "drainA", "drainB", "health", "led",
                                          "host", "usbout", "hwfilter", "ota", "button" };
static uint32_t diagPassUs = 0, diagRxWaitUs = 0, diagTxWaitMs = 0, diagCmdUs = 0;
static bool     diagResetPending = false;
static char     diagCmd[12] = "";
static uint8_t  diagStep = 0;
static uint32_t diagStepUs = 0;

// CPU load (cycle counter; 160 cycles per microsecond at 160 MHz)
static uint64_t profBusyCycles  = 0;   // cycles spent on passes that routed frames
static uint32_t profMaxFrameCyc = 0;   // worst single frame since the last 'stat'

// ---------------------------------------------------------------------------
// TWAI driver glue
// ---------------------------------------------------------------------------
static twai_node_handle_t canNode[2];
static bool               canOk[2];
static TaskHandle_t       loopTaskHandle = nullptr;   // the CAN task (woken by the CAN interrupts)

// Transmit: one frame in flight per bus. The driver keeps a POINTER to the
// frame until it is sent, so it lives in this fixed buffer, which is only
// rewritten once txBusy is false again.
struct TxSlot { twai_frame_t frame; uint8_t data[8]; };
static TxSlot            txSlot[2];
static volatile bool     txBusy[2];
static volatile uint8_t  busState[2];        // twai_error_state_t, updated from the ISR
static volatile bool     busTrouble[2];      // went passive / bus-off since last LED update

static inline bool wakeLoopFromIsr() {
  BaseType_t woken = pdFALSE;
  if (loopTaskHandle) vTaskNotifyGiveFromISR(loopTaskHandle, &woken);
  return woken == pdTRUE;
}

// RX ISR: copy the frame into the ring and wake loop(). Keep minimal.
static bool onRxDone(twai_node_handle_t h, const twai_rx_done_event_data_t *, void *ctx) {
  const uint8_t bus = (uint8_t)(uintptr_t)ctx;
  uint8_t data[8] = { 0 };
  twai_frame_t f = {};
  f.buffer = data;
  f.buffer_len = sizeof(data);
  if (twai_node_receive_from_isr(h, &f) != ESP_OK) return false;
  CAN_message_t m;
  m.id             = f.header.id;
  m.flags.extended = f.header.ide;
  m.flags.remote   = f.header.rtr;
  uint16_t len     = twaifd_dlc2len(f.header.dlc);
  m.len            = len > 8 ? 8 : (uint8_t)len;
  memcpy(m.buf, data, 8);
  m.t100us         = (uint32_t)(esp_timer_get_time() / 100);
  if (!rxRing[bus].push(m)) rxOverflowIsr[bus] = rxOverflowIsr[bus] + 1;
  return wakeLoopFromIsr();
}

static bool onTxDone(twai_node_handle_t, const twai_tx_done_event_data_t *, void *ctx) {
  txBusy[(uint8_t)(uintptr_t)ctx] = false;
  return wakeLoopFromIsr();
}

static bool onStateChange(twai_node_handle_t, const twai_state_change_event_data_t *e, void *ctx) {
  const uint8_t bus = (uint8_t)(uintptr_t)ctx;
  busState[bus] = (uint8_t)e->new_sta;
  if (e->new_sta == TWAI_ERROR_PASSIVE || e->new_sta == TWAI_ERROR_BUS_OFF) busTrouble[bus] = true;
  // Leaving bus-off: the driver abandons the frame that was in flight without
  // a tx-done callback, so the slot is free again.
  if (e->old_sta == TWAI_ERROR_BUS_OFF) txBusy[bus] = false;
  return wakeLoopFromIsr();
}

static bool initBus(uint8_t bus, uint32_t baud) {
  const uint8_t ch = busChannel[bus];
  twai_onchip_node_config_t c = {};
  c.io_cfg.tx                = CH_TX[ch];
  c.io_cfg.rx                = CH_RX[ch];
  c.io_cfg.quanta_clk_out    = GPIO_NUM_NC;
  c.io_cfg.bus_off_indicator = GPIO_NUM_NC;
  c.bit_timing.bitrate       = baud;
  c.fail_retry_cnt           = -1;   // retransmit until ACKed (standard CAN behaviour)
  c.tx_queue_depth           = 2;    // we only ever have one frame in flight
  if (twai_new_node_onchip(&c, &canNode[bus]) != ESP_OK) return false;

  twai_event_callbacks_t cbs = {};
  cbs.on_rx_done      = onRxDone;
  cbs.on_tx_done      = onTxDone;
  cbs.on_state_change = onStateChange;
  if (twai_node_register_event_callbacks(canNode[bus], &cbs, (void *)(uintptr_t)bus) != ESP_OK) return false;
  busState[bus] = TWAI_ERROR_ACTIVE;
  return twai_node_enable(canNode[bus]) == ESP_OK;
}

// Firmware update state (logic: "Firmware update over the USB link")
static const uint32_t OTA_IDLE_ABORT_MS = 10000;
static const uint32_t FW_CONFIRM_MS     = 10000;   // new firmware must run this long before it confirms itself
static esp_ota_handle_t       otaHandle = 0;
static const esp_partition_t *otaPart = nullptr;
static bool     otaActive = false, otaReady = false;
static uint32_t otaSize = 0, otaWritten = 0, otaCrc = 0, otaLastMs = 0;
static bool     fwPendingVerify = false;           // running a just-updated image, not confirmed yet
static uint8_t  otaBuf[600];


// Hardware filter state (logic: HARDWARE FILTER, near setup())
struct HwFilter { bool acceptAll; uint16_t code1, mask1, code2, mask2; };
static HwFilter hwfWanted  = { true, 0, 0, 0, 0 };   // from the settings
static HwFilter hwfApplied = { true, 0, 0, 0, 0 };   // in the controller (reset = accept all)
static uint16_t hwfPassKeys = 2048;                   // 11-bit keys the wanted filter accepts
static uint16_t hwfNeedKeys = 0;
static uint32_t hwfChanges = 0, hwfPauseUs = 0, hwfErrors = 0;
// Why the filter is not active: 0 active, 1 setting off, 2 unrouted A->B IDs
// are forwarded, 3 a needed ID is 29-bit, 4 open for the sniffer / monitor,
// 5 internal check failed, 6 controller not running
static uint8_t  hwfWhy = 1;

// ---------------------------------------------------------------------------
// Frame monitor (on only while the tool asks for it)
// ---------------------------------------------------------------------------
// Captures individual frames that match up to MON_MAX_FILTERS filters, both
// received (RX) and sent by the isolator (TX), into a ring the tool empties.
// "where" bits: 1 = Bus A RX, 2 = Bus B RX, 4 = Bus A TX, 8 = Bus B TX.
static const uint8_t  MON_MAX_FILTERS = 8;
static const uint16_t MON_RING = 1024;          // power of 2
static const uint32_t MON_AUTO_OFF_MS = 5000;   // tool closed: stop capturing
static const uint16_t MON_PER_REPLY   = 64;     // frames per 'mon' reply (keeps each reply quick; the tool asks again)
struct MonFilter { uint32_t first, last; uint8_t ext; uint8_t where; };   // ext 2 = either
struct MonRec    { uint32_t t; uint32_t id; uint8_t where, ext, len; uint8_t data[8]; };
static MonFilter monF[MON_MAX_FILTERS];
static uint8_t   monN = 0;
static bool      monOn = false;
static uint32_t  monPollMs;
static MonRec    monRing[MON_RING];
static uint16_t  monHead = 0, monTail = 0;
static uint32_t  monLost = 0;

static void monRecord(uint8_t whereBit, const CAN_message_t &m, uint32_t t100us) {
  for (uint8_t i = 0; i < monN; i++) {
    const MonFilter &f = monF[i];
    if (!(f.where & whereBit)) continue;
    if (f.ext != 2 && f.ext != (uint8_t)m.flags.extended) continue;
    if (m.id < f.first || m.id > f.last) continue;
    uint16_t next = (monHead + 1) & (MON_RING - 1);
    if (next == monTail) { monLost++; return; }
    MonRec &r = monRing[monHead];
    r.t = t100us; r.id = m.id; r.where = whereBit; r.ext = m.flags.extended; r.len = m.len;
    memcpy(r.data, m.buf, 8);
    monHead = next;
    return;
  }
}

// Each queued frame carries the low 16 bits of millis() from when it was
// queued, in its 'timestamp' field.
static void drainTx(uint8_t bus) {
  if (!canOk[bus]) return;
  const uint16_t now16 = (uint16_t)millis();
  const CAN_message_t *m;
  while ((m = txRing[bus].peek()) != nullptr) {
    if ((uint16_t)(now16 - m->timestamp) > cfg.txMaxAgeMs) {
      txRing[bus].drop();          // too old to be useful: discard, try the next one
      stats[bus].txStale++;
      continue;
    }
    if (txBusy[bus] || busState[bus] == TWAI_ERROR_BUS_OFF) return;

    TxSlot &s = txSlot[bus];
    s.frame = twai_frame_t();
    s.frame.header.id  = m->id;
    s.frame.header.ide = m->flags.extended;
    s.frame.header.rtr = m->flags.remote;
    s.frame.header.dlc = m->len;
    memcpy(s.data, m->buf, 8);
    s.frame.buffer     = s.data;
    s.frame.buffer_len = m->flags.remote ? 0 : m->len;

    txBusy[bus] = true;            // before transmit: tx-done may fire immediately
    if (twai_node_transmit(canNode[bus], &s.frame, 0) != ESP_OK) {
      txBusy[bus] = false;         // bus-off or queue full: retry on a later pass
      return;
    }
    if (monOn) monRecord(4 << bus, *m, (uint32_t)(esp_timer_get_time() / 100));
    { uint32_t w = (uint16_t)(now16 - m->timestamp); if (w > diagTxWaitMs) diagTxWaitMs = w; }
    txRing[bus].drop();
    stats[bus].tx++;
  }
}

static bool queueTx(uint8_t dstBus, const CAN_message_t &m) {
  CAN_message_t q = m;
  q.timestamp = (uint16_t)millis();
  if (txRing[dstBus].push(q)) return true;
  stats[dstBus].txOverflow++;
  return false;
}

// ---------------------------------------------------------------------------
// Routing
// ---------------------------------------------------------------------------
static inline bool isRange(const RouteCfg &r) { return r.idLast != 0 && r.idLast != r.idFirst; }

static int findRouteSlow(uint32_t id, bool ext, uint8_t dirBit) {
  for (uint16_t i = 0; i < cfg.numRoutes; i++) {
    const RouteCfg &r = cfg.routes[i];
    if (!(r.dir & dirBit)) continue;
    if ((bool)r.ext != ext) continue;
    uint32_t last = r.idLast ? r.idLast : r.idFirst;
    if (id >= r.idFirst && id <= last) return i;
  }
  return -1;
}

// Constant-time lookup for 11-bit IDs: the result of the table scan for every
// possible standard ID, per direction, recomputed whenever the routes change.
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

// Clean copy: carries only what belongs on the wire.
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
  const RouteCfg &r  = cfg.routes[ri];
  RouteState     &st = routeState[ri][src];
  const uint8_t dst = src ^ 1;

  if (r.onChange && st.everSent && sameFrame(out, st.lastSent) &&
      (r.hbMs == 0 || (now - st.lastSentMs) < r.hbMs)) {
    st.hasPending = false;          // newest data equals what the far side already has
    stats[src].suppressed++;
    return;
  }

  if (r.minMs && st.everSent && (now - st.lastSentMs) < r.minMs) {
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
  for (uint16_t ri = 0; ri < cfg.numRoutes; ri++) {
    for (uint8_t src = 0; src < 2; src++) {
      RouteState &st = routeState[ri][src];
      if (!st.hasPending) continue;
      if (st.everSent && (now - st.lastSentMs) < cfg.routes[ri].minMs) continue;
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
static uint32_t genDueMs[MAX_GEN];
static bool     genStarted[MAX_GEN];
static uint32_t genSent[MAX_GEN];       // frames queued
static uint32_t genNoSource[MAX_GEN];   // periods skipped: a source frame missing / stale / too short
static uint8_t  genLastBuf[MAX_GEN][8];
static uint32_t genCounters[MAX_GEN][MAX_GEN_SIG];
// Derived from the settings by applyConfig():
static uint8_t  genSrcCache[MAX_GEN][MAX_GEN_SIG];    // frameCache index of each field's source
static uint8_t  genSrcMinLen[MAX_GEN][MAX_GEN_SIG];   // source must be at least this long
static uint8_t  genFixedMask[MAX_GEN][8];             // bits no field writes (constant)
static bool     anyGenEnabled = false;

// Build generated frame i into out[]. 'counters' is advanced only when the
// frame is really sent (the copy check passes a scratch copy).
static bool buildGenerated(uint8_t i, uint8_t out[8], uint32_t counters[MAX_GEN_SIG]) {
  const GenCfg &g = cfg.gens[i];
  const uint32_t now = millis();
  return genBuild(g, out, counters, [&](uint8_t k) -> const uint8_t * {
    const CachedFrame &cf = frameCache[genSrcCache[i][k]];
    if (!cf.valid || cf.msg.len < genSrcMinLen[i][k]) return nullptr;
    if (g.sig[k].staleMs && (now - cf.rxMs) > g.sig[k].staleMs) return nullptr;
    return cf.msg.buf;
  });
}

static void runGenerators(uint32_t now) {
  for (uint8_t i = 0; i < cfg.numGen; i++) {
    if (!genEnabled(i)) continue;
    const GenCfg &g = cfg.gens[i];
    if (genStarted[i] && (int32_t)(now - genDueMs[i]) < 0) continue;   // not due yet
    if (genStarted[i] && (now - genDueMs[i]) < g.periodMs) genDueMs[i] += g.periodMs;
    else                                                   genDueMs[i]  = now + g.periodMs;
    genStarted[i] = true;

    CAN_message_t out;
    out.id             = g.id;
    out.flags.extended = g.ext;
    out.len            = g.len;
    if (!buildGenerated(i, out.buf, genCounters[i])) { genNoSource[i]++; continue; }
    if (queueTx(g.bus, out)) {
      genSent[i]++;
      memcpy(genLastBuf[i], out.buf, 8);
    }
  }
}

// ---------------------------------------------------------------------------
// Test frames from the tool (send once, or repeat in one of TXP_SLOTS slots)
// ---------------------------------------------------------------------------
// Repeating test frames stop by themselves if the tool goes quiet for
// TXP_AUTO_OFF_MS (closed, unplugged, crashed), so a forgotten test can
// never keep sending into the car.
static const uint8_t  TXP_SLOTS = 4;
static const uint32_t TXP_AUTO_OFF_MS = 5000;
struct TxPeriodic { bool on; uint8_t bus; uint16_t periodMs; uint32_t dueMs; CAN_message_t m; uint32_t sent; };
static TxPeriodic txp[TXP_SLOTS];
static uint32_t   lastHostMs = 0;      // last command line received from the tool
static uint32_t   testSent[2];         // test frames queued per bus (once + periodic)

static void runTestFrames(uint32_t now) {
  bool any = false;
  for (uint8_t i = 0; i < TXP_SLOTS; i++) any |= txp[i].on;
  if (!any) return;
  if (now - lastHostMs > TXP_AUTO_OFF_MS) { for (uint8_t i = 0; i < TXP_SLOTS; i++) txp[i].on = false; return; }
  for (uint8_t i = 0; i < TXP_SLOTS; i++) {
    TxPeriodic &t = txp[i];
    if (!t.on || (int32_t)(now - t.dueMs) < 0) continue;
    t.dueMs = ((now - t.dueMs) < t.periodMs) ? t.dueMs + t.periodMs : now + t.periodMs;
    if (queueTx(t.bus, t.m)) { t.sent++; testSent[t.bus]++; }
  }
}

// The M1's own copies of the generated IDs: compare each one with what we'd
// build right now (constant bits must always match), and report whether it
// must be dropped (frame enabled) or routed normally (frame off).
static uint32_t verChecked[MAX_GEN];
static uint32_t verFixedErr[MAX_GEN];   // constant bits differ: MUST stay 0
static uint32_t verLiveDiff[MAX_GEN];   // field values differ (timing/rounding)
static uint32_t m1Seen[MAX_GEN];        // copies seen on the other bus
static uint8_t  m1LastBuf[MAX_GEN][8];
static uint8_t  m1LastLen[MAX_GEN];

// A frame with a generated frame's ID arriving on the OTHER bus (normally the
// M1's own copy): compare it with what we'd build right now (constant bits
// must always match) and say whether to drop it (frame enabled).
static bool isGeneratedCopy(uint8_t src, const CAN_message_t &m1) {
  for (uint8_t i = 0; i < cfg.numGen; i++) {
    const GenCfg &g = cfg.gens[i];
    if (g.id != m1.id || g.ext != (uint8_t)m1.flags.extended || src == g.bus) continue;
    m1Seen[i]++;
    memcpy(m1LastBuf[i], m1.buf, 8);
    m1LastLen[i] = m1.len;
    uint8_t ours[8];
    uint32_t scratch[MAX_GEN_SIG];
    memcpy(scratch, genCounters[i], sizeof(scratch));
    if (buildGenerated(i, ours, scratch)) {
      verChecked[i]++;
      bool fixedBad = (m1.len != g.len), liveBad = false;
      for (uint8_t b = 0; b < g.len; b++) {
        uint8_t diff = m1.buf[b] ^ ours[b];
        if (diff & genFixedMask[i][b])  fixedBad = true;
        if (diff & ~genFixedMask[i][b]) liveBad  = true;
      }
      if (liveBad)  verLiveDiff[i]++;
      if (fixedBad) verFixedErr[i]++;
    }
    return g.enabled;
  }
  return false;
}

// ---------------------------------------------------------------------------
// ID sniffer (on only while the tool asks for it)
// ---------------------------------------------------------------------------
static const uint16_t SNIFF_SLOTS = 256;   // hash table, power of 2
static const uint16_t SNIFF_MAX   = 200;   // distinct IDs per bus
struct SniffEntry { uint32_t id; uint32_t count; uint8_t used, ext, len; uint8_t data[8]; };
static SniffEntry sniffTab[2][SNIFF_SLOTS];
static uint16_t   sniffN[2];
static uint32_t   sniffLost[2];
static bool       sniffOn = false;
static uint32_t   sniffPollMs, sniffSinceMs;
static const uint32_t SNIFF_AUTO_OFF_MS = 5000;   // tool closed: stop sniffing

static void sniffClear() {
  memset(sniffTab, 0, sizeof(sniffTab));
  sniffN[0] = sniffN[1] = 0;
  sniffLost[0] = sniffLost[1] = 0;
}

static void sniffRecord(uint8_t bus, const CAN_message_t &m) {
  uint32_t h = ((m.id * 2654435761u) >> 24) ^ (m.flags.extended ? 0x55 : 0);
  for (uint16_t p = 0; p < SNIFF_SLOTS; p++) {
    SniffEntry &e = sniffTab[bus][(h + p) & (SNIFF_SLOTS - 1)];
    if (!e.used) {
      if (sniffN[bus] >= SNIFF_MAX) { sniffLost[bus]++; return; }
      sniffN[bus]++;
      e.used = 1; e.id = m.id; e.ext = m.flags.extended; e.count = 0;
    } else if (e.id != m.id || e.ext != (uint8_t)m.flags.extended) {
      continue;
    }
    e.count++; e.len = m.len; memcpy(e.data, m.buf, 8);
    return;
  }
}

// ---------------------------------------------------------------------------
// Custom transforms (data, see GenEngine.h)
// ---------------------------------------------------------------------------
static uint8_t  xfSrcCache[MAX_XF][MAX_XF_OPS];
static uint8_t  xfSrcMinLen[MAX_XF][MAX_XF_OPS];
static uint32_t xfCounters[MAX_XF][MAX_XF_OPS];

static bool runCustomXf(uint8_t xi, CAN_message_t &out, const CAN_message_t &rx) {
  const XfCfg &x = cfg.xforms[xi];
  uint32_t id = out.id;
  uint8_t  ext = out.flags.extended, len = out.len;
  const uint32_t now = millis();
  bool keep = xfApply(x, id, ext, len, out.buf, rx.buf, rx.len, xfCounters[xi], [&](uint8_t k) -> const uint8_t * {
    const CachedFrame &cf = frameCache[xfSrcCache[xi][k]];
    if (!cf.valid || cf.msg.len < xfSrcMinLen[xi][k]) return nullptr;
    if (x.ops[k].staleMs && (now - cf.rxMs) > x.ops[k].staleMs) return nullptr;
    return cf.msg.buf;
  });
  if (!keep) return false;
  out.id = id; out.flags.extended = ext; out.len = len;
  return true;
}

// ---------------------------------------------------------------------------
// Frame handling
// ---------------------------------------------------------------------------
static void handleFrame(uint8_t src, const CAN_message_t &rxRaw, uint32_t now) {
  // The ISR copies all 8 data bytes whatever the frame's length (DLC). Bytes
  // past the DLC can be leftovers from an earlier frame: zero them, so a
  // short frame can never leak junk into a merged frame or the on-change
  // comparison.
  CAN_message_t rx = rxRaw;
  if (rx.len > 8) rx.len = 8;
  for (uint8_t i = rx.len; i < 8; i++) rx.buf[i] = 0;

  stats[src].rx++;
  {
    uint32_t waitUs = ((uint32_t)(esp_timer_get_time() / 100) - rx.t100us) * 100;
    if (waitUs > diagRxWaitUs) diagRxWaitUs = waitUs;
  }
  cacheUpdate(src, rx, now);
  if (sniffOn) sniffRecord(src, rx);
  if (monOn) monRecord(1 << src, rx, rx.t100us);
  if (anyGenEnabled && isGeneratedCopy(src, rx)) { stats[src].filtered++; return; }

  const uint8_t dst    = src ^ 1;
  const uint8_t dirBit = (src == BUS_A) ? A_TO_B : B_TO_A;
  const int     ri     = findRoute(rx, dirBit);

  if (ri < 0) {
    if (cfg.defFwd[src]) {
      CAN_message_t out; copyFrame(out, rx);
      if (queueTx(dst, out)) stats[src].fwd++;
    } else {
      stats[src].filtered++;
    }
    return;
  }

  const RouteCfg &r = cfg.routes[ri];
  if (r.action == DROP) { stats[src].filtered++; return; }

  CAN_message_t out; copyFrame(out, rx);
  if (r.xf >= XF_CUSTOM) {
    if (!runCustomXf(r.xf - XF_CUSTOM, out, rx)) { stats[src].filtered++; return; }
  } else {
    TransformFn xf = XF_FNS[r.xf];
    if (xf && !xf(out)) { stats[src].filtered++; return; }
  }
  if (out.len > 8) out.len = 8;

  if (isRange(r) || (r.minMs == 0 && !r.onChange)) {
    if (queueTx(dst, out)) stats[src].fwd++;
    return;
  }
  offerRateLimited((uint16_t)ri, src, out, now);
}

// ---------------------------------------------------------------------------
// Bus health
// ---------------------------------------------------------------------------
// The TWAI controller stays off the bus after bus-off until it is told to
// recover, so this starts recovery (which itself waits for 128 x 11
// recessive bits, as the CAN standard requires) and counts events.
static void checkBusHealth() {
  static bool wasBusOff[2] = { false, false };
  for (uint8_t b = 0; b < 2; b++) {
    if (!canOk[b]) continue;
    bool busOff = busState[b] == TWAI_ERROR_BUS_OFF;
    if (busOff && !wasBusOff[b]) {
      stats[b].busOffEvents++;
      twai_node_recover(canNode[b]);
    }
    wasBusOff[b] = busOff;
    stats[b].rxOverflow = rxOverflowIsr[b];
  }
}

// ---------------------------------------------------------------------------
// Watchdog (ESP-IDF task watchdog on the loop task)
// ---------------------------------------------------------------------------
#ifdef WATCHDOG_ENABLE
static void watchdogStart() {
  esp_task_wdt_config_t c = {};
  c.timeout_ms     = WATCHDOG_TIMEOUT_MS;
  c.idle_core_mask = 0;          // only watch the loop task
  c.trigger_panic  = true;       // panic -> reboot
  if (esp_task_wdt_reconfigure(&c) != ESP_OK) esp_task_wdt_init(&c);
  esp_task_wdt_add(nullptr);     // the current (loop) task
}
static inline void watchdogFeed() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last >= 50) { last = now; esp_task_wdt_reset(); }
}
#else
static inline void watchdogStart() {}
static inline void watchdogFeed() {}
#endif

// ---------------------------------------------------------------------------
// Status LED
// ---------------------------------------------------------------------------
static bool ledOverride = false;   // factory-reset flash owns the LED

static void updateLed(uint32_t now) {
  static uint32_t last = 0, prevRx[2] = { 0, 0 };
  static uint32_t shown = 0xFFFFFFFF;
  if (ledOverride) { shown = 0xFFFFFFFF; return; }
  if (now - last < 500) return;
  last = now;
  bool trafficA = stats[BUS_A].rx != prevRx[BUS_A];
  bool trafficB = stats[BUS_B].rx != prevRx[BUS_B];
  prevRx[BUS_A] = stats[BUS_A].rx;
  prevRx[BUS_B] = stats[BUS_B].rx;
  bool trouble = busTrouble[BUS_A] || busTrouble[BUS_B] ||
                 busState[BUS_A] >= TWAI_ERROR_PASSIVE || busState[BUS_B] >= TWAI_ERROR_PASSIVE;
  busTrouble[BUS_A] = false;
  busTrouble[BUS_B] = false;
  uint32_t rgb = (!canOk[0] || !canOk[1]) ? 0x400040
               : trouble ? 0x200000 : (trafficA && trafficB) ? 0x002000 : (trafficA || trafficB) ? 0x000020 : 0;
  if (rgb != shown) {            // the LED write takes ~30 us, so only on change
    shown = rgb;
    rgbLedWrite(PIN_LED, (rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF);
  }
}

// ---------------------------------------------------------------------------
// Configuration: defaults, checking, applying, flash storage
// ---------------------------------------------------------------------------
static const char *const CFG_SOURCE_NAMES[] = { "defaults", "saved", "defaults (saved settings were invalid)" };
static CfgSource cfgSource = SRC_DEFAULTS;
static bool      haveSaved = false;     // flash holds a valid config
static uint32_t  savedCrc  = 0;         // its CRC (to tell whether RAM differs)
static uint32_t  defaultsCrc = 0;       // CRC of the factory defaults

static uint32_t cfgCrc(const Config &c) {
  return esp_rom_crc32_le(0, (const uint8_t *)&c, offsetof(Config, crc));
}

// Notes: printable ASCII only, no quote or backslash (they go into JSON as is).
static void sanitizeNote(char *dst, const char *src) {
  uint8_t n = 0;
  for (uint8_t i = 0; i < NOTE_LEN - 1 && src[i]; i++) {
    char c = src[i];
    if (c >= 0x20 && c <= 0x7E && c != '"' && c != '\\') dst[n++] = c;
  }
  while (n < NOTE_LEN) dst[n++] = 0;
}

// Rewrites every route and map row through a zeroed copy and clears unused
// slots, so equal settings always give byte-identical structs (and CRCs).
static void normalizeConfig(Config &c) {
  c.magic = CFG_MAGIC; c.version = CFG_VERSION; c.size = sizeof(Config);
  if (c.numRoutes > MAX_ROUTES) c.numRoutes = MAX_ROUTES;
  if (c.numButtonMap > MAX_BTN) c.numButtonMap = MAX_BTN;
  for (uint16_t i = 0; i < MAX_ROUTES; i++) {
    RouteCfg t; memset(&t, 0, sizeof(t));
    if (i < c.numRoutes) {
      const RouteCfg &s = c.routes[i];
      t.idFirst = s.idFirst; t.idLast = s.idLast; t.ext = s.ext; t.dir = s.dir;
      t.action = s.action; t.xf = s.xf; t.minMs = s.minMs; t.onChange = s.onChange; t.hbMs = s.hbMs;
      char note[NOTE_LEN]; memcpy(note, s.note, NOTE_LEN); note[NOTE_LEN - 1] = 0;
      sanitizeNote(t.note, note);
    }
    memcpy(&c.routes[i], &t, sizeof(t));
  }
  for (uint8_t i = 0; i < MAX_BTN; i++) {
    ButtonMapCfg t = { 0, 0, 0, 0 };
    if (i < c.numButtonMap) t = c.buttonMap[i];
    c.buttonMap[i] = t;
  }
  memset(c.reserved, 0, sizeof(c.reserved));
  memset(c.xfPad, 0, sizeof(c.xfPad));
  if (c.numXf > MAX_XF) c.numXf = MAX_XF;
  for (uint8_t i = 0; i < MAX_XF; i++) {
    XfCfg t; memset(&t, 0, sizeof(t));
    if (i < c.numXf) {
      const XfCfg &x = c.xforms[i];
      char name[GEN_NAME_LEN]; memcpy(name, x.name, GEN_NAME_LEN); name[GEN_NAME_LEN - 1] = 0;
      sanitizeNote(t.name, name);
      t.numOps = x.numOps > MAX_XF_OPS ? MAX_XF_OPS : x.numOps;
      for (uint8_t k = 0; k < t.numOps; k++) {
        const XfOpCfg &a = x.ops[k];
        XfOpCfg &o = t.ops[k];
        o.srcId = a.srcId; o.value = a.value; o.factor = a.factor; o.offset = a.offset; o.staleMs = a.staleMs;
        o.kind = a.kind; o.srcBus = a.srcBus; o.srcExt = a.srcExt; o.srcStart = a.srcStart; o.srcLen = a.srcLen;
        o.srcOrder = a.srcOrder; o.srcSigned = a.srcSigned; o.dstStart = a.dstStart; o.dstLen = a.dstLen;
        o.dstOrder = a.dstOrder; o.dstSigned = a.dstSigned; o.cmp = a.cmp; o.step = a.step; o.flags = a.flags;
      }
    }
    memcpy(&c.xforms[i], &t, sizeof(t));
  }
  if (c.numGen > MAX_GEN) c.numGen = MAX_GEN;
  for (uint8_t i = 0; i < MAX_GEN; i++) {
    GenCfg t; memset(&t, 0, sizeof(t));
    if (i < c.numGen) {
      const GenCfg &s = c.gens[i];
      t.id = s.id; t.periodMs = s.periodMs; t.ext = s.ext; t.bus = s.bus; t.len = s.len; t.enabled = s.enabled;
      t.numSig = s.numSig > MAX_GEN_SIG ? MAX_GEN_SIG : s.numSig;
      memcpy(t.base, s.base, 8);
      for (uint8_t b = t.len; b < 8; b++) t.base[b] = 0;
      char name[GEN_NAME_LEN]; memcpy(name, s.name, GEN_NAME_LEN); name[GEN_NAME_LEN - 1] = 0;
      sanitizeNote(t.name, name);
      for (uint8_t k = 0; k < t.numSig; k++) {
        const GenSigCfg &a = s.sig[k];
        GenSigCfg &f = t.sig[k];
        f.kind = a.kind; f.dstStart = a.dstStart; f.dstLen = a.dstLen; f.dstOrder = a.dstOrder; f.dstSigned = a.dstSigned;
        if (a.kind == GS_SOURCE) {
          f.srcId = a.srcId; f.factor = a.factor; f.offset = a.offset; f.staleMs = a.staleMs;
          f.srcBus = a.srcBus; f.srcExt = a.srcExt; f.srcStart = a.srcStart; f.srcLen = a.srcLen;
          f.srcOrder = a.srcOrder; f.srcSigned = a.srcSigned;
        } else if (a.kind == GS_COUNTER) {
          f.step = a.step;
        }
      }
    }
    memcpy(&c.gens[i], &t, sizeof(t));
  }
  c.crc = 0;
}

static void loadDefaults(Config &c) {
  memset(&c, 0, sizeof(c));
  c.baud[BUS_A]     = DEFAULT_BAUD_A;
  c.baud[BUS_B]     = DEFAULT_BAUD_B;
  c.swapChannels    = DEFAULT_SWAP_CAN_CHANNELS;
  c.defFwd[BUS_A]   = DEFAULT_FORWARD_A_TO_B;
  c.defFwd[BUS_B]   = DEFAULT_FORWARD_B_TO_A;
  c.txMaxAgeMs      = DEFAULT_TX_MAX_AGE_MS;
  c.buttonStaleMs   = DEFAULT_BUTTON_STALE_MS;
  c.buttonsId       = ID_BUTTONS;
  c.buttonsExt      = 0;
  c.numButtonMap    = sizeof(DEFAULT_BUTTON_MAP) / sizeof(DEFAULT_BUTTON_MAP[0]);
  c.numRoutes       = sizeof(DEFAULT_ROUTES) / sizeof(DEFAULT_ROUTES[0]);
  static_assert(sizeof(DEFAULT_BUTTON_MAP) / sizeof(DEFAULT_BUTTON_MAP[0]) <= MAX_BTN, "button map too long");
  static_assert(sizeof(DEFAULT_ROUTES) / sizeof(DEFAULT_ROUTES[0]) <= MAX_ROUTES, "route table too long");
  memcpy(c.buttonMap, DEFAULT_BUTTON_MAP, sizeof(DEFAULT_BUTTON_MAP));
  memcpy(c.routes, DEFAULT_ROUTES, sizeof(DEFAULT_ROUTES));
  loadDefaultGens(c);
  c.hwFilterA = DEFAULT_HW_FILTER_A;
  normalizeConfig(c);
}

static bool validBaud(uint32_t b) {
  for (uint32_t v : VALID_BAUDS) if (v == b) return true;
  return false;
}

static bool validId(uint32_t id, uint8_t ext) { return id <= (ext ? 0x1FFFFFFFu : 0x7FFu); }

// Every field the firmware relies on. err gets a short reason.
static bool validateConfig(const Config &c, char *err, size_t errLen) {
  if (!validBaud(c.baud[0]) || !validBaud(c.baud[1])) { snprintf(err, errLen, "unsupported bit rate"); return false; }
  if (c.swapChannels > 1 || c.defFwd[0] > 1 || c.defFwd[1] > 1 || c.hwFilterA > 1) { snprintf(err, errLen, "bad on/off value"); return false; }
  if (c.txMaxAgeMs < 5 || c.txMaxAgeMs > 1000) { snprintf(err, errLen, "TX max age must be 5-1000 ms"); return false; }
  if (c.buttonStaleMs < 10 || c.buttonStaleMs > 5000) { snprintf(err, errLen, "button timeout must be 10-5000 ms"); return false; }
  if (c.buttonsExt > 1 || !validId(c.buttonsId, c.buttonsExt)) { snprintf(err, errLen, "bad buttons ID"); return false; }
  if (c.numButtonMap > MAX_BTN) { snprintf(err, errLen, "too many button map rows (max %u)", MAX_BTN); return false; }
  for (uint8_t i = 0; i < c.numButtonMap; i++) {
    const ButtonMapCfg &m = c.buttonMap[i];
    if (m.src > 7 || m.dst > 7 || m.catchTaps > 1) { snprintf(err, errLen, "button map row %u: bad value", i + 1); return false; }
  }
  if (c.numRoutes > MAX_ROUTES) { snprintf(err, errLen, "too many routes (max %u)", MAX_ROUTES); return false; }
  for (uint16_t i = 0; i < c.numRoutes; i++) {
    const RouteCfg &r = c.routes[i];
    if (r.ext > 1 || r.onChange > 1)  { snprintf(err, errLen, "route %u: bad on/off value", i + 1); return false; }
    if (!validId(r.idFirst, r.ext))   { snprintf(err, errLen, "route %u: ID too big for %s", i + 1, r.ext ? "29-bit" : "11-bit"); return false; }
    if (r.idLast && (r.idLast < r.idFirst || !validId(r.idLast, r.ext))) { snprintf(err, errLen, "route %u: bad range end", i + 1); return false; }
    if (r.dir < 1 || r.dir > 3)       { snprintf(err, errLen, "route %u: bad direction", i + 1); return false; }
    if (r.action > DROP)              { snprintf(err, errLen, "route %u: bad action", i + 1); return false; }
    if (r.xf >= XF_CUSTOM + c.numXf)  { snprintf(err, errLen, "route %u: unknown transform", i + 1); return false; }
  }
  if (c.numXf > MAX_XF) { snprintf(err, errLen, "too many transforms (max %u)", MAX_XF); return false; }
  for (uint8_t i = 0; i < c.numXf; i++) {
    const XfCfg &x = c.xforms[i];
    if (x.numOps > MAX_XF_OPS) { snprintf(err, errLen, "transform %u: too many steps (max %u)", i + 1, MAX_XF_OPS); return false; }
    for (uint8_t k = 0; k < x.numOps; k++) {
      const XfOpCfg &o = x.ops[k];
      bool bad = false;
      const char *why = "bad value";
      switch (o.kind) {
        case XO_SET_ID:    bad = o.srcExt > 1 || !validId(o.srcId, o.srcExt); why = "bad ID"; break;
        case XO_SET_LEN:   bad = o.value < 0 || o.value > 8; why = "length must be 0-8"; break;
        case XO_SET_FIELD:
        case XO_COUNTER:   bad = o.dstSigned > 1 || !genFieldFits(o.dstStart, o.dstLen, o.dstOrder, 8); why = "field outside 8 bytes"; break;
        case XO_COPY:
        case XO_COND:
          if (o.srcBus > XS_THIS || o.srcSigned > 1 || o.flags > 1 || !genFieldFits(o.srcStart, o.srcLen, o.srcOrder, 8)) { bad = true; why = "bad source field"; break; }
          if (o.srcBus != XS_THIS && (o.srcExt > 1 || !validId(o.srcId, o.srcExt))) { bad = true; why = "bad source ID"; break; }
          if (o.kind == XO_COND && o.cmp >= XC_COUNT) { bad = true; why = "bad comparison"; break; }
          if (o.kind == XO_COPY && (o.dstSigned > 1 || !genFieldFits(o.dstStart, o.dstLen, o.dstOrder, 8))) { bad = true; why = "field outside 8 bytes"; break; }
          if (o.kind == XO_COPY && (!isfinite(o.factor) || !isfinite(o.offset))) { bad = true; why = "bad factor / offset"; }
          break;
        default: bad = true; why = "unknown step";
      }
      if (bad) { snprintf(err, errLen, "transform %u step %u: %s", i + 1, k + 1, why); return false; }
    }
  }
  if (c.numGen > MAX_GEN) { snprintf(err, errLen, "too many generated frames (max %u)", MAX_GEN); return false; }
  for (uint8_t i = 0; i < c.numGen; i++) {
    const GenCfg &g = c.gens[i];
    if (g.ext > 1 || !validId(g.id, g.ext)) { snprintf(err, errLen, "generated frame %u: bad ID", i + 1); return false; }
    if (g.bus > 1 || g.enabled > 1 || g.len > 8) { snprintf(err, errLen, "generated frame %u: bad bus / length", i + 1); return false; }
    if (g.periodMs < 1 || g.periodMs > 60000) { snprintf(err, errLen, "generated frame %u: period must be 1-60000 ms", i + 1); return false; }
    if (g.numSig > MAX_GEN_SIG) { snprintf(err, errLen, "generated frame %u: too many fields (max %u)", i + 1, MAX_GEN_SIG); return false; }
    for (uint8_t k = 0; k < g.numSig; k++) {
      const GenSigCfg &f = g.sig[k];
      if (f.kind != GS_SOURCE && f.kind != GS_COUNTER) { snprintf(err, errLen, "generated frame %u field %u: bad type", i + 1, k + 1); return false; }
      if (f.dstSigned > 1 || !genFieldFits(f.dstStart, f.dstLen, f.dstOrder, g.len)) {
        snprintf(err, errLen, "generated frame %u field %u: bits outside the %u-byte frame", i + 1, k + 1, g.len); return false;
      }
      if (f.kind == GS_SOURCE) {
        if (f.srcBus > 1 || f.srcExt > 1 || !validId(f.srcId, f.srcExt) || f.srcSigned > 1 || !genFieldFits(f.srcStart, f.srcLen, f.srcOrder, 8)) {
          snprintf(err, errLen, "generated frame %u field %u: bad source", i + 1, k + 1); return false;
        }
        if (!isfinite(f.factor) || !isfinite(f.offset)) { snprintf(err, errLen, "generated frame %u field %u: bad factor / offset", i + 1, k + 1); return false; }
      }
    }
  }
  return true;
}

// Make the running isolator follow 'cfg'. Bit rate / connector changes wait
// for a restart. Called between loop passes, so no frame is ever routed
// half under the old settings and half under the new.
static uint8_t cacheSlot(CacheDef *list, uint8_t &n, uint8_t bus, uint32_t id, bool ext) {
  for (uint8_t i = 0; i < n; i++) if (list[i].bus == bus && list[i].id == id && list[i].extended == ext) return i;
  list[n] = { bus, id, ext };
  return n++;
}

static void applyConfig() {
  // Frame cache: wheel frame first, then each distinct generated-frame
  // source. Frames already cached under the same (bus, ID) are kept, so a
  // settings change doesn't pause the merged buttons or generated frames.
  CacheDef newIds[MAX_CACHED];
  uint8_t  n = 0;
  cacheSlot(newIds, n, BUS_A, cfg.buttonsId, cfg.buttonsExt);
  for (uint8_t i = 0; i < cfg.numGen; i++) {
    const GenCfg &g = cfg.gens[i];
    memset(genFixedMask[i], 0, 8);
    uint8_t written[8] = { 0 };
    for (uint8_t k = 0; k < g.numSig; k++) {
      const GenSigCfg &f = g.sig[k];
      genMark(written, f.dstStart, f.dstLen, f.dstOrder);
      if (f.kind == GS_SOURCE) {
        genSrcCache[i][k] = cacheSlot(newIds, n, f.srcBus, f.srcId, f.srcExt);
        uint8_t m[8] = { 0 }, minLen = 0;
        genMark(m, f.srcStart, f.srcLen, f.srcOrder);
        for (uint8_t b = 0; b < 8; b++) if (m[b]) minLen = b + 1;
        genSrcMinLen[i][k] = minLen;
      }
    }
    for (uint8_t b = 0; b < g.len; b++) genFixedMask[i][b] = ~written[b];
  }
  for (uint8_t i = 0; i < cfg.numXf; i++) {
    const XfCfg &x = cfg.xforms[i];
    for (uint8_t k = 0; k < x.numOps; k++) {
      const XfOpCfg &o = x.ops[k];
      if ((o.kind == XO_COPY || o.kind == XO_COND) && o.srcBus != XS_THIS) {
        xfSrcCache[i][k] = cacheSlot(newIds, n, o.srcBus, o.srcId, o.srcExt);
        uint8_t m[8] = { 0 }, minLen = 0;
        genMark(m, o.srcStart, o.srcLen, o.srcOrder);
        for (uint8_t b = 0; b < 8; b++) if (m[b]) minLen = b + 1;
        xfSrcMinLen[i][k] = minLen;
      }
    }
  }
  memset(xfCounters, 0, sizeof(xfCounters));
  static CachedFrame newCache[MAX_CACHED];
  memset(newCache, 0, sizeof(newCache));
  for (uint8_t i = 0; i < n; i++)
    for (uint8_t j = 0; j < numCached; j++)
      if (cacheIds[j].bus == newIds[i].bus && cacheIds[j].id == newIds[i].id && cacheIds[j].extended == newIds[i].extended)
        newCache[i] = frameCache[j];
  memcpy(cacheIds, newIds, sizeof(CacheDef) * n);
  memcpy(frameCache, newCache, sizeof(frameCache));
  numCached = n;

  buildRouteIndex();
  memset(routeState, 0, sizeof(routeState));
  computeHwFilter();

  // Generated frames: restart scheduling and counters, clear their stats.
  anyGenEnabled = false;
  for (uint8_t i = 0; i < cfg.numGen; i++) anyGenEnabled |= cfg.gens[i].enabled;
  memset(genStarted, 0, sizeof(genStarted));
  memset(genCounters, 0, sizeof(genCounters));
  memset(genSent, 0, sizeof(genSent));         memset(genNoSource, 0, sizeof(genNoSource));
  memset(verChecked, 0, sizeof(verChecked));   memset(verFixedErr, 0, sizeof(verFixedErr));
  memset(verLiveDiff, 0, sizeof(verLiveDiff)); memset(m1Seen, 0, sizeof(m1Seen));
}

static bool needsRestart() {
  return cfg.baud[0] != hwBaud[0] || cfg.baud[1] != hwBaud[1] || cfg.swapChannels != hwSwap;
}

// True when the running settings differ from what a restart would run
// (the saved settings, or the factory defaults if nothing is saved).
static bool isDirty() { return cfgCrc(cfg) != (haveSaved ? savedCrc : defaultsCrc); }

static const char *PREFS_NS = "isolator", *PREFS_KEY = "cfg";
static const char *CFG_PART = "isocfg";        // settings partition (partitions.csv); nullptr = default 'nvs'
static bool cfgPartOk = false;                 // isocfg usable (else settings stay in the default area)

// Open the settings namespace: the isocfg partition if the board has it.
static bool prefsOpen(Preferences &p, bool readOnly, bool legacy = false) {
  if (legacy || !cfgPartOk) return p.begin(PREFS_NS, readOnly);
  return p.begin(PREFS_NS, readOnly, CFG_PART);
}

// Prepare isocfg (fresh boards: the area held SPIFFS or nothing -> format it).
static void cfgPartInit() {
  const esp_partition_t *part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, CFG_PART);
  if (!part) return;                            // old partition table: keep using the default area
  esp_err_t e = nvs_flash_init_partition(CFG_PART);
  if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase_partition(CFG_PART);
    e = nvs_flash_init_partition(CFG_PART);
  }
  cfgPartOk = (e == ESP_OK);
}
static const size_t CFG_V2_SIZE = offsetof(Config, hwFilterA) + 4;   // layout 2 ended with the CRC here
static const size_t CFG_V3_SIZE = offsetof(Config, numXf) + 4;       // layout 3 likewise

// Loads the saved config into 'out' if it exists and passes every check.
static bool loadSaved(Config &out, bool &existed, bool legacy = false) {
  Preferences p;
  existed = false;
  if (!prefsOpen(p, true, legacy)) return false;   // namespace doesn't exist yet
  size_t n = p.getBytesLength(PREFS_KEY);
  existed = n > 0;
  bool ok = false;
  char err[64];
  if (n == sizeof(Config) && p.getBytes(PREFS_KEY, &out, sizeof(Config)) == sizeof(Config)) {
    ok = out.magic == CFG_MAGIC && out.version == CFG_VERSION && out.size == sizeof(Config) &&
         out.crc == cfgCrc(out) && validateConfig(out, err, sizeof(err));
  } else if ((n == CFG_V2_SIZE || n == CFG_V3_SIZE) && p.getBytes(PREFS_KEY, &out, n) == n) {
    // Saved by firmware 2.2 (layout 2) or 2.3-2.4 (layout 3): the same fields
    // as now up to that point, then the CRC. Keep everything and add the
    // newer settings with their defaults.
    const uint16_t ver = (n == CFG_V2_SIZE) ? 2 : 3;
    uint32_t crcOld;
    memcpy(&crcOld, (const uint8_t *)&out + n - 4, 4);
    if (out.magic == CFG_MAGIC && out.version == ver && out.size == n &&
        crcOld == esp_rom_crc32_le(0, (const uint8_t *)&out, n - 4)) {
      if (ver == 2) { out.hwFilterA = DEFAULT_HW_FILTER_A; memset(out.reserved, 0, sizeof(out.reserved)); }
      out.numXf = 0;
      normalizeConfig(out);
      ok = validateConfig(out, err, sizeof(err));
    }
  }
  p.end();
  return ok;
}

static bool saveConfig() {
  normalizeConfig(cfg);
  cfg.crc = cfgCrc(cfg);
  Preferences p;
  if (!prefsOpen(p, false)) return false;
  bool ok = p.putBytes(PREFS_KEY, &cfg, sizeof(Config)) == sizeof(Config);
  p.end();
  if (ok) { haveSaved = true; savedCrc = cfg.crc; cfgSource = SRC_SAVED; }
  return ok;
}

static bool eraseSaved() {
  Preferences p;
  if (!prefsOpen(p, false)) return false;
  bool ok = p.clear();
  p.end();
  if (ok) haveSaved = false;
  return ok;
}

// ---------------------------------------------------------------------------
// USB link: non-blocking output queue
// ---------------------------------------------------------------------------
// Replies are queued here and handed to the USB driver only as fast as it
// can take them, a little each loop pass. A reply that doesn't fit is dropped
// whole (never half a line), so a stalled or absent computer can never make
// the isolator wait.
static char     outQ[24576];
static uint32_t outHead = 0, outTail = 0;   // free-running indexes
static uint32_t outDropped = 0;             // reply lines dropped

static void outEnqueue(const char *s, size_t n) {
  if (sizeof(outQ) - (outHead - outTail) < n) { outDropped++; return; }
  uint32_t pos = outHead % sizeof(outQ);
  size_t first = n < sizeof(outQ) - pos ? n : sizeof(outQ) - pos;   // up to the end of the buffer
  memcpy(&outQ[pos], s, first);
  memcpy(outQ, s + first, n - first);                                // wrapped part (often 0 bytes)
  outHead += n;
}

static void outPump() {
  if (outHead == outTail) return;
  int room = Serial.availableForWrite();
  if (room <= 0) return;
  uint32_t pos = outTail % sizeof(outQ);
  uint32_t n = outHead - outTail;
  if (n > sizeof(outQ) - pos) n = sizeof(outQ) - pos;   // contiguous part
  if (n > (uint32_t)room) n = room;
  size_t w = Serial.write((const uint8_t *)&outQ[pos], n);
  outTail += w;
}

// One reply line is built here, then queued.
static char   lb[12288];
static size_t lbLen;
static bool   lbFull;

static void lbStart() { lbLen = 0; lbFull = false; }
static void lbf(const char *fmt, ...) {
  if (lbFull) return;
  va_list ap; va_start(ap, fmt);
  int n = vsnprintf(lb + lbLen, sizeof(lb) - lbLen, fmt, ap);
  va_end(ap);
  if (n < 0 || lbLen + n >= sizeof(lb) - 2) { lbFull = true; return; }
  lbLen += n;
}
static size_t lbRoom() { return sizeof(lb) - lbLen; }
static void lbSend() {
  if (lbFull) { lbStart(); lbf("{\"t\":\"err\",\"msg\":\"reply too long\"}"); }
  lb[lbLen++] = '\n';
  outEnqueue(lb, lbLen);
}
// Fast appenders for the replies the tool polls often (monitor, sniffer):
// printf-style formatting is slow on this chip (no FPU, big vsnprintf), and
// replies are built in the loop, so they must be quick.
static inline void lbPut(char c) { if (lbLen < sizeof(lb) - 2) lb[lbLen++] = c; else lbFull = true; }
static void lbStr(const char *s) { while (*s) lbPut(*s++); }
static void lbU(uint32_t v) {
  char t[10]; uint8_t n = 0;
  do { t[n++] = (char)('0' + v % 10); v /= 10; } while (v);
  while (n) lbPut(t[--n]);
}
static void lbHex(const uint8_t *d, uint8_t n) {
  static const char H[] = "0123456789ABCDEF";
  lbPut('"');
  for (uint8_t i = 0; i < n; i++) { lbPut(H[d[i] >> 4]); lbPut(H[d[i] & 15]); }
  lbPut('"');
}

static void replyOk(const char *cmd) { lbStart(); lbf("{\"t\":\"ok\",\"re\":\"%s\"}", cmd); lbSend(); }
static void replyErr(const char *cmd, const char *msg) {
  lbStart(); lbf("{\"t\":\"err\",\"re\":\"%s\",\"msg\":\"%s\"}", cmd, msg); lbSend();
}

// ---------------------------------------------------------------------------
// USB link: replies
// ---------------------------------------------------------------------------
static const char *resetReasonName() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "power-on";
    case ESP_RST_SW:       return "software restart";
    case ESP_RST_PANIC:    return "crash (panic)";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      return "watchdog (loop stalled)";
    case ESP_RST_BROWNOUT: return "brownout (supply dipped)";
    case ESP_RST_USB:      return "USB reset";
    case ESP_RST_JTAG:     return "JTAG / flashing";
    case ESP_RST_EXT:      return "reset pin";
    default:               return "other";
  }
}

static void sendHello() {
  lbStart();
  lbf("{\"t\":\"hello\",\"fw\":\"%s\",\"ver\":\"%s\",\"proto\":%u,\"maxRoutes\":%u,\"maxBtn\":%u,\"noteLen\":%u,",
      FW_NAME, FW_VERSION, PROTO_VERSION, MAX_ROUTES, MAX_BTN, NOTE_LEN - 1);
  lbf("\"xf\":[");
  for (uint8_t i = 0; i < NUM_XF; i++) lbf("%s\"%s\"", i ? "," : "", XF_NAMES[i]);
  lbf("],\"bauds\":[");
  for (size_t i = 0; i < sizeof(VALID_BAUDS) / sizeof(VALID_BAUDS[0]); i++) lbf("%s%lu", i ? "," : "", (unsigned long)VALID_BAUDS[i]);
  lbf("],\"maxGen\":%u,\"maxGenSig\":%u,\"genNameLen\":%u,\"maxXf\":%u,\"maxXfOps\":%u,\"xfCustom\":%u",
      MAX_GEN, MAX_GEN_SIG, GEN_NAME_LEN - 1, MAX_XF, MAX_XF_OPS, XF_CUSTOM);
  {
    const esp_partition_t *run = esp_ota_get_running_partition();
    const esp_partition_t *nxt = esp_ota_get_next_update_partition(nullptr);
    const esp_partition_t *last = esp_ota_get_last_invalid_partition();
    lbf(",\"fwTag\":\"%s\",\"slot\":\"%s\",\"otaMax\":%lu,\"pendingVerify\":%u,\"rolledBack\":%u,\"cfgPart\":\"%s\"",
        FW_TAG, run ? run->label : "?", nxt ? (unsigned long)nxt->size : 0UL, fwPendingVerify, last != nullptr,
        cfgPartOk ? CFG_PART : "nvs");
  }
  lbf(",\"reset\":\"%s\",\"canOk\":[%u,%u],\"features\":[\"mon\",\"txp\",\"gen2\",\"ota\",\"xf\"],\"monFilters\":%u,\"txpSlots\":%u}",
      resetReasonName(), canOk[0], canOk[1], MON_MAX_FILTERS, TXP_SLOTS);
  lbSend();
}

static void sendConfig() {
  lbStart();
  lbf("{\"t\":\"cfg\",\"baud\":[%lu,%lu],\"swap\":%u,\"def\":[%u,%u],\"txMaxAge\":%u,\"btnStale\":%u,"
      "\"btnId\":%lu,\"btnExt\":%u,\"hwFilterA\":%u,",
      (unsigned long)cfg.baud[0], (unsigned long)cfg.baud[1], cfg.swapChannels, cfg.defFwd[0], cfg.defFwd[1],
      cfg.txMaxAgeMs, cfg.buttonStaleMs, (unsigned long)cfg.buttonsId, cfg.buttonsExt, cfg.hwFilterA);
  lbf("\"btn\":[");
  for (uint8_t i = 0; i < cfg.numButtonMap; i++) {
    const ButtonMapCfg &m = cfg.buttonMap[i];
    lbf("%s[%u,%u,%u,%u]", i ? "," : "", m.src, m.dst, m.mask, m.catchTaps);
  }
  lbf("],\"routes\":[");
  for (uint16_t i = 0; i < cfg.numRoutes; i++) {
    const RouteCfg &r = cfg.routes[i];
    lbf("%s[%lu,%lu,%u,%u,%u,%u,%u,%u,%u,\"%s\"]", i ? "," : "",
        (unsigned long)r.idFirst, (unsigned long)r.idLast, r.ext, r.dir, r.action, r.xf,
        r.minMs, r.onChange, r.hbMs, r.note);
  }
  lbf("],\"xforms\":[");
  for (uint8_t i = 0; i < cfg.numXf; i++) {
    const XfCfg &x = cfg.xforms[i];
    lbf("%s{\"name\":\"%s\",\"ops\":[", i ? "," : "", x.name);
    for (uint8_t k = 0; k < x.numOps; k++) {
      const XfOpCfg &o = x.ops[k];
      // kind srcBus srcId srcExt srcStart srcLen srcOrder srcSigned dstStart dstLen dstOrder dstSigned value factor offset staleMs cmp step flags
      lbf("%s[%u,%u,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%ld,%.9g,%.9g,%u,%u,%u,%u]", k ? "," : "",
          o.kind, o.srcBus, (unsigned long)o.srcId, o.srcExt, o.srcStart, o.srcLen, o.srcOrder, o.srcSigned,
          o.dstStart, o.dstLen, o.dstOrder, o.dstSigned, (long)o.value, (double)o.factor, (double)o.offset,
          o.staleMs, o.cmp, o.step, o.flags);
    }
    lbf("]}");
  }
  lbf("],\"gens\":[");
  for (uint8_t i = 0; i < cfg.numGen; i++) {
    const GenCfg &g = cfg.gens[i];
    lbf("%s{\"name\":\"%s\",\"id\":%lu,\"ext\":%u,\"bus\":%u,\"period\":%u,\"len\":%u,\"on\":%u,\"base\":",
        i ? "," : "", g.name, (unsigned long)g.id, g.ext, g.bus, g.periodMs, g.len, g.enabled);
    lbHex(g.base, 8);
    lbf(",\"sig\":[");
    for (uint8_t k = 0; k < g.numSig; k++) {
      const GenSigCfg &f = g.sig[k];
      // kind srcBus srcId srcExt srcStart srcLen srcOrder srcSigned dstStart dstLen dstOrder dstSigned factor offset staleMs step
      lbf("%s[%u,%u,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%.9g,%.9g,%u,%u]", k ? "," : "",
          f.kind, f.srcBus, (unsigned long)f.srcId, f.srcExt, f.srcStart, f.srcLen, f.srcOrder, f.srcSigned,
          f.dstStart, f.dstLen, f.dstOrder, f.dstSigned, (double)f.factor, (double)f.offset, f.staleMs, f.step);
    }
    lbf("]}");
  }
  lbf("],\"src\":\"%s\",\"saved\":%u,\"dirty\":%u,\"restart\":%u,\"hw\":{\"baud\":[%lu,%lu],\"swap\":%u}}",
      CFG_SOURCE_NAMES[cfgSource], haveSaved, isDirty(), needsRestart(),
      (unsigned long)hwBaud[0], (unsigned long)hwBaud[1], hwSwap);
  lbSend();
}

static void sendStats() {
  static uint64_t prevBusy = 0;
  static uint32_t prevMs = 0;
  const uint32_t now = millis();
  const float cycPerMs = (float)getCpuFrequencyMhz() * 1000.0f;
  const uint32_t dt = now - prevMs;
  float cpu = (dt && prevMs) ? 100.0f * (float)(profBusyCycles - prevBusy) / (cycPerMs * dt) : 0.0f;
  float worstUs = (float)profMaxFrameCyc / (cycPerMs / 1000.0f);
  prevBusy = profBusyCycles; prevMs = now; profMaxFrameCyc = 0;

  lbStart();
  lbf("{\"t\":\"stat\",\"diag\":{\"passUs\":%lu,\"step\":\"%s\",\"stepUs\":%lu,\"rxWaitUs\":%lu,\"txWaitMs\":%lu,\"cmd\":\"%s\",\"cmdUs\":%lu},",
      (unsigned long)diagPassUs, STEP_NAMES[diagStep], (unsigned long)diagStepUs, (unsigned long)diagRxWaitUs, (unsigned long)diagTxWaitMs,
      diagCmd, (unsigned long)diagCmdUs);
  diagResetPending = true;         // cleared after this command, so it isn't counted in the next window
  const uint32_t cpu100 = (uint32_t)(cpu * 100.0f + 0.5f);          // integers only: float printf is slow here
  lbf("\"up\":%lu,\"cpu\":%lu.%02lu,\"worstUs\":%lu,\"heap\":%lu,\"outDrop\":%lu,\"sniff\":%u,\"bus\":[",
      (unsigned long)now, (unsigned long)(cpu100 / 100), (unsigned long)(cpu100 % 100), (unsigned long)(worstUs + 0.5f),
      (unsigned long)ESP.getFreeHeap(), (unsigned long)outDropped, sniffOn);
  for (uint8_t b = 0; b < 2; b++) {
    twai_node_status_t st = {};
    twai_node_record_t rec = {};
    if (canOk[b]) twai_node_get_info(canNode[b], &st, &rec);
    const BusStats &s = stats[b];
    lbf("%s{\"rx\":%lu,\"fwd\":%lu,\"filt\":%lu,\"supp\":%lu,\"tx\":%lu,\"rxOvf\":%lu,\"txOvf\":%lu,"
        "\"stale\":%lu,\"boff\":%lu,\"st\":%u,\"tec\":%u,\"rec\":%u,\"berr\":%lu,\"ch\":%u}",
        b ? "," : "", (unsigned long)s.rx, (unsigned long)s.fwd, (unsigned long)s.filtered,
        (unsigned long)s.suppressed, (unsigned long)s.tx, (unsigned long)s.rxOverflow,
        (unsigned long)s.txOverflow, (unsigned long)s.txStale, (unsigned long)s.busOffEvents,
        canOk[b] ? (unsigned)busState[b] : 9u, st.tx_error_count, st.rx_error_count,
        (unsigned long)rec.bus_err_num, busChannel[b]);
  }
  lbf("],\"test\":[%lu,%lu],\"txp\":[", (unsigned long)testSent[0], (unsigned long)testSent[1]);
  for (uint8_t i = 0; i < TXP_SLOTS; i++) lbf("%s[%u,%lu]", i ? "," : "", txp[i].on, (unsigned long)txp[i].sent);
  lbf("],\"mon\":%u,\"hwf\":{\"why\":%u,\"set\":%u,\"active\":%u,\"pass\":%u,\"need\":%u,\"changes\":%lu,\"pauseUs\":%lu,\"errors\":%lu,"
      "\"f\":[%u,%u,%u,%u]},\"gen\":[",
      monOn, canOk[BUS_A] ? hwfWhy : 6, cfg.hwFilterA, !hwfApplied.acceptAll, hwfWanted.acceptAll ? 2048 : hwfPassKeys, hwfNeedKeys,
      (unsigned long)hwfChanges, (unsigned long)hwfPauseUs, (unsigned long)hwfErrors,
      hwfApplied.code1, hwfApplied.mask1, hwfApplied.code2, hwfApplied.mask2);
  for (uint8_t i = 0; i < cfg.numGen; i++) {
    lbf("%s{\"sent\":%lu,\"nosrc\":%lu,\"m1\":%lu,\"vchk\":%lu,\"vfix\":%lu,\"vlive\":%lu,\"ours\":",
        i ? "," : "", (unsigned long)genSent[i], (unsigned long)genNoSource[i], (unsigned long)m1Seen[i],
        (unsigned long)verChecked[i], (unsigned long)verFixedErr[i], (unsigned long)verLiveDiff[i]);
    lbHex(genLastBuf[i], genSent[i] ? cfg.gens[i].len : 0);
    lbf(",\"m1data\":");
    lbHex(m1LastBuf[i], m1Seen[i] ? m1LastLen[i] : 0);
    lbf("}");
  }
  lbf("]}");
  lbSend();
}

// One line per bus: every ID seen since sniffing started, with the number
// of frames since the previous dump (the tool divides by dt for Hz).
static void sendSniff() {
  const uint32_t now = millis();
  const uint32_t dt = now - sniffSinceMs;
  sniffSinceMs = now;
  sniffPollMs  = now;
  for (uint8_t b = 0; b < 2; b++) {
    lbStart();
    lbStr("{\"t\":\"sniff\",\"b\":"); lbU(b); lbStr(",\"dt\":"); lbU(dt); lbStr(",\"lost\":"); lbU(sniffLost[b]); lbStr(",\"ids\":[");
    bool first = true, more = false;
    for (uint16_t i = 0; i < SNIFF_SLOTS; i++) {
      SniffEntry &e = sniffTab[b][i];
      if (!e.used) continue;
      if (lbRoom() < 80) { more = true; break; }
      if (!first) lbPut(',');
      lbPut('['); lbU(e.id); lbPut(','); lbU(e.ext); lbPut(','); lbU(e.count); lbPut(','); lbU(e.len); lbPut(',');
      lbHex(e.data, e.len);
      lbPut(']');
      e.count = 0;
      first = false;
    }
    lbStr("],\"more\":"); lbU(more); lbPut('}');
    lbSend();
  }
}

// ---------------------------------------------------------------------------
// USB link: commands
// ---------------------------------------------------------------------------
// One command per line (115200 8N1 nominal; USB ignores the rate). Numbers
// may be decimal or 0x hex. Every reply is one JSON object per line with a
// "t" (type) field; replies to commands carry "re" (the command).
//
//   hello                      -> {"t":"hello",...}   firmware, limits, transforms, generated frames
//   get                        -> {"t":"cfg",...}     running settings
//   stat                       -> {"t":"stat",...}    counters (cumulative), CPU load, bus state
//   sniff on | off             -> ok                  start (clears) / stop the ID sniffer
//   sniff                      -> {"t":"sniff","b":0..1,...} x2   (auto-off after 5 s without this)
//   ping                       -> ok                  (flow-control sync during uploads)
//   tx <bus> <id> <ext> <hex>  -> ok                  send one test frame on bus 0 (A) / 1 (B)
//   txp <slot> <bus> <ms> <id> <ext> <hex> -> ok      repeat a test frame (slots 0-3)
//   txp <slot> off | txp off   -> ok                  (all repeats stop 5 s after the tool goes quiet)
//   mon add <where> <ext> <first> <last> -> ok        frame monitor filter; where bits 1 A RX, 2 B RX,
//                                                     4 A TX, 8 B TX; ext 0/1, 2 = either
//   mon on | off | clear       -> ok
//   mon                        -> {"t":"mon","f":[[t_0.1ms,where,id,ext,"hex"],...],"more":0|1}
//
//   begin                      -> ok      start a new settings upload (starts from the running
//                                         settings with no routes, button map rows or generated frames)
//   set <key> <value>          (silent)   keys: baudA baudB swap defAB defBA txMaxAge
//                                               btnStale btnId btnExt hwFilterA
//   route <first> <last> <ext> <dir> <action> <xf> <minMs> <onChange> <hbMs> [note...]
//                              (silent)   append a route
//   btn <src> <dst> <mask> <catchTaps>
//                              (silent)   append a button map row (0-based bytes)
//   gen <periodMs> <id> <ext> <bus> <len> <on> <base hex x16> [name...]
//                              (silent)   append a generated frame
//   gsig <kind 1 source|2 counter> <srcBus> <srcId> <srcExt> <srcStart> <srcLen> <srcOrder>
//        <srcSigned> <dstStart> <dstLen> <dstOrder> <dstSigned> <factor> <offset> <staleMs> <step>
//                              (silent)   append a field to the last gen (order 0 Motorola, 1 Intel)
//   xf [name...]               (silent)   start a custom transform (routes use xf = xfCustom + index)
//   xop <kind> <srcBus 0|1|2=this> <srcId> <srcExt> <srcStart> <srcLen> <srcOrder> <srcSigned>
//       <dstStart> <dstLen> <dstOrder> <dstSigned> <value> <factor> <offset> <staleMs> <cmp> <step> <flags>
//                              (silent)   append a step to the last xf (see GenEngine.h)
//   commit                     -> ok {"restart":0|1} or err   check the upload, then run it (RAM)
//
//   save                       -> ok / err   store the running settings in flash
//   revert                     -> ok         go back to the saved settings (or defaults)
//   defaults                   -> ok         run the factory defaults (RAM only)
//   erase                      -> ok         delete the saved settings
//   reboot                     -> ok, then restart
//
// Errors in silent lines are remembered and reported by commit.
static bool stagingOpen = false;
static char stagingErr[64];

static void stageError(const char *fmt, ...) {
  if (stagingErr[0]) return;            // keep the first one
  va_list ap; va_start(ap, fmt);
  vsnprintf(stagingErr, sizeof(stagingErr), fmt, ap);
  va_end(ap);
}

static char *skipSpaces(char *p) { while (*p == ' ' || *p == '\t') p++; return p; }

static bool nextNum(char *&p, uint32_t &v) {
  p = skipSpaces(p);
  if (!*p) return false;
  char *e;
  v = strtoul(p, &e, 0);
  if (e == p || (*e && *e != ' ' && *e != '\t')) return false;
  p = e;
  return true;
}

// "<id> <ext 0|1> <hex>" -> frame (length = number of hex byte pairs, 0-8)
static bool parseFrame(char *&p, CAN_message_t &m) {
  uint32_t id, ext;
  if (!nextNum(p, id) || !nextNum(p, ext) || ext > 1 || !validId(id, ext)) return false;
  p = skipSpaces(p);
  m = CAN_message_t();
  m.id = id; m.flags.extended = ext; m.len = 0;
  while (*p && *p != ' ') {
    if (m.len >= 8 || !isxdigit((unsigned char)p[0]) || !isxdigit((unsigned char)p[1])) return false;
    char h[3] = { p[0], p[1], 0 };
    m.buf[m.len++] = (uint8_t)strtoul(h, nullptr, 16);
    p += 2;
  }
  return true;
}

// Captured frames since the last dump, as many as fit in one reply line
// ("more":1 = call again). Times are in 0.1 ms.
static void sendMon() {
  monPollMs = millis();
  lbStart();
  lbStr("{\"t\":\"mon\",\"on\":"); lbU(monOn); lbStr(",\"lost\":"); lbU(monLost); lbStr(",\"f\":[");
  monLost = 0;
  bool first = true;
  uint16_t count = 0;
  while (monTail != monHead && count < MON_PER_REPLY && lbRoom() > 80) {
    const MonRec &r = monRing[monTail];
    if (!first) lbPut(',');
    lbPut('['); lbU(r.t); lbPut(','); lbU(r.where); lbPut(','); lbU(r.id); lbPut(','); lbU(r.ext); lbPut(',');
    lbHex(r.data, r.len);
    lbPut(']');
    monTail = (monTail + 1) & (MON_RING - 1);
    first = false;
    count++;
  }
  lbStr("],\"more\":"); lbU(monTail != monHead); lbPut('}');
  lbSend();
}

// ---------------------------------------------------------------------------
// Firmware update over the USB link
// ---------------------------------------------------------------------------
//   ota begin <size>                     -> ok {"part","max"}   start writing the other app slot
//   ota data <offset> <crc32 hex> <b64>  -> ok {"at"}          up to 512 bytes per line, in order
//   ota end <crc32 hex of the whole>     -> ok / err           verify the image, make it the boot slot
//   ota abort                            -> ok
// The isolator keeps routing throughout (flash writes pause the CAN
// interrupts briefly). Nothing changes until 'ota end' has verified the
// whole image; 'reboot' then starts it. No data for 10 s aborts.
static void otaAbort() {
  if (otaActive) esp_ota_abort(otaHandle);
  otaActive = false;
}

static void cmdOta(char *p) {
  p = skipSpaces(p);
  char *sub = p;
  while (*p && *p != ' ') p++;
  if (*p) *p++ = 0;
  if (!strcmp(sub, "begin")) {
    uint32_t size;
    if (!nextNum(p, size) || size < 1024) { replyErr("ota", "use: ota begin <size>"); return; }
    otaAbort();
    otaReady = false;
    otaPart = esp_ota_get_next_update_partition(nullptr);
    if (!otaPart) { replyErr("ota", "no update slot (partition table without ota_1)"); return; }
    if (size > otaPart->size) { replyErr("ota", "image larger than the update slot"); return; }
    if (esp_ota_begin(otaPart, OTA_WITH_SEQUENTIAL_WRITES, &otaHandle) != ESP_OK) { replyErr("ota", "could not start the update"); return; }
    otaActive = true; otaSize = size; otaWritten = 0; otaCrc = 0; otaLastMs = millis();
    lbStart(); lbf("{\"t\":\"ok\",\"re\":\"ota\",\"part\":\"%s\",\"max\":%lu}", otaPart->label, (unsigned long)otaPart->size); lbSend();
  } else if (!strcmp(sub, "data")) {
    uint32_t off;
    if (!otaActive) { replyErr("ota", "no update in progress"); return; }
    if (!nextNum(p, off)) { replyErr("ota", "bad offset"); return; }
    p = skipSpaces(p);
    char *e; uint32_t crc = strtoul(p, &e, 16);
    if (e == p) { replyErr("ota", "bad crc"); return; }
    p = skipSpaces(e);
    size_t n = 0;
    if (mbedtls_base64_decode(otaBuf, sizeof(otaBuf), &n, (const unsigned char *)p, strlen(p)) != 0 || n == 0 || n > 512) { replyErr("ota", "bad data"); return; }
    if (off != otaWritten) {
      lbStart(); lbf("{\"t\":\"err\",\"re\":\"ota\",\"msg\":\"expected offset %lu\",\"at\":%lu}", (unsigned long)otaWritten, (unsigned long)otaWritten); lbSend();
      return;
    }
    if (esp_rom_crc32_le(0, otaBuf, n) != crc) { replyErr("ota", "chunk checksum mismatch"); return; }
    if (otaWritten + n > otaSize) { replyErr("ota", "more data than announced"); return; }
    esp_err_t we = esp_ota_write(otaHandle, otaBuf, n);
    if (we != ESP_OK) {
      otaAbort();
      replyErr("ota", we == ESP_ERR_OTA_VALIDATE_FAILED ? "not a valid ESP32-C6 firmware image" : "flash write failed");
      return;
    }
    otaCrc = esp_rom_crc32_le(otaCrc, otaBuf, n);
    otaWritten += n; otaLastMs = millis();
    lbStart(); lbf("{\"t\":\"ok\",\"re\":\"ota\",\"at\":%lu}", (unsigned long)otaWritten); lbSend();
  } else if (!strcmp(sub, "end")) {
    if (!otaActive) { replyErr("ota", "no update in progress"); return; }
    uint32_t crc = strtoul(skipSpaces(p), nullptr, 16);
    if (otaWritten != otaSize) { otaAbort(); replyErr("ota", "image incomplete"); return; }
    if (crc != otaCrc) { otaAbort(); replyErr("ota", "image checksum mismatch"); return; }
    otaActive = false;
    esp_err_t e = esp_ota_end(otaHandle);            // checks the image (format, chip, SHA-256)
    if (e != ESP_OK) { replyErr("ota", e == ESP_ERR_OTA_VALIDATE_FAILED ? "not a valid ESP32-C6 firmware image" : "could not finish the update"); return; }
    if (esp_ota_set_boot_partition(otaPart) != ESP_OK) { replyErr("ota", "could not select the new firmware"); return; }
    otaReady = true;
    lbStart(); lbf("{\"t\":\"ok\",\"re\":\"ota\",\"ready\":1}"); lbSend();
  } else if (!strcmp(sub, "abort")) {
    otaAbort(); replyOk("ota");
  } else replyErr("ota", "use: ota begin|data|end|abort");
}

// Called every loop pass: idle abort, and confirming a freshly updated image.
static void otaService(uint32_t now) {
  if (otaActive && millis() - otaLastMs > OTA_IDLE_ABORT_MS) otaAbort();   // not 'now': otaLastMs may be newer
  if (fwPendingVerify && now >= FW_CONFIRM_MS && canOk[0] && canOk[1]) {
    esp_ota_mark_app_valid_cancel_rollback();
    fwPendingVerify = false;
  }
}

static void cmdSet(char *p) {
  p = skipSpaces(p);
  char *key = p;
  while (*p && *p != ' ' && *p != '\t') p++;
  if (*p) *p++ = 0;
  uint32_t v;
  if (!nextNum(p, v)) { stageError("set %s: bad value", key); return; }
  if      (!strcmp(key, "baudA"))    staging.baud[BUS_A]   = v;
  else if (!strcmp(key, "baudB"))    staging.baud[BUS_B]   = v;
  else if (!strcmp(key, "swap"))     staging.swapChannels  = v > 255 ? 255 : v;
  else if (!strcmp(key, "defAB"))    staging.defFwd[BUS_A] = v > 255 ? 255 : v;
  else if (!strcmp(key, "defBA"))    staging.defFwd[BUS_B] = v > 255 ? 255 : v;
  else if (!strcmp(key, "txMaxAge")) staging.txMaxAgeMs    = v > 65535 ? 65535 : v;
  else if (!strcmp(key, "btnStale")) staging.buttonStaleMs = v > 65535 ? 65535 : v;
  else if (!strcmp(key, "btnId"))    staging.buttonsId     = v;
  else if (!strcmp(key, "btnExt"))   staging.buttonsExt    = v > 255 ? 255 : v;
  else if (!strcmp(key, "hwFilterA")) staging.hwFilterA    = v > 255 ? 255 : v;
  else stageError("unknown setting %s", key);
}

static void cmdRoute(char *p) {
  uint32_t v[9];
  for (uint8_t i = 0; i < 9; i++) {
    if (!nextNum(p, v[i])) { stageError("route %u: bad field %u", staging.numRoutes + 1, i + 1); return; }
  }
  if (staging.numRoutes >= MAX_ROUTES) { stageError("too many routes (max %u)", MAX_ROUTES); return; }
  for (uint8_t i = 2; i < 9; i++) {
    uint32_t lim = (i == 6 || i == 8) ? 65535 : 255;
    if (v[i] > lim) { stageError("route %u: field %u out of range", staging.numRoutes + 1, i + 1); return; }
  }
  RouteCfg r; memset(&r, 0, sizeof(r));
  r.idFirst = v[0]; r.idLast = v[1]; r.ext = v[2]; r.dir = v[3]; r.action = v[4];
  r.xf = v[5]; r.minMs = v[6]; r.onChange = v[7]; r.hbMs = v[8];
  sanitizeNote(r.note, skipSpaces(p));
  memcpy(&staging.routes[staging.numRoutes++], &r, sizeof(r));
}

static void cmdBtn(char *p) {
  uint32_t v[4];
  for (uint8_t i = 0; i < 4; i++) {
    if (!nextNum(p, v[i]) || v[i] > 255) { stageError("button map row %u: bad field %u", staging.numButtonMap + 1, i + 1); return; }
  }
  if (staging.numButtonMap >= MAX_BTN) { stageError("too many button map rows (max %u)", MAX_BTN); return; }
  ButtonMapCfg &m = staging.buttonMap[staging.numButtonMap++];
  m.src = v[0]; m.dst = v[1]; m.mask = v[2]; m.catchTaps = v[3];
}

// gen <periodMs> <id> <ext> <bus> <len> <on> <base hex, 16 digits> [name...]
static void cmdGen(char *p) {
  uint32_t v[6];
  for (uint8_t i = 0; i < 6; i++)
    if (!nextNum(p, v[i])) { stageError("generated frame %u: bad field %u", staging.numGen + 1, i + 1); return; }
  if (staging.numGen >= MAX_GEN) { stageError("too many generated frames (max %u)", MAX_GEN); return; }
  if (v[0] > 65535 || v[2] > 255 || v[3] > 255 || v[4] > 255 || v[5] > 255) { stageError("generated frame %u: value out of range", staging.numGen + 1); return; }
  GenCfg g; memset(&g, 0, sizeof(g));
  g.periodMs = v[0]; g.id = v[1]; g.ext = v[2]; g.bus = v[3]; g.len = v[4]; g.enabled = v[5];
  p = skipSpaces(p);
  for (uint8_t b = 0; b < 8; b++) {
    if (!isxdigit((unsigned char)p[0]) || !isxdigit((unsigned char)p[1])) { stageError("generated frame %u: base must be 16 hex digits", staging.numGen + 1); return; }
    char h[3] = { p[0], p[1], 0 };
    g.base[b] = (uint8_t)strtoul(h, nullptr, 16);
    p += 2;
  }
  sanitizeNote(g.name, skipSpaces(p));
  memcpy(&staging.gens[staging.numGen++], &g, sizeof(g));
}

// gsig <kind> <srcBus> <srcId> <srcExt> <srcStart> <srcLen> <srcOrder> <srcSigned>
//      <dstStart> <dstLen> <dstOrder> <dstSigned> <factor> <offset> <staleMs> <step>
// adds a field to the most recent 'gen'.
static void cmdGsig(char *p) {
  if (!staging.numGen) { stageError("gsig before any gen"); return; }
  GenCfg &g = staging.gens[staging.numGen - 1];
  if (g.numSig >= MAX_GEN_SIG) { stageError("generated frame %u: too many fields (max %u)", staging.numGen, MAX_GEN_SIG); return; }
  uint32_t v[12];
  for (uint8_t i = 0; i < 12; i++)
    if (!nextNum(p, v[i]) || (i != 2 && v[i] > 255)) { stageError("generated frame %u field %u: bad value %u", staging.numGen, g.numSig + 1, i + 1); return; }
  float fo[2];
  for (uint8_t i = 0; i < 2; i++) {
    p = skipSpaces(p);
    char *e;
    fo[i] = strtof(p, &e);
    if (e == p || (*e && *e != ' ' && *e != '\t')) { stageError("generated frame %u field %u: bad factor / offset", staging.numGen, g.numSig + 1); return; }
    p = e;
  }
  uint32_t stale, step;
  if (!nextNum(p, stale) || stale > 65535 || !nextNum(p, step) || step > 255) { stageError("generated frame %u field %u: bad stale / step", staging.numGen, g.numSig + 1); return; }
  GenSigCfg f; memset(&f, 0, sizeof(f));
  f.kind = v[0]; f.srcBus = v[1]; f.srcId = v[2]; f.srcExt = v[3]; f.srcStart = v[4]; f.srcLen = v[5];
  f.srcOrder = v[6]; f.srcSigned = v[7]; f.dstStart = v[8]; f.dstLen = v[9]; f.dstOrder = v[10]; f.dstSigned = v[11];
  f.factor = fo[0]; f.offset = fo[1]; f.staleMs = stale; f.step = step;
  memcpy(&g.sig[g.numSig++], &f, sizeof(f));
}

// xf [name...]   start a new custom transform
static void cmdXf(char *p) {
  if (staging.numXf >= MAX_XF) { stageError("too many transforms (max %u)", MAX_XF); return; }
  XfCfg &x = staging.xforms[staging.numXf++];
  memset(&x, 0, sizeof(x));
  sanitizeNote(x.name, skipSpaces(p));
}

// xop <kind> <srcBus> <srcId> <srcExt> <srcStart> <srcLen> <srcOrder> <srcSigned> <dstStart> <dstLen>
//     <dstOrder> <dstSigned> <value> <factor> <offset> <staleMs> <cmp> <step> <flags>
// adds a step to the last 'xf'. value is signed; factor/offset are decimals.
static void cmdXop(char *p) {
  if (!staging.numXf) { stageError("xop before any xf"); return; }
  XfCfg &x = staging.xforms[staging.numXf - 1];
  if (x.numOps >= MAX_XF_OPS) { stageError("transform %u: too many steps (max %u)", staging.numXf, MAX_XF_OPS); return; }
  double v[19];
  for (uint8_t i = 0; i < 19; i++) {
    p = skipSpaces(p);
    char *e;
    v[i] = strtod(p, &e);
    if (e == p || (*e && *e != ' ' && *e != '\t')) { stageError("transform %u step %u: bad value %u", staging.numXf, x.numOps + 1, i + 1); return; }
    p = e;
    bool isFloat = (i == 13 || i == 14), isSigned = (i == 12), isId = (i == 2);
    if (!isFloat && v[i] != (double)(int64_t)v[i]) { stageError("transform %u step %u: value %u must be whole", staging.numXf, x.numOps + 1, i + 1); return; }
    double lo = isSigned ? -2147483648.0 : 0, hi = isSigned ? 2147483647.0 : isId ? 4294967295.0 : (i == 15 ? 65535 : 255);
    if (!isFloat && (v[i] < lo || v[i] > hi)) { stageError("transform %u step %u: value %u out of range", staging.numXf, x.numOps + 1, i + 1); return; }
  }
  XfOpCfg o; memset(&o, 0, sizeof(o));
  o.kind = v[0]; o.srcBus = v[1]; o.srcId = (uint32_t)v[2]; o.srcExt = v[3]; o.srcStart = v[4]; o.srcLen = v[5];
  o.srcOrder = v[6]; o.srcSigned = v[7]; o.dstStart = v[8]; o.dstLen = v[9]; o.dstOrder = v[10]; o.dstSigned = v[11];
  o.value = (int32_t)v[12]; o.factor = (float)v[13]; o.offset = (float)v[14]; o.staleMs = v[15];
  o.cmp = v[16]; o.step = v[17]; o.flags = v[18];
  memcpy(&x.ops[x.numOps++], &o, sizeof(o));
}

static void runConfig(CfgSource src) {
  normalizeConfig(cfg);
  cfgSource = src;
  applyConfig();
}

static void handleCommand(char *line) {
  char *p = skipSpaces(line);
  if (!*p) return;
  char *cmd = p;
  while (*p && *p != ' ' && *p != '\t') p++;
  if (*p) *p++ = 0;

  if      (!strcmp(cmd, "set"))   { if (stagingOpen) cmdSet(p);   return; }
  else if (!strcmp(cmd, "route")) { if (stagingOpen) cmdRoute(p); return; }
  else if (!strcmp(cmd, "btn"))   { if (stagingOpen) cmdBtn(p);   return; }
  else if (!strcmp(cmd, "gen"))   { if (stagingOpen) cmdGen(p);   return; }
  else if (!strcmp(cmd, "gsig"))  { if (stagingOpen) cmdGsig(p);  return; }
  else if (!strcmp(cmd, "xf"))    { if (stagingOpen) cmdXf(p);    return; }
  else if (!strcmp(cmd, "xop"))   { if (stagingOpen) cmdXop(p);   return; }
  else if (!strcmp(cmd, "hello")) sendHello();
  else if (!strcmp(cmd, "get"))   sendConfig();
  else if (!strcmp(cmd, "stat"))  sendStats();
  else if (!strcmp(cmd, "ping"))  replyOk("ping");
  else if (!strcmp(cmd, "ota"))   cmdOta(p);
  else if (!strcmp(cmd, "tx")) {
    uint32_t bus; CAN_message_t m;
    if (!nextNum(p, bus) || bus > 1 || !parseFrame(p, m)) {
      replyErr("tx", "use: tx <bus 0|1> <id> <ext 0|1> <hex data, up to 8 bytes>"); return;
    }
    if (queueTx(bus, m)) { testSent[bus]++; replyOk("tx"); } else replyErr("tx", "TX queue full");
  }
  else if (!strcmp(cmd, "txp")) {
    // txp off | txp <slot> off | txp <slot> <bus> <periodMs> <id> <ext> <hex>
    p = skipSpaces(p);
    if (!strcmp(p, "off")) { for (uint8_t i = 0; i < TXP_SLOTS; i++) txp[i].on = false; replyOk("txp"); return; }
    uint32_t slot, bus, period; CAN_message_t m;
    if (!nextNum(p, slot) || slot >= TXP_SLOTS) { replyErr("txp", "bad slot"); return; }
    if (!strcmp(skipSpaces(p), "off")) { txp[slot].on = false; replyOk("txp"); return; }
    if (!nextNum(p, bus) || bus > 1 || !nextNum(p, period) || period < 1 || period > 60000 || !parseFrame(p, m)) {
      replyErr("txp", "use: txp <slot> <bus> <period 1-60000 ms> <id> <ext> <hex> | txp <slot> off | txp off"); return;
    }
    TxPeriodic &t = txp[slot];
    t.bus = bus; t.periodMs = period; t.m = m; t.sent = 0; t.dueMs = millis(); t.on = true;
    replyOk("txp");
  }
  else if (!strcmp(cmd, "mon")) {
    // mon on | off | clear | add <where 1-15> <ext 0|1|2> <first> <last> | (empty: dump)
    p = skipSpaces(p);
    if (!strcmp(p, "on"))         { monHead = monTail = 0; monLost = 0; monOn = true; monPollMs = millis(); replyOk("mon"); }
    else if (!strcmp(p, "off"))   { monOn = false; replyOk("mon"); }
    else if (!strcmp(p, "clear")) { monN = 0; monHead = monTail = 0; monLost = 0; replyOk("mon"); }
    else if (!strncmp(p, "add", 3)) {
      p += 3;
      uint32_t where, ext, first, last;
      if (!nextNum(p, where) || !nextNum(p, ext) || !nextNum(p, first) || !nextNum(p, last) ||
          where < 1 || where > 15 || ext > 2 || last < first) { replyErr("mon", "use: mon add <where 1-15> <ext 0|1|2> <first> <last>"); return; }
      if (monN >= MON_MAX_FILTERS) { replyErr("mon", "too many filters"); return; }
      monF[monN++] = { first, last, (uint8_t)ext, (uint8_t)where };
      replyOk("mon");
    }
    else if (!*p) sendMon();
    else replyErr("mon", "use: mon on | off | clear | add ...");
  }
  else if (!strcmp(cmd, "sniff")) {
    p = skipSpaces(p);
    if (!strcmp(p, "on"))       { sniffClear(); sniffOn = true; sniffSinceMs = sniffPollMs = millis(); replyOk("sniff"); }
    else if (!strcmp(p, "off")) { sniffOn = false; replyOk("sniff"); }
    else if (!*p)               { if (!sniffOn) { sniffClear(); sniffOn = true; sniffSinceMs = millis(); } sendSniff(); }
    else replyErr("sniff", "use: sniff on | off");
  }
  else if (!strcmp(cmd, "begin")) {
    staging = cfg;
    staging.numRoutes = 0;
    staging.numButtonMap = 0;
    staging.numGen = 0;
    staging.numXf = 0;
    stagingErr[0] = 0;
    stagingOpen = true;
    replyOk("begin");
  }
  else if (!strcmp(cmd, "commit")) {
    if (!stagingOpen) { replyErr("commit", "no upload in progress (send begin first)"); return; }
    stagingOpen = false;
    char err[64];
    if (stagingErr[0]) { replyErr("commit", stagingErr); return; }
    if (!validateConfig(staging, err, sizeof(err))) { replyErr("commit", err); return; }
    cfg = staging;
    runConfig(cfgSource);
    lbStart(); lbf("{\"t\":\"ok\",\"re\":\"commit\",\"restart\":%u}", needsRestart()); lbSend();
  }
  else if (!strcmp(cmd, "save")) {
    if (saveConfig()) { lbStart(); lbf("{\"t\":\"ok\",\"re\":\"save\",\"restart\":%u}", needsRestart()); lbSend(); }
    else replyErr("save", "flash write failed");
  }
  else if (!strcmp(cmd, "revert")) {
    bool existed;
    if (loadSaved(staging, existed)) { cfg = staging; runConfig(SRC_SAVED); }
    else { loadDefaults(cfg); runConfig(existed ? SRC_SAVED_BAD : SRC_DEFAULTS); }
    replyOk("revert");
  }
  else if (!strcmp(cmd, "defaults")) { loadDefaults(cfg); runConfig(SRC_DEFAULTS); replyOk("defaults"); }
  else if (!strcmp(cmd, "erase"))    { if (eraseSaved()) replyOk("erase"); else replyErr("erase", "flash erase failed"); }
  else if (!strcmp(cmd, "reboot")) {
    replyOk("reboot");
    uint32_t t0 = millis();
    while (outHead != outTail && millis() - t0 < 200) { outPump(); delay(1); }
    delay(50);
    esp_restart();
  }
  else replyErr(cmd, "unknown command");
}

static char    inLine[1024];             // fits an 'ota data' line (512 bytes as base64)
static uint16_t inLen = 0;
static bool    inTooLong = false;

// Reads and runs tool commands. Stops after HOST_BUDGET_US of command work
// in one loop pass, so CAN routing always runs between two slow commands.
static const uint32_t HOST_BUDGET_US = 300;

static void pollHost() {
  const uint32_t budgetT0 = micros();
  for (uint16_t n = 0; n < 1024 && Serial.available() > 0; n++) {
    int c = Serial.read();
    if (c < 0) break;
    if (c == '\r') continue;
    if (c == '\n') {
      if (inTooLong) { if (stagingOpen) stageError("line too long"); else replyErr("?", "line too long"); }
      else {
        inLine[inLen] = 0; lastHostMs = millis();
        char name[12]; uint8_t k = 0;
        while (k < sizeof(name) - 1 && inLine[k] && inLine[k] != ' ') { name[k] = inLine[k]; k++; }
        name[k] = 0;
        uint32_t t0 = micros();
        handleCommand(inLine);
        uint32_t d = micros() - t0;
        if (d > diagCmdUs) { diagCmdUs = d; memcpy(diagCmd, name, sizeof(diagCmd)); }
        if (diagResetPending) { diagCmdUs = 0; diagCmd[0] = 0; }
      }
      inLen = 0; inTooLong = false;
      if (micros() - budgetT0 > HOST_BUDGET_US) break;   // the rest waits for the next pass
      continue;
    } else if (inLen < sizeof(inLine) - 1) {
      inLine[inLen++] = (char)c;
    } else {
      inTooLong = true;
    }
  }
  if (sniffOn && millis() - sniffPollMs > SNIFF_AUTO_OFF_MS) sniffOn = false;
  if (monOn && millis() - monPollMs > MON_AUTO_OFF_MS) monOn = false;
}

// ---------------------------------------------------------------------------
// HARDWARE FILTER (Bus A)
// ---------------------------------------------------------------------------
// The C6 TWAI controller has one acceptance filter, used here in dual mode:
// two (code, mask) pairs over the 11 ID bits of standard frames. Measured on
// the bench: in this mode ALL 29-bit frames are rejected (even when their top
// bits match). So the filter is only used when nothing needed from Bus A is
// 29-bit; then unneeded 29-bit traffic is rejected too. The needed 11-bit IDs
// are split into the two groups that let the fewest other IDs through. Software still
// routes every accepted frame, so a filter that passes a few extra IDs is
// harmless; it never rejects an ID that a route, the button merge or a
// generated frame needs (checked after computing).
//
// Needed from Bus A: IDs of A->B PASS routes, the steering wheel frame, and
// generated-frame sources on Bus A. No filter (accept all) when the setting
// is off, when unrouted A->B IDs are forwarded, when a needed ID is 29-bit,
// or while the tool's sniffer or monitor captures Bus A RX. Changing the filter stops the controller for
// a moment (measured: hwf.pauseUs), so it only changes on those events.
// (state: see "Hardware filter state" near the frame monitor)

static uint16_t keyGroup(const uint16_t *k, uint16_t n, uint32_t sel, bool inSel, uint16_t &code, uint16_t &mask, uint32_t &cost) {
  uint16_t a = 0x7FF, o = 0, cnt = 0;
  for (uint16_t i = 0; i < n; i++) {
    bool in = (sel >> i) & 1;
    if (in != inSel) continue;
    a &= k[i]; o |= k[i]; cnt++;
  }
  uint16_t vary = (a ^ o) & 0x7FF;
  mask = ~vary & 0x7FF;
  code = a & mask;
  cost = cnt ? (1u << __builtin_popcount(vary)) : 0;
  return cnt;
}

static void computeHwFilter() {
  hwfWanted = { true, 0, 0, 0, 0 };
  hwfPassKeys = 2048;
  hwfNeedKeys = 0;
  if (!cfg.hwFilterA) { hwfWhy = 1; return; }
  if (cfg.defFwd[BUS_A]) { hwfWhy = 2; return; }
  for (uint16_t i = 0; i < cfg.numRoutes; i++) {
    const RouteCfg &r = cfg.routes[i];
    if ((r.dir & A_TO_B) && r.action == PASS && r.ext) { hwfWhy = 3; return; }
  }
  for (uint8_t i = 0; i < numCached; i++)
    if (cacheIds[i].bus == BUS_A && cacheIds[i].extended) { hwfWhy = 3; return; }

  static uint8_t need[256];
  memset(need, 0, sizeof(need));
  auto mark = [](uint32_t key) { need[key >> 3] |= 1u << (key & 7); };
  for (uint16_t i = 0; i < cfg.numRoutes; i++) {
    const RouteCfg &r = cfg.routes[i];
    if (!(r.dir & A_TO_B) || r.action != PASS) continue;
    uint32_t last = r.idLast ? r.idLast : r.idFirst;
    for (uint32_t id = r.idFirst; id <= last; id++) mark(id);
  }
  for (uint8_t i = 0; i < numCached; i++)
    if (cacheIds[i].bus == BUS_A) mark(cacheIds[i].id);

  static uint16_t keys[2048];
  uint16_t n = 0;
  for (uint16_t k = 0; k < 2048; k++) if (need[k >> 3] & (1u << (k & 7))) keys[n++] = k;
  hwfNeedKeys = n;

  HwFilter f = { false, 0x7FF, 0x7FF, 0x7FF, 0x7FF };   // n == 0: nothing needed (only key 7FF passes)
  if (n) {
    uint32_t best = UINT32_MAX, c1, c2;
    uint16_t a1, m1, a2, m2;
    auto consider = [&](uint32_t sel) {
      keyGroup(keys, n, sel, true, a1, m1, c1);
      uint16_t cntB = keyGroup(keys, n, sel, false, a2, m2, c2);
      if (!cntB) { a2 = a1; m2 = m1; }                    // one group: both filters the same
      if (c1 + c2 < best) { best = c1 + c2; f = { false, a1, m1, a2, m2 }; }
    };
    consider(0xFFFFFFFF);                                  // everything in one group
    if (n <= 14) {
      for (uint32_t sel = 1; sel < (1u << (n - 1)); sel++) consider(sel);   // every split
    } else {
      for (uint8_t b = 0; b < 11; b++) {                   // split on one ID bit
        // sel only covers 32 keys, so split by bit directly
        uint16_t ga[2] = { 0x7FF, 0x7FF }, go[2] = { 0, 0 }, gc[2] = { 0, 0 };
        for (uint16_t i = 0; i < n; i++) { uint8_t g = (keys[i] >> b) & 1; ga[g] &= keys[i]; go[g] |= keys[i]; gc[g]++; }
        if (!gc[0] || !gc[1]) continue;
        uint16_t v0 = (ga[0] ^ go[0]) & 0x7FF, v1 = (ga[1] ^ go[1]) & 0x7FF;
        uint32_t cost = (1u << __builtin_popcount(v0)) + (1u << __builtin_popcount(v1));
        if (cost < best) {
          best = cost;
          uint16_t mm0 = ~v0 & 0x7FF, mm1 = ~v1 & 0x7FF;
          f = { false, (uint16_t)(ga[0] & mm0), mm0, (uint16_t)(ga[1] & mm1), mm1 };
        }
      }
    }
  }
  // Safety check: every needed key must pass, otherwise don't filter.
  for (uint16_t i = 0; i < n; i++) {
    uint16_t k = keys[i];
    if ((k & f.mask1) != f.code1 && (k & f.mask2) != f.code2) { hwfErrors++; hwfWhy = 5; return; }
  }
  uint16_t pass = 0;
  for (uint16_t k = 0; k < 2048; k++) if ((k & f.mask1) == f.code1 || (k & f.mask2) == f.code2) pass++;
  hwfWanted = f;
  hwfPassKeys = pass;
  hwfWhy = 0;
}

static inline bool sameFilter(const HwFilter &a, const HwFilter &b) {
  if (a.acceptAll || b.acceptAll) return a.acceptAll == b.acceptAll;
  return a.code1 == b.code1 && a.mask1 == b.mask1 && a.code2 == b.code2 && a.mask2 == b.mask2;
}

static bool monCapturesARx() {
  if (!monOn) return false;
  for (uint8_t i = 0; i < monN; i++) if (monF[i].where & 1) return true;
  return false;
}

// Called every loop pass; touches the controller only when the wanted state changes.
static void updateHwFilter() {
  if (!canOk[BUS_A] || busState[BUS_A] == TWAI_ERROR_BUS_OFF) return;
  static const HwFilter OPEN = { true, 0, 0, 0, 0 };
  if (!hwfWanted.acceptAll) hwfWhy = (sniffOn || monCapturesARx()) ? 4 : 0;
  const HwFilter &want = (sniffOn || monCapturesARx()) ? OPEN : hwfWanted;
  if (sameFilter(want, hwfApplied)) return;

  twai_mask_filter_config_t mf = {};
  if (want.acceptAll) { mf.id = 0; mf.mask = 0; mf.is_ext = false; }     // mask 0 = every bit "any"
  else mf = twai_make_dual_filter(want.code1, want.mask1, want.code2, want.mask2, false);
  const uint32_t t0 = micros();
  bool ok = twai_node_disable(canNode[BUS_A]) == ESP_OK;
  ok = ok && twai_node_config_mask_filter(canNode[BUS_A], 0, &mf) == ESP_OK;
  bool en = twai_node_enable(canNode[BUS_A]) == ESP_OK;
  hwfPauseUs = micros() - t0;
  txBusy[BUS_A] = false;                     // a frame in flight was abandoned by the stop
  if (ok && en) { hwfApplied = want; hwfChanges++; }
  else { hwfErrors++; if (!en) canOk[BUS_A] = twai_node_enable(canNode[BUS_A]) == ESP_OK; hwfApplied = OPEN; }
}

// ---------------------------------------------------------------------------
// Factory reset: BOOT held for 3 s while running
// ---------------------------------------------------------------------------
static void checkFactoryReset(uint32_t now) {
  static uint32_t pressedSince = 0;
  static bool     pressed = false;
  if (digitalRead(PIN_BOOT) == LOW) {
    if (!pressed) { pressed = true; pressedSince = now; }
    else if (now - pressedSince >= FACTORY_RESET_HOLD_MS) {
      ledOverride = true;
      for (uint8_t i = 0; i < 6; i++) {             // flash white
        rgbLedWrite(PIN_LED, (i & 1) ? 0 : 0x30, (i & 1) ? 0 : 0x30, (i & 1) ? 0 : 0x30);
        uint32_t t0 = millis();
        while (millis() - t0 < 150) { watchdogFeed(); delay(5); }
      }
      eraseSaved();
      esp_restart();
    }
  } else {
    pressed = false;
  }
}

// The Arduino core would confirm a freshly updated image as soon as it boots;
// take that over so the image must first run properly (otaService()).
extern "C" bool verifyRollbackLater() { return true; }

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
  // CAN pins first, for BOTH channels, before any transceiver or controller
  // is enabled. GPIO16/17 (CAN0) are UART0's default pins and stay attached
  // to the UART after boot; the TWAI driver only routes signals through the
  // GPIO matrix, so make them plain GPIOs here. TX idles HIGH = recessive, so
  // neither transceiver can glitch its bus while the other channel starts.
  for (uint8_t ch = 0; ch < 2; ch++) {
    pinMode(CH_RX[ch], INPUT);
    digitalWrite(CH_TX[ch], HIGH);
    pinMode(CH_TX[ch], OUTPUT);
  }

  // Wake the transceivers: RS and shutdown LOW = normal mode.
  for (uint8_t ch = 0; ch < 2; ch++) {
    pinMode(CH_RS[ch], OUTPUT);
    digitalWrite(CH_RS[ch], LOW);
    pinMode(CH_SHDN[ch], OUTPUT);
    digitalWrite(CH_SHDN[ch], LOW);
  }
  pinMode(PIN_BOOT, INPUT_PULLUP);
  {
    esp_ota_img_states_t st;
    const esp_partition_t *running = esp_ota_get_running_partition();
    fwPendingVerify = running && esp_ota_get_state_partition(running, &st) == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY;
  }

  // Settings: saved ones if they pass every check, otherwise the defaults.
  loadDefaults(staging);
  defaultsCrc = cfgCrc(staging);
  cfgPartInit();
  bool existed;
  bool fromLegacy = false;
  if (cfgPartOk && !loadSaved(staging, existed) && !existed) {
    // Nothing in isocfg yet: settings saved by firmware before 2.4 are in the
    // default area. Use them, and move them over once below.
    bool legacyExisted;
    if (loadSaved(staging, legacyExisted, true)) fromLegacy = true;
    existed = legacyExisted;
  }
  if (fromLegacy || loadSaved(staging, existed)) {
    cfg = staging;
    normalizeConfig(cfg);
    haveSaved = true;
    savedCrc  = cfgCrc(cfg);
    cfgSource = SRC_SAVED;
    if (fromLegacy && saveConfig()) {           // move to isocfg, then clear the old copy
      Preferences old;
      if (old.begin(PREFS_NS, false)) { old.clear(); old.end(); }
    }
  } else {
    loadDefaults(cfg);
    cfgSource = existed ? SRC_SAVED_BAD : SRC_DEFAULTS;
  }

  hwSwap = cfg.swapChannels;
  busChannel[BUS_A] = hwSwap ? 1 : 0;
  busChannel[BUS_B] = hwSwap ? 0 : 1;
  hwBaud[BUS_A] = cfg.baud[BUS_A];
  hwBaud[BUS_B] = cfg.baud[BUS_B];

  applyConfig();                                  // before the buses start delivering frames
  canOk[BUS_A] = initBus(BUS_A, hwBaud[BUS_A]);
  canOk[BUS_B] = initBus(BUS_B, hwBaud[BUS_B]);

  // USB link. Larger buffers so a settings upload never overflows; writes
  // never block (see outPump).
  Serial.setRxBufferSize(4096);
  Serial.setTxBufferSize(2048);
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);

  xTaskCreate(canTask, "can", 8192, nullptr, CAN_TASK_PRIORITY, &loopTaskHandle);
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
// Returns the number of frames routed.
static uint32_t routeFrames(uint32_t now) {
  CAN_message_t m;
  uint32_t frames = 0;
  // Alternate between the buses so a flood on one side can't starve the other.
  for (uint16_t n = 0; n < 64; n++) {
    uint32_t f0 = esp_cpu_get_cycle_count();
    bool gotA = rxRing[BUS_A].pop(m); if (gotA) handleFrame(BUS_A, m, now);
    bool gotB = rxRing[BUS_B].pop(m); if (gotB) handleFrame(BUS_B, m, now);
    if (!gotA && !gotB) break;
    uint8_t k = (uint8_t)gotA + (uint8_t)gotB;
    frames += k;
    uint32_t per = (esp_cpu_get_cycle_count() - f0) / k;
    if (per > profMaxFrameCyc) profMaxFrameCyc = per;
  }
  return frames;
}

// Times one step of the loop pass for the timing health numbers.
static uint32_t stepT0, passT0, stepWorst; static uint8_t stepWorstId;
static inline void stepDone(uint8_t id) {
  uint32_t t = micros(), d = t - stepT0;
  if (d > stepWorst) { stepWorst = d; stepWorstId = id; }
  stepT0 = t;
}

// The CAN work runs in its own task, above Arduino's loop task: on single-core
// chips the Arduino loop task sleeps 5 ms every 2 s by design (yieldIfNecessary),
// which would hold up forwarding. This task still sleeps whenever it has
// nothing to do, so lower-priority tasks get their time.
static void canLoopOnce();
static void canTask(void *) {
  watchdogStart();                 // watch this task (not Arduino's loop task)
  for (;;) canLoopOnce();
}

void loop() {
  vTaskDelay(portMAX_DELAY);       // nothing to do here: see canTask()
}

static void canLoopOnce() {
  watchdogFeed();
  const uint32_t now = millis();
  const uint32_t passStart = esp_cpu_get_cycle_count();
  passT0 = stepT0 = micros(); stepWorst = 0;

  uint32_t frames = routeFrames(now);  stepDone(0);
  flushPending(now);                   stepDone(1);
  runGenerators(now);                  stepDone(2);
  runTestFrames(now);                  stepDone(3);
  drainTx(BUS_A);                      stepDone(4);
  drainTx(BUS_B);                      stepDone(5);
  checkBusHealth();                    stepDone(6);
  updateLed(now);                      stepDone(7);

  // Count the whole pass (routing + TX + health) only when it did real
  // work. Idle time isn't load. ISR time isn't included.
  if (frames) profBusyCycles += esp_cpu_get_cycle_count() - passStart;

  pollHost();                          stepDone(8);
  outPump();                           stepDone(9);
  updateHwFilter();                    stepDone(10);
  otaService(now);                     stepDone(11);
  checkFactoryReset(now);              stepDone(12);
  uint32_t pass = micros() - passT0;
  if (diagResetPending) { diagResetPending = false; diagPassUs = diagStepUs = diagRxWaitUs = diagTxWaitMs = 0; pass = 0; }
  if (pass > diagPassUs) { diagPassUs = pass; diagStep = stepWorstId; diagStepUs = stepWorst; }

  // Nothing received this pass: sleep until a CAN interrupt (RX, TX done,
  // state change) or the next tick (1 ms) for timers, rate limits and USB.
  if (!frames && rxRing[BUS_A].empty() && rxRing[BUS_B].empty()) ulTaskNotifyTake(pdTRUE, 1);
}
