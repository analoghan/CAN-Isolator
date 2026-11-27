// GenEngine.h - data-driven generated frames for CanIsolatorC6.
//
// A generated frame is: ID, bus, period, length, 8 constant bytes, and up to
// MAX_GEN_SIG signal fields written over the constant bytes:
//
//   GS_SOURCE  : read a field from a cached source frame, then
//                out = round(raw * factor + offset), clamped to the field.
//   GS_COUNTER : a rolling counter, +step every frame sent, wraps at the
//                field size.
//
// Bit numbering follows DBC files: position = byte * 8 + bit, bit 0 = least
// significant bit of the byte.
//   Intel    (little-endian, DBC @1): start = least significant bit; the
//            field continues upward (into the next byte's bit 0).
//   Motorola (big-endian,    DBC @0): start = most significant bit; the
//            field continues downward, then into the NEXT byte's bit 7.
//
// Plain C++ with no Arduino dependencies, so it can be unit-tested on a PC.
#pragma once
#include <stdint.h>
#include <string.h>
#include <math.h>

static const uint8_t MAX_GEN     = 8;    // generated frames
static const uint8_t MAX_GEN_SIG = 4;    // fields per generated frame
static const uint8_t GEN_NAME_LEN = 24;  // including the terminating 0

enum GenSigKind  : uint8_t { GS_NONE = 0, GS_SOURCE = 1, GS_COUNTER = 2 };
enum GenBitOrder : uint8_t { BO_MOTOROLA = 0, BO_INTEL = 1 };   // DBC @0 / @1

struct GenSigCfg {
  uint32_t srcId;       // GS_SOURCE: source frame
  float    factor;      // GS_SOURCE: out = raw * factor + offset
  float    offset;
  uint16_t staleMs;     // GS_SOURCE: source older than this = skip the frame (0 = any age)
  uint8_t  kind;        // GenSigKind
  uint8_t  srcBus;      // 0 = Bus A, 1 = Bus B
  uint8_t  srcExt;
  uint8_t  srcStart, srcLen, srcOrder, srcSigned;
  uint8_t  dstStart, dstLen, dstOrder, dstSigned;
  uint8_t  step;        // GS_COUNTER: increment per frame
};

struct GenCfg {
  uint32_t  id;
  uint16_t  periodMs;
  uint8_t   ext;
  uint8_t   bus;        // bus the frame is SENT on
  uint8_t   len;        // 0-8 data bytes
  uint8_t   enabled;
  uint8_t   numSig;
  uint8_t   base[8];    // constant bytes (fields are written over them)
  char      name[GEN_NAME_LEN];
  GenSigCfg sig[MAX_GEN_SIG];
};

// Next bit position of a field, walking from its start.
static inline int genNextPos(int pos, uint8_t order) {
  if (order == BO_INTEL) return pos + 1;
  return (pos % 8 == 0) ? pos + 15 : pos - 1;
}

// True if every bit of the field lies inside the first 'bytes' bytes.
static inline bool genFieldFits(uint8_t start, uint8_t len, uint8_t order, uint8_t bytes) {
  if (len < 1 || len > 32 || start > 63 || order > BO_INTEL) return false;
  int pos = start;
  for (uint8_t i = 0; i < len; i++) {
    if (pos < 0 || pos >= bytes * 8) return false;
    pos = genNextPos(pos, order);
  }
  return true;
}

// Read a field as an unsigned number (caller checked it fits).
static inline uint32_t genGet(const uint8_t *b, uint8_t start, uint8_t len, uint8_t order) {
  uint32_t v = 0;
  int pos = start;
  if (order == BO_INTEL) {
    for (uint8_t i = 0; i < len; i++, pos++) v |= (uint32_t)((b[pos >> 3] >> (pos & 7)) & 1) << i;
  } else {
    for (uint8_t i = 0; i < len; i++) { v = (v << 1) | ((b[pos >> 3] >> (pos & 7)) & 1); pos = genNextPos(pos, order); }
  }
  return v;
}

// Write the low 'len' bits of v into a field; other bits are untouched.
static inline void genPut(uint8_t *b, uint8_t start, uint8_t len, uint8_t order, uint32_t v) {
  int pos = start;
  if (order == BO_INTEL) {
    for (uint8_t i = 0; i < len; i++, pos++) {
      uint8_t m = 1u << (pos & 7);
      if ((v >> i) & 1) b[pos >> 3] |= m; else b[pos >> 3] &= ~m;
    }
  } else {
    for (int i = len - 1; i >= 0; i--) {
      uint8_t m = 1u << (pos & 7);
      if ((v >> i) & 1) b[pos >> 3] |= m; else b[pos >> 3] &= ~m;
      pos = genNextPos(pos, order);
    }
  }
}

// Mark the field's bits in a mask (used to find the constant bits).
static inline void genMark(uint8_t *mask, uint8_t start, uint8_t len, uint8_t order) {
  int pos = start;
  for (uint8_t i = 0; i < len; i++) { mask[pos >> 3] |= 1u << (pos & 7); pos = genNextPos(pos, order); }
}

// Source raw -> destination raw: sign-extend, scale in float, round half
// up, clamp to the destination field's range.
static inline uint32_t genScale(uint32_t raw, const GenSigCfg &s) {
  float x;
  if (s.srcSigned && s.srcLen < 32 && ((raw >> (s.srcLen - 1)) & 1)) x = (float)((int64_t)raw - ((int64_t)1 << s.srcLen));
  else if (s.srcSigned && s.srcLen == 32) x = (float)(int32_t)raw;
  else x = (float)raw;
  float v = x * s.factor + s.offset;
  int64_t lo, hi;
  if (s.dstSigned) { lo = -((int64_t)1 << (s.dstLen - 1)); hi = ((int64_t)1 << (s.dstLen - 1)) - 1; }
  else             { lo = 0;                                hi = ((int64_t)1 << s.dstLen) - 1; }
  if (!(v == v)) v = 0.0f;                                  // NaN
  if (v <= (float)lo) return (uint32_t)lo;
  if (v >= (float)hi) return (uint32_t)hi;
  int64_t r = (v > 0.0f) ? (int64_t)(v + 0.5f) : (int64_t)floorf(v + 0.5f);
  if (r < lo) r = lo;
  if (r > hi) r = hi;
  return (uint32_t)r;                                       // two's complement for negatives
}

// Build one frame. getSrc(i) must return a pointer to the 8 data bytes of
// signal i's source frame, or nullptr if it is missing / stale / too short.
// counters[] holds each counter field's state. Returns false (frame
// skipped, counters unchanged) if any source is missing.
template <typename GetSrc>
static inline bool genBuild(const GenCfg &g, uint8_t out[8], uint32_t counters[MAX_GEN_SIG], GetSrc getSrc) {
  memcpy(out, g.base, 8);
  for (uint8_t i = 0; i < g.numSig; i++)
    if (g.sig[i].kind == GS_SOURCE && !getSrc(i)) return false;
  for (uint8_t i = 0; i < g.numSig; i++) {
    const GenSigCfg &s = g.sig[i];
    if (s.kind == GS_SOURCE) {
      uint32_t raw = genGet(getSrc(i), s.srcStart, s.srcLen, s.srcOrder);
      genPut(out, s.dstStart, s.dstLen, s.dstOrder, genScale(raw, s));
    } else if (s.kind == GS_COUNTER) {
      genPut(out, s.dstStart, s.dstLen, s.dstOrder, counters[i]);
    }
  }
  for (uint8_t i = 0; i < g.numSig; i++) {
    const GenSigCfg &s = g.sig[i];
    if (s.kind == GS_COUNTER) {
      uint32_t mod = s.dstLen >= 32 ? 0 : (1u << s.dstLen);
      counters[i] += s.step;
      if (mod) counters[i] %= mod;
    }
  }
  return true;
}

// ===========================================================================
// Custom route transforms
// ===========================================================================
// A transform is a list of steps applied, in order, to a frame a route
// forwards. Fields use the same DBC bit numbering as generated frames.
//
//   XO_SET_ID    : the frame leaves with ID srcId (29-bit if srcExt)
//   XO_SET_LEN   : the frame leaves with 'value' data bytes (0-8; new bytes are 0)
//   XO_SET_FIELD : write the constant 'value' into the dst field
//   XO_COPY      : read a src field, out = round(raw * factor + offset) clamped,
//                  write it into the dst field
//   XO_COND      : forward the frame only if the src field (raw, signed if
//                  srcSigned) compares true with 'value' (cmp)
//   XO_COUNTER   : write a rolling counter into the dst field, +step per frame
//
// Sources: srcBus 0 / 1 = the latest cached frame srcId on Bus A / B,
//          XS_THIS (2)  = this frame as it was received (before any step).
// A cached source that is missing or older than staleMs: the frame is dropped,
// unless flags & XF_KEEP_IF_MISSING (the step is skipped) or, for COPY,
// flags & XF_ZERO_IF_MISSING (0 is written into the field, e.g. "released").
// COPY options:
//   XF_MASK       : only the bits set in 'value' (the dst field's raw bits) are
//                   written; the others keep the frame's own value.
//   XF_CATCH_TAPS : (cached source) a bit reads 1 if it was set in ANY source
//                   frame since this transform last ran, so a press shorter
//                   than one forwarded frame still shows once. Bit-per-button
//                   data only. The caller provides that data through getSrc.
// Together these do what the old Buttons map did (mergeButtons).
static const uint8_t MAX_XF     = 8;     // custom transforms
static const uint8_t MAX_XF_OPS = 6;     // steps per transform
static const uint8_t XS_THIS    = 2;
static const uint8_t XF_KEEP_IF_MISSING = 0x01;
static const uint8_t XF_ZERO_IF_MISSING = 0x02;   // COPY only
static const uint8_t XF_CATCH_TAPS      = 0x04;   // COPY from a cached frame only
static const uint8_t XF_MASK            = 0x08;   // COPY only: 'value' = bits to write
static const uint8_t XF_FLAGS_ALL       = 0x0F;

enum XfOpKind : uint8_t { XO_NONE = 0, XO_SET_ID = 1, XO_SET_LEN = 2, XO_SET_FIELD = 3, XO_COPY = 4, XO_COND = 5, XO_COUNTER = 6 };
enum XfCmp    : uint8_t { XC_EQ = 0, XC_NE, XC_LT, XC_LE, XC_GT, XC_GE, XC_ALL_BITS, XC_ANY_BIT, XC_COUNT };

struct XfOpCfg {
  uint32_t srcId;       // source frame (COPY / COND from a cached frame); new ID (SET_ID)
  int32_t  value;       // SET_FIELD constant, COND operand, SET_LEN length
  float    factor;      // COPY
  float    offset;
  uint16_t staleMs;     // cached source older than this = missing (0 = any age)
  uint8_t  kind, srcBus, srcExt, srcStart, srcLen, srcOrder, srcSigned;
  uint8_t  dstStart, dstLen, dstOrder, dstSigned;
  uint8_t  cmp, step, flags;
};

struct XfCfg {
  char    name[GEN_NAME_LEN];
  uint8_t numOps;
  uint8_t pad[3];
  XfOpCfg ops[MAX_XF_OPS];
};

static inline int64_t xfSigned(uint32_t raw, uint8_t len, bool sgn) {
  if (sgn && len < 32 && ((raw >> (len - 1)) & 1)) return (int64_t)raw - ((int64_t)1 << len);
  if (sgn && len == 32) return (int32_t)raw;
  return raw;
}

static inline bool xfCompare(int64_t a, uint32_t rawA, const XfOpCfg &o) {
  int64_t b = o.value;
  switch (o.cmp) {
    case XC_EQ: return a == b;
    case XC_NE: return a != b;
    case XC_LT: return a <  b;
    case XC_LE: return a <= b;
    case XC_GT: return a >  b;
    case XC_GE: return a >= b;
    case XC_ALL_BITS: return (rawA & (uint32_t)o.value) == (uint32_t)o.value;
    case XC_ANY_BIT:  return (rawA & (uint32_t)o.value) != 0;
  }
  return false;
}

// Apply transform x to a frame. id/ext/len/data are the outgoing frame (start
// as a copy of the received one); orig/origLen are the received data.
// getSrc(i) returns the 8 data bytes of step i's cached source frame, or
// nullptr if missing / stale / too short. Returns false = drop the frame.
// counters[] advance only when the frame is forwarded.
// Writes v into the COPY step's dst field; with XF_MASK only the bits in 'value'.
static inline void xfPutMasked(uint8_t data[8], const XfOpCfg &o, uint32_t v) {
  if (o.flags & XF_MASK) {
    const uint32_t fm = o.dstLen >= 32 ? 0xFFFFFFFFu : ((1u << o.dstLen) - 1);
    const uint32_t m = (uint32_t)o.value & fm;
    v = (genGet(data, o.dstStart, o.dstLen, o.dstOrder) & ~m) | (v & m);
  }
  genPut(data, o.dstStart, o.dstLen, o.dstOrder, v);
}

template <typename GetSrc>
static inline bool xfApply(const XfCfg &x, uint32_t &id, uint8_t &ext, uint8_t &len, uint8_t data[8],
                           const uint8_t orig[8], uint8_t origLen, uint32_t counters[MAX_XF_OPS], GetSrc getSrc) {
  for (uint8_t i = 0; i < x.numOps; i++) {
    const XfOpCfg &o = x.ops[i];
    const uint8_t *src = nullptr;
    if (o.kind == XO_COPY || o.kind == XO_COND) {
      if (o.srcBus == XS_THIS) src = genFieldFits(o.srcStart, o.srcLen, o.srcOrder, origLen) ? orig : nullptr;
      else src = getSrc(i);
      if (!src) {
        if (o.flags & XF_KEEP_IF_MISSING) continue;
        if (o.kind == XO_COPY && (o.flags & XF_ZERO_IF_MISSING)) { xfPutMasked(data, o, 0); continue; }
        return false;
      }
    }
    switch (o.kind) {
      case XO_SET_ID:  id = o.srcId; ext = o.srcExt; break;
      case XO_SET_LEN: {
        uint8_t n = o.value < 0 ? 0 : o.value > 8 ? 8 : (uint8_t)o.value;
        for (uint8_t b = len; b < n; b++) data[b] = 0;
        len = n;
        break;
      }
      case XO_SET_FIELD: {
        uint32_t m = o.dstLen >= 32 ? 0xFFFFFFFFu : ((1u << o.dstLen) - 1);
        genPut(data, o.dstStart, o.dstLen, o.dstOrder, (uint32_t)o.value & m);
        break;
      }
      case XO_COPY: {
        GenSigCfg s; memset(&s, 0, sizeof(s));
        s.srcLen = o.srcLen; s.srcSigned = o.srcSigned; s.dstLen = o.dstLen; s.dstSigned = o.dstSigned;
        s.factor = o.factor; s.offset = o.offset;
        xfPutMasked(data, o, genScale(genGet(src, o.srcStart, o.srcLen, o.srcOrder), s));
        break;
      }
      case XO_COND: {
        uint32_t raw = genGet(src, o.srcStart, o.srcLen, o.srcOrder);
        if (!xfCompare(xfSigned(raw, o.srcLen, o.srcSigned), raw, o)) return false;
        break;
      }
      case XO_COUNTER:
        genPut(data, o.dstStart, o.dstLen, o.dstOrder, counters[i]);
        break;
    }
  }
  for (uint8_t i = 0; i < x.numOps; i++) {
    const XfOpCfg &o = x.ops[i];
    if (o.kind == XO_COUNTER) {
      counters[i] += o.step;
      if (o.dstLen < 32) counters[i] %= (1u << o.dstLen);
    }
  }
  return true;
}
