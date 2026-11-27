// CanProbe.ino - transceiver / wiring check for the CanIsolator hardware.
// Teensy 4.0. Flash this INSTEAD of CanIsolator, open the Serial Monitor.
// It only ever LISTENS: it never transmits or ACKs, so it is safe on a live car bus.
// The test repeats every few seconds, so you can wiggle wires / probe while it runs.
//
// TEST 1 - pin level check (CAN controller not running, pins as plain GPIO)
//   For each transceiver it checks:
//   RX pin (transceiver RXD -> Teensy):
//     - follows the Teensy's pull-up AND pull-down -> nothing drives it:
//       transceiver unpowered, not connected, or wired to the wrong pin
//     - steady HIGH, no edges -> transceiver alive, but the bus is silent:
//       CAN-H/CAN-L not connected (or no node transmitting)
//     - HIGH with edges       -> transceiver alive and passing bus traffic. GOOD.
//     - steady LOW            -> stuck dominant: CAN-H/CAN-L swapped or shorted,
//                                or a transceiver that isn't powered properly
//   TX pin (Teensy -> transceiver TXD):
//     - edges seen            -> TX and RX are SWAPPED (an output drives the TX pin)
//
// TEST 2 - CAN controller in listen-only mode at several bit rates
//   Counts frames and errors. Frames at 500k = the whole chain works.
//   Frames at a different rate only = the bus isn't running at 500k.

#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

struct PinSet { const char *name; uint8_t rx; uint8_t tx; const char *side; };
// With SWAP_CAN_CONTROLLERS defined in CanIsolator (current build):
//   CAN2 = noisy side, CAN1 = MoTeC M1 side.
static const PinSet PINS[2] = {
  { "CAN1", 23, 22, "M1 side (with SWAP_CAN_CONTROLLERS)"    },
  { "CAN2",  0,  1, "noisy side (with SWAP_CAN_CONTROLLERS)" },
};

static const uint32_t RATES[] = { 500000, 250000, 1000000, 125000 };

// Count level changes on a pin for 'ms' milliseconds.
// One 500k bit is 2 us. The loop polls far faster than that.
// Low-pulse width statistics from the last countEdges() call.
// A real CAN dominant bit is 2 us at 500k (8 us at 125k). Pulses far shorter
// than that are noise / ringing, not CAN data.
static const uint32_t CYC_PER_US = F_CPU_ACTUAL / 1000000;
static uint32_t lowPulses, lowShort, lowBit, lowMinCyc, lowMaxCyc;

static uint32_t countEdges(uint8_t pin, uint32_t ms, uint8_t &finalLevel, uint32_t &highSamples, uint32_t &samples) {
  uint32_t edges = 0;
  highSamples = 0; samples = 0;
  lowPulses = lowShort = lowBit = 0; lowMinCyc = 0xFFFFFFFF; lowMaxCyc = 0;
  uint8_t last = digitalReadFast(pin);
  uint32_t fallCyc = ARM_DWT_CYCCNT;
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    uint8_t v = digitalReadFast(pin);
    if (v != last) {
      uint32_t now = ARM_DWT_CYCCNT;
      edges++;
      if (v == 0) fallCyc = now;                       // went low
      else {                                           // came back high: one low pulse
        uint32_t w = now - fallCyc;
        lowPulses++;
        if (w < lowMinCyc) lowMinCyc = w;
        if (w > lowMaxCyc) lowMaxCyc = w;
        if (w < CYC_PER_US) lowShort++;                // < 1 us: shorter than any CAN bit up to 1 Mbps
        else                lowBit++;                  // >= 1 us: plausible CAN bit(s)
      }
      last = v;
    }
    highSamples += v; samples++;
  }
  finalLevel = last;
  return edges;
}

static void pinTest(const PinSet &p) {
  Serial.printf("\n--- TEST 1: %s pins (%s)  RX=pin %u  TX=pin %u ---\n", p.name, p.side, p.rx, p.tx);

  // Is anything driving the RX pin? Try both pulls.
  pinMode(p.rx, INPUT_PULLUP);   delay(2); uint8_t withUp   = digitalReadFast(p.rx);
  pinMode(p.rx, INPUT_PULLDOWN); delay(2); uint8_t withDown = digitalReadFast(p.rx);
  pinMode(p.rx, INPUT);          delay(2);
  uint8_t lvl; uint32_t hi, n;
  uint32_t edges = countEdges(p.rx, 500, lvl, hi, n);
  float pctHigh = n ? 100.0f * hi / n : 0;

  Serial.printf("RX pin: pull-up reads %u, pull-down reads %u, %lu edges in 0.5 s, high %.1f%% of the time\n",
                withUp, withDown, (unsigned long)edges, pctHigh);
  if (lowPulses)
    Serial.printf("RX low pulses: %lu total | %lu shorter than 1 us (noise) | %lu of 1 us or more (CAN-bit length) | "
                  "shortest %.3f us, longest %.2f us\n",
                  (unsigned long)lowPulses, (unsigned long)lowShort, (unsigned long)lowBit,
                  (float)lowMinCyc / CYC_PER_US, (float)lowMaxCyc / CYC_PER_US);

  if (withUp == 1 && withDown == 0 && edges == 0)
    Serial.println("  => RX pin is NOT DRIVEN. Transceiver unpowered, not wired to this pin, or dead.");
  else if (edges > 0 && lowPulses && lowBit * 10 < lowPulses)
    Serial.println("  => Transceiver POWERED, but its output is NOISE (sub-microsecond spikes), not CAN bits.\n"
                   "     Check: transceiver GND tied to CAR ground, BOTH CAN-H and CAN-L connected, H/L not swapped.");
  else if (edges > 0 && pctHigh > 50)
    Serial.printf("  => Transceiver ALIVE and passing bus traffic (~%lu edges/s). GOOD.\n", (unsigned long)edges * 2);
  else if (edges == 0 && withUp == 1 && withDown == 1)
    Serial.println("  => Transceiver ALIVE (driving recessive), but NO bus activity. Check CAN-H/CAN-L to this transceiver, and that the bus is live.");
  else if (edges == 0 && withUp == 0 && withDown == 0)
    Serial.println("  => RX STUCK LOW (dominant). CAN-H/CAN-L swapped or shorted, transceiver power/ground fault, or TX/RX swapped.");
  else
    Serial.println("  => Mostly LOW with activity: unusual. Check CAN-H/CAN-L polarity and termination.");

  // TX pin: nothing on the transceiver side should ever drive it.
  pinMode(p.tx, INPUT_PULLDOWN); delay(2);
  uint32_t txEdges = countEdges(p.tx, 200, lvl, hi, n);
  uint8_t txDown = digitalReadFast(p.tx);
  pinMode(p.tx, INPUT_PULLUP); delay(2);
  uint8_t txUp = digitalReadFast(p.tx);
  pinMode(p.tx, INPUT);
  Serial.printf("TX pin: pull-down reads %u, pull-up reads %u, %lu edges in 0.2 s\n", txDown, txUp, (unsigned long)txEdges);
  if (txEdges > 0)
    Serial.println("  => Activity on the TX pin: TX and RX wires are almost certainly SWAPPED.");
  else if (txDown == 1)
    Serial.println("  => TX pin held HIGH by the transceiver (normal for many parts: TXD has a pull-up).");
  else
    Serial.println("  => TX pin not driven (normal for transceivers without a TXD pull-up).");
}

// (No templates: the Arduino .ino prototype generator mangles them.)
static void busBegin(uint8_t i) {
  if (i == 0) { can1.begin(); can1.setMaxMB(16); can1.enableFIFO(); }
  else        { can2.begin(); can2.setMaxMB(16); can2.enableFIFO(); }
}
static void busRate(uint8_t i, uint32_t r) {
  if (i == 0) can1.setBaudRate(r, LISTEN_ONLY); else can2.setBaudRate(r, LISTEN_ONLY);
}
static int busRead(uint8_t i, CAN_message_t &m) { return i == 0 ? can1.read(m) : can2.read(m); }

static void canTest(uint8_t i) {
  const PinSet &p = PINS[i];
  Serial.printf("--- TEST 2: %s listen-only ---\n", p.name);
  const uint32_t base = (i == 0) ? (uint32_t)CAN1 : (uint32_t)CAN2;
  busBegin(i);
  for (uint8_t r = 0; r < sizeof(RATES) / sizeof(RATES[0]); r++) {
    busRate(i, RATES[r]);
    CAN_message_t m;
    while (busRead(i, m)) {}                     // flush
    (void)FLEXCANb_ESR1(base);                   // clear latched error flags
    uint32_t frames = 0, extFrames = 0, firstId = 0;
    const uint32_t t0 = millis();
    while (millis() - t0 < 1000) {
      while (busRead(i, m)) { if (!frames) firstId = m.id; frames++; if (m.flags.extended) extFrames++; }
    }
    uint32_t esr = FLEXCANb_ESR1(base);
    uint32_t ecr = FLEXCANb_ECR(base);
    Serial.printf("  %7lu bps: %5lu frames/s (%lu ext)", (unsigned long)RATES[r], (unsigned long)frames, (unsigned long)extFrames);
    if (frames) Serial.printf("  first ID %lX", (unsigned long)firstId);
    Serial.printf("  | RX err count %lu", (unsigned long)((ecr >> 8) & 0xFF));
    if (esr & (1UL << 10)) Serial.print(" STUFF");
    if (esr & (1UL << 11)) Serial.print(" FORM");
    if (esr & (1UL << 12)) Serial.print(" CRC");
    if (esr & (1UL << 15)) Serial.print(" BIT1");
    if (esr & (1UL << 14)) Serial.print(" BIT0");
    Serial.println();
  }
}

// Transceiver standby pins on the dual CAN adapter: LOW = normal mode.
// Set STANDBY_TEST to true to leave them HIGH and see what standby looks like.
static const uint8_t CAN_STANDBY_PINS[] = { 2, 3 };
static const bool    STANDBY_TEST = false;

void setup() {
  for (uint8_t i = 0; i < sizeof(CAN_STANDBY_PINS); i++) {
    pinMode(CAN_STANDBY_PINS[i], OUTPUT);
    digitalWriteFast(CAN_STANDBY_PINS[i], STANDBY_TEST ? HIGH : LOW);
  }
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 4000) {}
  Serial.println("\nCanProbe - transceiver and wiring check (listen-only, never transmits)");
  Serial.printf("Transceiver standby pins 2, 3 driven %s\n", STANDBY_TEST ? "HIGH (standby test)" : "LOW (normal mode)");
}

void loop() {
  // Test 1 uses the pins as GPIO, so run it before the CAN controllers take them.
  pinTest(PINS[0]);
  pinTest(PINS[1]);
  canTest(0);
  canTest(1);
  Serial.println("\n(repeating in 3 s)");
  delay(3000);
}
