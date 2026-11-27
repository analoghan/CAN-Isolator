// CanipulatorTest - hardware bring-up test for both CAN channels on the
// TechOverflow CANipulator (ESP32-C6-WROOM-1U, two on-chip TWAI controllers).
//
// Open the serial monitor at 115200 (USB-C). Commands:
//   a  run ALL tests (also runs at boot and when the BOOT button is pressed)
//   s  self-test each channel           (no wiring needed)
//   p  control-pin test (RS / shutdown) (no wiring needed)
//   x  cross test CAN0 <-> CAN1         (needs CAN0 wired to CAN1, see below)
//   l  listen-only monitor on both channels (safe on a live car bus); any key stops
//   1/2/5/8  bit rate 125k / 250k / 500k / 1M   (default 500k)
//   ?  help
//
// SELF TEST: each controller runs in "no-ACK" mode and sends frames with
//   self-reception. The frame goes MCU TX pin -> transceiver -> CAN-H/L ->
//   transceiver receiver -> MCU RX pin, so a pass proves the controller,
//   both GPIOs, and the transceiver's driver and receiver on that channel.
//
// CONTROL PIN TEST: the self test is repeated with the transceiver put in
//   standby (RS high), then in shutdown. It MUST fail in those states; if it
//   still passes, the control pin isn't reaching the transceiver.
//
// CROSS TEST wiring: CAN0-H to CAN1-H, CAN0-L to CAN1-L, plus ~120 ohm across
//   H/L (or bridge one termination solder jumper; both are open from the
//   factory). Each channel sends to the other and the other ACKs, so this
//   proves real two-node traffic in both directions.
//   The self/pin/cross tests TRANSMIT - do not run them while connected to the car.

#include <Arduino.h>
#include "driver/twai.h"

// ---- Pin map (CANipulator datasheet, "I/O Usage & Functions") ----
static const gpio_num_t CAN_RX[2]   = { GPIO_NUM_16, GPIO_NUM_18 };
static const gpio_num_t CAN_TX[2]   = { GPIO_NUM_17, GPIO_NUM_19 };
static const uint8_t    CAN_RS[2]   = { 0, 1 };  // high = standby, low = high-speed
static const uint8_t    CAN_SHDN[2] = { 3, 2 };  // high = shutdown (note CAN0=GPIO3, CAN1=GPIO2)
static const uint8_t    PIN_LED     = 8;         // addressable RGB LED
static const uint8_t    PIN_BUTTON  = 9;         // BOOT / user button, active low

static uint32_t bitrate = 500000;
static twai_handle_t h[2] = { nullptr, nullptr };

// ------------------------------------------------------------------ helpers

static void led(uint8_t r, uint8_t g, uint8_t b) { rgbLedWrite(PIN_LED, r, g, b); }

static void transceiverActive(int ch) {
  digitalWrite(CAN_SHDN[ch], LOW);
  digitalWrite(CAN_RS[ch], LOW);
}

static twai_timing_config_t timingFor(uint32_t rate) {
  switch (rate) {
    case 125000:  { twai_timing_config_t t = TWAI_TIMING_CONFIG_125KBITS(); return t; }
    case 250000:  { twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS(); return t; }
    case 1000000: { twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();   return t; }
    default:      { twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS(); return t; }
  }
}

static const char *stateName(twai_state_t s) {
  switch (s) {
    case TWAI_STATE_STOPPED:    return "STOPPED";
    case TWAI_STATE_RUNNING:    return "RUNNING";
    case TWAI_STATE_BUS_OFF:    return "BUS-OFF";
    case TWAI_STATE_RECOVERING: return "RECOVERING";
  }
  return "?";
}

static bool canStart(int ch, twai_mode_t mode) {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT_V2(ch, CAN_TX[ch], CAN_RX[ch], mode);
  g.tx_queue_len = 16;
  g.rx_queue_len = 64;
  twai_timing_config_t t = timingFor(bitrate);
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  esp_err_t e = twai_driver_install_v2(&g, &t, &f, &h[ch]);
  if (e != ESP_OK) { Serial.printf("  CAN%d driver install failed: %s\n", ch, esp_err_to_name(e)); h[ch] = nullptr; return false; }
  e = twai_start_v2(h[ch]);
  if (e != ESP_OK) { Serial.printf("  CAN%d start failed: %s\n", ch, esp_err_to_name(e)); return false; }
  return true;
}

static void canStop(int ch) {
  if (!h[ch]) return;
  twai_status_info_t st;
  if (twai_get_status_info_v2(h[ch], &st) == ESP_OK &&
      (st.state == TWAI_STATE_RUNNING || st.state == TWAI_STATE_RECOVERING)) {
    twai_stop_v2(h[ch]);
  }
  twai_driver_uninstall_v2(h[ch]);
  h[ch] = nullptr;
}

static void printStatus(int ch) {
  twai_status_info_t st;
  if (!h[ch] || twai_get_status_info_v2(h[ch], &st) != ESP_OK) return;
  Serial.printf("    CAN%d status: %s  TEC=%lu REC=%lu  busErr=%lu txFail=%lu arbLost=%lu rxMissed=%lu\n",
                ch, stateName(st.state), st.tx_error_counter, st.rx_error_counter,
                st.bus_error_count, st.tx_failed_count, st.arb_lost_count, st.rx_missed_count);
}

static void fillFrame(twai_message_t &m, uint32_t id, uint8_t seq, uint8_t ch) {
  memset(&m, 0, sizeof(m));
  m.identifier = id;
  m.data_length_code = 8;
  m.data[0] = seq;       m.data[1] = ch;
  m.data[2] = 0xA5;      m.data[3] = 0x5A;
  m.data[4] = 0xFF;      m.data[5] = 0x00;
  m.data[6] = (uint8_t)~seq;
  m.data[7] = 0x55;
}

static bool frameMatches(const twai_message_t &a, const twai_message_t &b) {
  return a.identifier == b.identifier && a.data_length_code == b.data_length_code &&
         memcmp(a.data, b.data, a.data_length_code) == 0;
}

// Drain anything left in an RX queue.
static void drain(int ch) {
  twai_message_t m;
  while (twai_receive_v2(h[ch], &m, 0) == ESP_OK) {}
}

// ------------------------------------------------------------------ self test

// Returns number of frames correctly self-received out of n.
static int selfTestOnce(int ch, int n, bool verbose) {
  if (!canStart(ch, TWAI_MODE_NO_ACK)) { canStop(ch); return 0; }
  int ok = 0;
  for (int i = 0; i < n; i++) {
    twai_message_t tx, rx;
    fillFrame(tx, 0x700 + ch, i, ch);
    tx.self = 1;
    if (twai_transmit_v2(h[ch], &tx, pdMS_TO_TICKS(20)) != ESP_OK) break;
    if (twai_receive_v2(h[ch], &rx, pdMS_TO_TICKS(20)) != ESP_OK) break;
    if (frameMatches(tx, rx)) ok++;
    else if (verbose) Serial.printf("    CAN%d frame %d came back corrupted\n", ch, i);
  }
  if (verbose && ok != n) printStatus(ch);
  canStop(ch);
  return ok;
}

static bool testSelf() {
  const int N = 50;
  bool all = true;
  Serial.printf("\n== SELF TEST (no-ACK + self-reception, %lu bit/s, %d frames) ==\n", bitrate, N);
  for (int ch = 0; ch < 2; ch++) {
    transceiverActive(ch);
    delay(2);
    // Idle check: with the controller not running, a live transceiver holds RX high (recessive).
    pinMode(CAN_RX[ch], INPUT_PULLDOWN);
    delayMicroseconds(200);
    int idle = digitalRead(CAN_RX[ch]);
    pinMode(CAN_RX[ch], INPUT);
    if (!idle) Serial.printf("  CAN%d: RX pin (GPIO%d) reads LOW at idle - transceiver unpowered, bus stuck dominant, or H/L shorted\n",
                             ch, CAN_RX[ch]);
    int ok = selfTestOnce(ch, N, true);
    bool pass = (ok == N);
    all &= pass;
    Serial.printf("  CAN%d: %d/%d frames looped through the transceiver  -> %s\n",
                  ch, ok, N, pass ? "PASS" : "FAIL");
  }
  if (!all)
    Serial.println("  Hint: if this passes at 125k ('1' then 's') but fails at 500k, the bus has no termination -\n"
                   "        add 120 ohm across H/L or bridge the termination jumper. 'd' measures edge timing.");
  return all;
}

// ------------------------------------------------------------------ control pin test

static bool testControlPins() {
  bool all = true;
  Serial.printf("\n== CONTROL PIN TEST (self test must FAIL when the transceiver is disabled) ==\n");
  for (int ch = 0; ch < 2; ch++) {
    // Standby via RS
    transceiverActive(ch);
    digitalWrite(CAN_RS[ch], HIGH);
    delay(2);
    int okStby = selfTestOnce(ch, 5, false);
    // Shutdown
    transceiverActive(ch);
    digitalWrite(CAN_SHDN[ch], HIGH);
    delay(2);
    int okShdn = selfTestOnce(ch, 5, false);
    transceiverActive(ch);
    delay(2);
    // And it must work again afterwards
    int okBack = selfTestOnce(ch, 5, false);

    bool stbyPass = (okStby == 0), shdnPass = (okShdn == 0), backPass = (okBack == 5);
    Serial.printf("  CAN%d: RS/standby  (GPIO%d high): %d/5 looped -> %s\n", ch, CAN_RS[ch], okStby,
                  stbyPass ? "PASS (blocked)" : "FAIL (pin has no effect)");
    Serial.printf("  CAN%d: shutdown    (GPIO%d high): %d/5 looped -> %s\n", ch, CAN_SHDN[ch], okShdn,
                  shdnPass ? "PASS (blocked)" : "FAIL (pin has no effect)");
    Serial.printf("  CAN%d: re-enabled                : %d/5 looped -> %s\n", ch, okBack,
                  backPass ? "PASS" : "FAIL");
    all &= stbyPass && shdnPass && backPass;
  }
  return all;
}

// ------------------------------------------------------------------ cross test

static int sendAcross(int from, int to, int n) {
  drain(to);
  int got = 0, txFail = 0;
  for (int i = 0; i < n; i++) {
    twai_message_t tx, rx;
    fillFrame(tx, 0x100 + from, i, from);
    if (twai_transmit_v2(h[from], &tx, pdMS_TO_TICKS(20)) != ESP_OK) { txFail++; break; }
    if (twai_receive_v2(h[to], &rx, pdMS_TO_TICKS(20)) == ESP_OK && frameMatches(tx, rx)) got++;
  }
  return got;
}

static bool testCross() {
  const int N = 100;
  Serial.printf("\n== CROSS TEST CAN0 <-> CAN1 (normal mode, %lu bit/s, %d frames each way) ==\n", bitrate, N);
  transceiverActive(0);
  transceiverActive(1);
  delay(2);
  bool started = canStart(0, TWAI_MODE_NORMAL) && canStart(1, TWAI_MODE_NORMAL);
  bool all = false;
  if (started) {
    int a = sendAcross(0, 1, N);
    int b = sendAcross(1, 0, N);
    bool pa = (a == N), pb = (b == N);
    Serial.printf("  CAN0 -> CAN1: %d/%d received -> %s\n", a, N, pa ? "PASS" : "FAIL");
    Serial.printf("  CAN1 -> CAN0: %d/%d received -> %s\n", b, N, pb ? "PASS" : "FAIL");
    all = pa && pb;
    if (!all) {
      printStatus(0);
      printStatus(1);
      Serial.println("    Check: CAN0-H<->CAN1-H, CAN0-L<->CAN1-L, ~120 ohm across H/L, nothing else on the wires.");
      Serial.println("    0 frames + rising TEC usually = no wiring between channels (no ACK). H/L swapped also gives this.");
    }
  }
  canStop(0);
  canStop(1);
  return all;
}

// ------------------------------------------------------------------ raw GPIO diagnostic

// With the CAN controller off, drive the TX pin by hand and read the RX pin.
// A working, enabled transceiver echoes TX to RX: TX low -> RX low (dominant).
// Each cell shows RX level for TX=LOW/TX=HIGH, under RX pull-down / pull-up:
//   "0 1 | 0 1" = RX follows TX (good)   "x x | 0 1" style = RX floating (nothing drives it)
static int readWithPull(uint8_t pin, bool up) {
  pinMode(pin, up ? INPUT_PULLUP : INPUT_PULLDOWN);
  delayMicroseconds(100);
  return digitalRead(pin);
}

static void rawProbe(uint8_t txPin, uint8_t rxPin, char *out) {
  pinMode(txPin, OUTPUT);
  int r[2][2];
  for (int up = 0; up < 2; up++)
    for (int lvl = 0; lvl < 2; lvl++) {
      digitalWrite(txPin, lvl);
      delayMicroseconds(50);
      r[up][lvl] = readWithPull(rxPin, up);
    }
  digitalWrite(txPin, HIGH);
  pinMode(txPin, INPUT);
  pinMode(rxPin, INPUT);
  const char *verdict;
  if (r[0][0] == 0 && r[0][1] == 1 && r[1][0] == 0 && r[1][1] == 1) verdict = "FOLLOWS TX";
  else if (r[0][0] == r[0][1] && r[1][0] == r[1][1] && r[0][0] != r[1][0]) verdict = "floating";
  else if (r[0][0] && r[0][1] && r[1][0] && r[1][1]) verdict = "stuck high";
  else if (!r[0][0] && !r[0][1] && !r[1][0] && !r[1][1]) verdict = "stuck low";
  else verdict = "odd";
  sprintf(out, "%d %d | %d %d  %-10s", r[0][0], r[0][1], r[1][0], r[1][1], verdict);
}

// Time (ns) for RX to go dominant after TX goes low, and back to recessive after TX
// is released. Without a termination resistor the bus has nothing to pull it back
// to recessive quickly, so the release time stretches to several microseconds -
// longer than a bit at 500k (2 us) - and the controller sees errors.
static void edgeTiming(int ch, uint32_t &fallNs, uint32_t &riseNs) {
  const uint32_t mhz = getCpuFrequencyMhz();
  const uint32_t timeout = 100 * mhz;  // 100 us
  pinMode(CAN_TX[ch], OUTPUT);
  pinMode(CAN_RX[ch], INPUT);
  uint32_t bestF = UINT32_MAX, worstR = 0;
  for (int i = 0; i < 20; i++) {
    digitalWrite(CAN_TX[ch], HIGH);
    delayMicroseconds(100);
    uint32_t t0 = esp_cpu_get_cycle_count();
    digitalWrite(CAN_TX[ch], LOW);
    while (digitalRead(CAN_RX[ch]) && esp_cpu_get_cycle_count() - t0 < timeout) {}
    uint32_t f = esp_cpu_get_cycle_count() - t0;
    delayMicroseconds(20);
    t0 = esp_cpu_get_cycle_count();
    digitalWrite(CAN_TX[ch], HIGH);
    while (!digitalRead(CAN_RX[ch]) && esp_cpu_get_cycle_count() - t0 < timeout) {}
    uint32_t r = esp_cpu_get_cycle_count() - t0;
    if (f < bestF) bestF = f;
    if (r > worstR) worstR = r;
  }
  pinMode(CAN_TX[ch], INPUT);
  fallNs = bestF * 1000 / mhz;
  riseNs = worstR * 1000 / mhz;
}

static void testEdgeTiming() {
  Serial.println("\n== BUS EDGE TIMING (controller off) ==");
  for (int ch = 0; ch < 2; ch++) {
    transceiverActive(ch);
    delay(2);
    uint32_t f, r;
    edgeTiming(ch, f, r);
    Serial.printf("  CAN%d: TX->RX dominant %lu ns, recessive release %lu ns", ch, f, r);
    if (r > 8000) Serial.print("  <- SLOW: bus not terminated? (limits bit rate)");
    Serial.println();
  }
  Serial.println("  (coarse: GPIO polling inflates these. Measured on this board: ~5.8 us terminated, ~11.4 us unterminated)");
}

static void testRawPins() {
  testEdgeTiming();
  Serial.println("\n== RAW GPIO DIAGNOSTIC (controller off; TX driven by hand) ==");
  Serial.println("  cells: RX for TX=0,1 with pulldown | with pullup");
  for (int ch = 0; ch < 2; ch++) {
    for (int rs = 0; rs < 2; rs++)
      for (int sd = 0; sd < 2; sd++) {
        digitalWrite(CAN_RS[ch], rs);
        digitalWrite(CAN_SHDN[ch], sd);
        delay(2);
        char a[48], b[48];
        rawProbe(CAN_TX[ch], CAN_RX[ch], a);
        rawProbe(CAN_RX[ch], CAN_TX[ch], b);  // swapped orientation
        Serial.printf("  CAN%d RS(GPIO%d)=%d SHDN(GPIO%d)=%d  normal tx%d->rx%d: %s  swapped tx%d->rx%d: %s\n",
                      ch, CAN_RS[ch], rs, CAN_SHDN[ch], sd,
                      CAN_TX[ch], CAN_RX[ch], a, CAN_RX[ch], CAN_TX[ch], b);
      }
    transceiverActive(ch);
  }
}

// ------------------------------------------------------------------ listen monitor

static void listenMonitor() {
  Serial.printf("\n== LISTEN-ONLY MONITOR (%lu bit/s). Never transmits or ACKs. Any key to stop. ==\n", bitrate);
  transceiverActive(0);
  transceiverActive(1);
  delay(2);
  if (!canStart(0, TWAI_MODE_LISTEN_ONLY) || !canStart(1, TWAI_MODE_LISTEN_ONLY)) { canStop(0); canStop(1); return; }
  uint32_t count[2] = { 0, 0 }, lastReport = millis();
  while (!Serial.available()) {
    for (int ch = 0; ch < 2; ch++) {
      twai_message_t m;
      while (twai_receive_v2(h[ch], &m, 0) == ESP_OK) {
        count[ch]++;
        if (count[ch] <= 20) {  // print the first few, then just count
          Serial.printf("  CAN%d %s 0x%03lX [%d]", ch, m.extd ? "EXT" : "STD", m.identifier, m.data_length_code);
          for (int i = 0; i < m.data_length_code; i++) Serial.printf(" %02X", m.data[i]);
          Serial.println();
        }
      }
    }
    if (millis() - lastReport >= 1000) {
      lastReport = millis();
      Serial.printf("  frames/s  CAN0=%lu  CAN1=%lu\n", count[0], count[1]);
      printStatus(0);
      printStatus(1);
      led(0, count[0] ? 20 : 0, count[1] ? 20 : 0);
      count[0] = count[1] = 0;
    }
    delay(1);
  }
  while (Serial.available()) Serial.read();
  canStop(0);
  canStop(1);
  Serial.println("  monitor stopped");
}

// ------------------------------------------------------------------ main

static void help() {
  Serial.printf("\nCANipulator dual-CAN test. Bit rate: %lu\n", bitrate);
  Serial.println("  a=all  s=self  p=control pins  x=cross (CAN0 wired to CAN1)  l=listen monitor");
  Serial.println("  1=125k 2=250k 5=500k 8=1M  ?=help   BOOT button = run all");
}

static void runAll() {
  led(20, 20, 0);  // yellow = running
  bool s = testSelf();
  bool p = testControlPins();
  bool x = testCross();
  Serial.printf("\n== SUMMARY @ %lu bit/s:  self %s | control pins %s | cross %s ==\n",
                bitrate, s ? "PASS" : "FAIL", p ? "PASS" : "FAIL", x ? "PASS" : "FAIL");
  if (s && p && x)  led(0, 30, 0);   // green  = everything passed
  else if (s && p)  led(0, 0, 30);   // blue   = board OK, cross test failed (check the H/L wiring)
  else              led(30, 0, 0);   // red    = channel hardware fault
}

void setup() {
  for (int ch = 0; ch < 2; ch++) {
    pinMode(CAN_RS[ch], OUTPUT);
    pinMode(CAN_SHDN[ch], OUTPUT);
    transceiverActive(ch);
  }
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  led(0, 0, 10);
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(300);
  help();
  runAll();
}

void loop() {
  if (digitalRead(PIN_BUTTON) == LOW) {
    delay(30);
    while (digitalRead(PIN_BUTTON) == LOW) delay(5);
    runAll();
  }
  if (!Serial.available()) { delay(5); return; }
  char c = Serial.read();
  switch (c) {
    case 'a': runAll(); break;
    case 's': testSelf(); break;
    case 'p': testControlPins(); break;
    case 'x': testCross(); break;
    case 'd': testRawPins(); break;
    case 'l': listenMonitor(); break;
    case '1': bitrate = 125000;  help(); break;
    case '2': bitrate = 250000;  help(); break;
    case '5': bitrate = 500000;  help(); break;
    case '8': bitrate = 1000000; help(); break;
    case '?': help(); break;
    default: break;
  }
}
