// ====================================================================
//  AUDIO FTP OS v4.29 — modular core + apps
//  Arduino Uno / ATmega328P @ 16 MHz
//  WIRING: RX on D8 (ICP1), TX on D9 (OC1A) on BOTH boards.
//  Core: modem, protocol, session, storage/transfer services, shell kernel, app manager.
//  Apps register via register_all_apps() (apps_bundle.cpp); core never names them.
//  TX gap fixed (DEFAULT_DELAY). Storage: EEPROM only. BOTH BOARDS MUST MATCH.
// ====================================================================
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include <stdarg.h>
#include "config.h"
#include "core_api.h"

enum Loc : uint8_t { L_ROOT,
                     L_NETWORK,
                     L_EEPROM,
                     L_REMOTE,
                     L_REMOTE_EEPROM };

#define TX_PIN 9   // must stay D9: Timer1 OC1A / PB1 (hardware tone output)
#define RX_PIN 8   // must stay D8: Timer1 ICP1 / PB0 (input capture)

// --------------------------------------------------------------------
// ANSI Escape Sequences for Tera Term
// --------------------------------------------------------------------
#define ANSI_CLS "\033[2J\033[H"
#define ANSI_RESET "\033[0m"
#define ANSI_RED "\033[31m"
#define ANSI_GREEN "\033[32m"
#define ANSI_YELLOW "\033[33m"
#define ANSI_MAGENTA "\033[35m"
#define ANSI_CYAN "\033[36m"
#define ANSI_WHITE "\033[37;1m"

// --------------------------------------------------------------------
// Tone map (3000 .. 5285 Hz)
//   control: preamble / space / repeat, then 10 possible layer shifts (60 Hz steps)
//   data:    13 tones per layer (123 Hz steps)
//            layers 0-6 : printable ASCII (91 chars)
//            layer 7    : '|' '}' '~', 4 opcode tokens, 6 command tokens
//            layer 8, 9 : opcode tokens (13 each)
// --------------------------------------------------------------------
#define FREQ_PREAMBLE 3000
#define FREQ_SPACE 3060
#define FREQ_REPEAT 3120
#define SHIFT_BASE 3180
#define SHIFT_STEP 60
#define LAYER_BASE 3800
#define LAYER_STEP 123
#define CTL_TOL 20
#define DATA_TOL 45

// Receiver band edges, derived from the map above
#define BAND_SPLIT ((SHIFT_BASE + 9 * SHIFT_STEP + LAYER_BASE) / 2)  // 3760, middle of the dead zone
#define DATA_TOP   (LAYER_BASE + 12 * LAYER_STEP + DATA_TOL)     // 5321

// Tone length in whole periods; one captured interval confirms a data tone.
#define DATA_PERIODS 2
#define CTL_PERIODS 2

// Silence after a layer-shift tone = gap * 3/5 (1.02 ms at the 1.7 ms default,
// i.e. same as the old fixed 1.0 ms there), so the delay setting scales it too.
#define POST_SHIFT_NUM 3
#define POST_SHIFT_DEN 5

#define TOK_NONE (-99)
#define TOK_PREAMBLE 200
#define TOK_SPACE (-2)
#define TOK_REPEAT (-3)

#define TOK_CMD0 300
#define CMD_COUNT 6
#define CMD_LAYER 7
#define CMD_IDX0 7

// Opcode tokens: TOK_OP0 + slot (slot table: OP_TOK below)
#define TOK_OP0 400
#define OP_SLOTS 25
#define OP_LAYER_A 8    // session/control opcodes (slots 0..9)
#define OP_LAYER_B 9    // file/control opcodes    (slots 10..21)
                        // slots 22..25 = layer 7 idx 3..6 (transfer opcodes)

#define GAP_PERIOD_US 450    // 450 us: below the 600 us post-shift gap at the 1 ms TX setting, but above all tone periods
#define RESET_GAP_MS 3

#ifndef DEFAULT_DELAY        // normally set in config.h
#define DEFAULT_DELAY 1      // fixed TX gap units (0.1 ms); both boards must match
#endif
#define MIN_SAFE_DELAY 1
#define SILENCE_MIN_MS 4
#define SILENCE_MARGIN 2
#define CS_MARGIN 1
#define FRAME_GAP_EXTRA 1
#define PREAMBLE_MS 3

// --------------------------------------------------------------------
// Timing / Limits
// --------------------------------------------------------------------
#define MAX_TRIES 4
// Initial retransmission timeout. It is adapted from successful first-try ACKs.
// Keep a floor high enough for the slow audio framing, but avoid waiting
// half a second after a consistently fast round trip.
#define ACK_TIMEOUT 450UL
#define ACK_JITTER 50
#define ACK_RTO_MIN 300UL
#define ACK_RTO_MAX 900UL
#define MAX_DEFER 8
#define NAK_MIN_INTERVAL 1500UL
#define PING_INTERVAL 10000UL
#define LINK_DROP_TIMEOUT 30000UL
#define REMOTE_TIMEOUT 15000UL
#define LOCK_TIMEOUT 180000UL

#define MAX_NAME 12
#define MAX_CONTENT 32
#define MAX_PAYLOAD 88
#define FRAME_MAX 104
#define QUEUE_LEN 2
#define INBUF_LEN 80

const char EMPTY[] = "";

#define MAX_EEPROM_FILE 900
#define CHUNK_DATA 72  // XDAT payload = 13 + data; MAX_PAYLOAD 88 -> 72 data bytes
#define XFER_TIMEOUT 10000UL

struct XferRx {
  bool active, upload;
  char name[MAX_NAME + 1];
  uint8_t nl;
  uint16_t total, got;
  int base;
  unsigned long last;
} xr;

struct XferTx {
  bool active, begun, ready;
  char name[MAX_NAME + 1];
  uint16_t total, sent, acked, qlen;   // acked = bytes confirmed by the peer
  int addr;
  unsigned long last;
} xs;

// Background EEPROM writer for received chunks (see eeBgPump). The chunk is
// copied here, the ACK goes out, and the bytes are stored while the next
// chunk is on the air.
uint8_t ewBuf[CHUNK_DATA];
uint8_t ewLen = 0, ewPos = 0;
int ewAddr = 0;

// Base91 reader state for 'view'. The Base91 stream is decoded
// directly from EEPROM, so the compressed image never needs a RAM buffer.
struct B91R {
  int addr;
  int end;
  int16_t pair;
  uint32_t acc;
  uint8_t bits;
};

// View decoder state. Must be declared up here (before the first function):
// the Arduino IDE auto-generates prototypes at the top of the file, so any
// struct used in a function signature has to be defined before them.
struct ViewDec {
  B91R b;
  uint32_t range, code;
  bool bad;       // invalid Base91 character seen
  uint8_t last;   // previous pixel differed from its left neighbour
  uint16_t pA[10], pB[3], pS[3], pM[14];
};

void eepromTouch();
void peekEnd();
void dropRemote();
void printPrompt();

// --------------------------------------------------------------------
// Shared strings & output helpers: every repeated text is stored once
// (colour codes live here once, not in every string)
// --------------------------------------------------------------------
const char S_eeprom[] PROGMEM = "eeprom";
const char S_net[] PROGMEM = "net";
const char S_rf[] PROGMEM = "-rf";
const char S_TXQ[] PROGMEM = "TX queue full.";
#define FS(p) ((const __FlashStringHelper*)(p))

// --------------------------------------------------------------------
// Flash-shared UI strings. Stored once in PROGMEM and reused through
// FS()/queueFmt() instead of creating another PSTR/F() object per call site.
// --------------------------------------------------------------------
const char S_ANSI_RESET[] PROGMEM = ANSI_RESET;
const char S_ANSI_GREEN[] PROGMEM = ANSI_GREEN;
const char S_ANSI_YELLOW[] PROGMEM = ANSI_YELLOW;
const char S_ANSI_CYAN[] PROGMEM = ANSI_CYAN;
const char S_ANSI_WHITE[] PROGMEM = ANSI_WHITE;
const char S_ANSI_QUOTE_CYAN[] PROGMEM = "'" ANSI_CYAN;
const char S_ANSI_YELLOW_CLOSE_RESET[] PROGMEM = ANSI_YELLOW "]" ANSI_RESET;

const char S_ERR_BUSY[] PROGMEM = "ERR:BUSY";
const char S_DELETED[] PROGMEM = "Deleted ";
const char S_UPLOADING[] PROGMEM = "Uploading";
const char S_SAVED[] PROGMEM = "Saved ";
const char S_TO_EEPROM[] PROGMEM = " to EEPROM.";
const char S_TRUNCATED[] PROGMEM = "Truncated image.";
const char S_PERIOD[] PROGMEM = ".";
const char S_BUSY[] PROGMEM = "Busy: waiting...";
const char S_NOTFOUND[] PROGMEM = "File not found.";
const char S_XFER_CANCEL[] PROGMEM = "\n[Transfer] Cancelled.";
const char S_XFER_TIMEOUT[] PROGMEM = "\n[Transfer] Timeout aborted.";
const char S_XFER_SYNC[] PROGMEM = "\n[Transfer Error] Sync fail.";

const char S_FROM_EE[] PROGMEM = " from EEPROM.";
const char S_LIST_FILES[] PROGMEM = "List files";
const char S_DOWNLOADING[] PROGMEM = "Downloading";
const char S_READING[] PROGMEM = "Reading";
const char S_DEL_REMOTE[] PROGMEM = "Deleting remote";
const char S_UP_CHUNK[] PROGMEM = "Uploading chunked";


// UTF-8 block glyphs (PROGMEM so they don't eat RAM)
const uint8_t GLYPH_FULL[] PROGMEM = {0xE2, 0x96, 0x88};  // █
const uint8_t GLYPH_UPPER[] PROGMEM = {0xE2, 0x96, 0x80}; // ▀
const uint8_t GLYPH_LOWER[] PROGMEM = {0xE2, 0x96, 0x84}; // ▄
static void writeGlyph(const uint8_t* g) {
  for (uint8_t i = 0; i < 3; i++) Serial.write(pgm_read_byte(&g[i]));
}



bool viewing = false;  // 'view' is drawing: notices go to the status area under the picture
const __FlashStringHelper* viewNote = nullptr;  // latest notice waiting for the status area
bool viewNoteNode = false;                      // prefix it with the peer's node name
uint16_t viewProgressDone = 0;
uint16_t viewProgressTotal = 0;

static void csi() { Serial.print(F("\033[")); }
static void clrLine() { Serial.print(F("\033[2K\033[1G")); }
static void clearScreen() { Serial.print(F("\033[r" ANSI_CLS)); }
static void endLine() { Serial.println(FS(S_ANSI_RESET)); }
static void errHead() { Serial.print(F(ANSI_RED "Error: ")); }
static void indent() { Serial.print(F("  ")); }
static void printB() { Serial.print(F(" B")); }
static void barLine() { Serial.println(F(ANSI_CYAN "================================" ANSI_RESET)); }
static void titleOpen() {
  barLine();
  Serial.print(F(ANSI_GREEN " Audio FTP Terminal " ANSI_YELLOW "["));
}
static void titleClose() {
  Serial.println(FS(S_ANSI_YELLOW_CLOSE_RESET));
}
static void printErr(const __FlashStringHelper* s) {
  if (viewing) return;  // never write into the picture
  Serial.print(F(ANSI_RED));
  Serial.print(s);
  endLine();
}
static void printErrMsg(const __FlashStringHelper* s) {
  if (viewing) return;
  Serial.print(F(ANSI_RED "\n[Error] "));
  Serial.print(s);
  endLine();
}

static void printE(const __FlashStringHelper* s) {
  if (viewing) return;
  errHead();
  Serial.print(s);
  endLine();
}
static void printWarn(const __FlashStringHelper* s) {
  if (viewing) return;
  Serial.print(FS(S_ANSI_YELLOW));
  Serial.print(s);
  endLine();
}

// Notices may arrive asynchronously while the normal command prompt is
// sitting on the current line. Clear that line first so we never leave a
// stale "ftp:/>" (or path prompt) in front of the notice.
static void noticeHead() {
  clrLine();
  Serial.print(F(ANSI_YELLOW "[Notice] "));
}
static void printNotice(const __FlashStringHelper* s) {
  if (viewing) {
    viewNote = s;
    viewNoteNode = false;
    return;
  }
  noticeHead();
  Serial.print(s);
  endLine();
}
static void printOk(const __FlashStringHelper* s) {
  if (viewing) return;
  Serial.print(FS(S_ANSI_GREEN));
  Serial.print(s);
  endLine();
}

static void noticePrompt(const __FlashStringHelper* s) {
  printNotice(s);
  printPrompt();
}
static void errPrompt(const __FlashStringHelper* s) {
  printErr(s);
  printPrompt();
}
static void errMsgPrompt(const __FlashStringHelper* s) {
  printErrMsg(s);
  printPrompt();
}
static void warnPrompt(const __FlashStringHelper* s) {
  printWarn(s);
  printPrompt();
}
static void okPrompt(const __FlashStringHelper* s) {
  printOk(s);
  printPrompt();
}

static void errNF() {
  printE(FS(S_NOTFOUND));
}

static void errQ() {
  printE(FS(S_TXQ));
}

static void errStore() {
  printE(F("Not in storage dir."));
}

static void errLimit() {
  printE(F("EEPROM file limit."));
}

// "<verb> 'name'..."
static void act(const __FlashStringHelper* v, const char* n) {
  Serial.print(v);
  Serial.print(F(" '"));
  Serial.print(n);
  Serial.println(F("'..."));
}

// green  <pre>'name'<post>
static void okName(const __FlashStringHelper* pre, const char* name, const __FlashStringHelper* post) {
  if (viewing) return;
  Serial.print(FS(S_ANSI_GREEN));
  Serial.print(pre);
  Serial.print(FS(S_ANSI_QUOTE_CYAN));
  Serial.print(name);
  Serial.print(F(ANSI_GREEN "'"));
  if (post) Serial.print(post);
  endLine();
}

bool progressActive = false;
unsigned long xferT0 = 0;   // transfer start time, for the bps readout
unsigned long xferT1 = 0;   // transfer end time (0 = still running); freezes bps

static void printProgressBar(uint16_t done, uint16_t total) {
  if (viewing) {
    // The picture owns the terminal rows. Save the newest transfer state and
    // draw it in the two-line status area after the current frame finishes.
    progressActive = true;
    viewProgressDone = done;
    viewProgressTotal = total;
    return;
  }
  const uint8_t width = 10;
  progressActive = true;

  uint8_t filled = 0;
  uint8_t percent = 0;

  if (total > 0) {
    // MAX_EEPROM_FILE is 900, so done*10 stays within uint16_t.
    // Compute the exact percentage in two 16-bit steps instead of
    // pulling the AVR's much larger 32-bit multiply/divide helpers.
    uint16_t x10 = (uint16_t)done * 10;
    filled = x10 / total;
    uint16_t rem = x10 % total;
    percent = (uint8_t)(filled * 10 + (uint16_t)(rem * 10) / total);
  }

  // Keep the progress display on ONE physical line.
  // Use ANSI horizontal positioning instead of CR. Some terminals
  // interpret CR as CR/LF, which would create a new line per update.
  // ESC[2K clears only the current line; ESC[1G moves to column 1.
  clrLine();

  Serial.print(F(ANSI_GREEN "["));
  Serial.print(FS(S_ANSI_CYAN));

  for (uint8_t i = 0; i < width; i++) {
    if (i < filled)
      writeGlyph(GLYPH_FULL);
    else
      Serial.write('=');
  }

  Serial.print(F(ANSI_GREEN "]" ANSI_WHITE " "));

  if (percent < 100) Serial.print(' ');
  if (percent < 10) Serial.print(' ');
  Serial.print(percent);
  Serial.print(F("%  "));

  Serial.print(done);
  Serial.print('/');
  Serial.print(total);
  printB();

  // Effective throughput: file data bits over wall-clock time (includes ACKs,
  // retries and the handshake). done <= 900, so done * 8000 fits in 32 bits.
  uint32_t ms = (xferT1 ? xferT1 : millis()) - xferT0;
  if (done && ms) {
    Serial.print(F("  "));
    Serial.print((uint32_t)done * 8000UL / ms);
    Serial.print(F(" bps"));
  }

  Serial.print(FS(S_ANSI_RESET));
}

// "[Notice] Transfer initiated." + empty bar (sender and receiver share it)
static void xferStart(uint16_t total) {
  xferT0 = millis();
  xferT1 = 0;
  printNotice(F("Transfer start."));
  printProgressBar(0, total);
}

// --------------------------------------------------------------------
// Tiny formatter (replaces snprintf/vsnprintf: saves the whole printf engine)
// Supports %s (RAM string), %u (unsigned), %d (int). Format lives in PROGMEM.
// --------------------------------------------------------------------
static uint8_t utoaDec(char* o, uint16_t v) {
  char t[5];
  uint8_t n = 0, k = 0;
  do {
    t[n++] = '0' + v % 10;
    v /= 10;
  } while (v);
  while (n) o[k++] = t[--n];
  return k;
}

static void vfmtP(char* out, size_t cap, const char* f, va_list ap) {
  char* o = out;
  char* e = out + cap - 1;
  char c;
  while ((c = pgm_read_byte(f++)) && o < e) {
    if (c != '%') {
      *o++ = c;
      continue;
    }
    c = pgm_read_byte(f++);
    if (!c) break;
    if (c == 's') {
      const char* s = va_arg(ap, const char*);
      while (*s && o < e) *o++ = *s++;
    } else {
      uint16_t v = (uint16_t)va_arg(ap, unsigned);
      if (e - o >= 5) o += utoaDec(o, v);
    }
  }
  *o = 0;
}

// Identity / state
const char* localNodeName = "";
const char* remoteNodeName = "";
bool nodeIdentified = false;
bool isNode1 = false;
int myRandomId = 0;
unsigned long lastHelloTime = 0;
unsigned long lastPeerActivityTime = 0;
unsigned long lastPingQueuedTime = 0;

Loc loc = L_ROOT;

enum Mode : uint8_t { M_MENU, M_OFFLINE, M_ONLINE };
Mode mode = M_MENU;

const uint8_t delayTX = DEFAULT_DELAY;  // fixed; change DEFAULT_DELAY and reflash both boards

bool remoteSpaceLocked = false;
bool localInRemoteSpace = false;
bool peeking = false;  // one-shot remote listing is borrowing the peer lock
unsigned long remoteLockTime = 0;
bool waitingForRemoteResponse = false;
unsigned long remoteRequestTime = 0;

char inBuf[INBUF_LEN + 2];
uint8_t inLen = 0;
bool inOverflow = false;
bool lastWasCR = false;
bool stActive = false;
uint8_t stErr = 0;  // put result: 0 ok, 1..3 eepromPutCap code, 255 = already reported
char stName[MAX_NAME + 1];

// Timing Helpers
uint8_t effDelay() {
  return delayTX < MIN_SAFE_DELAY ? MIN_SAFE_DELAY : delayTX;
}

uint16_t effGapUs() {
  return (uint16_t)effDelay() * 100U;
}

uint16_t silenceMs() {
  uint16_t s = (uint16_t)(((uint16_t)2 * effGapUs() + 999U) / 1000U) + SILENCE_MARGIN;
  return (s < SILENCE_MIN_MS) ? SILENCE_MIN_MS : s;
}

uint16_t interFrameGapMs() {
  return silenceMs() + FRAME_GAP_EXTRA;
}
uint16_t csQuietMs() {
  return (uint16_t)((effGapUs() + 999U) / 1000U) + CS_MARGIN;
}

static void copyStr(char* dst, const char* src, size_t cap) {
  size_t i = 0;
  while (i < cap && src[i]) { dst[i] = src[i]; i++; }
  dst[i] = 0;
}

static bool validName(const char* n) {
  size_t l = strlen(n);
  if (l == 0 || l > MAX_NAME) return false;
  for (; *n; n++) {
    char c = *n;
    if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
          c == '.' || c == '_' || c == '-')) return false;
  }
  return true;
}

// --------------------------------------------------------------------
// EEPROM wear levelling (address translation layer)
//
// Physical layout:  [0]=magic  [1..2]=rotation offset (lo,hi)  [3..]=data
// Logical addresses (what the record code uses) map to physical cells via
//   phys = EE_HDR + ((logical + eeOff) % eeLen())
// eeRotate() cyclically shifts the whole data area and bumps eeOff, so the
// logical hot spots (terminator, low addresses) move to different cells.
// ALL store access must use eeGet/eePut/eeLen, never EEPROM.* directly
// (except inside the header / rotation code).
// --------------------------------------------------------------------
#define EEPROM_MAGIC 0x5C   // reformats stores that lack wear-level header
#define EE_HDR 3
#define EE_WEAR_ODDS 40     // on average one rotation per ~40 modifying ops

uint16_t eeOff = 0;
bool eeRotatePending = false;

inline uint16_t eeLen() { return EEPROM.length() - EE_HDR; }
inline uint16_t eePhys(uint16_t a) { return EE_HDR + (uint16_t)(a + eeOff) % eeLen(); }
uint8_t eeGet(int a) { return EEPROM.read(eePhys(a)); }
void eePut(int a, uint8_t v) { EEPROM.update(eePhys(a), v); }

// --------------------------------------------------------------------
// EEPROM store.
// Every record: [nl][name][0xFF][cl_lo][cl_hi][content].
// --------------------------------------------------------------------
bool eepromRemove(const char* n);

uint16_t recContentLen(int addr, uint8_t nl) {
  return eeGet(addr + 2 + nl) | ((uint16_t)eeGet(addr + 3 + nl) << 8);
}

uint16_t recSize(int addr, uint8_t nl) {
  return 4 + nl + recContentLen(addr, nl);
}

uint16_t recContentOffset(int, uint8_t nl) {
  return 4 + nl;
}

// Is there a valid record at addr?  Fills name length / content length.
bool recAt(int addr, uint8_t* nl, uint16_t* cl) {
  if (addr >= (int)eeLen()) return false;
  uint8_t n = eeGet(addr);
  if (n == 0 || n == 0xFF || n > MAX_NAME || eeGet(addr + 1 + n) != 0xFF) return false;
  uint16_t c = recContentLen(addr, n);
  if (c > MAX_EEPROM_FILE) return false;
  uint16_t end = (uint16_t)addr + recSize(addr, n);
  if (end > eeLen()) return false;
  *nl = n;
  *cl = c;
  return true;
}

// Read n bytes from EEPROM into d and NUL-terminate (d needs n+1 bytes).
void eeRead(int a, uint16_t n, char* d) {
  for (uint16_t i = 0; i < n; i++) d[i] = eeGet(a + i);
  d[n] = 0;
}

void eepromFormat() {
  eepromTouch();
  eePut(1, 0);                  // empty record list; rotation offset is kept on purpose
  EEPROM.update(0, EEPROM_MAGIC);
}

int eepromEnd() {
  int a = 1;
  uint8_t nl;
  uint16_t cl;
  while (recAt(a, &nl, &cl)) a += recSize(a, nl);
  int total = eeLen();
  return a < total ? a : total;
}

int getFreeEEPROMBytes() {
  return (int)eeLen() - eepromEnd();
}

int eepromFind(const char* name, uint8_t* outNameLen, uint16_t* outContentLen, int* outContentAddr) {
  int a = 1;
  uint8_t nl;
  uint16_t cl;
  uint8_t nameLen = strlen(name);

  while (recAt(a, &nl, &cl)) {
    bool match = (nl == nameLen);
    if (match) {
      for (uint8_t i = 0; i < nl; i++) {
        if (eeGet(a + 1 + i) != (uint8_t)name[i]) {
          match = false;
          break;
        }
      }
    }
    if (match) {
      if (outNameLen) *outNameLen = nl;
      if (outContentLen) *outContentLen = cl;
      if (outContentAddr) *outContentAddr = a + recContentOffset(a, nl);
      return a;
    }
    a += recSize(a, nl);
  }
  return -1;
}

// Write "[nl][name][0xFF][cl_lo][cl_hi]" header at a
void recHeader(int a, const char* name, uint8_t nl, uint16_t cl) {
  eePut(a, nl);
  for (uint8_t i = 0; i < nl; i++) eePut(a + 1 + i, name[i]);
  eePut(a + 1 + nl, 0xFF);
  eePut(a + 2 + nl, (uint8_t)(cl & 0xFF));
  eePut(a + 3 + nl, (uint8_t)(cl >> 8));
}

// ---- Unix-style listing helpers ----
void padNum(int v, uint8_t w) {
  uint8_t d = 1;
  for (int t = v; t >= 10; t /= 10) d++;
  while (d++ < w) Serial.print(' ');
  Serial.print(v);
}

// type: 'd' directory, '-' file, 'l' link
void lsHead(char type, int size) {
  indent();
  Serial.print(type);
  Serial.print(' ');
  if (type == '-') padNum(size, 5);
  else Serial.print(F("    -"));
  indent();
  if (type == 'd') Serial.print(FS(S_ANSI_CYAN));
  else if (type == 'l') Serial.print(F(ANSI_MAGENTA));
  else Serial.print(FS(S_ANSI_WHITE));
}

void lsTail(char type) {
  if (type == 'd') Serial.print('/');
  else if (type == 'l') Serial.print(F(" -> /"));
  endLine();
}

void printLsRow(char type, const char* name, int size) {
  lsHead(type, size);
  Serial.print(name);
  lsTail(type);
}

void dirRow(const __FlashStringHelper* n) {
  lsHead('d', 0);
  Serial.print(n);
  lsTail('d');
}

void printCountBytes(uint8_t n, int bytes) {
  Serial.print(n);
  Serial.print(n == 1 ? F(" file, ") : F(" files, "));
  Serial.print(bytes);
  printB();
}

// "  (empty)" + "  N files, M B, "  (caller prints what follows)
void listFoot(uint8_t files, int bytes) {
  if (!files) printWarn(F("  (empty)"));
  indent();
  printCountBytes(files, bytes);
  Serial.print(F(", "));
}

// ... "N B free"
void listFootB(uint8_t files, int bytes, int freeB) {
  listFoot(files, bytes);
  Serial.print(freeB);
  printB();
  Serial.println(F(" free"));
}

void printEEPROMList() {
  int a = 1;
  uint8_t nl, files = 0;
  uint16_t cl;
  int bytes = 0;
  char name[MAX_NAME + 1];
  while (recAt(a, &nl, &cl)) {
    eeRead(a + 1, nl, name);
    printLsRow('-', name, cl);
    files++;
    bytes += cl;
    a += recSize(a, nl);
  }
  listFootB(files, bytes, getFreeEEPROMBytes());
}

uint8_t eepromPutCap(const char* n, const char* c, size_t cap) {
  if (!validName(n)) return 1;
  size_t cl = strlen(c);
  if (cl > cap) return 2;
  eepromTouch();
  uint8_t nl = strlen(n);
  int neededSpace = 4 + nl + cl;

  eepromRemove(n);
  if (getFreeEEPROMBytes() < neededSpace) return 3;

  int addr = eepromEnd();
  recHeader(addr, n, nl, cl);
  for (size_t i = 0; i < cl; i++) eePut(addr + 4 + nl + i, c[i]);
  if (addr + neededSpace < (int)eeLen()) eePut(addr + neededSpace, 0);
  return 0;
}

uint8_t eepromPut(const char* n, const char* c) {
  return eepromPutCap(n, c, MAX_CONTENT);
}

bool eepromRemove(const char* n) {
  uint8_t nl = 0;
  uint16_t cl = 0;
  int foundAddr = eepromFind(n, &nl, &cl, nullptr);
  if (foundAddr < 0) return false;
  eepromTouch();

  int recordSize = recSize(foundAddr, nl);
  int end = eepromEnd();
  for (int src = foundAddr + recordSize, dst = foundAddr; src < end; src++, dst++)
    eePut(dst, eeGet(src));
  int newEnd = end - recordSize;
  if (newEnd < (int)eeLen()) eePut(newEnd, 0);
  return true;
}

void loadEEPROM_Storage() {
  if (EEPROM.read(0) == EEPROM_MAGIC) {
    eeOff = EEPROM.read(1) | ((uint16_t)EEPROM.read(2) << 8);
    if (eeOff >= eeLen()) eeOff = 0;
  } else {
    // magic == 0 means a wear-levelling rotation was cut short (power loss)
    if (!EEPROM.read(0)) printWarn(F("[EEPROM] Wear reset."));
    eeOff = 0;
    EEPROM.update(1, 0);  // offset first, magic last (eepromFormat writes magic)
    EEPROM.update(2, 0);
    eepromFormat();
  }
}

static void printPutError(uint8_t code) {
  errHead();
  if (code == 1) {
    Serial.print(F("Bad name (max "));
    Serial.print(MAX_NAME);
    Serial.print(F(": a-z0-9._-)."));
  } else if (code == 2) {
    Serial.print(F("Content too long (max "));
    Serial.print(MAX_CONTENT);
    Serial.print(F(")."));
  } else {
    Serial.print(F("Storage full."));
  }
  endLine();
}

void sanitize(char* s) {
  char* w = s;
  bool pendingSpace = false;
  for (char* r = s; *r; r++) {
    unsigned char c = *r;
    if (c < 33 || c > 126) {
      pendingSpace = (w != s);
    } else {
      if (pendingSpace) {
        *w++ = ' ';
        pendingSpace = false;
      }
      *w++ = c;
    }
  }
  *w = 0;
}

char* nextField(char** p) {
  char* s = *p ? *p : EMPTY;
  char* c = strchr(s, ':');
  if (c) {
    *c = 0;
    *p = c + 1;
  } else {
    *p = nullptr;
  }
  return s;
}

uint16_t crc16(const char* d, uint8_t n) {
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= (uint16_t)(uint8_t)(*d++) << 8;
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
  }
  return crc;
}

bool frameCrcOk(const char* f, uint8_t len) {
  if (len < 8 || f[len - 5] != '#') return false;
  uint16_t v = 0;
  for (uint8_t i = len - 4; i < len; i++) {
    char c = f[i];
    uint8_t d;
    if (c >= '0' && c <= '9') d = c - '0';
    else if ((c | 32) >= 'a' && (c | 32) <= 'f') d = (c | 32) - 'a' + 10;
    else return false;
    v = (v << 4) | d;
  }
  return v == crc16(f, len - 5);
}

#define AL_COUNT 32
// Packed protocol dictionary: only wire tokens still used by the core remain.
const char AL_LONG[] PROGMEM = "HELLO\0HACK\0ACK\0NAK\0PING\0LOCK\0LKOK\0LKNO\0UNLK\0LS:eeprom\0LSD:eeprom\0CAT:eeprom\0CATD\0GET:eeprom\0GETD\0RM:eeprom\0PUT:eeprom\0OK:Saved\0OK:Deleted\0OK:EEPROM formatted\0ERR:NOLOCK\0ERR:DENIED\0ERR:NOTFOUND\0ERR:BADNAME\0ERR:TOOLONG\0ERR:FULL\0ERR\0XBEG\0XDAT\0XABT\0XGO\0APP\0";
const uint8_t AL_OFF[AL_COUNT] PROGMEM = {0,6,11,15,19,24,29,34,39,44,54,65,76,81,92,97,107,118,127,138,158,169,180,193,205,217,226,230,235,240,245,249};
const char AL_SHORT[AL_COUNT + 1] PROGMEM = "HYANPKOXUCcGgRTDVvfLdFbtlEBZzWJa";

// The dictionary is the ONLY place the protocol op names are stored.
#define ALP(i) (AL_LONG + pgm_read_byte(&AL_OFF[i]))
// Indices into AL_LONG (keep in sync with its order). O_* = op names,
// A_* = full dictionary strings that are queued as-is.
enum : uint8_t {
  O_HELLO = 0, O_HACK = 1, O_ACK = 2, O_NAK = 3, O_PING = 4, O_LOCK = 5, O_LKOK = 6, O_LKNO = 7, O_UNLK = 8,
  O_LS = 9, O_LSD = 10, O_CAT = 11, O_CATD = 12, O_GET = 13, O_GETD = 14, O_RM = 15, O_PUT = 16,
  O_OK = 17, A_Saved = 17, A_Deleted = 18, A_OKFMT = 19,
  O_ERR = 20, A_NOLOCK = 20, A_DENIED = 21, A_NOTFOUND = 22, A_BADNAME = 23, A_TOOLONG = 24, A_FULL = 25,
  O_XBEG = 27, O_XDAT = 28, O_XABT = 29, O_XGO = 30,
  O_APP = 31  // generic app message "APP:<app id>:<data>" (sent as alias 'a', no tone slot needed)
};
static_assert(CORE_OP_APP == O_APP, "CORE_OP_APP (core_api.h) out of sync with the dictionary");
static_assert(CORE_APP_MAX_DATA == MAX_PAYLOAD - 6, "CORE_APP_MAX_DATA (core_api.h) out of sync");

// Opcodes owned by apps: delivered to apps, delivery result reported to apps,
// never reported by the core as shell errors.
static inline bool isAppOp(int8_t id) {
  return id == O_APP;
}

// --------------------------------------------------------------------
// Opcode tokens: one tone per protocol opcode (see header, v4.27).
// slot -> dictionary index. Slots 0..8 = layer 8 (session/control),
// 9..20 = layer 9 (file ops), 21..24 = layer 7 idx 3..6 (transfer).
// Opcodes not listed here (OK:EEPROM formatted, ERR:DENIED, ERR:BADNAME,
// ERR:TOOLONG, ERR:FULL, ERR) still use the one-character alias.
// --------------------------------------------------------------------
const uint8_t OP_TOK[OP_SLOTS] PROGMEM = {
  O_HELLO, O_HACK, O_ACK, O_NAK, O_PING, O_LOCK, O_LKOK, O_LKNO, O_UNLK,
  O_LS, O_LSD, O_CAT, O_CATD, O_GET, O_GETD, O_RM, O_PUT, A_Saved, A_Deleted, A_NOLOCK, A_NOTFOUND,
  O_XBEG, O_XDAT, O_XABT, O_XGO
};

// slot -> (layer, tone index within that layer)
void opPos(uint8_t s, uint8_t* tl, uint8_t* ti) {
  if (s < 9) {
    *tl = OP_LAYER_A;
    *ti = s;
  } else if (s < 21) {
    *tl = OP_LAYER_B;
    *ti = s - 9;
  } else {
    *tl = CMD_LAYER;
    *ti = s - 21 + 3;
  }
}

// (layer, tone index) -> slot, or -1 if that tone carries no opcode
int8_t opSlotOf(uint8_t layer, uint8_t idx) {
  if (layer == OP_LAYER_A && idx < 9) return idx;
  if (layer == OP_LAYER_B && idx < 12) return 9 + idx;
  if (layer == CMD_LAYER && idx >= 3 && idx <= 6) return 21 + idx - 3;
  return -1;
}

// Index of the op name (text up to ':') in AL_LONG, or -1.
// First match wins, which is what the O_* values above rely on.
int8_t opId(const char* op) {
  for (uint8_t k = 0; k < AL_COUNT; k++) {
    const char* lp = ALP(k);
    uint8_t i = 0;
    char c;
    while ((c = pgm_read_byte(lp + i)) && c != ':' && op[i] == c) i++;
    if ((!c || c == ':') && (op[i] == 0 || op[i] == ':')) return k;
  }
  return -1;
}

static inline bool isTerm(char c) {
  return c == 0 || c == ':' || c == ' ';
}

char shortFor(const char* p, uint8_t* longLen) {
  char best = 0;
  uint8_t bl = 0;
  for (uint8_t k = 0; k < AL_COUNT; k++) {
    const char* lp = ALP(k);
    uint8_t l = strlen_P(lp);
    if (l > bl && strncmp_P(p, lp, l) == 0 && isTerm(p[l])) {
      best = (char)pgm_read_byte(&AL_SHORT[k]);
      bl = l;
    }
  }
  *longLen = bl;
  return best;
}

// Does the payload start with a tokenised opcode? Returns its slot (and the
// long name length) or -1. Must come after isTerm().
int8_t opSlotFor(const char* p, uint8_t* longLen) {
  for (uint8_t s = 0; s < OP_SLOTS; s++) {
    const char* lp = ALP(pgm_read_byte(&OP_TOK[s]));
    uint8_t l = strlen_P(lp);
    if (strncmp_P(p, lp, l) == 0 && isTerm(p[l])) {
      *longLen = l;
      return s;
    }
  }
  return -1;
}

// Expand a one-character protocol alias directly inside the receive buffer.
// rxBuf already has enough room for the fully expanded MAX_PAYLOAD message,
// so this avoids a second 81-byte stack buffer in handleFrame().
void expandAliasInPlace(char* s) {
  if (!s[0] || !isTerm(s[1])) return;

  for (uint8_t k = 0; k < AL_COUNT; k++) {
    if ((char)pgm_read_byte(&AL_SHORT[k]) == s[0]) {
      const char* lp = ALP(k);
      uint8_t l = strlen_P(lp);
      uint8_t tail = strlen(s + 1);
      if (l + tail > MAX_PAYLOAD) tail = MAX_PAYLOAD - l;

      // Make room for the long command, including the terminating NUL.
      memmove(s + l, s + 1, tail + 1);
      for (uint8_t i = 0; i < l; i++) s[i] = pgm_read_byte(lp + i);
      return;
    }
  }
}

// Tone dictionary tokens packed to avoid a 6-entry flash pointer table.
const char CMD_STR[] PROGMEM = ".txt\0.cfg\0.dat\0.log\0readme\0config\0";
const uint8_t CMD_OFF[CMD_COUNT] PROGMEM = {0,5,10,15,20,27};

int8_t matchCmd(const char* s, uint8_t* outLen) {
  int8_t best = -1;
  uint8_t bl = 0;
  for (uint8_t k = 0; k < CMD_COUNT; k++) {
    const char* p = CMD_STR + pgm_read_byte(&CMD_OFF[k]);
    uint8_t l = strlen_P(p);
    if (l > bl && strncmp_P(s, p, l) == 0) {
      best = k;
      bl = l;
    }
  }
  *outLen = bl;
  return best;
}

// --------------------------------------------------------------------
// Receiver buffer + Timer1 input capture (D8 / ICP1)
//
// Timer1 free-runs at clk/8 (0.5 us per tick at 16 MHz). Every rising edge on
// D8 latches the counter in hardware (ICR1), so ISR latency does not matter.
// rbData holds the raw tick count between consecutive rising edges
// (0xFFFF = longer than one counter wrap, i.e. a gap).
// While transmitting, Timer1 is switched to a CTC toggle on D9 (OC1A);
// rxTimerStart() restores capture mode afterwards.
// --------------------------------------------------------------------
#define RB_SIZE 64
volatile uint16_t rbData[RB_SIZE];
volatile uint8_t rbHead = 0, rbTail = 0;
volatile uint8_t t1Ovf = 0;
volatile uint16_t t1Last = 0;

#define T1_HZ (F_CPU / 8UL)
#define US_TO_TICKS(us) ((uint16_t)(((uint32_t)(us) * (F_CPU / 1000000UL)) / 8UL))

ISR(TIMER1_OVF_vect) {
  if (t1Ovf < 2) t1Ovf++;
}

ISR(TIMER1_CAPT_vect) {
  uint16_t now = ICR1;
  uint16_t last = t1Last;
  uint16_t d = now - last;
  uint8_t ov = t1Ovf;
  if ((TIFR1 & _BV(TOV1)) && now < 0x8000) {  // wrapped before this capture, OVF ISR not run yet
    ov++;
    TIFR1 = _BV(TOV1);                         // so it isn't counted twice
  }
  t1Ovf = 0;
  t1Last = now;
  if (ov > 1 || (ov == 1 && now >= last)) d = 0xFFFF;  // more than one wrap: a gap
  uint8_t n = (rbHead + 1) & (RB_SIZE - 1);
  if (n != rbTail) {
    rbData[rbHead] = d;
    rbHead = n;
  }
}

void rxTimerStart() {
  TCCR1A = 0;
  TCCR1B = _BV(ICNC1) | _BV(ICES1) | _BV(CS11);  // noise canceler, rising edge, clk/8
  TCNT1 = 0;
  t1Last = 0;
  t1Ovf = 0;
  TIFR1 = _BV(ICF1) | _BV(TOV1);
  TIMSK1 = _BV(ICIE1) | _BV(TOIE1);
}

// Hardware tone on D9 (OC1A): CTC toggle, exact to ~0.2 Hz, no software jitter.
// The pin starts LOW, so the first rising edge comes half a period after start.
void txToneOn(uint16_t f) {
  OCR1A = (uint16_t)(((F_CPU / 2UL) + f / 2) / f) - 1;
  TCNT1 = 0;
  TCCR1A = _BV(COM1A0);               // toggle OC1A on compare match
  TCCR1B = _BV(WGM12) | _BV(CS10);    // CTC, clk/1
}

void txToneOff() {
  TCCR1B = 0;
  TCCR1A = 0;
  PORTB &= ~_BV(PB1);                 // D9 low
}

char rxBuf[FRAME_MAX + 1];
uint8_t rxLen = 0;
bool rxOverflow = false;
char rxLast = 0;
int16_t lastConfirmedTok = TOK_NONE;
uint8_t receiverLayer = 0;
bool messageInProgress = false;
unsigned long lastValidSignalTime = 0;
unsigned long lastPulseMs = 0;
unsigned long lastNakTime = 0;
uint8_t lastRxSeq = 0;
bool skipTone = false;
uint8_t earlyLen = 0;

void resetDecoder() {
  lastConfirmedTok = TOK_NONE;
  receiverLayer = 0;
}

void flushRx() {
  noInterrupts();
  rbTail = rbHead;
  interrupts();
  resetDecoder();
  messageInProgress = false;
  rxLen = 0;
  rxOverflow = false;
  rxLast = 0;
  skipTone = false;
  earlyLen = 0;
  lastPulseMs = lastValidSignalTime = millis();
}

void rxAppend(char c) {
  if (rxLen < FRAME_MAX) rxBuf[rxLen++] = c;
  else rxOverflow = true;
  rxLast = c;
}

void expandCmd(uint8_t k) {
  if (k >= CMD_COUNT) return;
  const char* p = CMD_STR + pgm_read_byte(&CMD_OFF[k]);
  char c;
  while ((c = (char)pgm_read_byte(p++))) rxAppend(c);
  rxLast = 0;
}

// Opcode token -> the long opcode name (the same text the sender CRC'd)
void expandOp(uint8_t s) {
  if (s >= OP_SLOTS) return;
  const char* p = ALP(pgm_read_byte(&OP_TOK[s]));
  char c;
  while ((c = (char)pgm_read_byte(p++))) rxAppend(c);
  rxLast = 0;
}

void applyToken(int16_t t) {
  if (t == TOK_PREAMBLE) {
    rxLen = 0;
    rxOverflow = false;
    rxLast = 0;
    receiverLayer = 0;
  } else if (t <= -10) {
    receiverLayer = -10 - t;
  } else if (t == TOK_REPEAT) {
    if (rxLast && rxLast != ' ') rxAppend(rxLast);
  } else if (t == TOK_SPACE) {
    rxAppend(' ');
  } else if (t >= TOK_OP0) {   // must be tested before TOK_CMD0 (400 > 300)
    expandOp(t - TOK_OP0);
  } else if (t >= TOK_CMD0) {
    expandCmd(t - TOK_CMD0);
  } else {
    rxAppend((char)t);
  }
}

// 'ticks' = Timer1 ticks between two rising edges (see capture ISR above).
void processPulse(uint16_t ticks) {
  if (ticks < US_TO_TICKS(100)) return;
  if (skipTone) {
    if (ticks > US_TO_TICKS(GAP_PERIOD_US)) skipTone = false;
    return;
  }
  if (ticks > US_TO_TICKS(GAP_PERIOD_US)) {
    lastConfirmedTok = TOK_NONE;
    return;
  }

  uint16_t f = (uint16_t)((T1_HZ + ticks / 2) / ticks);  // Hz, rounded
  int16_t tok = TOK_NONE;

  if (f >= FREQ_PREAMBLE - CTL_TOL && f < BAND_SPLIT) {
    uint8_t k = (f - (FREQ_PREAMBLE - 30)) / 60;
    int center = FREQ_PREAMBLE + k * 60;
    if ((int)f - center <= CTL_TOL && center - (int)f <= CTL_TOL) {
      if (k == 0) tok = TOK_PREAMBLE;
      else if (k == 1) tok = TOK_SPACE;
      else if (k == 2) tok = TOK_REPEAT;
      else tok = -10 - (k - 3);
    }
  } else if (f >= BAND_SPLIT && f <= DATA_TOP) {
    uint8_t idx = (f - (LAYER_BASE - LAYER_STEP / 2)) / LAYER_STEP;
    int center = LAYER_BASE + idx * LAYER_STEP;
    if (idx < 13 && (int)f - center <= DATA_TOL && center - (int)f <= DATA_TOL) {
      int asc = 33 + receiverLayer * 13 + idx;
      if (asc <= 126) tok = asc;
      else if (receiverLayer == CMD_LAYER && idx >= CMD_IDX0)
        tok = TOK_CMD0 + (idx - CMD_IDX0);
      else {
        int8_t s = opSlotOf(receiverLayer, idx);
        if (s >= 0) tok = TOK_OP0 + s;
      }
    }
  }
  if (tok == TOK_NONE) return;

  lastValidSignalTime = millis();
  if (tok != lastConfirmedTok) {
    applyToken(tok);
    lastConfirmedTok = tok;
    messageInProgress = true;
  }
}

// Transmitter
unsigned long txNotBefore = 0;
uint8_t deferCount = 0;

static void waitUs(uint16_t us) {
  if (us >= 1000) { delay(us / 1000); us %= 1000; }
  if (us) delayMicroseconds(us);
}

// Tone length = whole number of periods, so the receiver always sees the
// same number of edges regardless of pitch or TX gap.
static uint16_t toneUsFor(uint16_t f, uint8_t periods) {
  return (uint16_t)(((uint32_t)periods * 1000000UL + f / 2) / f);
}

// Layer-shift tone followed by the post-shift silence
static void txShift(uint8_t to, uint16_t postUs) {
  uint16_t f = SHIFT_BASE + to * SHIFT_STEP;
  txToneOn(f);
  waitUs(toneUsFor(f, CTL_PERIODS));
  txToneOff();
  waitUs(postUs);
}

void transmitFrame(uint8_t seq, const char* payload) {
  char frame[FRAME_MAX];

  // "<seq> <opcode><rest>#XXXX"  built by hand (no printf).
  // Tokenised opcodes stay in LONG form inside the frame (the CRC covers the
  // long name) and are sent as ONE tone; other opcodes use the 1-char alias.
  uint8_t ll = 0, opLen = 0;
  int8_t opSlot = opSlotFor(payload, &opLen);
  char sc = (opSlot < 0) ? shortFor(payload, &ll) : 0;
  int n = utoaDec(frame, seq);
  frame[n++] = ' ';
  int opStart = n;  // index of the first opcode character in the frame
  if (sc) {
    frame[n++] = sc;
    payload += ll;
  }
  int room = (int)sizeof(frame) - 6 - n;
  int pl = strlen(payload);
  if (pl > room) pl = room;
  memcpy(frame + n, payload, pl);
  n += pl;
  uint16_t crc = crc16(frame, n);
  frame[n] = '#';
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t d = (crc >> (12 - 4 * i)) & 15;
    frame[n + 1 + i] = d < 10 ? '0' + d : 'A' + d - 10;
  }
  n += 5;
  frame[n] = 0;

  uint16_t gapUs = (uint16_t)((delayTX < MIN_SAFE_DELAY) ? MIN_SAFE_DELAY : delayTX) * 100;
  uint16_t postShiftUs = (uint16_t)(((uint32_t)gapUs * POST_SHIFT_NUM) / POST_SHIFT_DEN);

  // Timer1 leaves capture mode (RX) and becomes the hardware tone generator.
  TIMSK1 = 0;
  TCCR1B = 0;
  TCCR1A = 0;
  pinMode(TX_PIN, OUTPUT);
  PORTB &= ~_BV(PB1);

  txToneOn(FREQ_PREAMBLE);
  delay(PREAMBLE_MS);
  txToneOff();

  char last = 0;
  uint8_t layer = 0;
  int8_t lastTok = -1;
  for (int i = 0; i < n;) {
    char c = frame[i];
    int freq;
    uint8_t adv = 1, cl = 0;
    bool isOp = (opSlot >= 0 && i == opStart);
    int8_t k = (!isOp && i < n - 5) ? matchCmd(frame + i, &cl) : -1;
    bool isTok = (k >= 0 && k != lastTok);

    if (isOp) {
      uint8_t tl, ti;
      opPos(opSlot, &tl, &ti);
      if (tl != layer) {
        txShift(tl, postShiftUs);
        layer = tl;
      }
      freq = LAYER_BASE + ti * LAYER_STEP;
      adv = opLen;
    } else if (isTok) {
      if (layer != CMD_LAYER) {
        txShift(CMD_LAYER, postShiftUs);
        layer = CMD_LAYER;
      }
      freq = LAYER_BASE + (CMD_IDX0 + k) * LAYER_STEP;
      adv = cl;
    } else if (c == ' ') {
      freq = FREQ_SPACE;
    } else if (c == last) {
      freq = FREQ_REPEAT;
    } else if (c >= 33 && c <= 126) {
      uint8_t off = c - 33, tl = off / 13, ti = off % 13;
      if (tl != layer) {
        txShift(tl, postShiftUs);
        layer = tl;
      }
      freq = LAYER_BASE + ti * LAYER_STEP;
    } else {
      i++;
      continue;
    }
    txToneOn(freq);
    waitUs(toneUsFor(freq, freq >= LAYER_BASE ? DATA_PERIODS : CTL_PERIODS));
    txToneOff();
    waitUs(gapUs);
    if (isOp) {
      last = 0;
      lastTok = -1;
    } else if (isTok) {
      last = 0;
      lastTok = k;
    } else {
      last = c;
      lastTok = -1;
    }
    i += adv;
  }

  txToneOff();
  pinMode(TX_PIN, INPUT);
  rxTimerStart();  // back to capture mode
  flushRx();
  txNotBefore = millis() + interFrameGapMs();
}

// Send a bare control frame (O_ACK / O_NAK) with the given seq
void sendCtl(uint8_t seq, uint8_t op) {
  char t[4];
  strcpy_P(t, ALP(op));
  transmitFrame(seq, t);
}

bool canTransmit() {
  unsigned long now = millis();
  if ((long)(now - txNotBefore) < 0) return false;
  bool clear = !messageInProgress && (now - lastPulseMs > csQuietMs());
  if (clear || deferCount >= MAX_DEFER) {
    deferCount = 0;
    return true;
  }
  deferCount++;
  txNotBefore = now + random(40, 250);
  return false;
}

// Send Queue
char txQ[QUEUE_LEN][MAX_PAYLOAD + 1];
uint8_t qHead = 0, qCount = 0;
bool txInFlight = false;
uint8_t txAttempts = 0;
uint8_t txSeq = 0;
uint8_t txSeqCounter = 1;
unsigned long txDeadline = 0;
unsigned long txRto = ACK_TIMEOUT;
unsigned long txSentAt = 0;

void runReceiverLogic();
void displayMenu();
bool core_apps_stream(char* data, uint8_t len, bool finish);

// ---------------- Paths ----------------
void printPathOf(Loc l, bool trailing) {
  if (l == L_ROOT) {
    Serial.print('/');
    return;
  }
  if (l == L_NETWORK || l >= L_REMOTE) {
    Serial.print('/');
    Serial.print(FS(S_net));
    if (l >= L_REMOTE) {
      Serial.print('/');
      Serial.print(remoteNodeName);
    }
  }
  if (l == L_EEPROM || l == L_REMOTE_EEPROM) {
    Serial.print('/');
    Serial.print(FS(S_eeprom));
  }
  if (trailing) Serial.print('/');
}

static const AppHooks* g_apps[CORE_MAX_APPS];
static uint8_t g_appCount = 0;
static int8_t g_activeApp = -1;  // app that owns the shell (-1 = none)

void printPrompt() {
  if (viewing) return;
  if (mode == M_MENU) {
    Serial.print(F(ANSI_CYAN "menu" ANSI_RESET "> "));
    return;
  }

  // OS-style shell identity instead of exposing the AFTP protocol name.
  // Online nodes resolve to node1/node2; offline keeps a stable local prompt.
  Serial.print(F(ANSI_CYAN));
  if (nodeIdentified && *localNodeName) Serial.print(localNodeName);
  else Serial.print(F("uno"));
  Serial.print(F("@uno" ANSI_RESET ":"));

  // Use a familiar shell-style path: ~, ~/eeprom, ~/net, ~/net/node2, ...
  if (loc == L_ROOT) {
    Serial.print('~');
  } else if (loc == L_EEPROM) {
    Serial.print(F("~/"));
    Serial.print(FS(S_eeprom));
  } else {
    Serial.print(F("~/"));
    Serial.print(FS(S_net));
    if (loc >= L_REMOTE) {
      Serial.print('/');
      Serial.print(remoteNodeName);
    }
    if (loc == L_REMOTE_EEPROM) {
      Serial.print('/');
      Serial.print(FS(S_eeprom));
    }
  }
  if (g_activeApp >= 0) {  // an app is open: show which one, so `exit` is visible
    Serial.print(F(ANSI_MAGENTA " ("));
    Serial.print(g_apps[g_activeApp]->name);
    Serial.print(')');
  }
  Serial.print(F(ANSI_YELLOW "$" ANSI_RESET " "));
}

bool rawNext = false;  // set around one queueFmt call to skip sanitize() (XDAT file data)

bool queueFmt(const char* f, ...) {
  if (qCount >= QUEUE_LEN) return false;
  uint8_t idx = (qHead + qCount) % QUEUE_LEN;

  va_list ap;
  va_start(ap, f);
  vfmtP(txQ[idx], MAX_PAYLOAD + 1, f, ap);
  va_end(ap);
  if (!rawNext) sanitize(txQ[idx]);
  qCount++;
  return true;
}
#define QF(fmt, ...) queueFmt(PSTR(fmt), ##__VA_ARGS__)
#define QA(i) queueFmt(ALP(i))  // queue a dictionary string as-is

// dictionary string + sep + a [+ ':' + b]   (e.g. "PUT:eeprom" ':' name ':' content)
bool queueAl2(uint8_t i, char sep, const char* a, const char* b) {
  if (qCount >= QUEUE_LEN) return false;
  char* q = txQ[(qHead + qCount) % QUEUE_LEN];
  strcpy_P(q, ALP(i));
  size_t n = strlen(q);
  for (uint8_t k = 0; k < 2; k++) {
    const char* s = k ? b : a;
    if (!s) break;
    if (n < MAX_PAYLOAD) q[n++] = k ? ':' : sep;
    while (*s && n < MAX_PAYLOAD) q[n++] = *s++;
    q[n] = 0;
  }
  sanitize(q);
  qCount++;
  return true;
}
bool queueAl(uint8_t i, char sep, const char* a) {
  return queueAl2(i, sep, a, nullptr);
}

void popHead() {
  qHead = (qHead + 1) % QUEUE_LEN;
  if (qCount) qCount--;
  txInFlight = false;
  txAttempts = 0;
  txSentAt = 0;
}

void clearQueue() {
  qHead = qCount = 0;
  txInFlight = false;
  txAttempts = 0;
  txSentAt = 0;
  txRto = ACK_TIMEOUT;
}

void failHead() {
  char op[6];
  uint8_t i = 0;
  const char* p = txQ[qHead];
  while (p[i] && p[i] != ':' && i < 5) {
    op[i] = p[i];
    i++;
  }
  op[i] = 0;
  int8_t id = opId(op);
  char aid = txQ[qHead][4];  // "APP:<id>" -> which app sent it
  popHead();

  if (id == O_XBEG || id == O_XDAT) xs.active = false;

  if (isAppOp(id)) {  // the app decides how to tell the user (no shell error/prompt inside an app UI)
    core_apps_tx_done((uint8_t)id, aid, false);
    return;
  }
  if (id == O_PING || id == O_UNLK || id == O_LKOK || id == O_LKNO || id == O_XABT) return;

  if (viewing) {
    printNotice(F("No ACK."));
  } else {
    Serial.println();
    Serial.print(F(ANSI_RED "[Error] No ACK (op: "));
    Serial.print(op);
    Serial.println(F(")." ANSI_RESET));
  }
  waitingForRemoteResponse = false;
  if (id == O_LOCK) dropRemote();
  else peekEnd();
  printPrompt();
}

void serviceTx() {
  if (qCount == 0) return;
  unsigned long now = millis();

  if (txInFlight) {
    if ((long)(now - txDeadline) < 0) return;
    if (txAttempts >= MAX_TRIES) {
      failHead();
      return;
    }
  }
  if (!canTransmit()) return;

  if (txAttempts == 0) {
    txSeqCounter = (txSeqCounter % 255) + 1;
    txSeq = txSeqCounter;
  }
  transmitFrame(txSeq, txQ[qHead]);
  txAttempts++;
  txInFlight = true;
  txSentAt = millis();
  txDeadline = txSentAt + txRto + random(0, ACK_JITTER);
}

// Transfers
bool xferKill(bool tellPeer) {
  bool was = xr.active || xs.active;
  xr.active = xs.active = false;
  ewLen = ewPos = 0;   // never let a stale chunk land in a later file
  if (was) {
    if (!xferT1) xferT1 = millis();
    waitingForRemoteResponse = false;
    if (tellPeer) QA(O_XABT);
  }
  return was;
}

// Called before every modifying EEPROM operation.
// Also randomly schedules a wear-levelling rotation (runs later, when idle).
void eepromTouch() {
  if (xferKill(true)) printWarn(F("\n[Transfer] Cancelled: EEPROM modified."));
  if (random(EE_WEAR_ODDS) == 0) eeRotatePending = true;
}

// Shared "transfer finished" banner
void xferDone(const __FlashStringHelper* pre,
              const char* name,
              uint16_t n,
              const __FlashStringHelper* post) {

  xferT1 = millis();   // freeze bps at the moment the transfer completes

  if (viewing) {  // picture on screen: notice + final 100% bar in the status area
    printProgressBar(n, n);
    printNotice(F("Transfer done."));
    return;
  }

  // Keep the final 100% progress line visible.
  // The cursor is already at the end of that line, so just move
  // to the next line before printing the completion banner.
  if (progressActive) {
    Serial.println();
    progressActive = false;
  }

  Serial.print(FS(S_ANSI_GREEN));
  Serial.print(pre);
  Serial.print(FS(S_ANSI_QUOTE_CYAN));
  Serial.print(name);
  Serial.print(F(ANSI_GREEN "' (" ANSI_WHITE));
  Serial.print(n);
  Serial.print(F(ANSI_GREEN " B)"));
  if (post) Serial.print(post);
  endLine();
}

void xrCommit() {
  int a = xr.base;
  recHeader(a, xr.name, xr.nl, xr.total);
  int end = a + 4 + xr.nl + xr.total;
  if (end < (int)eeLen()) eePut(end, 0);
  xr.active = false;

  if (xr.upload) {
    xferDone(F("[Remote] Received "), xr.name, xr.total, nullptr);
    queueAl(A_Saved, ' ', xr.name);
  } else {
    waitingForRemoteResponse = false;
    xferDone(F("[Download Success] "), xr.name, xr.total, F(" saved."));
  }
  printPrompt();
}

// Finish the pending chunk synchronously (blocks up to ~3.4 ms per byte).
void eeDrain() {
  while (ewPos < ewLen) {
    if (!xr.active) {
      ewLen = ewPos = 0;
      return;
    }
    eePut(ewAddr + ewPos, ewBuf[ewPos]);
    ewPos++;
  }
}

// Called from runReceiverLogic(): store pending bytes without ever waiting
// for the EEPROM, and commit the record once the last chunk is fully stored.
void eeBgPump() {
  if (ewPos < ewLen) {
    if (!xr.active) ewLen = ewPos = 0;
    else
      while (ewPos < ewLen && !(EECR & _BV(EEPE))) {
        eePut(ewAddr + ewPos, ewBuf[ewPos]);   // update() skips bytes that already match
        ewPos++;
      }
  }
  if (xr.active && xr.got == xr.total && ewPos >= ewLen) xrCommit();
}

uint8_t xrBegin(const char* name, int total, bool upload) {
  if (!validName(name)) return 1;
  if (total < 1 || total > MAX_EEPROM_FILE) return 2;
  eepromRemove(name);
  int nl = strlen(name);
  int base = eepromEnd();
  if (base + 4 + nl + total + 1 > (int)eeLen()) return 3;
  xr.active = true;
  ewLen = ewPos = 0;
  xr.upload = upload;
  copyStr(xr.name, name, MAX_NAME);
  xr.nl = nl;
  xr.total = total;
  xr.got = 0;
  xr.base = base;
  xr.last = millis();
  return 0;
}

uint8_t xrChunk(int off, const char* data, uint8_t len) {
  if (!xr.active) return 0;
  if (off < xr.got) return 0;
  if (len == 0 || off != xr.got || xr.got + len > xr.total) return 1;
  if (!data || strlen(data) < (size_t)len || data[len] != ':') return 1;

  eeDrain();  // normally already empty: handleFrame drains before it ACKs
  memcpy(ewBuf, data, len);
  ewAddr = xr.base + 4 + xr.nl + xr.got;
  ewLen = len;
  ewPos = 0;
  xr.got += len;
  xr.last = millis();

  printProgressBar(xr.got, xr.total);
  return 0;  // eeBgPump() stores the bytes and commits after the last chunk
}

bool xsStart(const char* name, int contentAddr, uint16_t len) {
  if (xs.active) return false;
  copyStr(xs.name, name, MAX_NAME);
  xs.total = len;
  xs.sent = 0;
  xs.acked = xs.qlen = 0;
  xs.addr = contentAddr;
  xs.begun = xs.ready = false;
  xs.last = millis();
  xs.active = true;
  xferStart(xs.total);
  return true;
}

void xferPump() {
  unsigned long now = millis();
  if ((xr.active && now - xr.last > XFER_TIMEOUT) || (xs.active && now - xs.last > XFER_TIMEOUT)) {
    xferKill(true);
    errPrompt(FS(S_XFER_TIMEOUT));
    return;
  }
  if (!xs.active || qCount != 0) return;

  if (!xs.begun) {
    if (QF("XBEG:%s:%u", xs.name, (unsigned)xs.total)) {
      xs.begun = true;
      xs.last = now;
    }
    return;
  }
  if (!xs.ready) return;

  if (xs.sent >= xs.total) {
    xs.active = false;
    xferDone(F("[Transfer] Sent "), xs.name, xs.total, FS(S_PERIOD));
    printPrompt();
    return;
  }

  char buf[CHUNK_DATA + 1];
  uint16_t remaining = xs.total - xs.sent;
  uint8_t len = (remaining > CHUNK_DATA) ? CHUNK_DATA : (uint8_t)remaining;
  eeRead(xs.addr + xs.sent, len, buf);
  buf[len] = 0;

  // B91 image data is deliberately printable ASCII. rawNext preserves it
  // byte-for-byte, including punctuation and repeated spaces if a future
  // encoder chooses to use them. The B91 alphabet itself contains no ':'
  // or '#', so it cannot collide with the FTP field/frame delimiters.

  // Carry the exact chunk length and append a delimiter after the data.
  // rawNext skips sanitize(), so double / leading / trailing spaces are kept.
  rawNext = true;
  bool queued = QF("XDAT:%u:%u:%s:", (unsigned)xs.sent, (unsigned)len, buf);
  rawNext = false;
  if (queued) {
    xs.sent += len;
    xs.qlen = len;   // the bar advances when the peer ACKs this chunk
    xs.last = remoteRequestTime = now;
  }
}

// Handshake: HELLO / HACK carry our random id
void sendHs(uint8_t op) {
  char b[12];
  strcpy_P(b, ALP(op));
  uint8_t n = strlen(b);
  b[n++] = ':';
  n += utoaDec(b + n, (uint16_t)myRandomId);
  b[n] = 0;
  transmitFrame(0, b);
}

void sendHello() {
  sendHs(O_HELLO);
}

bool resolveIdentity(int peerId) {
  if (myRandomId > peerId) {
    localNodeName = "node1";
    remoteNodeName = "node2";
    isNode1 = true;
  } else if (myRandomId < peerId) {
    localNodeName = "node2";
    remoteNodeName = "node1";
    isNode1 = false;
  } else {
    myRandomId = random(0, 101);
    delay(random(50, 200));
    sendHello();
    return false;
  }
  nodeIdentified = true;
  return true;
}

void displayMenu() {
  inLen = 0;
  inOverflow = false;
  stActive = false;
  while (Serial.available() > 0) Serial.read();

  clearScreen();
  titleOpen();
  Serial.print(FS(S_ANSI_WHITE));
  Serial.print(localNodeName);
  titleClose();
  Serial.println(F(" Type " ANSI_YELLOW "help" ANSI_RESET));
  barLine();
  printPrompt();
}

void onHandshakeDone() {
  lastPeerActivityTime = lastPingQueuedTime = millis();
  lastRxSeq = 0;
  Serial.println();
  Serial.print(F(ANSI_GREEN "[Connected] Local: " ANSI_WHITE));
  Serial.print(localNodeName);
  Serial.print(F(ANSI_GREEN " | Remote: " ANSI_WHITE));
  Serial.println(remoteNodeName);
  endLine();
  displayMenu();
}

void resetSession() {
  nodeIdentified = false;
  core_apps_disconnect();
  loc = L_ROOT;
  remoteSpaceLocked = localInRemoteSpace = false;
  waitingForRemoteResponse = false;
  xr.active = xs.active = false;
  ewLen = ewPos = 0;
  clearQueue();
  lastRxSeq = 0;
  myRandomId = random(0, 101);
  printWarn(F("Searching... [q]=menu"));
  sendHello();
  lastHelloTime = millis();
}

// Request processing
void leaveRemote() {
  if (localInRemoteSpace) {
    QA(O_UNLK);
    localInRemoteSpace = false;
  }
}

// Peer refused or vanished: drop remote-space state (and our location if we were in it)
void dropRemote() {
  if (loc >= L_REMOTE) loc = L_NETWORK;
  localInRemoteSpace = peeking = false;
}

// End of a one-shot remote listing: hand the lock back (unless we cd'd in meanwhile)
void peekEnd() {
  if (!peeking) return;
  peeking = false;
  if (loc < L_REMOTE) leaveRemote();
}

bool requireLock() {
  if (!remoteSpaceLocked) {
    QA(A_NOLOCK);
    return false;
  }
  remoteLockTime = millis();
  return true;
}

// Common preamble for LS/CAT/GET/RM/PUT: lock check + "eeprom" store check.
// On success *rest points at the remaining ":"-separated arguments.
bool storeOp(char* args, char** rest) {
  if (!requireLock()) return false;
  char* a = args;
  char* store = nextField(&a);
  if (strcmp_P(store, S_eeprom)) {
    QA(A_DENIED);
    return false;
  }
  *rest = a ? a : EMPTY;
  return true;
}

void handleRequest(int8_t id, const char* op, char* args) {
  if (id == O_LOCK) {
    if (localInRemoteSpace && isNode1) {
      QA(O_LKNO);
      return;
    }
    if (localInRemoteSpace) {
      dropRemote();
      waitingForRemoteResponse = false;
    }
    remoteSpaceLocked = true;
    remoteLockTime = millis();
    QA(O_LKOK);
    noticePrompt(F("Peer connected to storage."));
    return;
  }
  if (id == O_LKOK) return;
  if (id == O_LKNO) {
    if (localInRemoteSpace) {
      dropRemote();
      waitingForRemoteResponse = false;
      errMsgPrompt(F("Remote access denied."));
    }
    return;
  }
  if (id == O_UNLK) {
    remoteSpaceLocked = false;
    noticePrompt(F("Peer disconnected."));
    return;
  }
  if (id == O_PING) {
    // Keep-alive is one-way. The sender reports its PING timing;
    // the receiver does not answer with PONG.
    return;
  }

  // Opcodes not handled by core FTP/session: deliver to whatever app wants them
  if (id == O_APP) {  // generic: routed to the app whose wire id matches
    core_apps_app_frame(args);
    return;
  }
  
  if (id == O_XBEG) {
    bool asServer = remoteSpaceLocked;
    bool asClient = localInRemoteSpace && waitingForRemoteResponse;
    if (!asServer && !asClient) {
      QA(A_NOLOCK);
      return;
    }
    if (asServer) remoteLockTime = millis();
    char* a = args;
    char* name = nextField(&a);
    int total = a ? atoi(a) : 0;
    if (viewing && eepromFind(name, nullptr, nullptr, nullptr) >= 0) {  // would shift the picture data
      queueFmt(S_ERR_BUSY);
      return;
    }
    uint8_t r = xrBegin(name, total, asServer);
    if (r == 0) {
      xferStart(xr.total);
      QA(O_XGO);
      return;
    }
    if (asServer) {
      if (r == 1) QA(A_BADNAME);
      else if (r == 2) QA(A_TOOLONG);
      else QA(A_FULL);
    } else {
      QA(A_FULL);
      waitingForRemoteResponse = false;
      errPrompt(F("\n[Download Error] Storage."));
    }
    return;
  }
  if (id == O_XGO) {
    if (xs.active) {
      xs.ready = true;
      xs.last = millis();
    }
    return;
  }
  if (id == O_XDAT) {
    if (remoteSpaceLocked) remoteLockTime = millis();
    remoteRequestTime = millis();
    char* a = args;
    int off = atoi(nextField(&a));
    uint8_t chunkLen = (uint8_t)atoi(nextField(&a));
    if (xrChunk(off, a ? a : EMPTY, chunkLen)) {
      xferKill(false);
      QF("ERR:XFER");
      errPrompt(FS(S_XFER_SYNC));
    }
    return;
  }
  if (id == O_XABT) {
    if (xferKill(false)) {
      warnPrompt(FS(S_XFER_CANCEL));
    }
    return;
  }

  if (id == O_LS) {
    char* rest;
    if (!storeOp(args, &rest)) return;

    char list[MAX_PAYLOAD + 1];
    list[0] = 0;
    uint8_t used = 0;
    int a = 1;
    uint8_t nl;
    uint16_t cl;
    while (recAt(a, &nl, &cl)) {
      uint8_t k = used;
      if (used) {
        if (used + 1 >= sizeof(list)) break;
        list[k++] = ',';
      }
      if (k + nl + 2 >= sizeof(list)) break;
      eeRead(a + 1, nl, list + k);
      k += nl;
      list[k++] = ':';
      k += utoaDec(list + k, cl);
      if (k < sizeof(list)) {
        used = k;
        list[used] = 0;
      } else {
        break;
      }
      a += recSize(a, nl);
    }
    QF("LSD:eeprom:%u:%s", (unsigned)getFreeEEPROMBytes(), list);
    return;
  }
  if (id == O_CAT || id == O_GET) {
    char* name;
    if (!storeOp(args, &name)) return;
    bool isGet = (id == O_GET);

    int contentAddr = 0;
    uint16_t cl = 0;
    if (eepromFind(name, nullptr, &cl, &contentAddr) < 0) {
      queueAl(A_NOTFOUND, ' ', name);
    } else if (isGet && cl > MAX_CONTENT) {
      if (!xsStart(name, contentAddr, cl)) queueFmt(S_ERR_BUSY);
    } else {
      if (cl > MAX_CONTENT) cl = MAX_CONTENT;
      char contentBuf[MAX_CONTENT + 1];
      eeRead(contentAddr, cl, contentBuf);
      QF("%sD:%s:%s", op, name, contentBuf);  // CATD / GETD
    }
    return;
  }
  if (id == O_RM) {
    char* name;
    if (!storeOp(args, &name)) return;
    if (viewing) {  // deleting / formatting would shift the picture data
      queueFmt(S_ERR_BUSY);
      return;
    }

    if (!strcmp_P(name, S_rf)) {
      eepromFormat();
      QA(A_OKFMT);
      okPrompt(F("\n[Remote] Storage formatted."));
      return;
    }
    if (eepromRemove(name)) {
      queueAl(A_Deleted, ' ', name);
      okName(F("\n[Remote Action] Deleted "), name, nullptr);
      printPrompt();
    } else {
      queueAl(A_NOTFOUND, ' ', name);
    }
    return;
  }
  if (id == O_PUT) {
    char* rest;
    if (!storeOp(args, &rest)) return;
    char* a = rest;
    char* name = nextField(&a);
    char* content = a ? a : EMPTY;

    if (viewing && eepromFind(name, nullptr, nullptr, nullptr) >= 0) {  // would shift the picture data
      queueFmt(S_ERR_BUSY);
      return;
    }
    uint8_t r = eepromPut(name, content);
    if (r == 0) {
      queueAl(A_Saved, ' ', name);
      okName(F("\n[Remote] Received file "), name, nullptr);
      printPrompt();
    } else if (r == 1) QA(A_BADNAME);
    else if (r == 2) QA(A_TOOLONG);
    else QA(A_FULL);
    return;
  }

  if (id == O_LSD) {
    waitingForRemoteResponse = false;

    char* a = args;

    // args:
    // eeprom:940:big.txt:74
    char* store = nextField(&a);

    if (strcmp_P(store, S_eeprom)) {
      printErr(F("[Remote] Invalid storage listing."));
      peekEnd();
      printPrompt();
      return;
    }

    // First field after "eeprom" is free space.
    char* freeStr = nextField(&a);
    int remoteFree = atoi(freeStr);

    // Remaining field is:
    // big.txt:74,other.txt:123,...
    uint8_t files = 0;
    int bytes = 0;

    if (a && *a) {
      char* item = a;

      while (item && *item) {
        char* comma = strchr(item, ',');
        if (comma) *comma = 0;

        char* colon = strchr(item, ':');
        int sz = 0;

        if (colon) {
          *colon = 0;
          sz = atoi(colon + 1);
        }

        printLsRow('-', item, sz);

        files++;
        bytes += sz;

        item = comma ? comma + 1 : nullptr;
      }
    }

    listFootB(files, bytes, remoteFree);

    peekEnd();
    printPrompt();
    return;
  }
  if (id == O_CATD) {
    waitingForRemoteResponse = false;
    char* a = args;
    char* name = nextField(&a);
    Serial.print(F(ANSI_CYAN "\n[" ANSI_WHITE));
    Serial.print(remoteNodeName);
    Serial.print(':');
    Serial.print(name);
    Serial.println(F(ANSI_CYAN "]:" ANSI_RESET));
    Serial.println(a ? a : EMPTY);
    printPrompt();
    return;
  }
  if (id == O_GETD) {
    waitingForRemoteResponse = false;
    char* a = args;
    char* name = nextField(&a);
    char* content = a ? a : EMPTY;

    uint8_t res = eepromPut(name, content);
    Serial.println();
    if (res == 0) okName(F("[Download Success] Saved "), name, FS(S_PERIOD));
    else printPutError(res);
    printPrompt();
    return;
  }
  if (id == O_OK) {
    waitingForRemoteResponse = false;
    Serial.print(F(ANSI_GREEN "\n[Remote] " ANSI_RESET));
    Serial.println(args);
    printPrompt();
    return;
  }
  if (id == O_ERR) {
    waitingForRemoteResponse = false;
    xs.active = false;
    Serial.print(F(ANSI_RED "\n[Remote Error] " ANSI_RESET));
    Serial.println(args);
    if (!strcmp_P(args, ALP(A_NOLOCK) + 4)) {  // skip "ERR:"
      dropRemote();
      printWarn(F("Session timeout. Re-enter dir."));
    }
    peekEnd();
    printPrompt();
    return;
  }
}

void maybeNak(uint8_t len) {
  if (!nodeIdentified || len < 10) return;
  if (millis() - lastNakTime < NAK_MIN_INTERVAL) return;
  lastNakTime = millis();
  sendCtl(0, O_NAK);
}

// Is this XDAT frame the final chunk of the running receive?
bool xdatIsLast(const char* a) {
  if (!xr.active) return false;
  const char* p = strchr(a, ':');
  return p && atoi(a) == (int)xr.got && (int)xr.got + atoi(p + 1) == (int)xr.total;
}

void handleFrame(char* f, uint8_t len) {
  if (!frameCrcOk(f, len)) {
    maybeNak(len);
    return;
  }

  f[len - 5] = 0;
  char* sp = strchr(f, ' ');
  if (!sp) return;
  *sp = 0;
  uint8_t seq = (uint8_t)atoi(f);

  char* op = sp + 1;
  expandAliasInPlace(op);  // no-op for token opcodes: they arrive already in long form
  char* args = strchr(op, ':');
  if (args) *args++ = 0;
  else args = EMPTY;
  int8_t id = opId(op);

  lastPeerActivityTime = millis();

  if (id == O_HELLO) {
    if (!nodeIdentified && resolveIdentity(atoi(args))) {
      delay(50);
      sendHs(O_HACK);
      onHandshakeDone();
    }
    return;
  }
  if (id == O_HACK) {
    if (!nodeIdentified && resolveIdentity(atoi(args))) onHandshakeDone();
    return;
  }
  if (!nodeIdentified) return;

  if (id == O_ACK) {
    if (txInFlight && qCount > 0 && seq == txSeq) {
      int8_t hid = opId(txQ[qHead]);
      char aid = txQ[qHead][4];
      if (xs.active && hid == O_XDAT) {  // chunk confirmed by the peer
        xs.acked += xs.qlen;
        printProgressBar(xs.acked, xs.total);
      }
      // Only use first-attempt ACKs as RTT samples. A retransmitted frame's
      // ACK cannot distinguish the original delay from a retry delay.
      if (txAttempts == 1 && txSentAt) {
        unsigned long rtt = millis() - txSentAt;
        // Jacobson-style lightweight EWMA: RTO follows the channel without
        // requiring floating point on the ATmega328P.
        unsigned long target = rtt + (rtt >> 1);
        if (target < ACK_RTO_MIN) target = ACK_RTO_MIN;
        if (target > ACK_RTO_MAX) target = ACK_RTO_MAX;
        txRto = (txRto * 3UL + target) / 4UL;
        if (txRto < ACK_RTO_MIN) txRto = ACK_RTO_MIN;
        if (txRto > ACK_RTO_MAX) txRto = ACK_RTO_MAX;
      }
      popHead();
      // The peer just answered, so the channel is ours: skip the random
      // 40..250 ms carrier-sense backoff the ACK's own tail would trigger.
      deferCount = MAX_DEFER;
      unsigned long tNext = millis() + 2;  // let the tail of the ACK tone end
      if ((long)(txNotBefore - tNext) < 0) txNotBefore = tNext;
      if (isAppOp(hid)) core_apps_tx_done((uint8_t)hid, aid, true);
    }
    return;
  }
  if (id == O_NAK) {
    if (txInFlight) txDeadline = millis();
    return;
  }

  if (seq == 0) return;

  // Chunks are ACKed as soon as the previous chunk is stored; the bytes of
  // this one are written in the background. Only the final chunk is stored
  // and committed before its ACK, so the file is complete when the sender
  // reports "Sent".
  bool ackLate = (id == O_XDAT) && xdatIsLast(args);
  if (id == O_XDAT) eeDrain();
  if (!ackLate) sendCtl(seq, O_ACK);
  if (seq != lastRxSeq) {
    lastRxSeq = seq;
    handleRequest(id, op, args);
  }
  if (ackLate) {
    eeDrain();
    eeBgPump();  // commits the record
    sendCtl(seq, O_ACK);
  }
}

void deliverFrame(bool early) {
  messageInProgress = false;
  uint8_t len = rxLen;
  bool ovf = rxOverflow;
  rxBuf[len] = 0;
  resetDecoder();
  rxLen = 0;
  rxOverflow = false;
  rxLast = 0;
  earlyLen = 0;
  skipTone = early;
  if (!ovf) handleFrame(rxBuf, len);
}

void runReceiverLogic() {
  eeBgPump();
  while (rbTail != rbHead) {
    uint16_t p = rbData[rbTail];
    rbTail = (rbTail + 1) & (RB_SIZE - 1);
    lastPulseMs = millis();
    processPulse(p);
  }

  unsigned long now = millis();
  if (now - lastValidSignalTime > RESET_GAP_MS) {
    lastConfirmedTok = TOK_NONE;
  }

  if (messageInProgress && !rxOverflow && rxLen >= 8 && rxLen != earlyLen) {
    earlyLen = rxLen;
    if (rxBuf[rxLen - 5] == '#') {
      rxBuf[rxLen] = 0;
      if (frameCrcOk(rxBuf, rxLen)) {
        deliverFrame(true);
        return;
      }
    }
  }

  if (messageInProgress && now - lastValidSignalTime > silenceMs()) deliverFrame(false);
}

void checkKeepAlive() {
  unsigned long now = millis();
  if (now - lastPeerActivityTime > LINK_DROP_TIMEOUT) {
    printErrMsg(F("Connection lost."));
    resetSession();
    return;
  }

  // Node 1 is the sole keep-alive initiator.
  if (isNode1 &&
      qCount == 0 &&
      now - lastPeerActivityTime > PING_INTERVAL &&
      now - lastPingQueuedTime > PING_INTERVAL) {
    QA(O_PING);
    lastPingQueuedTime = now;
  }
}

void checkRemoteTimeout() {
  if (waitingForRemoteResponse && millis() - remoteRequestTime > REMOTE_TIMEOUT) {
    waitingForRemoteResponse = false;
    printErrMsg(F("Remote timeout."));
    peekEnd();
    printPrompt();
  }
}

void checkLockExpiry() {
  if (remoteSpaceLocked && millis() - remoteLockTime > LOCK_TIMEOUT) {
    remoteSpaceLocked = false;
    QA(O_LKNO);
    noticePrompt(F("Peer timeout."));
  }
}

// Commands
bool beginRemote() {
  if (waitingForRemoteResponse) {
    printWarn(FS(S_BUSY));
    return false;
  }
  return true;
}

void armRemote(bool queued) {
  if (queued) {
    waitingForRemoteResponse = true;
    remoteRequestTime = millis();
  } else errQ();
}

// ---- Path resolution ----
const char* nextSeg(const char* p, char* seg, uint8_t cap) {
  while (*p == '/') p++;
  uint8_t n = 0;
  while (*p && *p != '/') {
    if (n < cap - 1) seg[n++] = *p;
    p++;
  }
  seg[n] = 0;
  return p;
}

const __FlashStringHelper* navStep(Loc cur, const char* s, Loc* out) {
  *out = cur;
  if (!*s || !strcmp_P(s, PSTR("."))) return nullptr;
  if (!strcmp_P(s, PSTR(".."))) {
    switch (cur) {
      case L_NETWORK:
      case L_EEPROM: *out = L_ROOT; break;
      case L_REMOTE: *out = L_NETWORK; break;
      case L_REMOTE_EEPROM: *out = L_REMOTE; break;
      default: break;
    }
    return nullptr;
  }
  switch (cur) {
    case L_ROOT:
      if (!strcmp_P(s, S_eeprom)) {
        *out = L_EEPROM;
        return nullptr;
      }
      if (!strcmp_P(s, S_net) || !strcmp_P(s, PSTR("network"))) {
        if (mode != M_ONLINE) return F("Offline - no network.");
        *out = L_NETWORK;
        return nullptr;
      }
      break;
    case L_NETWORK:
      if (!strcmp(s, localNodeName)) {
        *out = L_ROOT;
        return nullptr;
      }
      if (!strcmp(s, remoteNodeName)) {
        if (remoteSpaceLocked) return F("Dir locked by peer.");
        *out = L_REMOTE;
        return nullptr;
      }
      return F("Node not found.");
    case L_REMOTE:
      if (!strcmp_P(s, S_eeprom)) {
        *out = L_REMOTE_EEPROM;
        return nullptr;
      }
      break;
    default: break;
  }
  if (cur == L_EEPROM && eepromFind(s, nullptr, nullptr, nullptr) >= 0) return F("Not a dir.");
  return F("No such dir.");
}

const __FlashStringHelper* navPath(const char* path, Loc* out) {
  Loc cur = loc;
  if (*path == '/') cur = L_ROOT;
  char seg[MAX_NAME + 3];
  const char* q = path;
  for (;;) {
    q = nextSeg(q, seg, sizeof(seg));
    if (!seg[0]) break;
    const __FlashStringHelper* e = navStep(cur, seg, &cur);
    if (e) return e;
  }
  *out = cur;
  return nullptr;
}

bool moveTo(Loc to) {
  bool wasRemote = (loc == L_REMOTE || loc == L_REMOTE_EEPROM);
  bool toRemote = (to == L_REMOTE || to == L_REMOTE_EEPROM);
  if (loc == L_NETWORK && to == L_REMOTE) {
    if (!QA(O_LOCK)) {
      errQ();
      return false;
    }
    localInRemoteSpace = true;
  } else if (wasRemote && !toRemote) {
    leaveRemote();
  }
  if (to != loc) waitingForRemoteResponse = false;
  loc = to;
  return true;
}

bool fsChangeDir(const char* t) {
  const char* q = *t ? t : "/";

  Loc dest;
  const __FlashStringHelper* e = navPath(q, &dest);
  if (e) {
    printE(e);
    return false;
  }

  if (*q == '/' && !moveTo(L_ROOT)) return false;
  char seg[MAX_NAME + 3];
  for (;;) {
    q = nextSeg(q, seg, sizeof(seg));
    if (!seg[0]) break;
    Loc nx;
    if (navStep(loc, seg, &nx)) return false;
    if (!moveTo(nx)) return false;
  }
  return true;
}

void listDir(Loc at) {
  switch (at) {
    case L_ROOT:
      dirRow(FS(S_eeprom));
      if (mode == M_ONLINE) dirRow(FS(S_net));
      break;
    case L_NETWORK:
      printLsRow('l', localNodeName, 0);
      printLsRow('d', remoteNodeName, 0);
      break;
    case L_EEPROM:
      printEEPROMList();
      break;
    case L_REMOTE:
      dirRow(FS(S_eeprom));
      break;
    case L_REMOTE_EEPROM:
      if (!beginRemote()) return;
      Serial.print(F("Listing from "));
      Serial.print(remoteNodeName);
      Serial.println(F("..."));
      armRemote(QA(O_LS));
      break;
  }
}

bool fsList(const char* arg) {
  while (*arg == '-') {
    while (*arg && *arg != ' ') arg++;
    while (*arg == ' ') arg++;
  }
  Loc at = loc;
  if (*arg) {
    const __FlashStringHelper* e = navPath(arg, &at);
    if (e) { printE(e); return false; }
    Serial.print(FS(S_ANSI_YELLOW));
    printPathOf(at, false);
    Serial.println(F(":" ANSI_RESET));
  }
  if (at == L_REMOTE_EEPROM && !localInRemoteSpace) {
    if (!beginRemote()) return false;
    if (qCount + 2 > QUEUE_LEN || !QA(O_LOCK)) { errQ(); return false; }
    localInRemoteSpace = peeking = true;
  }
  listDir(at);
  return true;
}
bool transferDownload(const char* fname) {
  if (!*fname) { printE(F("Filename required.")); return false; }
  if (loc != L_REMOTE_EEPROM) { printE(F("Remote directory required.")); return false; }
  if (!validName(fname)) { printPutError(1); return false; }
  if (!beginRemote()) return false;
  act(FS(S_DOWNLOADING), fname);
  armRemote(queueAl(O_GET, ':', fname));
  return true;
}
bool fsRemove(const char* fname) {
  if (!*fname) { printE(F("Filename required.")); return false; }
  if (!strcmp_P(fname, S_rf)) {
    if (loc == L_EEPROM) {
      eepromFormat();
      printOk(F("EEPROM wiped."));
      return true;
    }
    if (loc == L_REMOTE_EEPROM) {
      if (!beginRemote()) return false;
      Serial.println(F("Wiping remote EEPROM..."));
      armRemote(queueAl(O_RM, ':', fname));
      return true;
    }
    printE(F("Storage directory required."));
    return false;
  }
  if (loc == L_EEPROM) {
    if (eepromRemove(fname)) okName(FS(S_DELETED), fname, FS(S_FROM_EE));
    else errNF();
    return true;
  }
  if (loc == L_REMOTE_EEPROM) {
    if (!validName(fname)) { printPutError(1); return false; }
    if (!beginRemote()) return false;
    act(FS(S_DEL_REMOTE), fname);
    armRemote(queueAl(O_RM, ':', fname));
    return true;
  }
  errStore();
  return false;
}
bool transferUploadStored(const char* fname) {
  if (!validName(fname)) { printPutError(1); return false; }
  int caddr = 0;
  uint16_t cl = 0;
  if (eepromFind(fname, nullptr, &cl, &caddr) < 0) {
    printE(F("File not found in EEPROM."));
    return false;
  }
  if (!beginRemote()) return false;
  if (cl <= MAX_CONTENT) {
    char buf[MAX_CONTENT + 1];
    eeRead(caddr, cl, buf);
    act(FS(S_UPLOADING), fname);
    armRemote(queueAl2(O_PUT, ':', fname, buf));
    return true;
  }
  if (xs.active) { printE(F("Transfer busy.")); return false; }
  act(FS(S_UP_CHUNK), fname);
  xsStart(fname, caddr, cl);
  waitingForRemoteResponse = true;
  remoteRequestTime = millis();
  return true;
}
// ---- put into this node's storage: ONE write path --------------------------
// Short `put name text` and long streamed puts both run  Begin -> (Chunk)* -> End.
// Begin creates/replaces the file with `first`; Chunk appends; End reports once.
// All text passed in is NUL-terminated (the buffers are the line editor's).
static void putLocalBegin(const char* name, const char* first) {
  stActive = true;
  copyStr(stName, name, MAX_NAME);
  stErr = eepromPutCap(name, first, MAX_EEPROM_FILE);
}

// Prints the result; true if the file was saved.
static bool putLocalEnd(void) {
  stActive = false;
  uint8_t e = stErr;
  stErr = 0;
  if (e == 0) okName(FS(S_SAVED), stName, FS(S_TO_EEPROM));
  else if (e == 2) errLimit();
  else if (e != 255) printPutError(e);
  return e == 0;
}

bool transferUpload(const char* arg) {
  if (!*arg) { printE(F("Transfer argument required.")); return false; }
  char buf[INBUF_LEN + 2];
  size_t n = strlen(arg);
  if (n >= sizeof(buf)) { printE(F("Transfer argument too long.")); return false; }
  memcpy(buf, arg, n + 1);
  char* content = strchr(buf, ' ');
  if (!content) {
    if (loc != L_REMOTE_EEPROM) { printE(F("A stored remote file is required.")); return false; }
    return transferUploadStored(buf);
  }
  *content++ = 0;
  if (loc == L_EEPROM) {
    putLocalBegin(buf, content);
    return putLocalEnd();
  }
  if (loc == L_REMOTE_EEPROM) {
    if (!validName(buf)) { printPutError(1); return false; }
    if (strlen(content) > MAX_CONTENT) { printWarn(F("Content too long.")); return false; }
    if (!beginRemote()) return false;
    act(FS(S_UPLOADING), buf);
    armRemote(queueAl2(O_PUT, ':', buf, content));
    return true;
  }
  errStore();
  return false;
}

// ---- File viewer: FTP-B91 v2 (Base91 -> range coder -> ANSI), no frame buffer ----
//
// FTP-B91 v2 image format (binary data before Base91):
//   0       0xB9 magic
//   1       0x02 = RGB palette, 0x03 = gray palette; +0x04 = animated (6 / 7)
//   2       bits 7-6 width code | bits 5-4 height code | bits 3-0 palette count-1
//           size code: 0=16, 1=32, 2=64, 3=96
//   2b      animated only: frame count (1 byte), then delay in 10 ms units
//           (1 byte). All frames share the palette and ONE range-coder stream.
//   ...     palette: RGB = N*3 bytes, gray = N bytes (sorted by luminance)
//   ...     adaptive binary range-coder payload (LZMA style, 12-bit probs,
//           shift 4). The always-zero first byte is omitted and trailing
//           zero bytes are trimmed: the decoder reads zeros past the end.
//
// Pixel model (causal, identical to the web encoder):
//   L = left, U = up, UL = up-left; at the edges the missing neighbour
//   mirrors the nearest one. t = relation class of L/U/UL (5 classes).
//   A: pix == L ?                  ctx = t*2 + lastMiss        (pA[10])
//   B: pix == U ?   (only if L!=U) ctx = t-2                   (pB[3])
//   C: escape: sign (ctx U>L / U<L / U==L), then unary |pix-L|-1 (pM[14]);
//      sign omitted / unary shortened when the palette edge leaves one choice.
//
// RAM: two image rows (2 x 96 B) + 48 B palette + 60 B model.
// Images up to 96 pixels wide need a terminal at least that many columns
// wide (autowrap is off, so a narrower window clips the right edge).
//
// Base91 alphabet = 91 printable chars excluding protocol delimiters.
// The decoder reads the stored file directly.

#define VIEW_COLS 96
#define VIEW_PB 12
#define VIEW_PSH 4

// Tera Term's 38;2 / 48;2 implementation picks the nearest entry of its
// 256-colour palette. We therefore program 16 unused palette slots with the
// EXACT image palette (OSC 4) and select them with indexed 38;5 / 48;5.
// Slots 240..255 leave the normal ANSI 0..15 colours untouched.
#define VIEW_TT_PALETTE_BASE 240

void viewSetPaletteColor(uint8_t slot, uint8_t r, uint8_t g, uint8_t b) {
  // OSC 4 ; slot ; #rrggbb BEL
  Serial.write(27);
  Serial.write(']');
  Serial.print(F("4;"));
  Serial.print(slot);
  Serial.print(F(";#"));

  const char hex[] = "0123456789ABCDEF";
  Serial.write(hex[(r >> 4) & 15]);
  Serial.write(hex[r & 15]);
  Serial.write(hex[(g >> 4) & 15]);
  Serial.write(hex[g & 15]);
  Serial.write(hex[(b >> 4) & 15]);
  Serial.write(hex[b & 15]);
  Serial.write(7);
}

// Emit one SGR sequence that sets whichever of fg / bg is >= 0
// (palette index 0..15). Merged into a single ESC[...m to save bytes.
void viewSgr(int8_t f, int8_t g) {
  if (f < 0 && g < 0) return;
  csi();
  if (f >= 0) {
    Serial.print(F("38;5;"));
    Serial.print(VIEW_TT_PALETTE_BASE + f);
  }
  if (f >= 0 && g >= 0) Serial.print(';');
  if (g >= 0) {
    Serial.print(F("48;5;"));
    Serial.print(VIEW_TT_PALETTE_BASE + g);
  }
  Serial.print('m');
}

// Draw one character cell holding pixel a (top) over pixel b (bottom),
// choosing the glyph that needs the fewest colour-change bytes.
// fg / bg track the terminal's current colours (-1 = unknown / default).
void viewCell(uint8_t a, uint8_t b, int8_t& fg, int8_t& bg) {
  if (a == b) {
    if (bg == (int8_t)a) {            // background already right: a space
      Serial.write(' ');
    } else if (fg == (int8_t)a) {     // foreground already right: full block
      writeGlyph(GLYPH_FULL);
    } else {
      viewSgr(-1, a);
      bg = a;
      Serial.write(' ');
    }
    return;
  }
  // upper half block: fg=a bg=b   |   lower half block: fg=b bg=a
  uint8_t costU = (fg != (int8_t)a) + (bg != (int8_t)b);
  uint8_t costL = (fg != (int8_t)b) + (bg != (int8_t)a);
  bool upper = costU <= costL;
  int8_t f = upper ? a : b, g = upper ? b : a;
  viewSgr(fg != f ? f : -1, bg != g ? g : -1);
  fg = f;
  bg = g;
  if (upper) writeGlyph(GLYPH_UPPER);
  else writeGlyph(GLYPH_LOWER);
}

// Value of a Base91 character (arithmetic, no table): the alphabet is the
// printable range 33..126 minus '#'(35), ':'(58) and '\\'(92).
int8_t viewB91Val(uint8_t c) {
  if (c < 33 || c > 126 || c == 35 || c == 58 || c == 92) return -1;
  return (int8_t)(c - 33 - (c > 35) - (c > 58) - (c > 92));
}

// Decode one byte from the Base91 stream in EEPROM.
// Returns -1 at end of stream, -2 on an invalid character.
int viewB91Next(B91R& b) {
  while (b.bits < 8) {
    if (b.addr >= b.end) {
      if (b.pair < 0) return -1;
      // single trailing character: it carries the last (partial) byte
      b.acc |= (uint32_t)b.pair << b.bits;
      b.bits += 8;
      b.pair = -1;
      break;
    }
    int8_t v = viewB91Val(eeGet(b.addr++));
    if (v < 0) return -2;
    if (b.pair < 0) {
      b.pair = v;
    } else {
      uint16_t q = (uint16_t)b.pair + (uint16_t)v * 91U;
      b.pair = -1;
      b.acc |= (uint32_t)q << b.bits;
      b.bits += ((q & 8191U) > 88U) ? 13 : 14;
    }
  }
  int out = b.acc & 255U;
  b.acc >>= 8;
  b.bits -= 8;
  return out;
}

// Range-coder input byte: zeros past the end of the stream (the encoder
// trims trailing zero bytes).
uint8_t viewRcByte(ViewDec& d) {
  int v = viewB91Next(d.b);
  if (v >= 0) return (uint8_t)v;
  if (v == -2) d.bad = true;
  return 0;
}

uint8_t viewBit(ViewDec& d, uint16_t* p) {
  while (d.range < 0x1000000UL) {
    d.range <<= 8;
    d.code = (d.code << 8) | viewRcByte(d);
  }
  uint16_t pr = *p;
  uint32_t bound = (d.range >> VIEW_PB) * pr;
  if (d.code < bound) {
    d.range = bound;
    *p = pr + ((4096U - pr) >> VIEW_PSH);
    return 0;
  }
  d.range -= bound;
  d.code -= bound;
  *p = pr - (pr >> VIEW_PSH);
  return 1;
}

// Decode one palette index. Returns -1 if the stream is corrupt.
int viewPix(ViewDec& d, uint8_t L, uint8_t U, uint8_t UL, uint8_t pc) {
  uint8_t t = (L == U) ? (UL == L ? 0 : 1) : (UL == L ? 2 : (UL == U ? 3 : 4));
  uint8_t pix;
  if (!viewBit(d, &d.pA[t * 2 + d.last])) {
    pix = L;
  } else if (L != U && !viewBit(d, &d.pB[t - 2])) {
    pix = U;
  } else {
    bool canNeg = L > 0, canPos = L < pc - 1, neg;
    if (canNeg && canPos) neg = viewBit(d, &d.pS[U > L ? 1 : (U < L ? 2 : 0)]);
    else if (canNeg) neg = true;
    else if (canPos) neg = false;
    else return -1;
    uint8_t maxK = (neg ? L : pc - 1 - L) - 1;
    uint8_t k = 0;
    while (k < maxK && viewBit(d, &d.pM[k])) k++;
    pix = neg ? L - (k + 1) : L + (k + 1);
  }
  d.last = (pix != L);
  return pix;
}

// Probability model (all 12-bit, start at 0.5) + range-coder start.
void viewRcInit(ViewDec& d) {
  for (uint8_t i = 0; i < 10; i++) d.pA[i] = 1 << (VIEW_PB - 1);
  for (uint8_t i = 0; i < 3; i++) d.pB[i] = d.pS[i] = 1 << (VIEW_PB - 1);
  for (uint8_t i = 0; i < 14; i++) d.pM[i] = 1 << (VIEW_PB - 1);
  d.last = 0;
  d.range = 0xFFFFFFFFUL;
  d.code = 0;
  for (uint8_t i = 0; i < 4; i++) d.code = (d.code << 8) | viewRcByte(d);
}

// 'view' blocks loop() (the animation runs until a key is pressed), so keep
// the radio link serviced from in here: receive + ACK, send keep-alive PINGs,
// run the TX queue. Without this both nodes drop the link after 30 s.
void linkPump() {
  if (mode != M_ONLINE || !nodeIdentified) return;
  unsigned long t = millis();
  do runReceiverLogic();
  while ((messageInProgress || rbTail != rbHead) && millis() - t < 400UL);
  checkKeepAlive();
  if (!nodeIdentified) return;  // link dropped
  checkLockExpiry();
  checkRemoteTimeout();
  xferPump();
  serviceTx();
}

// Write the pending notice and transfer progress in the status area below the
// picture. When a transfer is active, two terminal rows are reserved:
//   [Notice] Transfer initiated.
//   [██████====] 60%  538/896 B  2140 bps
// The next frame moves up by the picture height + one status row, so the
// picture itself never drifts.
void viewNoteFlush() {
  if (!viewNote && !progressActive) return;

  noticeHead();  // clears the first status line, prints "[Notice] "
  if (viewNote) {
    if (viewNoteNode) Serial.print(remoteNodeName);
    Serial.print(viewNote);
  }

  if (progressActive) {
    // The transfer gets a second status row for the live progress bar.
    endLine();

    // Reuse the normal progress renderer on that second row. Temporarily
    // clear 'viewing' so it does not defer itself back into this function.
    viewing = false;
    printProgressBar(viewProgressDone, viewProgressTotal);
    viewing = true;
  }

  // Back to column 1 for the next frame. Use ESC[1G, NOT a lone CR.
  Serial.print(F(ANSI_RESET "\033[1G"));
  if (!progressActive) viewNote = nullptr;
  viewNoteNode = false;
}

// Decode + draw one frame (the model carries over between frames).
// r0 / r1 are the two row buffers (row y uses r0 when y is even).
bool viewFrame(ViewDec& d, uint8_t w, uint8_t h, uint8_t pc, uint8_t* r0, uint8_t* r1) {
  for (uint8_t y = 0; y < h; y++) {
    uint8_t* cur = (y & 1) ? r1 : r0;
    uint8_t* prev = (y & 1) ? r0 : r1;
    // endLine() emits ANSI_RESET, so terminal colours are reset at every
    // rendered line. Keep the cache local to this line; carrying it across
    // lines causes missing SGR sequences and corrupted/glitchy frames.
    int8_t fg = -1, bg = -1;
    for (uint8_t x = 0; x < w; x++) {
      uint8_t U = y ? prev[x] : 0;
      uint8_t L = x ? cur[x - 1] : U;
      if (!y) U = L;
      uint8_t UL = (x && y) ? prev[x - 1] : U;
      int p = viewPix(d, L, U, UL, pc);
      if (p < 0 || d.bad) return false;
      cur[x] = (uint8_t)p;
      // Odd row: the top row (prev) is complete, so print this cell now.
      if (y & 1) viewCell(prev[x], (uint8_t)p, fg, bg);
    }
    if (y & 1) {
      endLine();
      linkPump();
    }
  }
  return true;
}

bool renderB91(int addr, uint16_t cl) {
  ViewDec d;
  d.b.addr = addr;
  d.b.end = addr + cl;
  d.b.pair = -1;
  d.b.acc = 0;
  d.b.bits = 0;
  d.bad = false;
  d.last = 0;

  int magic = viewB91Next(d.b);
  int ver = viewB91Next(d.b);
  int dim = viewB91Next(d.b);

  if (magic != 0xB9 || (ver & 0xFA) != 2 || dim < 0) {
    printE(F("Not FTP-B91 v2 (re-encode)."));
    return false;
  }
  bool gray = ver & 1;
  uint8_t lw = dim >> 6, lh = (dim >> 4) & 3;
  uint8_t w = lw == 3 ? 96 : 16 << lw;   // codes 0..3 = 16, 32, 64, 96
  uint8_t h = lh == 3 ? 96 : 16 << lh;
  uint8_t pc = (dim & 15) + 1;
  uint8_t nf = 1, dly = 0;
  if (ver & 4) {  // animated: frame count + delay (10 ms units)
    int a = viewB91Next(d.b), c = viewB91Next(d.b);
    if (a < 1 || c < 0) {
      printE(FS(S_TRUNCATED));
      return false;
    }
    nf = a;
    dly = c;
  }

  uint8_t pr[16], pg[16], pb[16];
  for (uint8_t i = 0; i < pc; i++) {
    int r = viewB91Next(d.b);
    int g = r, bl = r;
    if (!gray) {
      g = viewB91Next(d.b);
      bl = viewB91Next(d.b);
    }
    if (r < 0 || g < 0 || bl < 0) {
      printE(FS(S_TRUNCATED));
      return false;
    }
    pr[i] = r;
    pg[i] = g;
    pb[i] = bl;
  }

  // Install the image's exact RGB palette into Tera Term.
  for (uint8_t i = 0; i < pc; i++)
    viewSetPaletteColor(VIEW_TT_PALETTE_BASE + i, pr[i], pg[i], pb[i]);

  B91R rcStart = d.b;  // stream position after the palette: animation restarts here
  uint8_t buf0[VIEW_COLS], buf1[VIEW_COLS];  // row y lives in buf[y & 1]
  bool ok = true, stop = false, drawn = false, armed = false;

  Serial.print(F("\033[?7l"));                 // autowrap off: a narrow window clips
  if (nf > 1) Serial.print(F("\033[?25l"));    // hide the cursor while animating

  viewNote = nullptr;
  viewNoteNode = false;
  progressActive = false;
  viewing = true;
  do {
    d.b = rcStart;
    viewRcInit(d);
    for (uint8_t f = 0; f < nf && ok && !stop; f++) {
      unsigned long frameStart = millis();
      if (drawn) {  // back to the top-left of the picture
        csi();
        Serial.print((uint16_t)(h / 2 + (progressActive ? 1 : 0)));
        Serial.print('A');
      }
      drawn = true;
      ok = viewFrame(d, w, h, pc, buf0, buf1);
      if (ok) viewNoteFlush();
      if (ok && nf > 1) {
        // The encoded GIF delay is the TOTAL frame duration, not an extra
        // sleep after rendering. Rendering itself can take a large part of
        // the requested delay at 115200 baud, so only wait for the remainder.
        unsigned long elapsed = millis() - frameStart;
        unsigned long target = dly * 10UL;
        unsigned long wait = (elapsed < target) ? (target - elapsed) : 0;
        unsigned long t0 = millis();
        do linkPump();
        while (millis() - t0 < wait);
        if (mode == M_ONLINE && !nodeIdentified) stop = true;  // link dropped
        if (!armed) {  // drop the stray LF from Enter, then watch for real keys
          while (Serial.available() > 0) Serial.read();
          armed = true;
        } else if (Serial.available() > 0) {  // any key stops the animation
          while (Serial.available() > 0) Serial.read();
          stop = true;
        }
      }
    }
  } while (nf > 1 && ok && !stop);
  viewing = false;
  viewNote = nullptr;
  viewNoteNode = false;
  progressActive = false;

  Serial.println(F(ANSI_RESET "\033[?7h\033[?25h"));
  if (!ok) {
    printE(F("Corrupt image."));
    return false;
  }
  return true;
}

// true if name ends in ".b91" (case-insensitive)
bool hasB91Ext(const char* n) {
  size_t l = strlen(n);
  if (l < 5) return false;  // at least one character before ".b91"
  n += l - 4;
  return n[0] == '.' && (n[1] | 32) == 'b' && n[2] == '9' && n[3] == '1';
}

// .b91 -> image (EEPROM only); other files -> text (EEPROM / remote)
bool fileView(const char* fname) {
  if (!*fname) { printE(F("Filename required.")); return false; }
  bool img = hasB91Ext(fname);
  if (loc == L_REMOTE_EEPROM && !img) {
    if (!validName(fname)) { printPutError(1); return false; }
    if (!beginRemote()) return false;
    act(FS(S_READING), fname);
    armRemote(queueAl(O_CAT, ':', fname));
    return true;
  }
  if (loc != L_EEPROM) {
    if (img) printE(F("Images require EEPROM storage."));
    else errStore();
    return false;
  }
  int addr = 0;
  uint16_t cl = 0;
  if (img && !beginRemote()) return false;
  if (eepromFind(fname, nullptr, &cl, &addr) < 0) { errNF(); return false; }
  if (img) return renderB91(addr, cl);
  for (uint16_t i = 0; i < cl; i++) Serial.write(eeGet(addr + i));
  Serial.println();
  return true;
}

// One aligned help line: yellow command, description at a fixed column
void helpRow(const __FlashStringHelper* cmd, const __FlashStringHelper* desc) {
  Serial.print(F("  " ANSI_YELLOW));
  Serial.print(cmd);
  Serial.print(FS(S_ANSI_RESET));
  uint8_t l = strlen_P((PGM_P)cmd);
  while (l++ < 22) Serial.print(' ');
  Serial.println(desc);
}


void cmdHelp(char*) {
  Serial.println(F(ANSI_CYAN "\nCommands" ANSI_RESET));
  helpRow(F("cd [path]"), F("change directory"));
  helpRow(F("ls [path]"), F("list files"));
  helpRow(F("rm <file>"), F("delete file"));
  if (mode != M_ONLINE) helpRow(F("online"), F("connect to a peer"));
  helpRow(F("menu"), F("return to main menu"));
  helpRow(F("exit /"), F("leave active app"));

  if (g_appCount) {
    Serial.println(F(ANSI_CYAN "\nApplications" ANSI_RESET));
    for (uint8_t i = 0; i < g_appCount; i++) {
      Serial.print(F("  " ANSI_YELLOW));
      Serial.print(g_apps[i]->name ? g_apps[i]->name : "");
      Serial.print(FS(S_ANSI_RESET));
      uint8_t l = g_apps[i]->name ? strlen(g_apps[i]->name) : 0;
      while (l++ < 22) Serial.print(' ');
      Serial.println(g_apps[i]->blurb ? g_apps[i]->blurb : "");
    }
  }
}


// --------------------------------------------------------------------
// Main menu / mode switching
// --------------------------------------------------------------------
void resetInput() {
  stActive = false;
  inLen = 0;
  inOverflow = false;
  while (Serial.available() > 0) Serial.read();
  clearScreen();
}

void mainMenu() {
  mode = M_MENU;
  loc = L_ROOT;
  resetInput();
  titleOpen();
  Serial.print(F(ANSI_WHITE "main"));
  titleClose();
  Serial.println(F("  " ANSI_YELLOW "[1]" ANSI_RESET " offline\n  "
                   ANSI_YELLOW "[2]" ANSI_RESET " online"));
  barLine();
  printPrompt();
}

void offlineBanner() {
  resetInput();
  titleOpen();
  Serial.print(F(ANSI_MAGENTA "OFFLINE"));
  titleClose();
  Serial.println(F(" Local only. " ANSI_YELLOW "help" ANSI_RESET "\n "
                   ANSI_YELLOW "online" ANSI_RESET " | " ANSI_YELLOW "menu" ANSI_RESET));
  barLine();
  printPrompt();
}

void goOffline() {
  mode = M_OFFLINE;
  loc = L_ROOT;
  offlineBanner();
}

void goOnline() {
  mode = M_ONLINE;
  loc = L_ROOT;
  nodeIdentified = false;
  clearQueue();
  lastRxSeq = 0;
  flushRx();  // drop anything the ISR heard while we were offline
  myRandomId = random(0, 101);
  resetInput();
  printOk(F("[Online] Broadcasting..."));
  printWarn(F("Waiting for peer. [q]=menu."));
  sendHello();
  lastHelloTime = millis();
}

void leaveOnline() {
  if (nodeIdentified) {
    core_apps_disconnect();
    xferKill(true);
    leaveRemote();
    unsigned long t = millis();  // give UNLK/XABT a moment to get out
    while (qCount && millis() - t < 800UL) {
      runReceiverLogic();
      serviceTx();
    }
  }
  clearQueue();
  nodeIdentified = false;
  remoteSpaceLocked = localInRemoteSpace = waitingForRemoteResponse = false;
  localNodeName = remoteNodeName = "";
}

void cmdMenu(char*) {
  core_app_close();
  if (mode == M_ONLINE) leaveOnline();
  mainMenu();
}

void cmdOnline(char*) {
  if (mode == M_ONLINE) printWarn(F("Already online."));
  else goOnline();
}

void handleMenuLine(char* line) {
  if (*line == '[') {  // tolerate "[1]" typed literally
    line++;
    char* br = line;
    while (*br && *br != ']') br++;
    *br = 0;
  }
  if (line[0] == '1' && !line[1]) {
    goOffline();
  } else if (line[0] == '2' && !line[1]) {
    goOnline();
  } else {
    errPrompt(F("\nUnknown option."));
  }
}

// Resolve an application-supplied path and temporarily select its storage location.
bool resolveServicePath(char*& arg) {
  char* sp = strchr(arg, ' ');
  char* slash = nullptr;
  for (char* p = arg; *p && p != sp; p++)
    if (*p == '/') slash = p;
  if (!slash) return true;

  Loc d;
  const __FlashStringHelper* e = nullptr;
  if (slash == arg) {
    d = L_ROOT;
  } else {
    *slash = 0;
    e = navPath(arg, &d);
  }
  if (e) {
    printE(e);
    return false;
  }
  if ((d == L_REMOTE || d == L_REMOTE_EEPROM) && d != loc) {
    printE(F("Enter the remote storage directory first."));
    return false;
  }
  loc = d;  // restored by the caller after the service completes
  arg = slash + 1;
  return true;
}

// --------------------------------------------------------------------
// Compact kernel service dispatcher. Apps own command words; this exposes
// storage/transfer capabilities without duplicating path/buffer code.
static bool fsServiceRun(uint8_t service, const char* arg) {
  char buf[INBUF_LEN + 2];
  size_t n = arg ? strlen(arg) : 0;
  if (n >= sizeof(buf)) { printE(F("Path too long.")); return false; }
  if (arg) memcpy(buf, arg, n + 1); else buf[0] = 0;

  char* a = buf;
  if (!resolveServicePath(a)) return false;
  Loc saved = loc;
  bool ok = false;
  switch (service) {
    case CORE_FS_GET: ok = transferDownload(a); break;
    case CORE_FS_PUT: ok = transferUpload(a); break;
    case CORE_FS_VIEW: ok = fileView(a); break;
  }
  loc = saved;
  return ok;
}

bool core_fs_service(uint8_t service, const char* arg) {
  bool ok = fsServiceRun(service, arg);
  if (!waitingForRemoteResponse && !stActive && mode != M_MENU && (mode == M_OFFLINE || nodeIdentified))
    printPrompt();
  return ok;
}


void processLine(char* line) {
  sanitize(line);
  if (mode == M_MENU) {
    if (!*line) { printPrompt(); return; }
    handleMenuLine(line);
    return;
  }

  // Echo the completed command on its own line before an app/core handler
  // redraws the prompt.  Without this, app services that refresh the prompt
  // immediately after handling a command concatenate the new prompt onto
  // the typed command.
  Serial.println();

  // Core-owned app exit aliases. Exact "/" keeps absolute paths like /net/node1 valid.
  if (!strcmp(line, "/") || !strcmp(line, "exit")) {
    if (g_activeApp >= 0) { core_app_close(); printPrompt(); }
    else printErr(F("No active application."));
    return;
  }

  // Direct app launch: typing an app name opens it.
  if (g_activeApp < 0) {
    for (uint8_t i = 0; i < g_appCount; i++) {
      if (!strcmp(line, g_apps[i]->name)) { core_app_open(line); return; }
    }
  }

  // The active app gets the line first (including an empty line)
  int8_t wasApp = g_activeApp;
  if (core_apps_line(line)) return;

  if (!*line) {
    printPrompt();
    return;
  }

  char* arg = strchr(line, ' ');
  if (arg) {
    *arg++ = 0;
    while (*arg == ' ') arg++;
  } else arg = EMPTY;

  bool found = false;
  if (!strcmp(line, "help")) {
    cmdHelp(arg);
    found = true;
  } else if (!strcmp(line, "cd")) {
    fsChangeDir(arg);
    found = true;
  } else if (!strcmp(line, "ls")) {
    fsList(arg);
    found = true;
  } else if (!strcmp(line, "rm")) {
    Loc saved = loc;
    char* a = arg;
    if (resolveServicePath(a)) fsRemove(a);
    loc = saved;  // a dir/ prefix only applies to this command
    found = true;
  } else if (!strcmp(line, "menu")) {
    cmdMenu(arg);
    found = true;
  } else if (!strcmp(line, "online")) {
    cmdOnline(arg);
    found = true;
  }
  if (!found) printErr(F("Unknown command. Type 'help'."));
  bool appJustOpened = (wasApp < 0 && g_activeApp >= 0);  // it drew its own prompt
  if (!waitingForRemoteResponse && mode != M_MENU && (mode == M_OFFLINE || nodeIdentified)
      && !appJustOpened) printPrompt();
}

// Streamed put (a typed line longer than the line editor): the FTP app calls
// start once with the first part, chunk for every further full buffer and for the
// tail, and end when Enter is pressed. Same Begin/Chunk/End path as a short put.
// `name` may carry a dir/ prefix. Every text argument is NUL-terminated.
bool core_transfer_stream_start(const char* name, const char* initial) {
  char buf[MAX_NAME + 24];
  size_t n = name ? strlen(name) : 0;
  stActive = true;  // swallow the rest of the line even when we refuse it
  stErr = 255;
  if (!n || n >= sizeof(buf)) { printE(F("Bad name.")); return true; }
  memcpy(buf, name, n + 1);

  Loc saved = loc;
  char* a = buf;
  bool ok = resolveServicePath(a);
  Loc where = loc;
  loc = saved;
  if (!ok) return true;  // path error already printed
  if (where != L_EEPROM) {
    if (where == L_REMOTE_EEPROM) printWarn(F("Content too long."));  // same limit as a remote put
    else errStore();
    return true;
  }
  putLocalBegin(a, initial);
  return true;
}

bool core_transfer_stream_chunk(const char* data, uint8_t len) {
  if (!stActive) return false;
  if (stErr || !data || !len) return true;  // already failed / nothing to add
  uint8_t nl = 0; uint16_t cl = 0; int caddr = 0;
  int rec = eepromFind(stName, &nl, &cl, &caddr);
  if (rec < 0) { stErr = 1; return true; }
  int add = len;
  if ((uint32_t)cl + add > MAX_EEPROM_FILE || eepromEnd() + add >= (int)eeLen()) { stErr = 2; return true; }
  eepromTouch();
  int tail = caddr + cl;
  int end = eepromEnd();
  for (int i = end; i >= tail; i--) eePut(i + add, eeGet(i));  // open a gap behind the file
  for (int i = 0; i < add; i++) eePut(tail + i, data[i]);
  eePut(rec + 1 + nl, 0xFF);
  eePut(rec + 2 + nl, (uint8_t)((cl + add) & 0xFF));
  eePut(rec + 3 + nl, (uint8_t)((cl + add) >> 8));
  return true;
}

bool core_transfer_stream_end(void) {
  if (!stActive) return false;
  putLocalEnd();
  printPrompt();
  return true;
}

void pollSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    // Tera Term Backspace / Delete support
    if (c == '\b' || c == 127) {
      if (inLen > 0) {
        inLen--;
        Serial.print(F("\b \b"));
      }
      continue;
    }

    if (c == '\n' && lastWasCR) {
      lastWasCR = false;
      continue;
    }
    lastWasCR = (c == '\r');
    if (c == '\r' || c == '\n') {
      if (stActive) {
        inBuf[inLen] = 0;
        core_apps_stream(inBuf, inLen, true);
        inLen = 0;
        return;
      }
      if (inOverflow) {
        inOverflow = false;
        inLen = 0;
        errHead();
        Serial.print(F("Line too long (max "));
        Serial.print(INBUF_LEN);
        Serial.println(F(" chars)." ANSI_RESET));
        printPrompt();
        return;
      }
      inBuf[inLen] = 0;
      inLen = 0;
      processLine(inBuf);
      return;
    }
    if (inLen >= INBUF_LEN) {
      if (inOverflow) continue;
      inBuf[inLen] = 0;
      if (core_apps_stream(inBuf, inLen, false)) {
        inLen = 0;
        continue;
      }
      inOverflow = true;
      continue;
    }
    inBuf[inLen++] = c;
    Serial.print(c);
  }
}

// --------------------------------------------------------------------
// EEPROM wear-levelling rotation
// Cyclic shift of the whole data area by d bytes; each cell is written
// exactly once. Single cycle only because eeLen()==1021 is prime.
// Runs only while idle (menu / offline).
// Fail-safe, NOT resumable: the magic byte is cleared while shifting and
// restored last, so a power loss mid-rotation is detected at the next
// boot (loadEEPROM_Storage) and the store is reformatted.
// --------------------------------------------------------------------
void eeRotate() {
  eeRotatePending = false;
  uint16_t L = eeLen();  // 1021 on the 328P (prime) -> any d gives one cycle
  uint16_t d = random(1, L);
  printWarn(F("\n[EEPROM] Wear levelling..."));
  EEPROM.update(0, 0);  // store invalid while shifting
  uint8_t carry = EEPROM.read(EE_HDR);
  uint16_t j = 0;
  do {
    j = (j + d) % L;
    uint8_t t = EEPROM.read(EE_HDR + j);
    EEPROM.update(EE_HDR + j, carry);
    carry = t;
  } while (j != 0);
  eeOff = (eeOff + d) % L;
  EEPROM.update(1, eeOff & 0xFF);
  EEPROM.update(2, eeOff >> 8);
  EEPROM.update(0, EEPROM_MAGIC);  // valid again, written last
  okPrompt(F("[EEPROM] Done."));
}

// Setup & Loop

// ====================================================================
//  Core API implementation (apps call these; see core_api.h)
// ====================================================================
bool core_register_app(const AppHooks* app) {
  if (g_appCount >= CORE_MAX_APPS || !app) return false;
  if (app->id) {  // wire id: printable, not ':', unique
    if (app->id < 33 || app->id > 126 || app->id == ':') return false;
    for (uint8_t i = 0; i < g_appCount; i++)
      if (g_apps[i]->id == app->id) return false;
  }
  g_apps[g_appCount++] = app;
  return true;
}

void core_apps_disconnect(void) {
  for (uint8_t i = 0; i < g_appCount; i++)  // not only the active app: any app may hold pending state
    if (g_apps[i]->on_disconnect) g_apps[i]->on_disconnect();
  g_activeApp = -1;
  stActive = false;  // a streamed put cannot finish: do not leave the line editor in stream mode
  stErr = 0;
}


// Delivery result of a queued app frame. Generic frames go only to their sender.
void core_apps_tx_done(uint8_t op_id, char app_id, bool ok) {
  for (uint8_t i = 0; i < g_appCount; i++) {
    const AppHooks* a = g_apps[i];
    if (!a->on_tx_done) continue;
    if (op_id == O_APP && a->id != app_id) continue;
    a->on_tx_done(op_id, ok);
  }
}

// "APP:<id>:<data>" -> on_frame(CORE_OP_APP, data) of the app owning <id>.
void core_apps_app_frame(const char* args) {
  char id = args[0];
  if (!id) return;
  const char* data = (args[1] == ':') ? args + 2 : EMPTY;
  for (uint8_t i = 0; i < g_appCount; i++)
    if (g_apps[i]->id == id && g_apps[i]->on_frame) g_apps[i]->on_frame(CORE_OP_APP, data);
}

bool core_apps_line(char* line) {
  if (g_activeApp < 0 || g_activeApp >= g_appCount) return false;
  // false = not the app's line: the core shell then tries its own commands
  return g_apps[g_activeApp]->on_line && g_apps[g_activeApp]->on_line(line);
}

bool core_apps_stream(char* data, uint8_t len, bool finish) {
  if (g_activeApp < 0 || g_activeApp >= g_appCount) return false;
  const AppHooks* a = g_apps[g_activeApp];
  return a->on_stream && a->on_stream(data, len, finish);
}

bool core_app_open(const char* name) {
  for (uint8_t i = 0; i < g_appCount; i++) {
    if (!strcmp(g_apps[i]->name, name)) {
      if (g_activeApp == (int8_t)i) return true;
      if (g_activeApp >= 0) {
        printE(F("Exit the current app first."));
        return false;
      }
      if (!g_apps[i]->on_open) {
        printE(F("App cannot be opened."));
        return false;
      }
      if (!g_apps[i]->on_open()) return false;
      g_activeApp = (int8_t)i;
      return true;
    }
  }
  printE(F("Unknown app."));
  return false;
}

void core_app_close(void) {
  if (g_activeApp >= 0) {  // confirm the exit (the app then prints the plain prompt)
    noticeHead();
    Serial.print(F("Left "));
    Serial.print(g_apps[g_activeApp]->name);
    endLine();
  }
  g_activeApp = -1;
}

bool core_online(void) {
  return mode == M_ONLINE && nodeIdentified;
}

const char* core_peer_name(void) {
  return remoteNodeName;
}

// Generic app message: "APP:<id>:<data>". Printable ASCII only (whitespace is collapsed).
bool core_app_send_p(char app_id, const char* progmem_fmt, ...) {
  if (app_id < 33 || app_id > 126 || app_id == ':' || qCount >= QUEUE_LEN || !core_online()) return false;
  // Do not put an undeliverable generic APP frame on the wire. The API
  // contract says invalid app IDs fail at queue time.
  bool registered = false;
  for (uint8_t i = 0; i < g_appCount; i++) {
    if (g_apps[i]->id == app_id) {
      registered = true;
      break;
    }
  }
  if (!registered) return false;
  char* q = txQ[(qHead + qCount) % QUEUE_LEN];
  strcpy_P(q, ALP(O_APP));
  uint8_t n = 3;
  q[n++] = ':';
  q[n++] = app_id;
  q[n++] = ':';
  va_list ap;
  va_start(ap, progmem_fmt);
  vfmtP(q + n, MAX_PAYLOAD + 1 - n, progmem_fmt, ap);
  va_end(ap);
  sanitize(q);
  qCount++;
  return true;
}

bool core_app_send(char app_id, const char* data) {
  return core_app_send_p(app_id, PSTR("%s"), data);
}



void core_clear_screen(void) { clearScreen(); }
void core_clear_line(void) { clrLine(); }
void core_print_prompt(void) {
  // Leaving an app: restore the current shell location without redrawing the banner.
  printPrompt();
}
void core_notice(const __FlashStringHelper* s) { printNotice(s); }
void core_notice_peer(const __FlashStringHelper* s) {
  if (viewing) {
    viewNote = s;
    viewNoteNode = true;
    return;
  }
  noticeHead();
  Serial.print(remoteNodeName);
  Serial.print(s);
  endLine();
  core_prompt_refresh();
}
// Redraw the shell prompt, then the half-typed input (use after async output).
void core_prompt_refresh(void) {
  if (viewing) return;
  printPrompt();
  if (!stActive && inLen) Serial.write((const uint8_t*)inBuf, inLen);
}
bool core_display_locked(void) { return viewing; }
uint8_t core_input_len(void) { return inLen; }
const char* core_input_buffer(void) { return inBuf; }  // inLen bytes, NOT NUL-terminated
void core_err(const __FlashStringHelper* s) { printErr(s); }

void setup() {
  Serial.begin(115200);  // optional: 115200 for faster 'view' (set Tera Term to match)

  randomSeed(analogRead(A1) + micros());  // before the EEPROM load: rotation amounts differ per boot
  loadEEPROM_Storage();

  myRandomId = random(0, 101);
  txSeqCounter = random(1, 256);

  pinMode(TX_PIN, INPUT);
  pinMode(RX_PIN, INPUT);
  rxTimerStart();  // Timer1 input capture on D8 (must run after the core's init())

  lastValidSignalTime = lastPulseMs = millis();
  mainMenu();  // boots silent: no broadcast until 'online'
}

void loop() {
  if (mode != M_ONLINE) {  // main menu / offline: radio stays idle
    pollSerial();
    if (eeRotatePending && inLen == 0 && !stActive) eeRotate();
    return;
  }
  runReceiverLogic();

  if (!nodeIdentified) {
    while (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'q' || c == 'Q') {
        leaveOnline();
        mainMenu();
        return;
      }
    }

    if (millis() - lastHelloTime > 2000 && canTransmit()) {
      sendHello();
      lastHelloTime = millis();
      Serial.print(F(ANSI_CYAN "." ANSI_RESET));
    }
    return;
  }

  pollSerial();
  checkKeepAlive();
  if (!nodeIdentified) return;
  checkLockExpiry();
  checkRemoteTimeout();
  xferPump();
  serviceTx();
}