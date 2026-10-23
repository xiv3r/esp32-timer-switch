#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <uri/UriBraces.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <RTClib.h>
#include <time.h>
#include <sys/time.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <type_traits>
#include "lwip/dns.h"
#include "lwip/tcpip.h"
#include "lwip/sockets.h"
#include "lwip/ip_addr.h"

#define FW_VERSION        "1.0.0"
#define ASSET_VERSION     "5"

static const uint8_t NUM_RELAYS   = 16;
static const uint8_t NUM_PAIRS    = 8;
static const uint8_t NTP_SERVERS  = 4;
static const size_t  SPEC_MAX     = 56;
static const size_t  RELAY_NAME_MAX     = 24;
static const int     PIN_BOOT_BTN = 0;
static const int     PIN_I2C_SDA  = 21;
static const int     PIN_I2C_SCL  = 22;
static const int     DAY_LOOKBACK = 366 * 8 + 2;
static const time_t  MIN_VALID_EPOCH = 1577836800;
static const uint32_t FACTORY_HOLD_MS = 5000;

static const int8_t DEFAULT_PINS[NUM_RELAYS] = {23, 32, 33, 25, 26, 27, 14, 13, 1, 3, 19, 18, 5, 4, 2, 15};
static const int8_t ALLOWED_PINS[] = {1, 2, 3, 4, 5, 13, 14, 15, 16, 17, 18, 19, 23, 25, 26, 27, 32, 33};
static const uint8_t ALLOWED_PIN_COUNT = sizeof(ALLOWED_PINS) / sizeof(ALLOWED_PINS[0]);

static const char* CFG_DIR   = "/cfg";
static const char* F_NET     = "/cfg/net.json";
static const char* F_TIME    = "/cfg/time.json";
static const char* F_GPIO    = "/cfg/gpio.json";

static const uint8_t TX_POWER_VALUES[] = {78, 68, 60, 52, 44, 34, 28, 20, 8};
static const char* const TX_POWER_LABELS[] = {"19.5 dBm (max)", "17 dBm", "15 dBm", "13 dBm", "11 dBm", "8.5 dBm", "7 dBm", "5 dBm", "2 dBm"};
static const uint8_t TX_POWER_COUNT = sizeof(TX_POWER_VALUES) / sizeof(TX_POWER_VALUES[0]);

struct StaCfg {
  bool     enabled;
  char     ssid[33];
  char     pass[65];
  bool     dhcp;
  uint32_t ip, gw, mask, dns1, dns2;
};

struct ApCfg {
  bool     enabled;
  char     ssid[33];
  char     pass[65];
  uint8_t  channel;
  bool     hidden;
  uint8_t  maxConn;
  uint8_t  txPower;
  uint32_t ip;
  bool     captive;
  uint8_t  mode;
};

struct NetCfg {
  char   hostname[33];
  StaCfg sta;
  ApCfg  ap;
};

struct TimeCfg {
  char     ntp[NTP_SERVERS][49];
  uint16_t intervalMin;
  uint16_t timeoutMs;
  bool     rotate;
  bool     rtcEnabled;
  bool     autoBrowser;
  uint8_t  tzMode;
  int32_t  gmtOffset;
  bool     dst;
  int32_t  dstOffset;
  uint8_t  dsM, dsW, dsD, dsH;
  uint8_t  deM, deW, deD, deH;
  char     posix[65];
  char     acName[41];
  char     acPosix[65];
  int32_t  acOffset;
  bool     acValid;
};

struct GpioCfg {
  int8_t pin[NUM_RELAYS];
  bool   activeLow[NUM_RELAYS];
};

enum TimeSrc : uint8_t { SRC_NONE = 0, SRC_RTC, SRC_NTP, SRC_BROWSER };
struct TimeTIs32 {};
struct TimeTIs64 {};

struct Spec {
  uint64_t sec;
  uint64_t min;
  uint32_t hour;
  uint32_t dom;
  uint16_t mon;
  uint8_t  dow;
  bool     never;
  uint16_t y0, y1;
};

struct Pair {
  bool en;
  bool active;
  char aText[SPEC_MAX + 1];
  char bText[SPEC_MAX + 1];
  Spec a;
  Spec b;
};

struct Relay {
  char    name[RELAY_NAME_MAX + 1];
  uint8_t mode;
  bool    state;
  bool    driven;
  Pair    p[NUM_PAIRS];
};

NetCfg   netCfg;
TimeCfg  timeCfg;
GpioCfg  gpioCfg;
Relay    relays[NUM_RELAYS];

WebServer  server(80);
DNSServer  dnsServer;
WiFiUDP    ntpUdp;
RTC_DS3231 rtc;

static const uint32_t D_NET = 1u << 0, D_TIME = 1u << 1, D_GPIO = 1u << 2, D_RELAY0 = 1u << 8;
static uint32_t dirtyMask = 0;
static uint32_t dirtyAtMs = 0;

static uint32_t restartAtMs = 0;
static bool     factoryPending = false;
static bool     fsOk = false;

static char jbuf[4096];

static inline bool timeIsValid(time_t t) { return t >= MIN_VALID_EPOCH; }
static inline bool clockValid() { return timeIsValid(time(nullptr)); }
static inline uint32_t uptimeSec() { return (uint32_t)(esp_timer_get_time() / 1000000ULL); }
static inline bool elapsed(uint32_t since, uint32_t ms) { return (uint32_t)(millis() - since) >= ms; }

static void strcopy(char* dst, size_t n, const char* src) {
  if (!dst || n == 0) return;
  if (!src) { dst[0] = 0; return; }
  strncpy(dst, src, n - 1);
  dst[n - 1] = 0;
}

static void trimInPlace(char* s) {
  if (!s) return;
  char* p = s;
  while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
  if (p != s) memmove(s, p, strlen(p) + 1);
  size_t l = strlen(s);
  while (l && (s[l - 1] == ' ' || s[l - 1] == '\t' || s[l - 1] == '\r' || s[l - 1] == '\n')) s[--l] = 0;
}

static bool validHostname(const char* h) {
  size_t l = strlen(h);
  if (l < 1 || l > 32) return false;
  if (h[0] == '-' || h[l - 1] == '-') return false;
  for (size_t i = 0; i < l; i++) {
    char c = h[i];
    if (!(isalnum((unsigned char)c) || c == '-')) return false;
  }
  return true;
}

static bool validHostOrIp(const char* h) {
  size_t l = strlen(h);
  if (l < 1 || l > 48) return false;
  for (size_t i = 0; i < l; i++) {
    char c = h[i];
    if (!(isalnum((unsigned char)c) || c == '-' || c == '.' || c == '_')) return false;
  }
  return true;
}

static bool pinAllowed(int pin) {
  for (uint8_t i = 0; i < ALLOWED_PIN_COUNT; i++) if (ALLOWED_PINS[i] == pin) return true;
  return false;
}

static bool pinReserved(int pin) {
  return pin == PIN_BOOT_BTN || pin == PIN_I2C_SDA || pin == PIN_I2C_SCL;
}

static bool txPowerAllowed(uint8_t v) {
  for (uint8_t i = 0; i < TX_POWER_COUNT; i++) if (TX_POWER_VALUES[i] == v) return true;
  return false;
}

static int64_t daysFromCivil(int y, int m, int d) {
  y -= m <= 2;
  const int64_t era = (y >= 0 ? y : y - 399) / 400;
  const int yoe = (int)(y - era * 400);
  const int doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + doe - 719468;
}

static void civilFromDays(int64_t z, int& y, int& m, int& d) {
  z += 719468;
  const int64_t era = (z >= 0 ? z : z - 146096) / 146097;
  const int doe = (int)(z - era * 146097);
  const int yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const int64_t yy = (int64_t)yoe + era * 400;
  const int doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const int mp = (5 * doy + 2) / 153;
  d = doy - (153 * mp + 2) / 5 + 1;
  m = mp + (mp < 10 ? 3 : -9);
  y = (int)(yy + (m <= 2));
}

static inline int dowFromDays(int64_t days) {
  int64_t r = (days + 4) % 7;
  if (r < 0) r += 7;
  return (int)r;
}

static int64_t localPseudoEpoch(const struct tm& t) {
  return daysFromCivil(t.tm_year + 1900, t.tm_mon + 1, t.tm_mday) * 86400LL + t.tm_hour * 3600 + t.tm_min * 60 + t.tm_sec;
}

static int32_t currentUtcOffset(time_t t) {
  struct tm lt, gt;
  localtime_r(&t, &lt);
  gmtime_r(&t, &gt);
  return (int32_t)(localPseudoEpoch(lt) - localPseudoEpoch(gt));
}

static uint32_t crc32Update(uint32_t crc, const uint8_t* data, size_t len) {
  crc = ~crc;
  while (len--) {
    crc ^= *data++;
    for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

[[deprecated("This Arduino-ESP32 core has a 32-bit time_t: the clock cannot represent dates after 2038-01-19. Build with core 3.x for post-2038 operation.")]]
static inline void checkTimeTWidth(TimeTIs32) {}
static inline void checkTimeTWidth(TimeTIs64) {}
static inline bool timeT64() { return sizeof(time_t) >= 8; }

static const char* resetReasonStr() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "Power-on";
    case ESP_RST_EXT: return "External reset";
    case ESP_RST_SW: return "Software reset";
    case ESP_RST_PANIC: return "Panic / exception";
    case ESP_RST_INT_WDT: return "Interrupt watchdog";
    case ESP_RST_TASK_WDT: return "Task watchdog";
    case ESP_RST_WDT: return "Other watchdog";
    case ESP_RST_DEEPSLEEP: return "Deep-sleep wake";
    case ESP_RST_BROWNOUT: return "Brownout";
    case ESP_RST_SDIO: return "SDIO";
    default: return "Unknown";
  }
}

static const char* wifiReasonStr(uint8_t r) {
  switch (r) {
    case 1: return "Unspecified";
    case 2: return "Auth expired";
    case 3: return "Auth leave";
    case 4: return "Assoc expired";
    case 5: return "AP too busy";
    case 6: case 7: return "Class 2/3 frame from non-auth STA";
    case 8: return "Assoc leave";
    case 15: return "4-way handshake timeout (bad password?)";
    case 16: return "Group key update timeout";
    case 200: return "Beacon timeout";
    case 201: return "AP not found";
    case 202: return "Auth failed";
    case 203: return "Assoc failed";
    case 204: return "Handshake timeout";
    case 205: return "Connection failed";
    case 206: return "AP TSF reset";
    case 207: return "Roaming";
    default: return "Other";
  }
}

class CrcFilePrint : public Print {
 public:
  explicit CrcFilePrint(File& f) : _f(f) {}
  size_t write(uint8_t c) override { _crc = crc32Update(_crc, &c, 1); _n++; return _f.write(c); }
  size_t write(const uint8_t* b, size_t l) override { _crc = crc32Update(_crc, b, l); _n += l; return _f.write(b, l); }
  uint32_t crc() const { return _crc; }
  size_t   count() const { return _n; }
 private:
  File& _f;
  uint32_t _crc = 0;
  size_t   _n = 0;
};

static bool fileCrc(const char* path, uint32_t& crc, size_t& size) {
  File f = LittleFS.open(path, "r");
  if (!f) return false;
  uint8_t buf[256];
  crc = 0; size = 0;
  while (f.available()) {
    int n = f.read(buf, sizeof(buf));
    if (n <= 0) break;
    crc = crc32Update(crc, buf, (size_t)n);
    size += (size_t)n;
  }
  f.close();
  return true;
}

static void pathWithSuffix(char* out, size_t n, const char* path, const char* suf) {
  snprintf(out, n, "%s%s", path, suf);
}

static bool saveJsonAtomic(const char* path, JsonDocument& doc) {
  if (!fsOk) return false;
  char tmp[48], bak[48];
  pathWithSuffix(tmp, sizeof(tmp), path, ".tmp");
  pathWithSuffix(bak, sizeof(bak), path, ".bak");

  LittleFS.remove(tmp);
  File f = LittleFS.open(tmp, "w");
  if (!f) { return false; }
  CrcFilePrint cp(f);
  size_t written = serializeJson(doc, cp);
  f.flush();
  f.close();
  if (written == 0 || written != cp.count()) { LittleFS.remove(tmp); return false; }

  uint32_t vcrc = 0; size_t vsize = 0;
  if (!fileCrc(tmp, vcrc, vsize) || vsize != written || vcrc != cp.crc()) {
    LittleFS.remove(tmp);
    return false;
  }
  if (LittleFS.exists(path)) {
    LittleFS.remove(bak);
    LittleFS.rename(path, bak);
  }
  if (!LittleFS.rename(tmp, path)) {
    if (!LittleFS.exists(path) && LittleFS.exists(bak)) LittleFS.rename(bak, path);
    LittleFS.remove(tmp);
    return false;
  }
  return true;
}

static bool loadJsonFrom(const char* path, JsonDocument& doc) {
  File f = LittleFS.open(path, "r");
  if (!f) return false;
  if (f.size() == 0 || f.size() > 16384) { f.close(); return false; }
  DeserializationError e = deserializeJson(doc, f);
  f.close();
  if (e) { return false; }
  return doc.is<JsonObject>();
}

static bool loadJsonAtomic(const char* path, JsonDocument& doc, bool* needsRewrite) {
  if (needsRewrite) *needsRewrite = false;
  if (!fsOk) return false;
  char tmp[48], bak[48];
  pathWithSuffix(tmp, sizeof(tmp), path, ".tmp");
  pathWithSuffix(bak, sizeof(bak), path, ".bak");
  if (loadJsonFrom(path, doc)) { LittleFS.remove(tmp); return true; }
  doc.clear();
  if (loadJsonFrom(tmp, doc)) {
    LittleFS.remove(path);
    LittleFS.rename(tmp, path);
    return true;
  }
  doc.clear();
  if (loadJsonFrom(bak, doc)) {
    if (needsRewrite) *needsRewrite = true;
    return true;
  }
  doc.clear();
  return false;
}

static void relayFilePath(char* out, size_t n, uint8_t i) { snprintf(out, n, "%s/relay_%02u.json", CFG_DIR, (unsigned)i); }

static void defaultNetCfg() {
  memset(&netCfg, 0, sizeof(netCfg));
  uint8_t mac[6];
  WiFi.macAddress(mac);
  strcopy(netCfg.hostname, sizeof(netCfg.hostname), "esp32-relay");
  netCfg.sta.enabled = false;
  netCfg.sta.dhcp = true;
  netCfg.ap.enabled = true;
  snprintf(netCfg.ap.ssid, sizeof(netCfg.ap.ssid), "ESP32-Relay-%02X%02X", mac[4], mac[5]);
  strcopy(netCfg.ap.pass, sizeof(netCfg.ap.pass), "esp32relay");
  netCfg.ap.channel = 1;
  netCfg.ap.hidden = false;
  netCfg.ap.maxConn = 4;
  netCfg.ap.txPower = 78;
  netCfg.ap.ip = (uint32_t)IPAddress(192, 168, 4, 1);
  netCfg.ap.captive = true;
  netCfg.ap.mode = 0;
}

static void defaultTimeCfg() {
  memset(&timeCfg, 0, sizeof(timeCfg));
  strcopy(timeCfg.ntp[0], 49, "pool.ntp.org");
  strcopy(timeCfg.ntp[1], 49, "time.google.com");
  strcopy(timeCfg.ntp[2], 49, "time.cloudflare.com");
  strcopy(timeCfg.ntp[3], 49, "time.nist.gov");
  timeCfg.intervalMin = 60;
  timeCfg.timeoutMs = 2500;
  timeCfg.rotate = false;
  timeCfg.rtcEnabled = true;
  timeCfg.autoBrowser = true;
  timeCfg.tzMode = 0;
  timeCfg.gmtOffset = 0;
  timeCfg.dst = false;
  timeCfg.dstOffset = 3600;
  timeCfg.dsM = 3; timeCfg.dsW = 2; timeCfg.dsD = 0; timeCfg.dsH = 2;
  timeCfg.deM = 11; timeCfg.deW = 1; timeCfg.deD = 0; timeCfg.deH = 2;
}

static void defaultGpioCfg() {
  for (uint8_t i = 0; i < NUM_RELAYS; i++) { gpioCfg.pin[i] = DEFAULT_PINS[i]; gpioCfg.activeLow[i] = true; }
}

static void parseSpecInto(Pair& p);
static void defaultRelay(uint8_t i) {
  Relay& r = relays[i];
  memset(&r, 0, sizeof(Relay));
  snprintf(r.name, sizeof(r.name), "Relay %u", (unsigned)(i + 1));
  r.mode = 0;
  for (uint8_t k = 0; k < NUM_PAIRS; k++) {
    r.p[k].en = false;
    r.p[k].aText[0] = 0;
    r.p[k].bText[0] = 0;
    parseSpecInto(r.p[k]);
  }
}

static void ipToStr(uint32_t ip, char* out, size_t n) { IPAddress a(ip); snprintf(out, n, "%u.%u.%u.%u", a[0], a[1], a[2], a[3]); }
static uint32_t strToIp(const char* s) { IPAddress a; if (s && a.fromString(s)) return (uint32_t)a; return 0; }

static void netToDoc(JsonDocument& d, bool secrets) {
  d["v"] = 1;
  d["host"] = netCfg.hostname;
  JsonObject s = d["sta"].to<JsonObject>();
  s["en"] = netCfg.sta.enabled;
  s["ssid"] = netCfg.sta.ssid;
  s["pass"] = secrets ? netCfg.sta.pass : "";
  s["passSet"] = netCfg.sta.pass[0] != 0;
  s["dhcp"] = netCfg.sta.dhcp;
  char b[16];
  ipToStr(netCfg.sta.ip, b, sizeof b);   s["ip"] = b;
  ipToStr(netCfg.sta.gw, b, sizeof b);   s["gw"] = b;
  ipToStr(netCfg.sta.mask, b, sizeof b); s["mask"] = b;
  ipToStr(netCfg.sta.dns1, b, sizeof b); s["dns1"] = b;
  ipToStr(netCfg.sta.dns2, b, sizeof b); s["dns2"] = b;
  JsonObject a = d["ap"].to<JsonObject>();
  a["en"] = netCfg.ap.enabled;
  a["ssid"] = netCfg.ap.ssid;
  a["pass"] = secrets ? netCfg.ap.pass : "";
  a["passSet"] = netCfg.ap.pass[0] != 0;
  a["ch"] = netCfg.ap.channel;
  a["hidden"] = netCfg.ap.hidden;
  a["max"] = netCfg.ap.maxConn;
  a["tx"] = netCfg.ap.txPower;
  ipToStr(netCfg.ap.ip, b, sizeof b); a["ip"] = b;
  a["captive"] = netCfg.ap.captive;
  a["mode"] = netCfg.ap.mode;
}

static void timeToDoc(JsonDocument& d) {
  d["v"] = 1;
  JsonArray n = d["ntp"].to<JsonArray>();
  for (uint8_t i = 0; i < NTP_SERVERS; i++) n.add(timeCfg.ntp[i]);
  d["interval"] = timeCfg.intervalMin;
  d["timeout"] = timeCfg.timeoutMs;
  d["rotate"] = timeCfg.rotate;
  d["rtc"] = timeCfg.rtcEnabled;
  d["autoBrowser"] = timeCfg.autoBrowser;
  d["tzMode"] = timeCfg.tzMode;
  d["gmtOffset"] = timeCfg.gmtOffset;
  d["dst"] = timeCfg.dst;
  d["dstOffset"] = timeCfg.dstOffset;
  d["dsM"] = timeCfg.dsM; d["dsW"] = timeCfg.dsW; d["dsD"] = timeCfg.dsD; d["dsH"] = timeCfg.dsH;
  d["deM"] = timeCfg.deM; d["deW"] = timeCfg.deW; d["deD"] = timeCfg.deD; d["deH"] = timeCfg.deH;
  d["posix"] = timeCfg.posix;
  d["acName"] = timeCfg.acName;
  d["acPosix"] = timeCfg.acPosix;
  d["acOffset"] = timeCfg.acOffset;
  d["acValid"] = timeCfg.acValid;
}

static void gpioToDoc(JsonDocument& d) {
  d["v"] = 1;
  JsonArray a = d["r"].to<JsonArray>();
  for (uint8_t i = 0; i < NUM_RELAYS; i++) {
    JsonObject o = a.add<JsonObject>();
    o["pin"] = gpioCfg.pin[i];
    o["al"] = gpioCfg.activeLow[i];
  }
}

static void relayToDoc(uint8_t i, JsonDocument& d, bool live) {
  Relay& r = relays[i];
  d["v"] = 1;
  d["i"] = i;
  d["name"] = r.name;
  d["mode"] = r.mode;
  if (live) {
    d["state"] = r.state;
    d["pin"] = gpioCfg.pin[i];
    d["al"] = gpioCfg.activeLow[i];
  }
  JsonArray s = d["s"].to<JsonArray>();
  for (uint8_t k = 0; k < NUM_PAIRS; k++) {
    JsonObject o = s.add<JsonObject>();
    o["en"] = r.p[k].en;
    o["a"] = r.p[k].aText;
    o["b"] = r.p[k].bText;
    if (live) o["act"] = r.p[k].active;
  }
}

static void netFromDoc(JsonDocument& d) {
  JsonObject s = d["sta"], a = d["ap"];
  if (d["host"].is<const char*>() && validHostname(d["host"])) strcopy(netCfg.hostname, sizeof(netCfg.hostname), d["host"]);
  if (!s.isNull()) {
    netCfg.sta.enabled = s["en"] | netCfg.sta.enabled;
    if (s["ssid"].is<const char*>()) strcopy(netCfg.sta.ssid, sizeof(netCfg.sta.ssid), s["ssid"]);
    if (s["pass"].is<const char*>()) strcopy(netCfg.sta.pass, sizeof(netCfg.sta.pass), s["pass"]);
    netCfg.sta.dhcp = s["dhcp"] | netCfg.sta.dhcp;
    if (s["ip"].is<const char*>())   netCfg.sta.ip = strToIp(s["ip"]);
    if (s["gw"].is<const char*>())   netCfg.sta.gw = strToIp(s["gw"]);
    if (s["mask"].is<const char*>()) netCfg.sta.mask = strToIp(s["mask"]);
    if (s["dns1"].is<const char*>()) netCfg.sta.dns1 = strToIp(s["dns1"]);
    if (s["dns2"].is<const char*>()) netCfg.sta.dns2 = strToIp(s["dns2"]);
  }
  if (!a.isNull()) {
    netCfg.ap.enabled = a["en"] | netCfg.ap.enabled;
    if (a["ssid"].is<const char*>() && strlen(a["ssid"]) > 0) strcopy(netCfg.ap.ssid, sizeof(netCfg.ap.ssid), a["ssid"]);
    if (a["pass"].is<const char*>()) strcopy(netCfg.ap.pass, sizeof(netCfg.ap.pass), a["pass"]);
    int ch = a["ch"] | (int)netCfg.ap.channel; if (ch >= 1 && ch <= 13) netCfg.ap.channel = ch;
    netCfg.ap.hidden = a["hidden"] | netCfg.ap.hidden;
    int mx = a["max"] | (int)netCfg.ap.maxConn; if (mx >= 1 && mx <= 8) netCfg.ap.maxConn = mx;
    int tx = a["tx"] | (int)netCfg.ap.txPower; if (txPowerAllowed(tx)) netCfg.ap.txPower = tx;
    if (a["ip"].is<const char*>()) { uint32_t ip = strToIp(a["ip"]); if (ip) netCfg.ap.ip = ip; }
    netCfg.ap.captive = a["captive"] | netCfg.ap.captive;
    int md = a["mode"] | (int)netCfg.ap.mode; if (md == 0 || md == 1) netCfg.ap.mode = md;
  }
  if (strlen(netCfg.ap.pass) > 0 && strlen(netCfg.ap.pass) < 8) netCfg.ap.pass[0] = 0;
}

static void timeFromDoc(JsonDocument& d) {
  JsonArray n = d["ntp"];
  if (!n.isNull()) {
    for (uint8_t i = 0; i < NTP_SERVERS; i++) {
      const char* v = (i < n.size()) ? (const char*)n[i] : nullptr;
      strcopy(timeCfg.ntp[i], 49, v ? v : "");
    }
  }
  int iv = d["interval"] | (int)timeCfg.intervalMin; if (iv >= 1 && iv <= 1440) timeCfg.intervalMin = iv;
  int to = d["timeout"] | (int)timeCfg.timeoutMs; if (to >= 500 && to <= 10000) timeCfg.timeoutMs = to;
  timeCfg.rotate = d["rotate"] | timeCfg.rotate;
  timeCfg.rtcEnabled = d["rtc"] | timeCfg.rtcEnabled;
  timeCfg.autoBrowser = d["autoBrowser"] | timeCfg.autoBrowser;
  int tm = d["tzMode"] | (int)timeCfg.tzMode; if (tm == 0 || tm == 1) timeCfg.tzMode = tm;
  long go = d["gmtOffset"] | (long)timeCfg.gmtOffset; if (go >= -50400 && go <= 50400) timeCfg.gmtOffset = go;
  timeCfg.dst = d["dst"] | timeCfg.dst;
  long dof = d["dstOffset"] | (long)timeCfg.dstOffset; if (dof >= -7200 && dof <= 7200) timeCfg.dstOffset = dof;
  auto rd = [&](const char* k, uint8_t& dst, int lo, int hi) { int v = d[k] | (int)dst; if (v >= lo && v <= hi) dst = v; };
  rd("dsM", timeCfg.dsM, 1, 12); rd("dsW", timeCfg.dsW, 1, 5); rd("dsD", timeCfg.dsD, 0, 6); rd("dsH", timeCfg.dsH, 0, 24);
  rd("deM", timeCfg.deM, 1, 12); rd("deW", timeCfg.deW, 1, 5); rd("deD", timeCfg.deD, 0, 6); rd("deH", timeCfg.deH, 0, 24);
  if (d["posix"].is<const char*>()) strcopy(timeCfg.posix, sizeof(timeCfg.posix), d["posix"]);
  if (d["acName"].is<const char*>()) strcopy(timeCfg.acName, sizeof(timeCfg.acName), d["acName"]);
  if (d["acPosix"].is<const char*>()) strcopy(timeCfg.acPosix, sizeof(timeCfg.acPosix), d["acPosix"]);
  timeCfg.acOffset = d["acOffset"] | timeCfg.acOffset;
  timeCfg.acValid = d["acValid"] | timeCfg.acValid;
}

static void gpioFromDoc(JsonDocument& d) {
  JsonArray a = d["r"];
  if (a.isNull()) return;
  for (uint8_t i = 0; i < NUM_RELAYS && i < a.size(); i++) {
    int pin = a[i]["pin"] | (int)gpioCfg.pin[i];
    gpioCfg.pin[i] = (pin == -1 || pinAllowed(pin)) ? (int8_t)pin : (int8_t)-1;
    gpioCfg.activeLow[i] = a[i]["al"] | gpioCfg.activeLow[i];
  }
  for (uint8_t i = 0; i < NUM_RELAYS; i++)
    for (uint8_t j = i + 1; j < NUM_RELAYS; j++)
      if (gpioCfg.pin[i] >= 0 && gpioCfg.pin[i] == gpioCfg.pin[j]) gpioCfg.pin[j] = -1;
}

static const char* parseSpec(const char* str, Spec& sp);
static void relayFromDoc(uint8_t i, JsonDocument& d) {
  Relay& r = relays[i];
  if (d["name"].is<const char*>() && strlen(d["name"]) > 0) { strcopy(r.name, sizeof(r.name), d["name"]); trimInPlace(r.name); if (!r.name[0]) snprintf(r.name, sizeof(r.name), "Relay %u", i + 1); }
  int m = d["mode"] | (int)r.mode; if (m >= 0 && m <= 2) r.mode = m;
  JsonArray s = d["s"];
  if (s.isNull()) return;
  for (uint8_t k = 0; k < NUM_PAIRS && k < s.size(); k++) {
    JsonObject o = s[k];
    Pair& p = r.p[k];
    p.en = o["en"] | false;
    strcopy(p.aText, sizeof(p.aText), o["a"] | "");
    strcopy(p.bText, sizeof(p.bText), o["b"] | "");
    trimInPlace(p.aText); trimInPlace(p.bText);
    parseSpecInto(p);
  }
}

static void parseSpecInto(Pair& p) {
  if (parseSpec(p.aText, p.a) != nullptr) { p.en = false; p.aText[0] = 0; parseSpec("", p.a); }
  if (parseSpec(p.bText, p.b) != nullptr) { p.en = false; p.bText[0] = 0; parseSpec("", p.b); }
  if (!p.en) p.active = false;
}

static bool saveNet()  { JsonDocument d; netToDoc(d, true); return saveJsonAtomic(F_NET, d); }
static bool saveTime() { JsonDocument d; timeToDoc(d);      return saveJsonAtomic(F_TIME, d); }
static bool saveGpio() { JsonDocument d; gpioToDoc(d);      return saveJsonAtomic(F_GPIO, d); }
static bool saveRelay(uint8_t i) { JsonDocument d; relayToDoc(i, d, false); char p[40]; relayFilePath(p, sizeof p, i); return saveJsonAtomic(p, d); }

static void markDirty(uint32_t bits) { dirtyMask |= bits; dirtyAtMs = millis(); }

static void saveDirtyNow() {
  if (!dirtyMask) return;
  uint32_t m = dirtyMask;
  dirtyMask = 0;
  if (m & D_NET)  saveNet();
  if (m & D_TIME) saveTime();
  if (m & D_GPIO) saveGpio();
  for (uint8_t i = 0; i < NUM_RELAYS; i++) if (m & (D_RELAY0 << i)) saveRelay(i);
}

static void storageTick() {
  if (dirtyMask && elapsed(dirtyAtMs, 1500)) saveDirtyNow();
}

static void loadAllConfig() {
  defaultNetCfg(); defaultTimeCfg(); defaultGpioCfg();
  for (uint8_t i = 0; i < NUM_RELAYS; i++) defaultRelay(i);
  if (!fsOk) return;
  bool rw;
  {
    JsonDocument d;
    if (loadJsonAtomic(F_NET, d, &rw)) { netFromDoc(d); if (rw) markDirty(D_NET); }
  }
  {
    JsonDocument d;
    if (loadJsonAtomic(F_TIME, d, &rw)) { timeFromDoc(d); if (rw) markDirty(D_TIME); }
  }
  {
    JsonDocument d;
    if (loadJsonAtomic(F_GPIO, d, &rw)) { gpioFromDoc(d); if (rw) markDirty(D_GPIO); }
  }
  for (uint8_t i = 0; i < NUM_RELAYS; i++) {
    JsonDocument d;
    char p[40]; relayFilePath(p, sizeof p, i);
    if (loadJsonAtomic(p, d, &rw)) { relayFromDoc(i, d); if (rw) markDirty(D_RELAY0 << i); }
  }
}

static void removeWithVariants(const char* path) {
  char b[48];
  LittleFS.remove(path);
  pathWithSuffix(b, sizeof b, path, ".tmp"); LittleFS.remove(b);
  pathWithSuffix(b, sizeof b, path, ".bak"); LittleFS.remove(b);
}

static void factoryResetFiles() {
  if (!fsOk) return;
  removeWithVariants(F_NET);
  removeWithVariants(F_TIME);
  removeWithVariants(F_GPIO);
  for (uint8_t i = 0; i < NUM_RELAYS; i++) { char p[40]; relayFilePath(p, sizeof p, i); removeWithVariants(p); }
  LittleFS.rmdir(CFG_DIR);
}

static void requestRestart(uint32_t delayMs) { restartAtMs = millis() + delayMs; if (restartAtMs == 0) restartAtMs = 1; }

static void requestFactoryReset() { factoryPending = true; requestRestart(700); }

static const char* const DOW_NAMES[7]  = {"sun", "mon", "tue", "wed", "thu", "fri", "sat"};
static const char* const MON_NAMES[12] = {"jan", "feb", "mar", "apr", "may", "jun", "jul", "aug", "sep", "oct", "nov", "dec"};

static bool parseValue(const char*& p, const char* const* names, int nameCount, int nameBase, int& out) {
  if (isdigit((unsigned char)*p)) {
    long v = 0; int digits = 0;
    while (isdigit((unsigned char)*p) && digits < 6) { v = v * 10 + (*p - '0'); p++; digits++; }
    if (isdigit((unsigned char)*p)) return false;
    out = (int)v;
    return true;
  }
  if (names && isalpha((unsigned char)*p)) {
    char nm[4] = {0, 0, 0, 0};
    int i = 0;
    while (isalpha((unsigned char)*p)) { if (i < 3) nm[i] = (char)tolower((unsigned char)*p); i++; p++; }
    if (i != 3) return false;
    for (int k = 0; k < nameCount; k++) if (strcmp(nm, names[k]) == 0) { out = nameBase + k; return true; }
  }
  return false;
}

static const char* parseField(const char* s, int lo, int hi, int stepMax, uint64_t& mask, const char* const* names, int nameCount, int nameBase) {
  mask = 0;
  const char* p = s;
  if (!*p) return "empty field";
  while (true) {
    int a, b, step = 1;
    bool star = false, range = false;
    if (*p == '*') { p++; a = lo; b = stepMax; star = true; }
    else {
      if (!parseValue(p, names, nameCount, nameBase, a)) return "bad value";
      b = a;
      if (*p == '-') { p++; if (!parseValue(p, names, nameCount, nameBase, b)) return "bad range"; range = true; }
    }
    if (*p == '/') {
      p++;
      if (!parseValue(p, nullptr, 0, 0, step) || step < 1) return "bad step";
      if (!star && !range) b = stepMax;
    }
    if (a < lo || a > hi || b < lo || b > hi) return "out of range";
    if (b < a) return "range reversed";
    for (int v = a; v <= b; v += step) mask |= (1ULL << v);
    if (*p == ',') { p++; if (!*p) return "trailing comma"; continue; }
    if (*p == 0) break;
    return "unexpected character";
  }
  return mask ? nullptr : "no values";
}

static const char* parseYearField(const char* s, uint16_t& y0, uint16_t& y1) {
  if (strcmp(s, "*") == 0) { y0 = 0; y1 = 65535; return nullptr; }
  const char* p = s; int a, b;
  if (!parseValue(p, nullptr, 0, 0, a)) return "bad year";
  b = a;
  if (*p == '-') { p++; if (!parseValue(p, nullptr, 0, 0, b)) return "bad year range"; }
  if (*p) return "unexpected character in year";
  if (a < 1970 || a > 2199 || b < 1970 || b > 2199) return "year out of range (1970-2199)";
  if (b < a) return "year range reversed";
  y0 = a; y1 = b;
  return nullptr;
}

static const char* FIELD_NAMES[7] = {"sec", "min", "hour", "dow", "dom", "month", "year"};
static char specErr[64];

static const char* parseSpec(const char* str, Spec& sp) {
  memset(&sp, 0, sizeof(sp));
  sp.never = true;
  if (!str) return nullptr;
  while (*str == ' ' || *str == '\t') str++;
  if (!*str) return nullptr;
  if (strlen(str) > SPEC_MAX) return "spec too long";

  char tokens[7][SPEC_MAX + 1];
  int n = 0;
  const char* p = str;
  while (*p && n < 7) {
    while (*p == ' ' || *p == '\t') p++;
    if (!*p) break;
    int i = 0;
    while (*p && *p != ' ' && *p != '\t') { if (i < (int)SPEC_MAX) tokens[n][i++] = *p; p++; }
    tokens[n][i] = 0;
    n++;
  }
  while (*p == ' ' || *p == '\t') p++;
  if (*p) return "too many fields (max 7)";
  for (int i = n; i < 7; i++) strcpy(tokens[i], "*");

  uint64_t m; const char* e;
  if ((e = parseField(tokens[0], 0, 59, 59, m, nullptr, 0, 0)))   { snprintf(specErr, sizeof specErr, "%s: %s", FIELD_NAMES[0], e); return specErr; } sp.sec = m;
  if ((e = parseField(tokens[1], 0, 59, 59, m, nullptr, 0, 0)))   { snprintf(specErr, sizeof specErr, "%s: %s", FIELD_NAMES[1], e); return specErr; } sp.min = m;
  if ((e = parseField(tokens[2], 0, 23, 23, m, nullptr, 0, 0)))   { snprintf(specErr, sizeof specErr, "%s: %s", FIELD_NAMES[2], e); return specErr; } sp.hour = (uint32_t)m;
  if ((e = parseField(tokens[3], 0, 7, 6, m, DOW_NAMES, 7, 0)))   { snprintf(specErr, sizeof specErr, "%s: %s", FIELD_NAMES[3], e); return specErr; }
  if (m & (1ULL << 7)) m |= 1ULL;
  sp.dow = (uint8_t)(m & 0x7F);
  if ((e = parseField(tokens[4], 1, 31, 31, m, nullptr, 0, 0)))   { snprintf(specErr, sizeof specErr, "%s: %s", FIELD_NAMES[4], e); return specErr; } sp.dom = (uint32_t)m;
  if ((e = parseField(tokens[5], 1, 12, 12, m, MON_NAMES, 12, 1))) { snprintf(specErr, sizeof specErr, "%s: %s", FIELD_NAMES[5], e); return specErr; } sp.mon = (uint16_t)m;
  if ((e = parseYearField(tokens[6], sp.y0, sp.y1)))         { snprintf(specErr, sizeof specErr, "%s: %s", FIELD_NAMES[6], e); return specErr; }
  sp.never = false;
  return nullptr;
}

static inline bool specMatch(const Spec& s, const struct tm& t) {
  if (s.never) return false;
  int y = t.tm_year + 1900;
  if (y < s.y0 || y > s.y1) return false;
  if (t.tm_sec > 59) return false;
  return ((s.sec >> t.tm_sec) & 1ULL) && ((s.min >> t.tm_min) & 1ULL) && ((s.hour >> t.tm_hour) & 1u) &&
         ((s.dow >> t.tm_wday) & 1u) && ((s.dom >> t.tm_mday) & 1u) && ((s.mon >> (t.tm_mon + 1)) & 1u);
}

static inline int highestBitLE(uint64_t mask, int k) {
  if (k < 0) return -1;
  if (k < 63) mask &= ((1ULL << (k + 1)) - 1ULL);
  if (!mask) return -1;
  return 63 - __builtin_clzll(mask);
}

static int latestTimeOfDay(const Spec& s, int limit) {
  int lh = limit / 3600, lm = (limit / 60) % 60, ls = limit % 60;
  int h = highestBitLE(s.hour, lh);
  while (h >= 0) {
    int mlim = (h == lh) ? lm : 59;
    int m = highestBitLE(s.min, mlim);
    while (m >= 0) {
      int slim = (h == lh && m == lm) ? ls : 59;
      int sec = highestBitLE(s.sec, slim);
      if (sec >= 0) return h * 3600 + m * 60 + sec;
      m = highestBitLE(s.min, m - 1);
    }
    h = highestBitLE(s.hour, h - 1);
  }
  return -1;
}

static int64_t findPrevMatch(const Spec& s, const struct tm& now) {
  if (s.never) return -1;
  const int64_t D0 = daysFromCivil(now.tm_year + 1900, now.tm_mon + 1, now.tm_mday);
  const int S0 = now.tm_hour * 3600 + now.tm_min * 60 + now.tm_sec;
  const int64_t Dmin = D0 - DAY_LOOKBACK;
  int64_t D = D0;
  int guard = 0;
  while (D > Dmin && guard++ < 20000) {
    int y, m, d;
    civilFromDays(D, y, m, d);
    if (y > s.y1) { D = daysFromCivil(s.y1, 12, 31); continue; }
    if (y < s.y0) return -1;
    if (!((s.mon >> m) & 1u)) { D = daysFromCivil(y, m, 1) - 1; continue; }
    if (!((s.dom >> d) & 1u) || !((s.dow >> dowFromDays(D)) & 1u)) { D--; continue; }
    int tod = latestTimeOfDay(s, (D == D0) ? S0 : 86399);
    if (tod >= 0) return D * 86400LL + tod;
    D--;
  }
  return -1;
}

static time_t  schedLastEval = 0;
static int64_t schedLastLocal = 0;
static bool    schedForceRecalc = true;
static uint32_t schedRecalcCount = 0;

static void requestRecalcAll() { schedForceRecalc = true; }

static void recalcRelay(uint8_t i, const struct tm& lt) {
  Relay& r = relays[i];
  for (uint8_t k = 0; k < NUM_PAIRS; k++) {
    Pair& p = r.p[k];
    if (!p.en) { p.active = false; continue; }
    int64_t ls = findPrevMatch(p.a, lt);
    int64_t le = findPrevMatch(p.b, lt);
    p.active = (ls >= 0) && (ls >= le);
  }
}

static void recalcRelayNow(uint8_t i) {
  time_t now = time(nullptr);
  if (!timeIsValid(now)) { for (uint8_t k = 0; k < NUM_PAIRS; k++) relays[i].p[k].active = false; return; }
  struct tm lt; localtime_r(&now, &lt);
  recalcRelay(i, lt);
}

static void recalcAll(const struct tm& lt) {
  for (uint8_t i = 0; i < NUM_RELAYS; i++) recalcRelay(i, lt);
  schedRecalcCount++;
}

static void evalSecond(const struct tm& lt) {
  for (uint8_t i = 0; i < NUM_RELAYS; i++) {
    Relay& r = relays[i];
    for (uint8_t k = 0; k < NUM_PAIRS; k++) {
      Pair& p = r.p[k];
      if (!p.en) continue;
      if (specMatch(p.a, lt)) p.active = true;
      else if (specMatch(p.b, lt)) p.active = false;
    }
  }
}

static void applyOutputs(bool force);

static void schedTick() {
  time_t now = time(nullptr);
  if (!timeIsValid(now)) {
    schedLastEval = 0;
    schedForceRecalc = true;
    applyOutputs(false);
    return;
  }
  if (!schedForceRecalc && now == schedLastEval) return;

  struct tm lt;
  if (schedForceRecalc || schedLastEval == 0 || now < schedLastEval || now - schedLastEval > 120) {
    localtime_r(&now, &lt);
    recalcAll(lt);
    schedForceRecalc = false;
  } else {
    for (time_t t = schedLastEval + 1; t <= now; t++) {
      localtime_r(&t, &lt);
      int64_t lp = localPseudoEpoch(lt);
      if (lp != schedLastLocal + 1) {
        recalcAll(lt);
        schedLastLocal = lp;
        continue;
      }
      evalSecond(lt);
      schedLastLocal = lp;
    }
  }
  localtime_r(&now, &lt);
  schedLastLocal = localPseudoEpoch(lt);
  schedLastEval = now;
  applyOutputs(false);
}

static inline int physicalLevel(uint8_t i, bool on) { return (on ^ gpioCfg.activeLow[i]) ? HIGH : LOW; }

static void releasePin(uint8_t i) {
  int pin = gpioCfg.pin[i];
  if (relays[i].driven && pin >= 0) pinMode(pin, INPUT);
  relays[i].driven = false;
}

static void setupPin(uint8_t i) {
  int pin = gpioCfg.pin[i];
  relays[i].driven = false;
  if (pin < 0 || pinReserved(pin) || !pinAllowed(pin)) return;
  int lvl = physicalLevel(i, relays[i].state);
  digitalWrite(pin, lvl);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, lvl);
  relays[i].driven = true;
}

static bool relayAnyActive(const Relay& r) {
  for (uint8_t k = 0; k < NUM_PAIRS; k++) if (r.p[k].en && r.p[k].active) return true;
  return false;
}

static bool desiredState(uint8_t i) {
  const Relay& r = relays[i];
  if (r.mode == 1) return true;
  if (r.mode == 2) return false;
  return clockValid() && relayAnyActive(r);
}

static void applyOutputs(bool force) {
  for (uint8_t i = 0; i < NUM_RELAYS; i++) {
    bool want = desiredState(i);
    if (want != relays[i].state || force) {
      relays[i].state = want;
      if (relays[i].driven) digitalWrite(gpioCfg.pin[i], physicalLevel(i, want));
    }
  }
}

static int readPinLevel(uint8_t i) {
  int pin = gpioCfg.pin[i];
  if (pin < 0) return -1;
  if (!relays[i].driven) return -1;
  return digitalRead(pin);
}

static void initAllPins() {
  for (uint8_t i = 0; i < NUM_RELAYS; i++) { relays[i].state = false; setupPin(i); }
}

static void reassignRelayPin(uint8_t i, int8_t newPin, bool newActiveLow) {
  if (newPin == gpioCfg.pin[i] && relays[i].driven) {
    gpioCfg.activeLow[i] = newActiveLow;
    digitalWrite(newPin, physicalLevel(i, relays[i].state));
    return;
  }
  releasePin(i);
  gpioCfg.pin[i] = newPin;
  gpioCfg.activeLow[i] = newActiveLow;
  setupPin(i);
}

static const char TZ_TABLE[] PROGMEM =
"\nEtc/UTC|UTC0\nUTC|UTC0\nEtc/GMT|UTC0"
"\nAmerica/New_York|EST5EDT,M3.2.0,M11.1.0\nAmerica/Detroit|EST5EDT,M3.2.0,M11.1.0\nAmerica/Toronto|EST5EDT,M3.2.0,M11.1.0"
"\nAmerica/Indiana/Indianapolis|EST5EDT,M3.2.0,M11.1.0\nAmerica/Kentucky/Louisville|EST5EDT,M3.2.0,M11.1.0\nAmerica/Nassau|EST5EDT,M3.2.0,M11.1.0"
"\nAmerica/Chicago|CST6CDT,M3.2.0,M11.1.0\nAmerica/Winnipeg|CST6CDT,M3.2.0,M11.1.0\nAmerica/Menominee|CST6CDT,M3.2.0,M11.1.0"
"\nAmerica/Denver|MST7MDT,M3.2.0,M11.1.0\nAmerica/Edmonton|MST7MDT,M3.2.0,M11.1.0\nAmerica/Boise|MST7MDT,M3.2.0,M11.1.0\nAmerica/Ciudad_Juarez|MST7MDT,M3.2.0,M11.1.0"
"\nAmerica/Phoenix|MST7\nAmerica/Hermosillo|MST7\nAmerica/Regina|CST6"
"\nAmerica/Los_Angeles|PST8PDT,M3.2.0,M11.1.0\nAmerica/Vancouver|PST8PDT,M3.2.0,M11.1.0\nAmerica/Tijuana|PST8PDT,M3.2.0,M11.1.0"
"\nAmerica/Anchorage|AKST9AKDT,M3.2.0,M11.1.0\nAmerica/Juneau|AKST9AKDT,M3.2.0,M11.1.0\nAmerica/Adak|HST10HDT,M3.2.0,M11.1.0\nPacific/Honolulu|HST10"
"\nAmerica/Halifax|AST4ADT,M3.2.0,M11.1.0\nAmerica/Moncton|AST4ADT,M3.2.0,M11.1.0\nAmerica/St_Johns|NST3:30NDT,M3.2.0,M11.1.0\nAmerica/Puerto_Rico|AST4\nAmerica/Santo_Domingo|AST4"
"\nAmerica/Mexico_City|CST6\nAmerica/Monterrey|CST6\nAmerica/Merida|CST6\nAmerica/Cancun|EST5\nAmerica/Mazatlan|MST7\nAmerica/Chihuahua|CST6"
"\nAmerica/Guatemala|CST6\nAmerica/El_Salvador|CST6\nAmerica/Tegucigalpa|CST6\nAmerica/Managua|CST6\nAmerica/Costa_Rica|CST6\nAmerica/Panama|EST5\nAmerica/Jamaica|EST5"
"\nAmerica/Havana|CST5CDT,M3.2.0/0,M11.1.0/1\nAmerica/Port-au-Prince|EST5EDT,M3.2.0,M11.1.0"
"\nAmerica/Bogota|<-05>5\nAmerica/Lima|<-05>5\nAmerica/Guayaquil|<-05>5\nAmerica/Caracas|<-04>4\nAmerica/La_Paz|<-04>4\nAmerica/Manaus|<-04>4\nAmerica/Guyana|<-04>4"
"\nAmerica/Santiago|<-04>4<-03>,M9.1.6/24,M4.1.6/24\nAmerica/Punta_Arenas|<-03>3\nAmerica/Asuncion|<-03>3\nAmerica/Montevideo|<-03>3"
"\nAmerica/Argentina/Buenos_Aires|<-03>3\nAmerica/Argentina/Cordoba|<-03>3\nAmerica/Sao_Paulo|<-03>3\nAmerica/Bahia|<-03>3\nAmerica/Fortaleza|<-03>3\nAmerica/Recife|<-03>3\nAmerica/Belem|<-03>3"
"\nAmerica/Paramaribo|<-03>3\nAmerica/Cayenne|<-03>3\nAmerica/Noronha|<-02>2"
"\nAtlantic/Bermuda|AST4ADT,M3.2.0,M11.1.0\nAtlantic/Azores|<-01>1<+00>,M3.5.0/0,M10.5.0/1\nAtlantic/Cape_Verde|<-01>1\nAtlantic/Reykjavik|GMT0\nAtlantic/Canary|WET0WEST,M3.5.0/1,M10.5.0\nAtlantic/Madeira|WET0WEST,M3.5.0/1,M10.5.0"
"\nEurope/London|GMT0BST,M3.5.0/1,M10.5.0\nEurope/Dublin|IST-1GMT0,M10.5.0,M3.5.0/1\nEurope/Lisbon|WET0WEST,M3.5.0/1,M10.5.0\nEurope/Jersey|GMT0BST,M3.5.0/1,M10.5.0\nEurope/Guernsey|GMT0BST,M3.5.0/1,M10.5.0\nEurope/Isle_of_Man|GMT0BST,M3.5.0/1,M10.5.0"
"\nEurope/Paris|CET-1CEST,M3.5.0,M10.5.0/3\nEurope/Berlin|CET-1CEST,M3.5.0,M10.5.0/3\nEurope/Madrid|CET-1CEST,M3.5.0,M10.5.0/3\nEurope/Rome|CET-1CEST,M3.5.0,M10.5.0/3\nEurope/Amsterdam|CET-1CEST,M3.5.0,M10.5.0/3"
"\nEurope/Brussels|CET-1CEST,M3.5.0,M10.5.0/3\nEurope/Vienna|CET-1CEST,M3.5.0,M10.5.0/3\nEurope/Zurich|CET-1CEST,M3.5.0,M10.5.0/3\nEurope/Stockholm|CET-1CEST,M3.5.0,M10.5.0/3\nEurope/Oslo|CET-1CEST,M3.5.0,M10.5.0/3"
"\nEurope/Copenhagen|CET-1CEST,M3.5.0,M10.5.0/3\nEurope/Warsaw|CET-1CEST,M3.5.0,M10.5.0/3\nEurope/Prague|CET-1CEST,M3.5.0,M10.5.0/3\nEurope/Budapest|CET-1CEST,M3.5.0,M10.5.0/3\nEurope/Belgrade|CET-1CEST,M3.5.0,M10.5.0/3"
"\nEurope/Zagreb|CET-1CEST,M3.5.0,M10.5.0/3\nEurope/Ljubljana|CET-1CEST,M3.5.0,M10.5.0/3\nEurope/Bratislava|CET-1CEST,M3.5.0,M10.5.0/3\nEurope/Luxembourg|CET-1CEST,M3.5.0,M10.5.0/3\nEurope/Monaco|CET-1CEST,M3.5.0,M10.5.0/3"
"\nEurope/Malta|CET-1CEST,M3.5.0,M10.5.0/3\nEurope/Andorra|CET-1CEST,M3.5.0,M10.5.0/3\nEurope/Tirane|CET-1CEST,M3.5.0,M10.5.0/3\nEurope/Sarajevo|CET-1CEST,M3.5.0,M10.5.0/3\nEurope/Skopje|CET-1CEST,M3.5.0,M10.5.0/3"
"\nEurope/Podgorica|CET-1CEST,M3.5.0,M10.5.0/3\nEurope/Vaduz|CET-1CEST,M3.5.0,M10.5.0/3\nEurope/San_Marino|CET-1CEST,M3.5.0,M10.5.0/3\nEurope/Vatican|CET-1CEST,M3.5.0,M10.5.0/3\nEurope/Gibraltar|CET-1CEST,M3.5.0,M10.5.0/3\nArctic/Longyearbyen|CET-1CEST,M3.5.0,M10.5.0/3"
"\nEurope/Athens|EET-2EEST,M3.5.0/3,M10.5.0/4\nEurope/Bucharest|EET-2EEST,M3.5.0/3,M10.5.0/4\nEurope/Sofia|EET-2EEST,M3.5.0/3,M10.5.0/4\nEurope/Helsinki|EET-2EEST,M3.5.0/3,M10.5.0/4\nEurope/Kiev|EET-2EEST,M3.5.0/3,M10.5.0/4\nEurope/Kyiv|EET-2EEST,M3.5.0/3,M10.5.0/4"
"\nEurope/Riga|EET-2EEST,M3.5.0/3,M10.5.0/4\nEurope/Tallinn|EET-2EEST,M3.5.0/3,M10.5.0/4\nEurope/Vilnius|EET-2EEST,M3.5.0/3,M10.5.0/4\nEurope/Chisinau|EET-2EEST,M3.5.0,M10.5.0/3\nAsia/Nicosia|EET-2EEST,M3.5.0/3,M10.5.0/4\nEurope/Nicosia|EET-2EEST,M3.5.0/3,M10.5.0/4"
"\nEurope/Istanbul|<+03>-3\nEurope/Moscow|MSK-3\nEurope/Minsk|<+03>-3\nEurope/Kaliningrad|EET-2\nEurope/Samara|<+04>-4\nEurope/Volgograd|MSK-3\nEurope/Simferopol|MSK-3"
"\nAfrica/Cairo|EET-2EEST,M4.5.5/0,M10.5.4/24\nAfrica/Johannesburg|SAST-2\nAfrica/Lagos|WAT-1\nAfrica/Nairobi|EAT-3\nAfrica/Casablanca|<+01>-1\nAfrica/Algiers|CET-1\nAfrica/Tunis|CET-1\nAfrica/Tripoli|EET-2"
"\nAfrica/Accra|GMT0\nAfrica/Abidjan|GMT0\nAfrica/Dakar|GMT0\nAfrica/Addis_Ababa|EAT-3\nAfrica/Dar_es_Salaam|EAT-3\nAfrica/Kampala|EAT-3\nAfrica/Khartoum|CAT-2\nAfrica/Maputo|CAT-2\nAfrica/Harare|CAT-2\nAfrica/Lusaka|CAT-2\nAfrica/Windhoek|CAT-2\nAfrica/Kinshasa|WAT-1\nAfrica/Luanda|WAT-1"
"\nAsia/Dubai|<+04>-4\nAsia/Muscat|<+04>-4\nAsia/Riyadh|<+03>-3\nAsia/Kuwait|<+03>-3\nAsia/Qatar|<+03>-3\nAsia/Bahrain|<+03>-3\nAsia/Baghdad|<+03>-3\nAsia/Aden|<+03>-3\nAsia/Tehran|<+0330>-3:30"
"\nAsia/Jerusalem|IST-2IDT,M3.4.4/26,M10.5.0\nAsia/Tel_Aviv|IST-2IDT,M3.4.4/26,M10.5.0\nAsia/Beirut|EET-2EEST,M3.5.0/0,M10.5.0/0\nAsia/Amman|<+03>-3\nAsia/Damascus|<+03>-3"
"\nAsia/Karachi|PKT-5\nAsia/Kolkata|IST-5:30\nAsia/Calcutta|IST-5:30\nAsia/Colombo|<+0530>-5:30\nAsia/Kathmandu|<+0545>-5:45\nAsia/Dhaka|<+06>-6\nAsia/Thimphu|<+06>-6\nAsia/Yangon|<+0630>-6:30\nAsia/Kabul|<+0430>-4:30"
"\nAsia/Bangkok|<+07>-7\nAsia/Ho_Chi_Minh|<+07>-7\nAsia/Saigon|<+07>-7\nAsia/Phnom_Penh|<+07>-7\nAsia/Vientiane|<+07>-7\nAsia/Jakarta|WIB-7\nAsia/Makassar|WITA-8\nAsia/Jayapura|WIT-9"
"\nAsia/Kuala_Lumpur|<+08>-8\nAsia/Singapore|<+08>-8\nAsia/Manila|PST-8\nAsia/Shanghai|CST-8\nAsia/Chongqing|CST-8\nAsia/Urumqi|<+06>-6\nAsia/Hong_Kong|HKT-8\nAsia/Macau|CST-8\nAsia/Taipei|CST-8\nAsia/Brunei|<+08>-8\nAsia/Ulaanbaatar|<+08>-8"
"\nAsia/Seoul|KST-9\nAsia/Pyongyang|KST-9\nAsia/Tokyo|JST-9\nAsia/Tashkent|<+05>-5\nAsia/Samarkand|<+05>-5\nAsia/Almaty|<+05>-5\nAsia/Bishkek|<+06>-6\nAsia/Dushanbe|<+05>-5\nAsia/Ashgabat|<+05>-5"
"\nAsia/Yekaterinburg|<+05>-5\nAsia/Omsk|<+06>-6\nAsia/Novosibirsk|<+07>-7\nAsia/Krasnoyarsk|<+07>-7\nAsia/Irkutsk|<+08>-8\nAsia/Yakutsk|<+09>-9\nAsia/Vladivostok|<+10>-10\nAsia/Magadan|<+11>-11\nAsia/Kamchatka|<+12>-12"
"\nAsia/Baku|<+04>-4\nAsia/Tbilisi|<+04>-4\nAsia/Yerevan|<+04>-4\nIndian/Maldives|<+05>-5\nIndian/Mauritius|<+04>-4\nIndian/Reunion|<+04>-4"
"\nAustralia/Sydney|AEST-10AEDT,M10.1.0,M4.1.0/3\nAustralia/Melbourne|AEST-10AEDT,M10.1.0,M4.1.0/3\nAustralia/Canberra|AEST-10AEDT,M10.1.0,M4.1.0/3\nAustralia/Hobart|AEST-10AEDT,M10.1.0,M4.1.0/3"
"\nAustralia/Brisbane|AEST-10\nAustralia/Lindeman|AEST-10\nAustralia/Adelaide|ACST-9:30ACDT,M10.1.0,M4.1.0/3\nAustralia/Broken_Hill|ACST-9:30ACDT,M10.1.0,M4.1.0/3\nAustralia/Darwin|ACST-9:30\nAustralia/Perth|AWST-8\nAustralia/Lord_Howe|<+1030>-10:30<+11>-11,M10.1.0,M4.1.0"
"\nPacific/Auckland|NZST-12NZDT,M9.5.0,M4.1.0/3\nPacific/Chatham|<+1245>-12:45<+1345>,M9.5.0/2:45,M4.1.0/3:45\nPacific/Fiji|<+12>-12\nPacific/Guam|ChST-10\nPacific/Port_Moresby|<+10>-10\nPacific/Noumea|<+11>-11"
"\nPacific/Tongatapu|<+13>-13\nPacific/Apia|<+13>-13\nPacific/Pago_Pago|SST11\nPacific/Tahiti|<-10>10\nPacific/Majuro|<+12>-12\nPacific/Guadalcanal|<+11>-11\nPacific/Efate|<+11>-11\nPacific/Kiritimati|<+14>-14\n";

static bool lookupPosixTz(const char* iana, char* out, size_t n) {
  if (!iana || !*iana || strlen(iana) > 40) return false;
  char needle[44];
  snprintf(needle, sizeof needle, "\n%s|", iana);
  const char* hit = strstr(TZ_TABLE, needle);
  if (!hit) return false;
  hit += strlen(needle);
  const char* end = strchr(hit, '\n');
  size_t len = end ? (size_t)(end - hit) : strlen(hit);
  if (len == 0 || len >= n) return false;
  memcpy(out, hit, len);
  out[len] = 0;
  return true;
}

static void fmtPosixOffset(char* o, size_t n, int32_t west) {
  const char* sign = west < 0 ? "-" : "";
  if (west < 0) west = -west;
  int h = west / 3600, m = (west / 60) % 60, s = west % 60;
  if (s) snprintf(o, n, "%s%d:%02d:%02d", sign, h, m, s);
  else if (m) snprintf(o, n, "%s%d:%02d", sign, h, m);
  else snprintf(o, n, "%s%d", sign, h);
}

static void buildFixedPosix(char* out, size_t n, int32_t gmtOffset) {
  char off[16];
  fmtPosixOffset(off, sizeof off, -gmtOffset);
  snprintf(out, n, "LT%s", off);
}

static void buildManualPosix(char* out, size_t n) {
  char off1[16], off2[16];
  fmtPosixOffset(off1, sizeof off1, -timeCfg.gmtOffset);
  if (!timeCfg.dst) { snprintf(out, n, "STD%s", off1); return; }
  fmtPosixOffset(off2, sizeof off2, -(timeCfg.gmtOffset + timeCfg.dstOffset));
  snprintf(out, n, "STD%sDST%s,M%u.%u.%u/%u,M%u.%u.%u/%u", off1, off2,
           timeCfg.dsM, timeCfg.dsW, timeCfg.dsD, timeCfg.dsH, timeCfg.deM, timeCfg.deW, timeCfg.deD, timeCfg.deH);
}

static char tzCurrent[80] = "UTC0";
static char tzSource[16] = "manual";

static void applyTimezone() {
  char buf[80];
  if (timeCfg.tzMode == 1 && timeCfg.acValid) {
    if (timeCfg.acPosix[0]) { strcopy(buf, sizeof buf, timeCfg.acPosix); strcopy(tzSource, sizeof tzSource, "auto-rule"); }
    else { buildFixedPosix(buf, sizeof buf, timeCfg.acOffset); strcopy(tzSource, sizeof tzSource, "auto-offset"); }
  } else if (timeCfg.tzMode == 1) {
    buildManualPosix(buf, sizeof buf);
    strcopy(tzSource, sizeof tzSource, "auto-pending");
  } else if (timeCfg.posix[0]) {
    strcopy(buf, sizeof buf, timeCfg.posix);
    strcopy(tzSource, sizeof tzSource, "posix");
  } else {
    buildManualPosix(buf, sizeof buf);
    strcopy(tzSource, sizeof tzSource, "manual");
  }
  if (strcmp(buf, tzCurrent) != 0 || getenv("TZ") == nullptr) {
    strcopy(tzCurrent, sizeof tzCurrent, buf);
    setenv("TZ", tzCurrent, 1);
    tzset();
  }
  requestRecalcAll();
}

static const char* const SRC_NAMES[] = {"none", "rtc", "ntp", "browser"};

static TimeSrc  timeSrc = SRC_NONE;
static time_t   lastSetEpoch = 0;
static uint32_t lastSetMs = 0;
static bool     rtcWritePending = false;
static uint32_t rtcWriteReqMs = 0;

static void rtcCancelAlign();

static void setSystemTime(double epoch, TimeSrc src) {
  struct timeval tv;
  tv.tv_sec = (time_t)epoch;
  tv.tv_usec = (suseconds_t)((epoch - (double)tv.tv_sec) * 1e6);
  if (tv.tv_usec < 0) tv.tv_usec = 0;
  if (tv.tv_usec > 999999) tv.tv_usec = 999999;
  settimeofday(&tv, nullptr);
  timeSrc = src;
  lastSetEpoch = tv.tv_sec;
  lastSetMs = millis();
  if (src != SRC_RTC) { rtcWritePending = true; rtcWriteReqMs = millis(); rtcCancelAlign(); }
  requestRecalcAll();
}

static bool     rtcPresent = false;
static bool     rtcLost = false;
static time_t   rtcEpoch = 0;
static float    rtcTemp = 0;
static int32_t  rtcDrift = 0;
static uint32_t rtcReadMs = 0, rtcTempMs = 0, rtcProbeMs = 0;
static bool     rtcAlignReq = false;
static uint32_t rtcAlignStartMs = 0;
static time_t   rtcAlignPrev = 0;
static uint32_t rtcSetCount = 0;

static void rtcProbe() {
  if (!timeCfg.rtcEnabled) { rtcPresent = false; return; }
  rtcPresent = rtc.begin(&Wire);
  if (rtcPresent) {
    rtcLost = rtc.lostPower();
    DateTime d = rtc.now();
    rtcEpoch = (time_t)d.unixtime();
    rtcTemp = rtc.getTemperature();
    rtcReadMs = rtcTempMs = millis();
    if (!rtcLost && !timeIsValid(rtcEpoch)) rtcLost = true;
  }
}

static bool rtcUsable() { return timeCfg.rtcEnabled && rtcPresent && !rtcLost && timeIsValid(rtcEpoch); }

static void rtcCancelAlign() { rtcAlignReq = false; }

static void rtcRequestAlignSet() {
  if (!rtcUsable()) return;
  rtcAlignPrev = (time_t)rtc.now().unixtime();
  rtcAlignReq = true;
  rtcAlignStartMs = millis();
}

static void rtcTick() {
  if (!timeCfg.rtcEnabled) { rtcPresent = false; rtcWritePending = false; rtcAlignReq = false; return; }
  if (!rtcPresent) {
    if (elapsed(rtcProbeMs, 30000)) { rtcProbeMs = millis(); rtcProbe(); }
    return;
  }
  if (rtcWritePending) {
    struct timeval tv; gettimeofday(&tv, nullptr);
    if (timeIsValid(tv.tv_sec) && (tv.tv_usec < 40000 || elapsed(rtcWriteReqMs, 1500))) {
      rtc.adjust(DateTime((uint32_t)tv.tv_sec));
      rtcWritePending = false;
      rtcLost = false;
      rtcEpoch = tv.tv_sec;
      rtcDrift = 0;
      rtcSetCount++;
      rtcReadMs = millis();
    }
    return;
  }
  if (rtcAlignReq) {
    DateTime d = rtc.now();
    time_t e = (time_t)d.unixtime();
    rtcEpoch = e; rtcReadMs = millis();
    if (e != rtcAlignPrev || elapsed(rtcAlignStartMs, 1300)) {
      rtcAlignReq = false;
      if (timeIsValid(e)) { setSystemTime((double)e + 0.001, SRC_RTC); rtcDrift = 0; }
    }
    return;
  }
  if (elapsed(rtcReadMs, 1000)) {
    rtcReadMs = millis();
    DateTime d = rtc.now();
    rtcEpoch = (time_t)d.unixtime();
    rtcDrift = (int32_t)(rtcEpoch - time(nullptr));
    if (!timeIsValid(rtcEpoch)) rtcLost = true;
  }
  if (elapsed(rtcTempMs, 10000)) { rtcTempMs = millis(); rtcTemp = rtc.getTemperature(); }
}

enum DnsOwner : uint8_t { DNS_FREE = 0, DNS_NTP = 1, DNS_TZ = 2 };
static volatile uint8_t  dnsState = 0;
static volatile uint32_t dnsResult = 0;
static volatile uint32_t dnsSeq = 0;
static uint8_t  dnsOwner = DNS_FREE;
static uint32_t dnsStartMs = 0;

static void dnsFoundCb(const char* name, const ip_addr_t* ipaddr, void* arg) {
  (void)name;
  if ((uint32_t)(uintptr_t)arg != dnsSeq) return;
  if (ipaddr && IP_IS_V4(ipaddr)) { dnsResult = ip_addr_get_ip4_u32(ipaddr); dnsState = 2; }
  else dnsState = 3;
}

static bool dnsBusy() { return dnsOwner != DNS_FREE; }

static void dnsRelease(uint8_t owner) {
  if (dnsOwner != owner) return;
  dnsSeq = dnsSeq + 1;
  dnsState = 0;
  dnsOwner = DNS_FREE;
}

static bool dnsResolveStart(const char* host, uint8_t owner) {
  if (dnsOwner != DNS_FREE) return false;
  dnsOwner = owner;
  dnsSeq = dnsSeq + 1;
  dnsStartMs = millis();
  IPAddress lit;
  if (lit.fromString(host)) { dnsResult = (uint32_t)lit; dnsState = 2; return true; }
  ip_addr_t addr;
  dnsState = 1;
  LOCK_TCPIP_CORE();
  err_t e = dns_gethostbyname(host, &addr, dnsFoundCb, (void*)(uintptr_t)dnsSeq);
  UNLOCK_TCPIP_CORE();
  if (e == ERR_OK) { if (IP_IS_V4(&addr)) { dnsResult = ip_addr_get_ip4_u32(&addr); dnsState = 2; } else dnsState = 3; }
  else if (e != ERR_INPROGRESS) dnsState = 3;
  if (dnsState == 3) { dnsRelease(owner); return false; }
  return true;
}

static int dnsResolvePoll(uint32_t& ip, uint8_t owner) {
  if (dnsOwner != owner) return -1;
  if (dnsState == 2) { ip = dnsResult; dnsRelease(owner); return 1; }
  if (dnsState == 3) { dnsRelease(owner); return -1; }
  if (elapsed(dnsStartMs, 6000)) { dnsRelease(owner); return -1; }
  return 0;
}

enum NtpState : uint8_t { NTP_IDLE = 0, NTP_RESOLVE, NTP_WAIT };
static NtpState ntpState = NTP_IDLE;
static uint32_t ntpDueMs = 0;
static bool     ntpDueNow = true;
static uint8_t  ntpIdx = 0;
static uint8_t  ntpCycleTries = 0;
static uint32_t ntpStateMs = 0;
static uint32_t ntpServerIp = 0;
static char     ntpServerCur[49] = "";
static uint8_t  ntpTxNonce[8];
static double   ntpT1 = 0;
static time_t   ntpLastOk = 0;
static char     ntpLastOkServer[49] = "";
static char     ntpLastErr[48] = "";
static uint32_t ntpFailCycles = 0;
static uint32_t ntpBackoffMs = 0;
static double   ntpLastOffsetMs = 0;
static uint32_t ntpOkCount = 0;
static bool     ntpUdpStarted = false;
static bool     staHasIp = false;

static uint8_t ntpServerCount() { uint8_t c = 0; for (uint8_t i = 0; i < NTP_SERVERS; i++) if (timeCfg.ntp[i][0]) c++; return c; }
static const char* ntpServerAt(uint8_t idx) {
  uint8_t c = 0;
  for (uint8_t i = 0; i < NTP_SERVERS; i++) if (timeCfg.ntp[i][0]) { if (c == idx) return timeCfg.ntp[i]; c++; }
  return "";
}

static void ntpTriggerNow() { ntpDueNow = true; }

static inline void udpDiscard(WiFiUDP& u) {
#if defined(ESP_ARDUINO_VERSION) && defined(ESP_ARDUINO_VERSION_VAL) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
  u.clear();
#else
  u.flush();
#endif
}

static void ntpAbort() {
  dnsRelease(DNS_NTP);
  ntpState = NTP_IDLE;
}

static double nowDouble() { struct timeval tv; gettimeofday(&tv, nullptr); return (double)tv.tv_sec + tv.tv_usec / 1e6; }

static void ntpFail(const char* why) {
  strcopy(ntpLastErr, sizeof ntpLastErr, why);
  uint8_t n = ntpServerCount();
  ntpIdx = n ? (ntpIdx + 1) % n : 0;
  ntpCycleTries++;
  if (ntpCycleTries >= n || n == 0) {
    ntpFailCycles++;
    ntpBackoffMs = ntpBackoffMs ? min<uint32_t>(ntpBackoffMs * 2, 600000) : 20000;
    ntpDueMs = millis() + ntpBackoffMs;
    ntpState = NTP_IDLE;
    ntpCycleTries = 0;
  } else {
    ntpState = NTP_IDLE;
    ntpDueNow = true;
  }
}

static void ntpStartServer() {
  uint8_t n = ntpServerCount();
  if (n == 0) { strcopy(ntpLastErr, sizeof ntpLastErr, "no servers configured"); ntpDueMs = millis() + 60000; ntpDueNow = false; return; }
  if (ntpIdx >= n) ntpIdx = 0;
  strcopy(ntpServerCur, sizeof ntpServerCur, ntpServerAt(ntpIdx));
  if (!dnsResolveStart(ntpServerCur, DNS_NTP)) { ntpFail("dns start"); return; }
  ntpState = NTP_RESOLVE;
  ntpStateMs = millis();
}

static void ntpSendPacket() {
  if (!ntpUdpStarted) { ntpUdpStarted = ntpUdp.begin(0) != 0; if (!ntpUdpStarted) { ntpFail("udp begin"); return; } }
  while (ntpUdp.parsePacket() > 0) udpDiscard(ntpUdp);
  uint8_t pkt[48];
  memset(pkt, 0, sizeof pkt);
  pkt[0] = 0x23;
  pkt[1] = 0; pkt[2] = 6; pkt[3] = 0xEC;
  for (int i = 0; i < 8; i++) ntpTxNonce[i] = (uint8_t)esp_random();
  memcpy(pkt + 40, ntpTxNonce, 8);
  ntpT1 = nowDouble();
  if (!ntpUdp.beginPacket(IPAddress(ntpServerIp), 123) || ntpUdp.write(pkt, 48) != 48 || !ntpUdp.endPacket()) { ntpFail("udp send"); return; }
  ntpState = NTP_WAIT;
  ntpStateMs = millis();
}

static inline uint32_t be32(const uint8_t* p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }
static inline double ntpTs(const uint8_t* p) {
  uint32_t s = be32(p);
  double secs = (s & 0x80000000u) ? ((double)s - 2208988800.0) : ((double)s + 2085978496.0);
  return secs + (double)be32(p + 4) / 4294967296.0;
}

static void ntpHandleReply() {
  uint8_t pkt[48];
  int len = ntpUdp.read(pkt, 48);
  double t4 = nowDouble();
  udpDiscard(ntpUdp);
  if (len < 48) { ntpFail("short reply"); return; }
  if (memcmp(pkt + 24, ntpTxNonce, 8) != 0) { ntpFail("originate mismatch"); return; }
  uint8_t li = pkt[0] >> 6, mode = pkt[0] & 7, stratum = pkt[1];
  if (mode != 4 && mode != 5) { ntpFail("bad mode"); return; }
  if (stratum == 0) { ntpFail("kiss-of-death"); return; }
  if (li == 3) { ntpFail("server unsynchronized"); return; }
  double T2 = ntpTs(pkt + 32), T3 = ntpTs(pkt + 40);
  if (T3 < (double)MIN_VALID_EPOCH) { ntpFail("implausible time"); return; }
  double rtt = t4 - ntpT1;
  if (rtt < 0 || rtt > 10) rtt = 0;
  double offset = ((T2 - ntpT1) + (T3 - t4)) / 2.0;
  double newTime = t4 + offset;
  bool had = clockValid();
  ntpLastOffsetMs = had ? offset * 1000.0 : 0;
  setSystemTime(newTime, SRC_NTP);
  ntpLastOk = (time_t)newTime;
  strcopy(ntpLastOkServer, sizeof ntpLastOkServer, ntpServerCur);
  ntpLastErr[0] = 0;
  ntpOkCount++;
  ntpBackoffMs = 0;
  ntpCycleTries = 0;
  ntpState = NTP_IDLE;
  ntpDueNow = false;
  ntpDueMs = millis() + (uint32_t)timeCfg.intervalMin * 60000UL;
  if (timeCfg.rotate) { uint8_t n = ntpServerCount(); if (n) ntpIdx = (ntpIdx + 1) % n; }
}

static void ntpTick() {
  switch (ntpState) {
    case NTP_IDLE:
      if (!staHasIp) return;
      if (ntpDueNow || (int32_t)(millis() - ntpDueMs) >= 0) {
        if (dnsBusy()) return;
        ntpDueNow = false;
        ntpStartServer();
      }
      break;
    case NTP_RESOLVE: {
      uint32_t ip;
      int r = dnsResolvePoll(ip, DNS_NTP);
      if (r == 1) { ntpServerIp = ip; ntpSendPacket(); }
      else if (r < 0) ntpFail("dns failed");
      break;
    }
    case NTP_WAIT: {
      int len = ntpUdp.parsePacket();
      if (len >= 48) ntpHandleReply();
      else if (len > 0) udpDiscard(ntpUdp);
      else if (elapsed(ntpStateMs, timeCfg.timeoutMs)) ntpFail("timeout");
      break;
    }
  }
}

static uint32_t ntpSecondsToNext() {
  if (!staHasIp || ntpDueNow || ntpState != NTP_IDLE) return 0;
  int32_t d = (int32_t)(ntpDueMs - millis());
  return d > 0 ? (uint32_t)d / 1000 : 0;
}

enum TzState : uint8_t { TZ_IDLE = 0, TZ_RESOLVE, TZ_CONNECT, TZ_SEND, TZ_RECV };
static TzState  tzState = TZ_IDLE;
static int      tzSock = -1;
static uint32_t tzStateMs = 0;
static uint32_t tzLastTryMs = 0;
static bool     tzTried = false;
static bool     tzDueNow = false;
static char     tzBuf[1024];
static size_t   tzLen = 0;
static char     tzLastResult[64] = "not yet";
static time_t   tzLastOkEpoch = 0;
static const char* TZ_HOST = "ip-api.com";

static void tzTriggerNow() { tzDueNow = true; }

static void tzClose() {
  if (tzSock >= 0) { lwip_close(tzSock); tzSock = -1; }
  dnsRelease(DNS_TZ);
  tzState = TZ_IDLE; tzLastTryMs = millis(); tzTried = true;
}

static void tzFail(const char* why) { strcopy(tzLastResult, sizeof tzLastResult, why); tzClose(); }

static void tzApplyResult(const char* name, long offset) {
  char posix[65] = "";
  bool found = lookupPosixTz(name, posix, sizeof posix);
  bool changed = (timeCfg.acValid != true) || strcmp(timeCfg.acName, name) != 0 || strcmp(timeCfg.acPosix, posix) != 0 || timeCfg.acOffset != offset;
  strcopy(timeCfg.acName, sizeof timeCfg.acName, name);
  strcopy(timeCfg.acPosix, sizeof timeCfg.acPosix, posix);
  timeCfg.acOffset = (int32_t)offset;
  timeCfg.acValid = true;
  snprintf(tzLastResult, sizeof tzLastResult, "%s (%s)", name, found ? "rule" : "fixed offset");
  tzLastOkEpoch = time(nullptr);
  if (changed) markDirty(D_TIME);
  applyTimezone();
}

static void tzParseResponse() {
  tzBuf[tzLen] = 0;
  if (strncmp(tzBuf, "HTTP/1.", 7) != 0 || tzLen < 12) { tzFail("bad http response"); return; }
  int code = atoi(tzBuf + 9);
  char* body = strstr(tzBuf, "\r\n\r\n");
  if (code != 200 || !body) { snprintf(tzLastResult, sizeof tzLastResult, "http %d", code); tzClose(); return; }
  body += 4;
  char* j = strchr(body, '{');
  if (!j) { tzFail("no json"); return; }
  char* je = strrchr(j, '}');
  if (je) je[1] = 0;
  JsonDocument d;
  if (deserializeJson(d, j)) { tzFail("json parse"); return; }
  const char* st = d["status"] | "";
  if (strcmp(st, "success") != 0) { const char* msg = d["message"] | "api error"; snprintf(tzLastResult, sizeof tzLastResult, "api: %.40s", msg); tzClose(); return; }
  const char* tzn = d["timezone"] | "";
  long off = d["offset"] | 0L;
  if (!tzn[0] || off < -50400 || off > 50400) { tzFail("bad api data"); return; }
  tzClose();
  tzApplyResult(tzn, off);
}

static void tzTick() {
  if (timeCfg.tzMode != 1 && !tzDueNow && tzState == TZ_IDLE) return;
  switch (tzState) {
    case TZ_IDLE: {
      if (!staHasIp) return;
      bool due = tzDueNow;
      if (!due) {
        if (!tzTried) due = true;
        else if (timeCfg.acValid && timeCfg.acPosix[0]) due = elapsed(tzLastTryMs, 6UL * 3600UL * 1000UL);
        else if (timeCfg.acValid) due = elapsed(tzLastTryMs, 3600UL * 1000UL);
        else due = elapsed(tzLastTryMs, 120000UL);
      }
      if (!due || dnsBusy() || ntpState != NTP_IDLE) return;
      tzDueNow = false;
      tzTried = true;
      tzLastTryMs = millis();
      if (!dnsResolveStart(TZ_HOST, DNS_TZ)) { tzFail("dns start"); return; }
      tzState = TZ_RESOLVE; tzStateMs = millis();
      break;
    }
    case TZ_RESOLVE: {
      uint32_t ip; int r = dnsResolvePoll(ip, DNS_TZ);
      if (r < 0) { tzFail("dns failed"); return; }
      if (r == 0) return;
      tzSock = lwip_socket(AF_INET, SOCK_STREAM, 0);
      if (tzSock < 0) { tzFail("socket"); return; }
      int fl = lwip_fcntl(tzSock, F_GETFL, 0);
      lwip_fcntl(tzSock, F_SETFL, fl | O_NONBLOCK);
      struct sockaddr_in sa; memset(&sa, 0, sizeof sa);
      sa.sin_family = AF_INET; sa.sin_port = htons(80); sa.sin_addr.s_addr = ip;
      int rc = lwip_connect(tzSock, (struct sockaddr*)&sa, sizeof sa);
      if (rc < 0 && errno != EINPROGRESS) { tzFail("connect"); return; }
      tzState = TZ_CONNECT; tzStateMs = millis();
      break;
    }
    case TZ_CONNECT: {
      fd_set wf; FD_ZERO(&wf); FD_SET(tzSock, &wf);
      struct timeval tv = {0, 0};
      int r = lwip_select(tzSock + 1, nullptr, &wf, nullptr, &tv);
      if (r > 0) {
        int err = 0; socklen_t l = sizeof err;
        lwip_getsockopt(tzSock, SOL_SOCKET, SO_ERROR, &err, &l);
        if (err) { tzFail("connect refused"); return; }
        tzState = TZ_SEND; tzStateMs = millis();
      } else if (elapsed(tzStateMs, 5000)) tzFail("connect timeout");
      break;
    }
    case TZ_SEND: {
      int n = snprintf(tzBuf, sizeof tzBuf, "GET /json/?fields=status,message,timezone,offset HTTP/1.1\r\nHost: %s\r\nUser-Agent: ESP32Relay/" FW_VERSION "\r\nConnection: close\r\n\r\n", TZ_HOST);
      int w = lwip_send(tzSock, tzBuf, n, 0);
      if (w == n) { tzLen = 0; tzState = TZ_RECV; tzStateMs = millis(); }
      else if (w < 0 && errno != EWOULDBLOCK && errno != EAGAIN) tzFail("send");
      else if (elapsed(tzStateMs, 3000)) tzFail("send timeout");
      break;
    }
    case TZ_RECV: {
      int r = lwip_recv(tzSock, tzBuf + tzLen, sizeof(tzBuf) - 1 - tzLen, MSG_DONTWAIT);
      if (r > 0) { tzLen += (size_t)r; if (tzLen >= sizeof(tzBuf) - 1) { tzParseResponse(); } }
      else if (r == 0) tzParseResponse();
      else if (errno != EWOULDBLOCK && errno != EAGAIN) tzFail("recv");
      else if (elapsed(tzStateMs, 6000)) { if (tzLen) tzParseResponse(); else tzFail("recv timeout"); }
      break;
    }
  }
}

struct NetEvt { uint8_t ev; uint8_t reason; };
static volatile NetEvt  evQ[8];
static volatile uint8_t evHead = 0, evTail = 0;
static const uint8_t EV_UP = 1, EV_DOWN = 2;

static bool     staConnecting = false;
static uint32_t staAttemptMs = 0;
static uint32_t staNextTryMs = 0;
static uint32_t staAttempts = 0;
static uint32_t staDisconnects = 0;
static uint32_t staBackoffMs = 0;
static uint8_t  staLastReason = 0;
static uint32_t staUpSinceMs = 0;
static time_t   staUpSinceEpoch = 0;

static bool     apActive = false;
static bool     dnsActive = false;
static bool     mdnsStarted = false;

static uint8_t  netApplyPending = 0;
static uint32_t netApplyAtMs = 0;
static bool     staRestartPending = false;
static uint32_t staRestartMs = 0;

static void evPush(uint8_t ev, uint8_t reason) {
  uint8_t h = evHead, n = (uint8_t)((h + 1) & 7);
  if (n == evTail) return;
  evQ[h].ev = ev; evQ[h].reason = reason;
  evHead = n;
}

static void onWiFiEvent(arduino_event_id_t ev, arduino_event_info_t info) {
  switch (ev) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:       evPush(EV_UP, 0); break;
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:      evPush(EV_DOWN, 0); break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED: evPush(EV_DOWN, info.wifi_sta_disconnected.reason); break;
    default: break;
  }
}

static bool staWanted() { return netCfg.sta.enabled && netCfg.sta.ssid[0] != 0; }

static bool apWanted() {
  if (!staWanted()) return true;
  if (!netCfg.ap.enabled) return false;
  if (netCfg.ap.mode == 0) return true;
  if (!staHasIp) return true;
  return !elapsed(staUpSinceMs, 60000);
}

static void startMdns() {
  if (mdnsStarted) MDNS.end();
  mdnsStarted = MDNS.begin(netCfg.hostname);
  if (mdnsStarted) MDNS.addService("http", "tcp", 80);
}

static void startAp() {
  WiFi.enableAP(true);
  const char* pass = netCfg.ap.pass[0] ? netCfg.ap.pass : nullptr;
  bool ok = WiFi.softAP(netCfg.ap.ssid, pass, netCfg.ap.channel, netCfg.ap.hidden ? 1 : 0, netCfg.ap.maxConn);
  if (!ok) {
    char fb[33]; uint8_t mac[6]; WiFi.macAddress(mac);
    snprintf(fb, sizeof fb, "ESP32-Relay-%02X%02X", mac[4], mac[5]);
    WiFi.softAP(fb, nullptr, 1, 0, 4);
  }
  IPAddress ip(netCfg.ap.ip);
  WiFi.softAPConfig(ip, ip, IPAddress(255, 255, 255, 0));
  WiFi.setTxPower((wifi_power_t)netCfg.ap.txPower);
  apActive = true;
  if (netCfg.ap.captive) {
    dnsServer.stop();
    dnsServer.setTTL(0);
    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    dnsActive = dnsServer.start(53, "*", WiFi.softAPIP());
  }
}

static void stopAp() {
  if (dnsActive) { dnsServer.stop(); dnsActive = false; }
  WiFi.softAPdisconnect(false);
  WiFi.enableAP(false);
  apActive = false;
}

static void staBegin() {
  WiFi.enableSTA(true);
  WiFi.setAutoReconnect(false);
  if (netCfg.sta.dhcp || netCfg.sta.ip == 0) {
    const IPAddress zero(0, 0, 0, 0);
    WiFi.config(zero, zero, zero);
  } else {
    WiFi.config(IPAddress(netCfg.sta.ip), IPAddress(netCfg.sta.gw), IPAddress(netCfg.sta.mask), IPAddress(netCfg.sta.dns1), IPAddress(netCfg.sta.dns2));
  }
  WiFi.begin(netCfg.sta.ssid, netCfg.sta.pass[0] ? netCfg.sta.pass : nullptr);
  WiFi.setTxPower((wifi_power_t)netCfg.ap.txPower);
  staConnecting = true;
  staAttemptMs = millis();
  staAttempts++;
}

static void staStop() {
  staConnecting = false;
  staHasIp = false;
  WiFi.disconnect(false, false);
  WiFi.enableSTA(false);
}

static void onStaUp() {
  staHasIp = true;
  staConnecting = false;
  staBackoffMs = 0;
  staUpSinceMs = millis();
  staUpSinceEpoch = time(nullptr);
  ntpTriggerNow();
  if (timeCfg.tzMode == 1) tzTriggerNow();
  startMdns();
}

static void onStaDown(uint8_t reason) {
  bool wasUp = staHasIp;
  staHasIp = false;
  staConnecting = false;
  staLastReason = reason;
  staDisconnects++;
  staBackoffMs = staBackoffMs ? min<uint32_t>(staBackoffMs * 2, 30000) : 2000;
  staNextTryMs = millis() + staBackoffMs;
  if (ntpState != NTP_IDLE) { ntpAbort(); ntpDueNow = true; }
  if (tzState != TZ_IDLE) tzClose();
  if (wasUp) rtcRequestAlignSet();
}

static void applyNetworkConfig() {
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  WiFi.setSleep(false);
  WiFi.setHostname(netCfg.hostname);
  bool wantSta = staWanted();
  bool wantAp = apWanted();
  WiFi.mode(wantSta ? (wantAp ? WIFI_AP_STA : WIFI_STA) : WIFI_AP);
  if (wantAp) startAp(); else stopAp();
  if (wantSta) staBegin(); else staStop();
  if (!wantSta) startMdns();
}

static void scheduleNetApply(uint8_t what) { netApplyPending |= what; netApplyAtMs = millis() + 600; }

static void wifiTick() {
  while (evTail != evHead) {
    NetEvt e; e.ev = evQ[evTail].ev; e.reason = evQ[evTail].reason;
    evTail = (uint8_t)((evTail + 1) & 7);
    if (e.ev == EV_UP) { if (!staHasIp) onStaUp(); }
    else { if (staWanted()) { if (staHasIp || staConnecting) onStaDown(e.reason); } else staHasIp = false; }
  }
  static uint32_t reconMs = 0;
  if (elapsed(reconMs, 1000)) {
    reconMs = millis();
    bool drvUp = (WiFi.status() == WL_CONNECTED) && ((uint32_t)WiFi.localIP() != 0);
    if (staWanted() && drvUp && !staHasIp) onStaUp();
    else if (staHasIp && !drvUp) onStaDown(0);
  }

  if (netApplyPending && (int32_t)(millis() - netApplyAtMs) >= 0) {
    uint8_t w = netApplyPending; netApplyPending = 0;
    if (w & 4) { WiFi.setHostname(netCfg.hostname); startMdns(); }
    if (w & 1) {
      staStop();
      staRestartPending = true;
      staRestartMs = millis() + 400;
    }
    if (w & 2) { if (apActive) stopAp(); if (apWanted()) startAp(); }
    if (!staWanted() && !apActive) startAp();
  }
  if (staRestartPending && (int32_t)(millis() - staRestartMs) >= 0) {
    staRestartPending = false;
    if (staWanted()) {
      if (!(WiFi.getMode() & WIFI_MODE_STA)) WiFi.mode(apActive ? WIFI_AP_STA : WIFI_STA);
      staBegin();
    }
  }

  if (staWanted() && !staHasIp && !staRestartPending && !netApplyPending) {
    if (staConnecting && elapsed(staAttemptMs, 20000)) {
      staConnecting = false;
      WiFi.disconnect(false, false);
      staBackoffMs = staBackoffMs ? min<uint32_t>(staBackoffMs * 2, 30000) : 2000;
      staNextTryMs = millis() + staBackoffMs;
    }
    if (!staConnecting && (int32_t)(millis() - staNextTryMs) >= 0) {
      if (!(WiFi.getMode() & WIFI_MODE_STA)) WiFi.mode(apActive ? WIFI_AP_STA : WIFI_STA);
      staBegin();
      staNextTryMs = millis() + staBackoffMs;
    }
  }

  static uint32_t apEvalMs = 0;
  if (elapsed(apEvalMs, 1000)) {
    apEvalMs = millis();
    bool want = apWanted();
    if (want && !apActive) { WiFi.mode(staWanted() ? WIFI_AP_STA : WIFI_AP); startAp(); }
    else if (!want && apActive) stopAp();
  }
}

static uint32_t btnDownMs = 0;
static bool     btnWasDown = false;
static bool     btnFired = false;

static void buttonTick() {
  bool down = digitalRead(PIN_BOOT_BTN) == LOW;
  if (down && !btnWasDown) { btnDownMs = millis(); btnFired = false; }
  if (down && !btnFired && elapsed(btnDownMs, FACTORY_HOLD_MS)) {
    btnFired = true;
    requestFactoryReset();
  }
  btnWasDown = down;
}

static uint32_t buttonHeldMs() { return (btnWasDown && !btnFired) ? (uint32_t)(millis() - btnDownMs) : 0; }

static char chunkBuf[1024];

class ChunkedOut : public Print {
 public:
  size_t write(uint8_t c) override { chunkBuf[_n++] = (char)c; if (_n >= sizeof(chunkBuf)) flushBuf(); return 1; }
  size_t write(const uint8_t* d, size_t l) override {
    size_t total = l;
    while (l) {
      size_t k = sizeof(chunkBuf) - _n; if (k > l) k = l;
      memcpy(chunkBuf + _n, d, k); _n += k; d += k; l -= k;
      if (_n >= sizeof(chunkBuf)) flushBuf();
    }
    return total;
  }
  void flushBuf() { if (_n) { server.sendContent(chunkBuf, _n); _n = 0; } }
  void begin(const char* type) { _n = 0; server.setContentLength(CONTENT_LENGTH_UNKNOWN); server.sendHeader("Cache-Control", "no-store"); server.send(200, type, ""); }
  void end() { flushBuf(); server.sendContent(""); }
 private:
  size_t _n = 0;
};

static void sendJsonText(int code, const char* txt) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", txt);
}

static void sendDoc(JsonDocument& d) {
  size_t need = measureJson(d) + 1;
  if (need <= sizeof(jbuf)) { serializeJson(d, jbuf, sizeof(jbuf)); sendJsonText(200, jbuf); return; }
  ChunkedOut co; co.begin("application/json"); serializeJson(d, co); co.end();
}

static void sendOk() { sendJsonText(200, "{\"ok\":true}"); }

static void sendErr(int code, const char* msg) {
  JsonDocument d; d["ok"] = false; d["err"] = msg;
  serializeJson(d, jbuf, sizeof(jbuf));
  sendJsonText(code, jbuf);
}

static bool readBody(JsonDocument& d) {
  if (!server.hasArg("plain")) { sendErr(400, "missing JSON body"); return false; }
  DeserializationError e = deserializeJson(d, server.arg("plain"));
  if (e) { char m[64]; snprintf(m, sizeof m, "bad JSON: %s", e.c_str()); sendErr(400, m); return false; }
  if (!d.is<JsonObject>()) { sendErr(400, "JSON object expected"); return false; }
  return true;
}

static int pathIndex(const char* prefix) {
  String u = server.uri();
  if (!u.startsWith(prefix)) return -1;
  const char* p = u.c_str() + strlen(prefix);
  if (!isdigit((unsigned char)*p)) return -1;
  int v = atoi(p);
  return (v >= 0 && v < NUM_RELAYS) ? v : -1;
}

static bool captiveRedirect() {
  if (!apActive || !netCfg.ap.captive) return false;
  IPAddress apIp = WiFi.softAPIP();
  if (server.client().localIP() != apIp) return false;
  if (server.uri().startsWith("/api/")) return false;
  String host = server.hostHeader();
  if (host.length() == 0) return false;
  int colon = host.indexOf(':');
  if (colon > 0) host = host.substring(0, colon);
  host.toLowerCase();
  if (host == apIp.toString()) return false;
  String hn = String(netCfg.hostname); hn.toLowerCase();
  if (host == hn || host == hn + ".local") return false;
  String apStr = apIp.toString();
  char loc[64];
  snprintf(loc, sizeof loc, "http://%s/", apStr.c_str());
  server.sendHeader("Location", loc, true);
  server.sendHeader("Cache-Control", "no-store");
  server.send(302, "text/plain", "");
  return true;
}

static void handleCaptiveProbe() {
  if (captiveRedirect()) return;
  String u = server.uri();
  if (u == "/generate_204" || u == "/gen_204") { server.send(204, "text/plain", ""); return; }
  if (u == "/hotspot-detect.html" || u == "/library/test/success.html") { server.send(200, "text/html", "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>"); return; }
  if (u == "/connecttest.txt") { server.send(200, "text/plain", "Microsoft Connect Test"); return; }
  if (u == "/ncsi.txt") { server.send(200, "text/plain", "Microsoft NCSI"); return; }
  if (u == "/success.txt") { server.send(200, "text/plain", "success\n"); return; }
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

static void handleNotFound() {
  if (captiveRedirect()) return;
  server.send(404, "text/plain", "Not found");
}

static const char* ntpStateStr() { return ntpState == NTP_IDLE ? (staHasIp ? "idle" : "offline") : ntpState == NTP_RESOLVE ? "resolving" : "waiting"; }

static void fillTimeStatus(JsonObject o) {
  struct timeval tv; gettimeofday(&tv, nullptr);
  o["e"] = (uint32_t)tv.tv_sec;
  o["ms"] = (int)(tv.tv_usec / 1000);
  o["o"] = currentUtcOffset(tv.tv_sec);
  o["v"] = timeIsValid(tv.tv_sec);
  o["src"] = SRC_NAMES[timeSrc];
  o["set"] = (uint32_t)lastSetEpoch;
  o["tz"] = tzCurrent;
  o["tzs"] = tzSource;
  o["tzn"] = timeCfg.acName;
  o["tzr"] = tzLastResult;
  o["tzMode"] = timeCfg.tzMode;
  o["ab"] = timeCfg.autoBrowser;
  JsonObject n = o["ntp"].to<JsonObject>();
  n["st"] = ntpStateStr();
  n["last"] = (uint32_t)ntpLastOk;
  n["srv"] = ntpLastOkServer;
  n["cur"] = ntpServerCur;
  n["next"] = ntpSecondsToNext();
  n["err"] = ntpLastErr;
  n["fails"] = ntpFailCycles;
  n["ok"] = ntpOkCount;
  n["off"] = (int64_t)ntpLastOffsetMs;
  n["n"] = ntpServerCount();
  n["iv"] = timeCfg.intervalMin;
  JsonObject r = o["rtc"].to<JsonObject>();
  r["en"] = timeCfg.rtcEnabled;
  r["ok"] = rtcPresent;
  r["lost"] = rtcLost;
  r["e"] = (uint32_t)rtcEpoch;
  r["t"] = rtcPresent ? rtcTemp : 0.0f;
  r["drift"] = rtcPresent ? (long)rtcDrift : 0L;
  r["w"] = rtcSetCount;
}

static const char* staStateStr() {
  if (!netCfg.sta.enabled) return "disabled";
  if (!netCfg.sta.ssid[0]) return "no SSID";
  if (staHasIp) return "connected";
  if (staConnecting) return "connecting";
  return "disconnected";
}

static void fillStaStatus(JsonObject s) {
  s["en"] = netCfg.sta.enabled;
  s["ssid"] = netCfg.sta.ssid;
  s["st"] = staStateStr();
  s["up"] = staHasIp;
  if (staHasIp) {
    s["ip"] = WiFi.localIP().toString();
    s["gw"] = WiFi.gatewayIP().toString();
    s["mask"] = WiFi.subnetMask().toString();
    s["dns"] = WiFi.dnsIP().toString();
    s["rssi"] = WiFi.RSSI();
    s["ch"] = WiFi.channel();
    s["bssid"] = WiFi.BSSIDstr();
    s["since"] = (uint32_t)staUpSinceEpoch;
  }
  s["mac"] = WiFi.macAddress();
  s["host"] = netCfg.hostname;
  s["tries"] = staAttempts;
  s["drops"] = staDisconnects;
  s["reason"] = staLastReason;
  s["reasonTxt"] = staLastReason ? wifiReasonStr(staLastReason) : "";
  s["dhcp"] = netCfg.sta.dhcp;
}

static void fillApStatus(JsonObject a) {
  a["on"] = apActive;
  a["en"] = netCfg.ap.enabled;
  a["ssid"] = apActive ? WiFi.softAPSSID() : String(netCfg.ap.ssid);
  a["ip"] = apActive ? WiFi.softAPIP().toString() : String("");
  a["ch"] = apActive ? WiFi.channel() : 0;
  a["clients"] = apActive ? WiFi.softAPgetStationNum() : 0;
  a["mac"] = WiFi.softAPmacAddress();
  a["captive"] = netCfg.ap.captive && dnsActive;
  a["hidden"] = netCfg.ap.hidden;
  a["mode"] = netCfg.ap.mode;
  a["tx"] = netCfg.ap.txPower;
}

static void fillRelaySummary(JsonArray arr) {
  for (uint8_t i = 0; i < NUM_RELAYS; i++) {
    Relay& r = relays[i];
    JsonObject o = arr.add<JsonObject>();
    o["n"] = r.name;
    o["m"] = r.mode;
    o["s"] = r.state;
    o["p"] = gpioCfg.pin[i];
    o["al"] = gpioCfg.activeLow[i];
    o["lv"] = readPinLevel(i);
    uint8_t act = 0, en = 0;
    for (uint8_t k = 0; k < NUM_PAIRS; k++) { if (r.p[k].en) en |= 1 << k; if (r.p[k].en && r.p[k].active) act |= 1 << k; }
    o["act"] = act;
    o["en"] = en;
  }
}

static void apiTimeGet() { JsonDocument d; fillTimeStatus(d.to<JsonObject>()); sendDoc(d); }

static void apiStatus() {
  JsonDocument d;
  d["fw"] = FW_VERSION;
  d["host"] = netCfg.hostname;
  d["up"] = uptimeSec();
  d["heap"] = ESP.getFreeHeap();
  d["minHeap"] = ESP.getMinFreeHeap();
  d["maxBlock"] = ESP.getMaxAllocHeap();
  d["btn"] = buttonHeldMs();
  d["clockValid"] = clockValid();
  fillTimeStatus(d["time"].to<JsonObject>());
  fillStaStatus(d["sta"].to<JsonObject>());
  fillApStatus(d["ap"].to<JsonObject>());
  fillRelaySummary(d["relays"].to<JsonArray>());
  sendDoc(d);
}

static void apiTimeCfgGet() { JsonDocument d; timeToDoc(d); d["tzNow"] = tzCurrent; sendDoc(d); }

static void apiTimeCfgPost() {
  JsonDocument d; if (!readBody(d)) return;
  TimeCfg t = timeCfg;
  JsonArray n = d["ntp"];
  if (!n.isNull()) {
    if (n.size() > NTP_SERVERS) { sendErr(400, "max 4 NTP servers"); return; }
    bool anyNtp = false;
    for (uint8_t i = 0; i < NTP_SERVERS; i++) {
      char h[49] = "";
      if (i < n.size() && n[i].is<const char*>()) strcopy(h, sizeof h, n[i]);
      trimInPlace(h);
      if (h[0] && !validHostOrIp(h)) { char m[80]; snprintf(m, sizeof m, "NTP server %u is not a valid host name", i + 1); sendErr(400, m); return; }
      strcopy(t.ntp[i], 49, h);
      if (t.ntp[i][0]) anyNtp = true;
    }
    if (!anyNtp) { sendErr(400, "at least one NTP server is required"); return; }
  }
  if (d["interval"].is<int>()) { int v = d["interval"]; if (v < 1 || v > 1440) { sendErr(400, "sync interval must be 1..1440 minutes"); return; } t.intervalMin = v; }
  if (d["timeout"].is<int>())  { int v = d["timeout"];  if (v < 500 || v > 10000) { sendErr(400, "timeout must be 500..10000 ms"); return; } t.timeoutMs = v; }
  if (d["rotate"].is<bool>()) t.rotate = d["rotate"];
  if (d["rtc"].is<bool>()) t.rtcEnabled = d["rtc"];
  if (d["autoBrowser"].is<bool>()) t.autoBrowser = d["autoBrowser"];
  if (d["tzMode"].is<int>()) { int v = d["tzMode"]; if (v != 0 && v != 1) { sendErr(400, "tzMode must be 0 or 1"); return; } t.tzMode = v; }
  if (d["gmtOffset"].is<long>()) { long v = d["gmtOffset"]; if (v < -50400 || v > 50400) { sendErr(400, "GMT offset must be -50400..50400 s"); return; } t.gmtOffset = v; }
  if (d["dst"].is<bool>()) t.dst = d["dst"];
  if (d["dstOffset"].is<long>()) { long v = d["dstOffset"]; if (v < -7200 || v > 7200) { sendErr(400, "DST offset must be -7200..7200 s"); return; } t.dstOffset = v; }
  struct { const char* k; uint8_t* f; int lo, hi; } rules[] = {
    {"dsM", &t.dsM, 1, 12}, {"dsW", &t.dsW, 1, 5}, {"dsD", &t.dsD, 0, 6}, {"dsH", &t.dsH, 0, 24},
    {"deM", &t.deM, 1, 12}, {"deW", &t.deW, 1, 5}, {"deD", &t.deD, 0, 6}, {"deH", &t.deH, 0, 24}};
  for (auto& ru : rules) {
    if (d[ru.k].is<int>()) { int v = d[ru.k]; if (v < ru.lo || v > ru.hi) { char m[64]; snprintf(m, sizeof m, "%s out of range", ru.k); sendErr(400, m); return; } *ru.f = v; }
  }
  if (d["posix"].is<const char*>()) {
    const char* p = d["posix"];
    if (strlen(p) > 64) { sendErr(400, "POSIX string too long (max 64)"); return; }
    for (const char* q = p; *q; q++) if (*q < 32 || *q > 126) { sendErr(400, "POSIX string has invalid characters"); return; }
    strcopy(t.posix, sizeof t.posix, p); trimInPlace(t.posix);
    if (t.posix[0] && (strlen(t.posix) < 3 || !(isalpha((unsigned char)t.posix[0]) || t.posix[0] == '<') || !strpbrk(t.posix, "0123456789"))) { sendErr(400, "POSIX string looks invalid (e.g. EST5EDT,M3.2.0,M11.1.0)"); return; }
  }
  bool tzChanged = t.tzMode != timeCfg.tzMode || t.gmtOffset != timeCfg.gmtOffset || t.dst != timeCfg.dst || t.dstOffset != timeCfg.dstOffset ||
                   memcmp(&t.dsM, &timeCfg.dsM, 8) != 0 || strcmp(t.posix, timeCfg.posix) != 0;
  bool ntpChanged = memcmp(t.ntp, timeCfg.ntp, sizeof(t.ntp)) != 0 || t.intervalMin != timeCfg.intervalMin;
  bool rtcChanged = t.rtcEnabled != timeCfg.rtcEnabled;
  timeCfg = t;
  markDirty(D_TIME);
  if (tzChanged) { applyTimezone(); if (timeCfg.tzMode == 1) tzTriggerNow(); }
  if (ntpChanged) { ntpAbort(); ntpIdx = 0; ntpCycleTries = 0; ntpBackoffMs = 0; ntpTriggerNow(); }
  if (rtcChanged) { if (timeCfg.rtcEnabled) rtcProbe(); else rtcPresent = false; }
  apiTimeCfgGet();
}

static void apiTimeSync() {
  if (!staHasIp) { sendErr(409, "station is not connected - NTP unreachable"); return; }
  if (ntpServerCount() == 0) { sendErr(400, "no NTP servers configured"); return; }
  ntpBackoffMs = 0; ntpCycleTries = 0;
  ntpAbort();
  ntpTriggerNow();
  sendOk();
}

static void apiTimeSet() {
  JsonDocument d; if (!readBody(d)) return;
  if (!d["epoch"].is<long long>() && !d["epoch"].is<long>() && !d["epoch"].is<double>()) { sendErr(400, "epoch missing"); return; }
  double epoch = d["epoch"].as<double>();
  int ms = d["ms"] | 0;
  if (ms < 0 || ms > 999) ms = 0;
  if (epoch < (double)MIN_VALID_EPOCH || epoch > 4102444800.0) { sendErr(400, "epoch out of range"); return; }
  setSystemTime(epoch + ms / 1000.0, SRC_BROWSER);
  JsonDocument o; fillTimeStatus(o.to<JsonObject>()); sendDoc(o);
}

static void apiTimeTz() {
  JsonDocument d; if (!readBody(d)) return;
  const char* iana = d["iana"] | "";
  long off = d["offset"] | 0L;
  if (off < -50400 || off > 50400) { sendErr(400, "offset out of range"); return; }
  char posix[65] = "";
  bool found = lookupPosixTz(iana, posix, sizeof posix);
  timeCfg.tzMode = 0;
  if (found) { strcopy(timeCfg.posix, sizeof timeCfg.posix, posix); }
  else { timeCfg.posix[0] = 0; timeCfg.gmtOffset = off; timeCfg.dst = false; }
  markDirty(D_TIME);
  applyTimezone();
  JsonDocument o; o["ok"] = true; o["found"] = found; o["posix"] = tzCurrent; o["iana"] = iana; sendDoc(o);
}

static void apiTimeTzDetect() {
  if (!staHasIp) { sendErr(409, "station is not connected"); return; }
  tzTriggerNow();
  sendOk();
}

static void apiTimeRtcRead() {
  if (!rtcUsable()) { sendErr(409, "DS3231 not available or its time is invalid"); return; }
  rtcRequestAlignSet();
  sendOk();
}

static void apiTimeRtcWrite() {
  if (!(timeCfg.rtcEnabled && rtcPresent)) { sendErr(409, "DS3231 not present"); return; }
  if (!clockValid()) { sendErr(409, "system clock is not valid"); return; }
  rtcWritePending = true; rtcWriteReqMs = millis();
  sendOk();
}

static void apiRelaysGet() { JsonDocument d; fillRelaySummary(d.to<JsonArray>()); sendDoc(d); }

static void apiRelayGet() {
  int i = pathIndex("/api/relay/"); if (i < 0) { sendErr(404, "bad relay index"); return; }
  JsonDocument d; relayToDoc(i, d, true); sendDoc(d);
}

static Pair tmpPairs[NUM_PAIRS];

static void apiRelayPost() {
  int i = pathIndex("/api/relay/"); if (i < 0) { sendErr(404, "bad relay index"); return; }
  JsonDocument d; if (!readBody(d)) return;
  Relay& r = relays[i];
  char name[RELAY_NAME_MAX + 1]; strcopy(name, sizeof name, r.name);
  if (d["name"].is<const char*>()) {
    const char* nm = d["name"];
    if (strlen(nm) > RELAY_NAME_MAX) { sendErr(400, "name too long (max 24)"); return; }
    strcopy(name, sizeof name, nm); trimInPlace(name);
    if (!name[0]) { sendErr(400, "name must not be empty"); return; }
  }
  int mode = d["mode"] | (int)r.mode;
  if (mode < 0 || mode > 2) { sendErr(400, "mode must be 0 (auto), 1 (on) or 2 (off)"); return; }
  bool hasS = d["s"].is<JsonArray>();
  if (hasS) {
    JsonArray s = d["s"];
    if (s.size() > NUM_PAIRS) { sendErr(400, "max 8 schedule pairs"); return; }
    memcpy(tmpPairs, r.p, sizeof(tmpPairs));
    for (uint8_t k = 0; k < s.size(); k++) {
      JsonObject o = s[k];
      if (o.isNull()) continue;
      Pair& p = tmpPairs[k];
      if (o["en"].is<bool>()) p.en = o["en"];
      char m[96];
      if (o["a"].is<const char*>()) { const char* a = o["a"]; if (strlen(a) > SPEC_MAX) { snprintf(m, sizeof m, "Pair %u START: too long (max %u chars)", k + 1, (unsigned)SPEC_MAX); sendErr(400, m); return; } strcopy(p.aText, sizeof p.aText, a); trimInPlace(p.aText); }
      if (o["b"].is<const char*>()) { const char* b = o["b"]; if (strlen(b) > SPEC_MAX) { snprintf(m, sizeof m, "Pair %u STOP: too long (max %u chars)", k + 1, (unsigned)SPEC_MAX); sendErr(400, m); return; } strcopy(p.bText, sizeof p.bText, b); trimInPlace(p.bText); }
      const char* e = parseSpec(p.aText, p.a);
      if (e) { snprintf(m, sizeof m, "Pair %u START: %s", k + 1, e); sendErr(400, m); return; }
      e = parseSpec(p.bText, p.b);
      if (e) { snprintf(m, sizeof m, "Pair %u STOP: %s", k + 1, e); sendErr(400, m); return; }
      if (p.en && p.a.never && p.b.never) { snprintf(m, sizeof m, "Pair %u is enabled but has neither START nor STOP", k + 1); sendErr(400, m); return; }
    }
  }
  strcopy(r.name, sizeof r.name, name);
  r.mode = (uint8_t)mode;
  if (hasS) { memcpy(r.p, tmpPairs, sizeof(tmpPairs)); recalcRelayNow(i); }
  markDirty(D_RELAY0 << i);
  applyOutputs(false);
  JsonDocument out; relayToDoc(i, out, true); sendDoc(out);
}

static void apiRelayMode() {
  int i = pathIndex("/api/relay/"); if (i < 0) { sendErr(404, "bad relay index"); return; }
  JsonDocument d; if (!readBody(d)) return;
  int mode = d["mode"] | -1;
  if (mode < 0 || mode > 2) { sendErr(400, "mode must be 0,1,2"); return; }
  relays[i].mode = (uint8_t)mode;
  markDirty(D_RELAY0 << i);
  applyOutputs(false);
  JsonDocument o; o["ok"] = true; o["mode"] = relays[i].mode; o["state"] = relays[i].state; sendDoc(o);
}

static void apiGpioGet() {
  JsonDocument d;
  JsonArray pins = d["pins"].to<JsonArray>();
  for (uint8_t i = 0; i < ALLOWED_PIN_COUNT; i++) if (!pinReserved(ALLOWED_PINS[i])) pins.add(ALLOWED_PINS[i]);
  d["serialDebug"] = false;
  fillRelaySummary(d["r"].to<JsonArray>());
  sendDoc(d);
}

static void apiGpioSet() {
  int i = pathIndex("/api/gpio/"); if (i < 0) { sendErr(404, "bad relay index"); return; }
  JsonDocument d; if (!readBody(d)) return;
  int pin = d["pin"] | (int)gpioCfg.pin[i];
  bool al = d["al"] | gpioCfg.activeLow[i];
  if (pin != -1) {
    if (!pinAllowed(pin)) { sendErr(400, "GPIO not allowed for relays"); return; }
    if (pinReserved(pin)) { sendErr(400, "GPIO is reserved (UART/I2C/boot)"); return; }
    for (uint8_t j = 0; j < NUM_RELAYS; j++) if (j != i && gpioCfg.pin[j] == pin) { char m[64]; snprintf(m, sizeof m, "GPIO %d is already used by %s", pin, relays[j].name); sendErr(409, m); return; }
  }
  reassignRelayPin(i, (int8_t)pin, al);
  markDirty(D_GPIO);
  sendOk();
}

static void apiGpioRemove() {
  int i = pathIndex("/api/gpio/"); if (i < 0) { sendErr(404, "bad relay index"); return; }
  reassignRelayPin(i, -1, gpioCfg.activeLow[i]);
  markDirty(D_GPIO);
  sendOk();
}

static void apiGpioLevel() {
  int i = pathIndex("/api/gpio/"); if (i < 0) { sendErr(404, "bad relay index"); return; }
  JsonDocument d; if (!readBody(d)) return;
  int lvl = d["level"] | -1;
  if (lvl != 0 && lvl != 1) { sendErr(400, "level must be 0 or 1"); return; }
  if (gpioCfg.pin[i] < 0) { sendErr(409, "relay has no GPIO assigned"); return; }
  if (!relays[i].driven) { sendErr(409, "GPIO is reserved or not drivable in this build"); return; }
  relays[i].mode = lvl ? 1 : 2;
  markDirty(D_RELAY0 << i);
  applyOutputs(false);
  sendOk();
}

static void apiGpioAll() {
  JsonDocument d; if (!readBody(d)) return;
  if (d["auto"] | false) {
    for (uint8_t i = 0; i < NUM_RELAYS; i++) if (relays[i].mode != 0) { relays[i].mode = 0; recalcRelayNow(i); markDirty(D_RELAY0 << i); }
    applyOutputs(false); sendOk(); return;
  }
  int lvl = d["level"] | -1;
  if (lvl != 0 && lvl != 1) { sendErr(400, "level must be 0 or 1"); return; }
  for (uint8_t i = 0; i < NUM_RELAYS; i++) if (gpioCfg.pin[i] >= 0) { relays[i].mode = lvl ? 1 : 2; markDirty(D_RELAY0 << i); }
  applyOutputs(false);
  sendOk();
}

static void apiNetGet() { JsonDocument d; netToDoc(d, false); sendDoc(d); }

static void apiNetStaPost() {
  JsonDocument d; if (!readBody(d)) return;
  StaCfg s = netCfg.sta;
  char host[33]; strcopy(host, sizeof host, netCfg.hostname);
  if (d["host"].is<const char*>()) { strcopy(host, sizeof host, d["host"]); trimInPlace(host); if (!validHostname(host)) { sendErr(400, "hostname: 1-32 chars, letters/digits/hyphen"); return; } }
  if (d["en"].is<bool>()) s.enabled = d["en"];
  if (d["ssid"].is<const char*>()) { const char* v = d["ssid"]; if (strlen(v) > 32) { sendErr(400, "SSID too long"); return; } strcopy(s.ssid, sizeof s.ssid, v); }
  if (d["pass"].is<const char*>()) { const char* v = d["pass"]; if (strlen(v) > 64) { sendErr(400, "password too long"); return; } if (strlen(v) > 0 && strlen(v) < 8) { sendErr(400, "WPA password needs 8+ chars (leave empty for open network)"); return; } strcopy(s.pass, sizeof s.pass, v); }
  if (d["dhcp"].is<bool>()) s.dhcp = d["dhcp"];
  auto ipField = [&](const char* k, uint32_t& dst, bool required) -> bool {
    if (!d[k].is<const char*>()) return true;
    const char* v = d[k];
    if (!*v) { dst = 0; return !required; }
    IPAddress a; if (!a.fromString(v)) return false;
    dst = (uint32_t)a; return true;
  };
  if (!ipField("ip", s.ip, false))     { sendErr(400, "invalid static IP"); return; }
  if (!ipField("gw", s.gw, false))     { sendErr(400, "invalid gateway"); return; }
  if (!ipField("mask", s.mask, false)) { sendErr(400, "invalid subnet mask"); return; }
  if (!ipField("dns1", s.dns1, false)) { sendErr(400, "invalid DNS 1"); return; }
  if (!ipField("dns2", s.dns2, false)) { sendErr(400, "invalid DNS 2"); return; }
  if (s.enabled && !s.ssid[0]) { sendErr(400, "SSID required to enable the station"); return; }
  if (!s.dhcp && (s.ip == 0 || s.gw == 0 || s.mask == 0)) { sendErr(400, "static mode needs IP, gateway and mask"); return; }
  bool hostChanged = strcmp(host, netCfg.hostname) != 0;
  bool staChanged = memcmp(&s, &netCfg.sta, sizeof(StaCfg)) != 0;
  netCfg.sta = s;
  strcopy(netCfg.hostname, sizeof netCfg.hostname, host);
  if (staChanged || hostChanged) markDirty(D_NET);
  uint8_t what = 0;
  if (staChanged || (d["connect"] | false)) { what |= 1; if (staChanged) staBackoffMs = 0; }
  if (hostChanged) what |= 4;
  if (what) scheduleNetApply(what);
  JsonDocument o; o["ok"] = true; o["apply"] = what != 0; sendDoc(o);
}

static void apiNetApPost() {
  JsonDocument d; if (!readBody(d)) return;
  ApCfg a = netCfg.ap;
  if (d["en"].is<bool>()) a.enabled = d["en"];
  if (d["ssid"].is<const char*>()) { const char* v = d["ssid"]; if (strlen(v) < 1 || strlen(v) > 32) { sendErr(400, "AP SSID must be 1-32 chars"); return; } strcopy(a.ssid, sizeof a.ssid, v); }
  if (d["pass"].is<const char*>()) { const char* v = d["pass"]; if (strlen(v) > 63) { sendErr(400, "AP password too long (max 63)"); return; } if (strlen(v) > 0 && strlen(v) < 8) { sendErr(400, "AP password needs 8+ chars (empty = open network)"); return; } strcopy(a.pass, sizeof a.pass, v); }
  if (d["ch"].is<int>()) { int v = d["ch"]; if (v < 1 || v > 13) { sendErr(400, "channel must be 1..13"); return; } a.channel = v; }
  if (d["hidden"].is<bool>()) a.hidden = d["hidden"];
  if (d["max"].is<int>()) { int v = d["max"]; if (v < 1 || v > 8) { sendErr(400, "max clients must be 1..8"); return; } a.maxConn = v; }
  if (d["tx"].is<int>()) { int v = d["tx"]; if (!txPowerAllowed(v)) { sendErr(400, "invalid TX power value"); return; } a.txPower = v; }
  if (d["ip"].is<const char*>()) {
    IPAddress ip; if (!ip.fromString((const char*)d["ip"]) || ip[3] == 0 || ip[3] == 255 || ip[0] == 0) { sendErr(400, "invalid AP IP"); return; }
    a.ip = (uint32_t)ip;
  }
  if (d["captive"].is<bool>()) a.captive = d["captive"];
  if (d["mode"].is<int>()) { int v = d["mode"]; if (v != 0 && v != 1) { sendErr(400, "mode must be 0 or 1"); return; } a.mode = v; }
  bool changed = memcmp(&a, &netCfg.ap, sizeof(ApCfg)) != 0;
  netCfg.ap = a;
  if (changed) { markDirty(D_NET); scheduleNetApply(2); }
  JsonDocument o; o["ok"] = true; o["apply"] = changed; o["forced"] = !staWanted(); sendDoc(o);
}

static void apiNetScan() {
  int16_t n = WiFi.scanComplete();
  JsonDocument d;
  if (n == WIFI_SCAN_RUNNING) { d["st"] = "running"; sendDoc(d); return; }
  if (n == WIFI_SCAN_FAILED) {
    int16_t r = WiFi.scanNetworks(true, true);
    d["st"] = (r == WIFI_SCAN_FAILED) ? "busy" : "started";
    sendDoc(d); return;
  }
  d["st"] = "done";
  JsonArray arr = d["nets"].to<JsonArray>();
  int cnt = n > 40 ? 40 : n;
  for (int i = 0; i < cnt; i++) {
    JsonObject o = arr.add<JsonObject>();
    o["ssid"] = WiFi.SSID(i);
    o["rssi"] = WiFi.RSSI(i);
    o["ch"] = WiFi.channel(i);
    o["enc"] = (int)WiFi.encryptionType(i);
    o["bssid"] = WiFi.BSSIDstr(i);
  }
  WiFi.scanDelete();
  sendDoc(d);
}

static void apiSystem() {
  JsonDocument d;
  d["fw"] = FW_VERSION;
  d["sdk"] = ESP.getSdkVersion();
  d["chip"] = ESP.getChipModel();
  d["rev"] = ESP.getChipRevision();
  d["cores"] = ESP.getChipCores();
  d["mhz"] = ESP.getCpuFreqMHz();
  d["flash"] = ESP.getFlashChipSize();
  d["sketch"] = ESP.getSketchSize();
  d["sketchFree"] = ESP.getFreeSketchSpace();
  d["heap"] = ESP.getFreeHeap();
  d["heapTotal"] = ESP.getHeapSize();
  d["minHeap"] = ESP.getMinFreeHeap();
  d["maxBlock"] = ESP.getMaxAllocHeap();
  d["psram"] = ESP.getPsramSize();
  d["up"] = uptimeSec();
  d["reset"] = resetReasonStr();
  d["fsOk"] = fsOk;
  d["fsTotal"] = fsOk ? LittleFS.totalBytes() : 0;
  d["fsUsed"] = fsOk ? LittleFS.usedBytes() : 0;
  d["dirty"] = dirtyMask != 0;
  d["recalcs"] = schedRecalcCount;
  d["serialDebug"] = false;
  d["timeT64"] = timeT64();
  d["btn"] = buttonHeldMs();
  d["i2c"] = String("SDA ") + PIN_I2C_SDA + " / SCL " + PIN_I2C_SCL;
  JsonArray files = d["files"].to<JsonArray>();
  if (fsOk) {
    File dir = LittleFS.open(CFG_DIR);
    if (dir && dir.isDirectory()) {
      File f = dir.openNextFile();
      int guard = 0;
      while (f && guard++ < 80) {
        JsonObject o = files.add<JsonObject>();
        const char* nm = f.name(); const char* base = strrchr(nm, '/');
        o["n"] = base ? base + 1 : nm;
        o["s"] = (uint32_t)f.size();
        f.close();
        f = dir.openNextFile();
      }
      dir.close();
    }
  }
  fillTimeStatus(d["time"].to<JsonObject>());
  fillStaStatus(d["sta"].to<JsonObject>());
  fillApStatus(d["ap"].to<JsonObject>());
  sendDoc(d);
}

static void apiReboot() { saveDirtyNow(); requestRestart(600); sendOk(); }
static void apiFactory() { requestFactoryReset(); sendOk(); }

static void apiBackup() {
  saveDirtyNow();
  ChunkedOut co;
  server.sendHeader("Content-Disposition", "attachment; filename=\"esp32-relay-backup.json\"");
  co.begin("application/json");
  co.print("{\"fw\":\"" FW_VERSION "\",\"net\":");
  { JsonDocument d; netToDoc(d, true); serializeJson(d, co); }
  co.print(",\"time\":");
  { JsonDocument d; timeToDoc(d); serializeJson(d, co); }
  co.print(",\"gpio\":");
  { JsonDocument d; gpioToDoc(d); serializeJson(d, co); }
  co.print(",\"relays\":[");
  for (uint8_t i = 0; i < NUM_RELAYS; i++) { if (i) co.print(','); JsonDocument d; relayToDoc(i, d, false); serializeJson(d, co); }
  co.print("]}");
  co.end();
}

static const char PAGE_TMPL[] PROGMEM =
"<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<title>ESP32 Relay</title><link rel=\"stylesheet\" href=\"/app.css?v=" ASSET_VERSION "\"></head>"
"<body data-page=\"%s\"><div id=\"app\"><noscript>JavaScript required</noscript></div><script src=\"/app.js?v=" ASSET_VERSION "\"></script></body></html>";

static const char APP_CSS[] PROGMEM = R"CSS(
:root{--bg:#0f1620;--card:#172130;--line:#263345;--tx:#e6edf3;--mut:#8b9bb0;--acc:#3b82f6;--ok:#22c55e;--bad:#ef4444;--warn:#f59e0b;--btn:#243247}
*{box-sizing:border-box}html,body{margin:0;background:var(--bg);color:var(--tx);font:15px/1.4 system-ui,Segoe UI,Roboto,Helvetica,Arial,sans-serif}
a{color:var(--acc);text-decoration:none}
header{position:sticky;top:0;z-index:5;display:flex;align-items:center;gap:10px;padding:8px 12px;background:#0b1118;border-bottom:1px solid var(--line);flex-wrap:wrap}
.brand{font-weight:700;letter-spacing:.3px;white-space:nowrap}
nav{display:flex;gap:4px;flex-wrap:wrap;flex:1}nav a{display:inline-flex;align-items:center;min-height:36px;padding:4px 10px;border-radius:8px;color:var(--mut);font-size:14px;touch-action:manipulation}nav a.on{background:var(--acc);color:#fff}nav a:hover{color:#fff}
.clock{margin-left:auto;text-align:right;font-variant-numeric:tabular-nums;font-family:ui-monospace,Menlo,Consolas,monospace;font-size:15px;white-space:nowrap}
.clock small{display:block;font-size:11px;color:var(--mut);font-family:inherit}.clock .bad{color:var(--bad)}.clock .ok{color:var(--tx)}
main{padding:12px;max-width:1280px;margin:0 auto}
h1{font-size:20px;margin:4px 0 12px}h2{font-size:16px;margin:0 0 10px;color:#c9d4e0}h3{font-size:14px;margin:12px 0 6px;color:var(--mut)}
.grid{display:grid;gap:12px;grid-template-columns:repeat(auto-fill,minmax(290px,1fr))}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px}
.kv{display:grid;grid-template-columns:auto 1fr;gap:4px 12px;font-size:14px}.kv b{color:var(--mut);font-weight:500}.kv span{overflow-wrap:anywhere;min-width:0}
.btn{display:inline-block;background:var(--btn);color:var(--tx);border:1px solid var(--line);border-radius:8px;padding:7px 12px;font-size:14px;cursor:pointer;margin:2px;min-height:36px;touch-action:manipulation}
.btn:hover{filter:brightness(1.15)}.btn:disabled{opacity:.5;cursor:default}.btn.p{background:var(--acc);border-color:var(--acc);color:#fff}.btn.g{background:var(--ok);border-color:var(--ok);color:#06240f}
.btn.r{background:var(--bad);border-color:var(--bad);color:#fff}.btn.s{padding:4px 8px;font-size:12px;min-height:30px}.btn.on{outline:2px solid #fff}.btn.ic{padding:2px 7px;font-size:14px;min-height:28px;line-height:1}
.badge{display:inline-block;padding:2px 8px;border-radius:999px;font-size:12px;font-weight:600;background:#334155;color:#cbd5e1}
.badge.on{background:var(--ok);color:#06240f}.badge.off{background:#475569;color:#e2e8f0}.badge.warn{background:var(--warn);color:#2a1a00}.badge.bad{background:var(--bad);color:#fff}.badge.ok{background:var(--ok);color:#06240f}
label{display:block;font-size:13px;color:var(--mut);margin:8px 0 3px}
input[type=text],input[type=password],input[type=number],select{width:100%;background:#0f1722;color:var(--tx);border:1px solid var(--line);border-radius:8px;padding:7px 9px;font-size:14px;min-height:36px;touch-action:manipulation}
input[type=checkbox]{width:18px;height:18px;vertical-align:middle;touch-action:manipulation}
.row{display:flex;gap:10px;flex-wrap:wrap;align-items:flex-end}.row>*{flex:1;min-width:120px}.row>.fix{flex:0 0 auto;min-width:0}
.chk{display:flex;align-items:center;gap:8px;margin:8px 0;font-size:14px;color:var(--tx)}
table{width:100%;border-collapse:collapse;font-size:13px}th,td{padding:6px 6px;border-bottom:1px solid var(--line);text-align:left;vertical-align:middle}th{color:var(--mut);font-weight:600;font-size:12px}
.sched{overflow-x:auto}.sched input[data-k]{width:calc(3ch + 16px);min-width:calc(3ch + 16px);min-height:30px;padding:4px 5px;font-size:13px;text-align:center;font-family:ui-monospace,Menlo,Consolas,monospace}
.sched td{white-space:nowrap}.sched .f{display:inline-flex;gap:3px;flex-wrap:wrap}.fi{display:inline-flex;flex-direction:column;align-items:center}.fi small{font-size:10px;color:var(--mut);line-height:1.2}
.sched td.lab::before{display:none;content:attr(data-l);color:var(--mut);font-size:11px;font-weight:600;margin-right:6px}
.relay{display:flex;align-items:center;gap:10px;flex-wrap:wrap;padding:10px 12px;border:1px solid var(--line);border-radius:12px;background:var(--card);margin-bottom:8px}
.relay .nm{font-weight:700;font-size:16px;cursor:pointer;user-select:none;-webkit-user-select:none;min-width:120px;touch-action:manipulation}.relay .nm:hover{text-decoration:underline dotted}
.relay .nm input{font-size:16px;font-weight:700;width:14em;padding:3px 6px}
.relay .meta{color:var(--mut);font-size:13px}.relay .sp{flex:1}
.tile{border:1px solid var(--line);border-radius:10px;padding:8px;background:#121b27;display:flex;flex-direction:column;gap:4px}
.tile .n{font-weight:600;font-size:13px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}.tile .st{display:flex;justify-content:space-between;align-items:center;font-size:12px;color:var(--mut)}
.tiles{display:grid;gap:8px;grid-template-columns:repeat(auto-fill,minmax(140px,1fr))}
.hint{font-size:12px;color:var(--mut);margin-top:8px}.hint code{background:#0f1722;padding:1px 5px;border-radius:5px}
#toast{position:fixed;left:50%;bottom:18px;transform:translateX(-50%);background:#1e293b;color:#fff;padding:10px 16px;border-radius:10px;border:1px solid var(--line);opacity:0;pointer-events:none;transition:.2s;max-width:92vw;z-index:20}
#toast.show{opacity:1}#toast.err{background:#7f1d1d}
.nets{max-height:260px;overflow:auto;margin-top:8px}.nets div{padding:6px 8px;border-bottom:1px solid var(--line);cursor:pointer;display:flex;justify-content:space-between}.nets div:hover{background:#1f2b3c}
.mono{font-family:ui-monospace,Menlo,Consolas,monospace}.mut{color:var(--mut)}.warnbox{border:1px solid var(--warn);background:#2a1f0a;color:#fde68a;border-radius:10px;padding:8px 10px;font-size:13px;margin:8px 0}
.prog{height:6px;background:#0f1722;border-radius:4px;overflow:hidden;margin-top:4px}.prog i{display:block;height:100%;background:var(--acc)}
@media(pointer:coarse){.btn.s{min-height:40px;padding:8px 10px;font-size:14px}.btn{min-height:42px}.btn.ic{min-height:36px;padding:4px 10px}input[type=checkbox]{width:22px;height:22px}.sched input[data-k]{min-height:38px}nav a{min-height:40px}}
.gt td::before{display:none;content:attr(data-l);color:var(--mut);font-size:11px;font-weight:600;width:5.5em;flex:none}
@media(max-width:700px){.gt table,.gt tbody,.gt tr,.gt td{display:block}.gt thead{display:none}.gt tr{border:1px solid var(--line);border-radius:10px;padding:8px;margin-bottom:8px}.gt td{border:0;padding:4px 0;display:flex;align-items:center;gap:6px;flex-wrap:wrap}
.gt td::before{display:inline-block}.gt td.ix{font-weight:700}.gt td.ix::before{display:none}.gt td select{flex:1;width:auto!important}.gt td.ac{justify-content:flex-start}}
@media(max-width:700px){input[type=text],input[type=password],input[type=number],select,.sched input{font-size:16px}
.sched table,.sched tbody,.sched tr,.sched td{display:block}.sched thead{display:none}.sched tr{border:1px solid var(--line);border-radius:10px;padding:8px;margin-bottom:8px}.sched td{border:0;padding:4px 0;white-space:normal}
.sched td.lab::before{display:inline-block}.sched .f{display:flex}.sched input[data-k]{min-width:calc(3ch + 22px)}.sched select{width:100%!important}}
@media(max-width:900px){header{row-gap:6px}.brand{order:1}.clock{order:2;margin-left:auto;text-align:right}nav{order:3;width:100%;flex:none;flex-wrap:nowrap;overflow-x:auto;-webkit-overflow-scrolling:touch;scrollbar-width:none;padding-bottom:2px}nav::-webkit-scrollbar{display:none}nav a{white-space:nowrap;flex:none}}
@media(pointer:coarse){input[type=text],input[type=password],input[type=number],select,.sched input[data-k]{font-size:16px}}
@media(max-width:600px){.clock{font-size:13px}nav a{padding:6px 10px;font-size:13px}main{padding:10px}h1{font-size:18px}}
)CSS";

static const char APP_JS[] PROGMEM = R"JS(
(function(){
'use strict';
var PAGES=[['/','dash','Dashboard'],['/relays','relays','Relays'],['/gpio','gpio','GPIO'],['/time','time','Time'],['/sta','sta','WiFi STA'],['/ap','ap','WiFi AP'],['/system','system','System']];
var page=document.body.getAttribute('data-page')||'dash';
var app=document.getElementById('app');
var timers=[];
var $=function(s,r){return (r||document).querySelector(s);};
var $$=function(s,r){return Array.prototype.slice.call((r||document).querySelectorAll(s));};
var esc=function(s){return String(s==null?'':s).replace(/[&<>"']/g,function(c){return {'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c];});};
var attrEsc=function(s){return String(s==null?'':s).replace(/[&<>"'\x00-\x1F]/g,function(c){var m={'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'};return m[c]||('&#'+c.charCodeAt(0)+';');});};
var p2=function(n){return (n<10?'0':'')+n;};
var every=function(ms,fn){timers.push(setInterval(fn,ms));};
var api=async function(path,body,method){
  var o={method:method||(body?'POST':'GET'),headers:{},cache:'no-store'};
  if(body){o.headers['Content-Type']='application/json';o.body=JSON.stringify(body);}
  var r=await fetch(path,o);var j=null;try{j=await r.json();}catch(e){}
  if(!r.ok)throw new Error((j&&j.err)||('HTTP '+r.status));
  return j;
}
var toastT;
var toast=function(msg,err){var t=$('#toast');t.textContent=msg;t.className='show'+(err?' err':'');clearTimeout(toastT);toastT=setTimeout(function(){t.className='';},err?5000:2500);};
var fail=function(e){toast((e&&e.message)||String(e),true);};
var fmtUp=function(s){s=Math.floor(s);var d=Math.floor(s/86400),h=Math.floor(s%86400/3600),m=Math.floor(s%3600/60);return (d?d+'d ':'')+p2(h)+':'+p2(m)+':'+p2(s%60);};
var kb=function(n){return n>=1048576?(n/1048576).toFixed(2)+' MB':n>=1024?(n/1024).toFixed(1)+' KB':n+' B';};
var DOW=['SUN','MON','TUE','WED','THU','FRI','SAT'];
var fmtOff=function(ms){var a=Math.abs(ms),s=ms<0?'-':'+';if(a<1000)return ms+' ms';if(a<60000)return s+(a/1000).toFixed(2)+' s';if(a<86400000)return s+fmtUp(a/1000);return s+(a/86400000).toFixed(1)+' days';};
var tm={ok:false};
var devNowMs=function(){return tm.ok?(tm.e*1000+tm.ms)+(performance.now()-tm.at):null;};
var fmtDate=function(d){return DOW[d.getUTCDay()]+' '+p2(d.getUTCMonth()+1)+'/'+p2(d.getUTCDate())+'/'+p2(d.getUTCFullYear()%100);};
var fmtTime=function(d){return p2(d.getUTCHours())+':'+p2(d.getUTCMinutes())+':'+p2(d.getUTCSeconds());};
var fmtEpoch=function(e){if(!e)return '-';var d=new Date((e+(tm.ok?tm.o:0))*1000);return fmtDate(d)+' '+fmtTime(d);};
var renderClock=function(){var el=$('#clock');if(!el)return;var n=devNowMs();if(n==null){el.innerHTML='<span class="bad">--- --/--/-- --:--:--</span><small>no link</small>';return;};
  var d=new Date(n+tm.o*1000);el.innerHTML='<span class="'+(tm.v?'ok':'bad')+'">'+fmtDate(d)+' '+fmtTime(d)+'</span><small>'+(tm.v?'device time &middot; '+esc(tm.src):'CLOCK NOT SET')+'</small>';}
var autoSynced=false;
var pollTime=async function(){try{var t=await api('/api/time');tm={ok:true,e:t.e,ms:t.ms,o:t.o,v:t.v,src:t.src,at:performance.now()};renderClock();
  if(!t.v&&t.ab&&!autoSynced){autoSynced=true;try{var now=Date.now();await api('/api/time/set',{epoch:Math.floor(now/1000),ms:now%1000});toast('Device clock was not set - synchronized from this browser');pollTime();}catch(e){}}
  }catch(e){}}
var browserSync=function(){var now=Date.now();return api('/api/time/set',{epoch:Math.floor(now/1000),ms:now%1000}).then(function(){toast('Device clock set from browser');pollTime();}).catch(fail);};
var shell=function(){
  app.innerHTML='<header><div class="brand">ESP32 Relay</div><nav>'+PAGES.map(function(p){return '<a href="'+p[0]+'" class="'+(p[1]===page?'on':'')+'">'+p[2]+'</a>';}).join('')+'</nav><div id="clock" class="clock"></div></header><main id="main"></main><div id="toast"></div>';
  renderClock();pollTime();every(1000,renderClock);every(10000,pollTime);
}
var modeName=function(m){return m===1?'Manual ON':m===2?'Manual OFF':'Auto';};
var stBadge=function(s){return '<span class="badge '+(s?'on':'off')+'">'+(s?'ON':'OFF')+'</span>';};
var modeBtns=function(i,m){return '<span class="modes" data-i="'+i+'"><button class="btn s'+(m===0?' on':'')+'" data-m="0">Auto</button><button class="btn s'+(m===1?' on':'')+'" data-m="1">On</button><button class="btn s'+(m===2?' on':'')+'" data-m="2">Off</button></span>';};
var setMode=async function(i,m){try{await api('/api/relay/'+i+'/mode',{mode:m});}catch(e){fail(e);}};
var bindModes=function(root,after){$$('.modes button',root).forEach(function(b){b.onclick=function(){var i=+b.parentNode.getAttribute('data-i');setMode(i,+b.getAttribute('data-m')).then(after);};});};
var dash=function(){
  var m=$('#main');m.innerHTML='<h1>Dashboard</h1><div class="grid" id="cards"></div><div style="margin-top:8px"><button class="btn s" id="bsync">Sync NTP</button><button class="btn s" id="bbr">Sync from browser</button></div><h2 style="margin-top:14px">Relays</h2><div class="tiles" id="tiles"></div><p class="hint">Auto = follows schedules &middot; manual overrides persist until set back to Auto. Names, pins and polarity are managed on the Relays / GPIO pages.</p>';
  $('#bsync').onclick=function(){api('/api/time/sync',{}).then(function(){toast('NTP sync requested');}).catch(fail);};
  $('#bbr').onclick=browserSync;
  var load=async function(){try{var s=await api('/api/status');var t=s.time;
    var cards='<div class="card"><h2>Time</h2><div class="kv"><b>Local</b><span class="mono">'+fmtEpoch(t.e)+'</span><b>Clock</b><span>'+(t.v?'<span class="badge ok">VALID</span>':'<span class="badge bad">NOT SET</span>')+' '+esc(t.src)+'</span><b>Zone</b><span class="mono">'+esc(t.tz)+'</span><b>NTP</b><span>'+esc(t.ntp.st)+(t.ntp.last?' &middot; last '+fmtEpoch(t.ntp.last):'')+(t.ntp.err?' &middot; <span class="mut">'+esc(t.ntp.err)+'</span>':'')+'</span><b>DS3231</b><span>'+(t.rtc.en?(t.rtc.ok?(t.rtc.lost?'<span class="badge warn">LOST POWER</span>':'<span class="badge ok">OK</span>')+' '+t.rtc.t.toFixed(1)+'&deg;C &middot; drift '+t.rtc.drift+'s':'<span class="badge bad">NOT FOUND</span>'):'disabled')+'</span></div></div>';
    cards+='<div class="card"><h2>WiFi Station</h2><div class="kv"><b>State</b><span>'+esc(s.sta.st)+'</span><b>SSID</b><span>'+esc(s.sta.ssid||'-')+'</span><b>IP</b><span class="mono">'+esc(s.sta.ip||'-')+'</span><b>RSSI</b><span>'+(s.sta.up?s.sta.rssi+' dBm (ch '+s.sta.ch+')':'-')+'</span><b>Drops</b><span>'+s.sta.drops+(s.sta.reasonTxt?' &middot; '+esc(s.sta.reasonTxt):'')+'</span></div></div>';
    cards+='<div class="card"><h2>WiFi Access Point</h2><div class="kv"><b>State</b><span>'+(s.ap.on?'<span class="badge ok">ON</span>':'<span class="badge off">OFF</span>')+(s.ap.captive?' captive portal':'')+'</span><b>SSID</b><span>'+esc(s.ap.ssid)+(s.ap.hidden?' (hidden)':'')+'</span><b>IP</b><span class="mono">'+esc(s.ap.ip||'-')+'</span><b>Clients</b><span>'+s.ap.clients+'</span><b>Channel</b><span>'+(s.ap.ch||'-')+'</span></div></div>';
    cards+='<div class="card"><h2>System</h2><div class="kv"><b>Host</b><span>'+esc(s.host)+'.local</span><b>Uptime</b><span>'+fmtUp(s.up)+'</span><b>Heap</b><span>'+kb(s.heap)+' free &middot; min '+kb(s.minHeap)+'</span><b>Largest block</b><span>'+kb(s.maxBlock)+'</span><b>Firmware</b><span>'+esc(s.fw)+'</span></div>'+(s.btn?'<div class="warnbox">BOOT button held '+(s.btn/1000).toFixed(1)+' s - factory reset at 5 s</div>':'')+'</div>';
    $('#cards').innerHTML=cards;
    var tl=$('#tiles');if(tl.children.length===s.relays.length){s.relays.forEach(function(r,i){var t=tl.children[i];$('.n',t).textContent=(i+1)+'. '+r.n;$('.sb',t).innerHTML=stBadge(r.s);$('.pn',t).textContent=r.p>=0?'GPIO'+r.p:'no pin';$('.mn',t).textContent=modeName(r.m);var mb=$('.mbx',t);if(mb.getAttribute('data-m')!=String(r.m)){mb.setAttribute('data-m',r.m);mb.innerHTML=modeBtns(i,r.m);bindModes(mb,load);}});return;}
    tl.innerHTML=s.relays.map(function(r,i){return '<div class="tile"><div class="n" title="'+esc(r.n)+'">'+(i+1)+'. '+esc(r.n)+'</div><div class="st"><span class="sb">'+stBadge(r.s)+'</span><span class="pn">'+(r.p>=0?'GPIO'+r.p:'no pin')+'</span></div><div class="st"><span class="mn">'+modeName(r.m)+'</span></div><div class="mbx" data-m="'+r.m+'">'+modeBtns(i,r.m)+'</div></div>';}).join('');
    bindModes(tl,load);
  }catch(e){}}
  load();every(3000,load);
}
var FIELDS=[['sec','0-59'],['min','0-59'],['hour','0-23'],['dow','0-6'],['dom','1-31'],['mon','1-12'],['year','*']];
var PRESETS=[['','Preset...'],['0 0 8 * * * *|0 0 17 * * * *','Daily 08:00 - 17:00'],['0 30 7 1-5 * * *|0 0 18 1-5 * * *','Weekdays 07:30 - 18:00'],['0 0 22 * * * *|0 0 6 * * * *','Overnight 22:00 - 06:00'],['0 0 9 0,6 * * *|0 0 21 0,6 * * *','Weekend 09:00 - 21:00'],['0 */15 * * * * *|30 */15 * * * * *','Every 15 min, on for 30 s'],['0 0 0 * 1 * *|0 0 0 * 2 * *','Monthly: 1st 00:00 - 2nd 00:00'],['0 0 12 * 29 2 *|0 0 13 * 29 2 *','Leap day Feb 29 12:00 - 13:00'],['clear','Clear pair']];
var splitSpec=function(s){s=(s||'').trim();if(!s)return ['','','','','','',''];var t=s.split(/\s+/);while(t.length<7)t.push('*');return t.slice(0,7);};
var specInputs=function(prefix,s){var v=splitSpec(s);return '<span class="f">'+FIELDS.map(function(f,i){return '<label class="fi"><small>'+f[0]+'</small><input type="text" autocapitalize="off" autocomplete="off" spellcheck="false" data-k="'+prefix+'" data-f="'+i+'" value="'+esc(v[i])+'" placeholder="*" title="'+f[0]+' ('+f[1]+')"'+(i===6?' class="y"':'')+'></label>';}).join('')+'</span>';};
var joinSpec=function(root,prefix){var v=$$('input[data-k="'+prefix+'"]',root).map(function(i){return i.value.replace(/\s+/g,'');});if(v.every(function(x){return !x;}))return '';return v.map(function(x){return x||'*';}).join(' ');};
var relays=function(){
  var m=$('#main');m.innerHTML='<h1>Relays</h1><p class="hint">Double-tap / double-click a relay name to rename it. Each relay has 8 START/STOP pairs; the relay is ON while any enabled pair window is active. Fields: <code>sec min hour dow dom month year</code> &middot; syntax <code>*</code> <code>5</code> <code>1,3,5</code> <code>1-5</code> <code>*/10</code> <code>8-17/2</code> <code>mon-fri</code> <code>jan,jul</code> &middot; dow 0=Sun (7 also = Sun) &middot; year <code>*</code> or <code>2026</code> or <code>2026-2030</code> &middot; all fields must match (dow AND dom) &middot; empty START/STOP = never.</p><div id="list"></div>';
  var open={};
  var refresh=async function(){try{var rs=await api('/api/relays');var list=$('#list');
    if(list.children.length!==rs.length){list.innerHTML=rs.map(function(r,i){return '<div class="relay" data-i="'+i+'"><span class="mut">#'+(i+1)+'</span><span class="nm" data-i="'+i+'"></span><button class="btn ic pen" title="Rename" aria-label="Rename relay">&#9998;</button><span class="state"></span><span class="meta"></span><span class="sp"></span><span class="mb"></span><button class="btn s tog">Schedules</button><div class="ed" style="flex-basis:100%;display:none"></div></div>';}).join('');
      $$('.relay',list).forEach(function(el){var i=+el.getAttribute('data-i');$('.tog',el).onclick=function(){if(open[i]){open[i]=false;$('.ed',el).style.display='none';$('.ed',el).innerHTML='';$('.tog',el).textContent='Schedules';}else{open[i]=true;$('.tog',el).textContent='Hide';loadEditor(i,el);}};bindName(el,i);});}
    rs.forEach(function(r,i){var el=list.children[i];var nm=$('.nm',el);if(!nm.querySelector('input')&&nm.textContent!==r.n)nm.textContent=r.n;
      $('.state',el).innerHTML=stBadge(r.s);$('.meta',el).innerHTML=(r.p>=0?'GPIO'+r.p+' '+(r.al?'active-LOW':'active-HIGH'):'<span class="badge warn">NO PIN</span>')+' &middot; '+modeName(r.m)+' &middot; '+bits(r.en)+' pair(s)'+(r.act?' &middot; <span class="badge ok">window active</span>':'');
      var mb=$('.mb',el);if(mb.getAttribute('data-m')!=String(r.m)){mb.setAttribute('data-m',r.m);mb.innerHTML=modeBtns(i,r.m);bindModes(mb,refresh);}
      if(open[i]){$$('.act',el).forEach(function(a,k){a.innerHTML=(r.act>>k)&1?'<span class="badge ok">active</span>':'<span class="badge off">idle</span>';});}
    });}catch(e){}}
  var bits=function(n){var c=0;while(n){c+=n&1;n>>=1;}return c;};
  var bindName=function(el,i){var nm=$('.nm',el);var last=0;
    var edit=function(){if(nm.querySelector('input'))return;var cur=nm.textContent;nm.innerHTML='<input type="text" maxlength="24" value="'+esc(cur)+'">';var inp=nm.querySelector('input');inp.focus();inp.select();var done=false;
      var finish=function(save){if(done)return;done=true;var v=inp.value.trim();if(save&&v&&v!==cur){api('/api/relay/'+i,{name:v}).then(function(){nm.textContent=v;toast('Renamed');}).catch(function(e){nm.textContent=cur;fail(e);});}else nm.textContent=cur;};
      inp.onkeydown=function(e){if(e.key==='Enter')finish(true);if(e.key==='Escape')finish(false);};inp.onblur=function(){finish(true);};}
    nm.ondblclick=function(e){e.preventDefault();edit();};
    nm.addEventListener('touchend',function(e){var n=Date.now();if(n-last<350){e.preventDefault();edit();}last=n;},{passive:false});
    nm.title='Double-tap to rename';$('.pen',el).onclick=edit;}
  var loadEditor=async function(i,el){var ed=$('.ed',el);ed.style.display='block';ed.innerHTML='<span class="mut">loading...</span>';
    try{var r=await api('/api/relay/'+i);
      var h='<div class="sched"><table><thead><tr><th>Pair</th><th>START (relay turns ON)</th><th>STOP (relay turns OFF)</th><th>Now</th><th>Preset</th></tr></thead><tbody>';
      r.s.forEach(function(p,k){h+='<tr data-k="'+k+'"><td class="num"><label class="chk" style="margin:0"><input type="checkbox" class="en"'+(p.en?' checked':'')+'> <b>#'+(k+1)+'</b> <span class="mut">enabled</span></label></td><td class="lab" data-l="START">'+specInputs('a'+k,p.a)+'</td><td class="lab" data-l="STOP">'+specInputs('b'+k,p.b)+'</td><td class="lab act" data-l="NOW">'+(p.act?'<span class="badge ok">active</span>':'<span class="badge off">idle</span>')+'</td><td><select class="pre" style="width:9em" title="Fill this pair from a preset">'+PRESETS.map(function(x){return '<option value="'+esc(x[0])+'">'+esc(x[1])+'</option>';}).join('')+'</select></td></tr>';});
      h+='</tbody></table></div><div style="margin-top:8px"><button class="btn p save">Save schedules</button><button class="btn reload">Reload</button> <span class="mut msg"></span></div>';
      ed.innerHTML=h;
      var fit=function(i){i.style.width='calc('+Math.max(3,i.value.length+1)+'ch + 16px)';};$$('input[data-k]',ed).forEach(function(i){fit(i);i.oninput=function(){fit(i);};});
      $$('.pre',ed).forEach(function(sel){sel.onchange=function(){var tr=sel.closest('tr');var k=tr.getAttribute('data-k');var v=sel.value;sel.value='';if(!v)return;var a='',b='';if(v!=='clear'){var parts=v.split('|');a=parts[0];b=parts[1]||'';}
        var va=splitSpec(a),vb=splitSpec(b);$$('input[data-k="a'+k+'"]',tr).forEach(function(inp,f){inp.value=va[f];fit(inp);});$$('input[data-k="b'+k+'"]',tr).forEach(function(inp,f){inp.value=vb[f];fit(inp);});$('.en',tr).checked=v!=='clear';};});
      $('.save',ed).onclick=async function(){var s=[];$$('tbody tr',ed).forEach(function(tr,k){s.push({en:$('.en',tr).checked,a:joinSpec(tr,'a'+k),b:joinSpec(tr,'b'+k)});});
        try{await api('/api/relay/'+i,{s:s});$('.msg',ed).textContent='Saved';toast('Schedules saved');refresh();}catch(e){$('.msg',ed).textContent=e.message;fail(e);}};
      $('.reload',ed).onclick=function(){loadEditor(i,el);};
    }catch(e){ed.innerHTML='<span class="badge bad">'+esc(e.message)+'</span>';}}
  refresh();every(2000,refresh);
}
var gpio=function(){
  var m=$('#main');m.innerHTML='<h1>GPIO</h1><div class="card"><div><button class="btn p" id="gh">All ON</button><button class="btn p" id="gl">All OFF</button><button class="btn" id="ga">All relays back to Auto</button></div><p class="hint"><b>Force ON</b> / <b>Force OFF</b> and <b>All ON</b> / <b>All OFF</b> set the relay\'s logical state (State column) and switch it to manual mode; the physical pin level is derived from the Polarity column (Level column) &mdash; with Active LOW the pin goes LOW for ON, with Active HIGH the pin goes HIGH for ON. Remove releases the GPIO (input, relay unassigned). Changes to pin / polarity apply immediately and are saved.</p><div id="sd"></div></div><div class="card gt" style="margin-top:12px;overflow-x:auto"><table id="gt"><thead><tr><th>#</th><th>Relay</th><th>GPIO</th><th>Polarity</th><th>Level</th><th>State</th><th>Mode</th><th>Actions</th></tr></thead><tbody></tbody></table></div>';
  var pins=[];
  var load=async function(){try{var g=await api('/api/gpio');pins=g.pins;if(g.serialDebug)$('#sd').innerHTML='<div class="warnbox">Serial debug build: GPIO1/GPIO3 are reserved for the UART and will not be driven.</div>';
    var used={};g.r.forEach(function(r){if(r.p>=0)used[r.p]=r.n;});
    var tb=$('#gt tbody');var ae=document.activeElement;
    var lvl=function(r){return r.lv<0?'<span class="mut">-</span>':'<span class="badge '+(r.lv?'on':'off')+'">'+(r.lv?'HIGH':'LOW')+'</span>';};
    if(tb.children.length===g.r.length){
      g.r.forEach(function(r,i){var tr=tb.children[i];$('.rn',tr).textContent=r.n;$('.lv',tr).innerHTML=lvl(r);$('.st',tr).innerHTML=stBadge(r.s);$('.md',tr).textContent=modeName(r.m);
        var sp=$('.pin',tr),sa=$('.al',tr);
        if(ae!==sp){if(String(sp.value)!==String(r.p))sp.value=String(r.p);}
        $$('option',sp).forEach(function(o){var p=+o.value;o.disabled=p>=0&&!!used[p]&&p!==r.p;o.textContent=p<0?'none':'GPIO'+p+(o.disabled?' (used)':'');});
        if(ae!==sa){if(sa.value!==(r.al?'1':'0'))sa.value=r.al?'1':'0';}});
      return;}
    tb.innerHTML=g.r.map(function(r,i){var opts='<option value="-1"'+(r.p<0?' selected':'')+'>none</option>'+pins.map(function(p){var u=used[p]&&p!==r.p;return '<option value="'+p+'"'+(p===r.p?' selected':'')+(u?' disabled':'')+'>GPIO'+p+(u?' (used)':'')+'</option>';}).join('');
      return '<tr data-i="'+i+'"><td class="ix">#'+(i+1)+'</td><td class="rn" data-l="Relay">'+esc(r.n)+'</td><td data-l="GPIO"><select class="pin">'+opts+'</select></td><td data-l="Polarity"><select class="al"><option value="1"'+(r.al?' selected':'')+'>Active LOW</option><option value="0"'+(!r.al?' selected':'')+'>Active HIGH</option></select></td><td class="lv" data-l="Level">'+lvl(r)+'</td><td class="st" data-l="State">'+stBadge(r.s)+'</td><td class="md" data-l="Mode">'+modeName(r.m)+'</td><td class="ac" data-l="Actions"><button class="btn s hi" title="Switch to manual mode and turn the relay ON">Force ON</button><button class="btn s lo" title="Switch to manual mode and turn the relay OFF">Force OFF</button><button class="btn s au">Auto</button><button class="btn s r rm">Remove</button></td></tr>';}).join('');
    $$('#gt tbody tr').forEach(function(tr){var i=+tr.getAttribute('data-i');
      $('.pin',tr).onchange=function(){api('/api/gpio/'+i,{pin:+this.value,al:$('.al',tr).value==='1'}).then(function(){toast('GPIO updated');load();}).catch(function(e){fail(e);load();});};
      $('.al',tr).onchange=function(){api('/api/gpio/'+i,{pin:+$('.pin',tr).value,al:this.value==='1'}).then(function(){toast('Polarity updated');load();}).catch(function(e){fail(e);load();});};
      $('.hi',tr).onclick=function(){api('/api/gpio/'+i+'/level',{level:1}).then(load).catch(function(e){fail(e);load();});};
      $('.lo',tr).onclick=function(){api('/api/gpio/'+i+'/level',{level:0}).then(load).catch(function(e){fail(e);load();});};
      $('.au',tr).onclick=function(){api('/api/relay/'+i+'/mode',{mode:0}).then(load).catch(function(e){fail(e);load();});};
      $('.rm',tr).onclick=function(){if(confirm('Unassign the GPIO of relay '+(i+1)+'?'))api('/api/gpio/'+i+'/remove',{}).then(function(){toast('GPIO removed');load();}).catch(fail);};});
  }catch(e){}}
  $('#gh').onclick=function(){if(confirm('Turn ALL assigned relays ON (manual mode)?'))api('/api/gpio/all',{level:1}).then(load).catch(fail);};
  $('#gl').onclick=function(){if(confirm('Turn ALL assigned relays OFF (manual mode)?'))api('/api/gpio/all',{level:0}).then(load).catch(fail);};
  $('#ga').onclick=function(){api('/api/gpio/all',{auto:true}).then(function(){toast('All relays in Auto');load();}).catch(fail);};
  load();every(2000,load);
}
var timePage=function(){
  var m=$('#main');m.innerHTML='<h1>Time</h1><div class="grid"><div class="card" id="tstat"></div><div class="card"><h2>Actions</h2><button class="btn p" id="bn">Sync NTP now</button><button class="btn" id="bb">Sync from browser</button><button class="btn" id="bz">Use browser time zone</button><button class="btn" id="bd">Detect time zone (IP)</button><button class="btn" id="br">Load clock from DS3231</button><button class="btn" id="bw">Write clock to DS3231</button><p class="hint">"Sync from browser" sets the device clock (and the DS3231) from this device - use it when NTP is unreachable. With "automatic browser sync" enabled below, any opened page does this by itself while the clock is invalid.</p></div></div><div class="card" style="margin-top:12px"><h2>Settings</h2><form id="tf"></form></div>';
  var stat=async function(){try{var t=await api('/api/time');var n=t.ntp,r=t.rtc;
    $('#tstat').innerHTML='<h2>Status</h2><div class="kv"><b>Local time</b><span class="mono">'+fmtEpoch(t.e)+'</span><b>UTC</b><span class="mono">'+(function(){var d=new Date(t.e*1000);return fmtDate(d)+' '+fmtTime(d);})()+'</span><b>UTC offset</b><span>'+(t.o>=0?'+':'')+(t.o/3600).toFixed(2)+' h ('+t.o+' s)</span><b>Clock</b><span>'+(t.v?'<span class="badge ok">VALID</span>':'<span class="badge bad">NOT SET</span>')+' source: '+esc(t.src)+(t.set?' &middot; set '+fmtEpoch(t.set):'')+'</span><b>TZ rule</b><span class="mono">'+esc(t.tz)+' <span class="mut">('+esc(t.tzs)+')</span></span>'+(t.tzMode==1?'<b>Auto TZ</b><span>'+esc(t.tzr)+'</span>':'')+'<b>NTP</b><span>'+esc(n.st)+(n.cur&&n.st!=='idle'&&n.st!=='offline'?' '+esc(n.cur):'')+'</span><b>Last NTP</b><span>'+(n.last?fmtEpoch(n.last)+' via '+esc(n.srv)+' (step '+fmtOff(n.off)+')':'never')+'</span><b>Next NTP</b><span>'+(n.next?fmtUp(n.next):'-')+' &middot; interval '+n.iv+' min &middot; ok '+n.ok+' / failed cycles '+n.fails+(n.err?' &middot; <span class="badge warn">'+esc(n.err)+'</span>':'')+'</span><b>DS3231</b><span>'+(r.en?(r.ok?(r.lost?'<span class="badge warn">LOST POWER</span>':'<span class="badge ok">PRESENT</span>')+' '+fmtEpoch(r.e)+' &middot; '+r.t.toFixed(2)+'&deg;C &middot; drift '+r.drift+' s &middot; writes '+r.w:'<span class="badge bad">NOT FOUND</span> (SDA 21 / SCL 22)'):'disabled')+'</span></div>';
  }catch(e){}}
  var sel=function(name,opts,val){return '<select name="'+name+'">'+opts.map(function(o){return '<option value="'+o[0]+'"'+(String(o[0])===String(val)?' selected':'')+'>'+esc(o[1])+'</option>';}).join('')+'</select>';};
  var MONTHS=[[1,'Jan'],[2,'Feb'],[3,'Mar'],[4,'Apr'],[5,'May'],[6,'Jun'],[7,'Jul'],[8,'Aug'],[9,'Sep'],[10,'Oct'],[11,'Nov'],[12,'Dec']],WEEKS=[[1,'1st'],[2,'2nd'],[3,'3rd'],[4,'4th'],[5,'last']],DAYS=[[0,'Sun'],[1,'Mon'],[2,'Tue'],[3,'Wed'],[4,'Thu'],[5,'Fri'],[6,'Sat']];
  var form=async function(){try{var c=await api('/api/time/cfg');var f=$('#tf');
    f.innerHTML='<h3>NTP</h3><div class="row">'+[0,1,2,3].map(function(i){return '<div><label>Server '+(i+1)+'</label><input type="text" name="ntp'+i+'" value="'+esc(c.ntp[i]||'')+'" maxlength="48"></div>';}).join('')+'</div><div class="row"><div><label>Auto sync interval (minutes, 1 - 1440 = 24 h)</label><input type="number" name="interval" min="1" max="1440" value="'+c.interval+'"></div><div><label>Reply timeout per server (ms)</label><input type="number" name="timeout" min="500" max="10000" value="'+c.timeout+'"></div></div><label class="chk"><input type="checkbox" name="rotate"'+(c.rotate?' checked':'')+'> Rotate servers (round-robin each sync). Off = always start with server 1, fall back to the next on failure.</label>'
      +'<h3>Clock sources</h3><label class="chk"><input type="checkbox" name="rtc"'+(c.rtc?' checked':'')+'> Use DS3231 RTC (I2C SDA 21 / SCL 22): seeds the clock at boot, keeps it disciplined while offline, written after every NTP / browser sync.</label><label class="chk"><input type="checkbox" name="autoBrowser"'+(c.autoBrowser?' checked':'')+'> Automatic browser sync when the clock is not set (NTP unreachable, no RTC).</label>'
      +'<h3>Time zone</h3><div class="row"><div><label>Mode</label>'+sel('tzMode',[[0,'Manual (offset / rules / POSIX)'],[1,'Automatic (IP geolocation + DST rules)']],c.tzMode)+'</div><div><label>GMT offset in seconds (e.g. -18000 = UTC-5, 3600 = UTC+1) <span class="mut" id="offh"></span></label><input type="number" name="gmtOffset" min="-50400" max="50400" step="1" value="'+c.gmtOffset+'"></div><div><label>DST offset (seconds)</label><input type="number" name="dstOffset" min="-7200" max="7200" value="'+c.dstOffset+'"></div></div>'
      +'<label class="chk"><input type="checkbox" name="dst"'+(c.dst?' checked':'')+'> Daylight saving time rules enabled</label><div class="row"><div><label>DST starts</label><div class="row">'+sel('dsM',MONTHS,c.dsM)+sel('dsW',WEEKS,c.dsW)+sel('dsD',DAYS,c.dsD)+'<input type="number" name="dsH" min="0" max="24" value="'+c.dsH+'" title="hour (local standard time)"></div></div><div><label>DST ends</label><div class="row">'+sel('deM',MONTHS,c.deM)+sel('deW',WEEKS,c.deW)+sel('deD',DAYS,c.deD)+'<input type="number" name="deH" min="0" max="24" value="'+c.deH+'" title="hour (local DST time)"></div></div></div>'
      +'<div><span class="mut" style="font-size:12px">Presets:</span> <button type="button" class="btn s" data-p="us">USA/Canada</button><button type="button" class="btn s" data-p="eu">EU (CET)</button><button type="button" class="btn s" data-p="uk">UK/Portugal</button><button type="button" class="btn s" data-p="au">Australia SE</button><button type="button" class="btn s" data-p="nz">New Zealand</button></div>'
      +'<label>POSIX TZ override (manual mode; empty = use the fields above) e.g. <code>EST5EDT,M3.2.0,M11.1.0</code></label><input type="text" name="posix" maxlength="64" value="'+esc(c.posix)+'">'
      +(c.acValid?'<p class="hint">Last automatic detection: <b>'+esc(c.acName)+'</b> &rarr; '+(c.acPosix?'<code>'+esc(c.acPosix)+'</code>':'fixed offset '+c.acOffset+' s (no rule in table - refreshed hourly)')+'</p>':'')
      +'<p class="hint">Active rule now: <code>'+esc(c.tzNow)+'</code></p><div style="margin-top:10px"><button class="btn p" type="submit">Save time settings</button></div>';
    var offh=function(){var v=+f.gmtOffset.value||0,a=Math.abs(v);$('#offh').textContent='= UTC'+(v<0?'-':'+')+p2(Math.floor(a/3600))+':'+p2(Math.floor(a%3600/60))+(a%60?':'+p2(a%60):'');};f.gmtOffset.oninput=offh;offh();
    var P={us:[-18000,3,2,0,2,11,1,0,2],eu:[3600,3,5,0,2,10,5,0,3],uk:[0,3,5,0,1,10,5,0,2],au:[36000,10,1,0,2,4,1,0,3],nz:[43200,9,5,0,2,4,1,0,3]};
    $$('button[data-p]',f).forEach(function(b){b.onclick=function(){var p=P[b.getAttribute('data-p')];f.gmtOffset.value=p[0];f.dst.checked=true;f.dstOffset.value=3600;f.dsM.value=p[1];f.dsW.value=p[2];f.dsD.value=p[3];f.dsH.value=p[4];f.deM.value=p[5];f.deW.value=p[6];f.deD.value=p[7];f.deH.value=p[8];f.posix.value='';f.tzMode.value='0';offh();};});
    f.onsubmit=async function(e){e.preventDefault();var b={ntp:[0,1,2,3].map(function(i){return f['ntp'+i].value.trim();}),interval:+f.interval.value,timeout:+f.timeout.value,rotate:f.rotate.checked,rtc:f.rtc.checked,autoBrowser:f.autoBrowser.checked,tzMode:+f.tzMode.value,gmtOffset:+f.gmtOffset.value,dst:f.dst.checked,dstOffset:+f.dstOffset.value,posix:f.posix.value.trim()};
      ['dsM','dsW','dsD','dsH','deM','deW','deD','deH'].forEach(function(k){b[k]=+f[k].value;});
      try{await api('/api/time/cfg',b);toast('Time settings saved');form();stat();pollTime();}catch(e){fail(e);}};
  }catch(e){fail(e);}}
  $('#bn').onclick=function(){api('/api/time/sync',{}).then(function(){toast('NTP sync started');setTimeout(stat,1500);}).catch(fail);};
  $('#bb').onclick=function(){browserSync().then(stat);};
  $('#bz').onclick=function(){var iana='';try{iana=Intl.DateTimeFormat().resolvedOptions().timeZone||'';}catch(e){}var off=-new Date().getTimezoneOffset()*60;
    api('/api/time/tz',{iana:iana,offset:off}).then(function(r){toast(r.found?'Zone '+iana+' applied with DST rules':'Zone '+(iana||'?')+' not in table - fixed offset '+off+' s applied');form();stat();pollTime();}).catch(fail);};
  $('#bd').onclick=function(){api('/api/time/tzdetect',{}).then(function(){toast('Detection started (needs internet)');setTimeout(function(){form();stat();pollTime();},4000);}).catch(fail);};
  $('#br').onclick=function(){api('/api/time/rtcread',{}).then(function(){toast('Clock will be loaded from DS3231');setTimeout(function(){stat();pollTime();},1500);}).catch(fail);};
  $('#bw').onclick=function(){api('/api/time/rtcwrite',{}).then(function(){toast('DS3231 will be written');setTimeout(stat,1500);}).catch(fail);};
  stat();form();every(2000,stat);
}
var sta=function(){
  var m=$('#main');m.innerHTML='<h1>WiFi Station</h1><div class="grid"><div class="card" id="ss"></div><div class="card"><h2>Settings</h2><form id="sf"></form></div></div>';
  var stat=async function(){try{var s=await api('/api/status');var t=s.sta;$('#ss').innerHTML='<h2>Status</h2><div class="kv"><b>State</b><span>'+esc(t.st)+'</span><b>SSID</b><span>'+esc(t.ssid||'-')+'</span><b>IP</b><span class="mono">'+esc(t.ip||'-')+'</span><b>Gateway</b><span class="mono">'+esc(t.gw||'-')+'</span><b>Mask</b><span class="mono">'+esc(t.mask||'-')+'</span><b>DNS</b><span class="mono">'+esc(t.dns||'-')+'</span><b>RSSI</b><span>'+(t.up?t.rssi+' dBm, channel '+t.ch:'-')+'</span><b>BSSID</b><span class="mono">'+esc(t.bssid||'-')+'</span><b>MAC</b><span class="mono">'+esc(t.mac)+'</span><b>Hostname</b><span>'+esc(t.host)+'.local</span><b>Connected since</b><span>'+(t.up&&t.since?fmtEpoch(t.since):'-')+'</span><b>Attempts / drops</b><span>'+t.tries+' / '+t.drops+'</span><b>Last reason</b><span>'+(t.reason?t.reason+' '+esc(t.reasonTxt):'-')+'</span></div><p class="hint">While disconnected the station retries every 2 seconds. Every successful (re)connect triggers an NTP sync; on disconnect the clock is re-disciplined from the DS3231.</p>';}catch(e){}};
  var form=async function(){try{var c=await api('/api/net');var s=c.sta;var f=$('#sf');
    f.innerHTML='<label class="chk"><input type="checkbox" name="en"'+(s.en?' checked':'')+'> Station enabled</label><label>SSID</label><div class="row"><input type="text" name="ssid" maxlength="32" value="'+esc(s.ssid)+'"><button type="button" class="btn fix" id="scan">Scan</button></div><div class="nets" id="nets" style="display:none"></div><label>Password '+(s.passSet?'<span class="mut">(set - leave blank to keep)</span>':'')+'</label><input type="password" name="pass" maxlength="64" autocomplete="new-password"><label>Hostname (mDNS: name.local)</label><input type="text" name="host" maxlength="32" value="'+esc(c.host)+'"><label class="chk"><input type="checkbox" name="dhcp"'+(s.dhcp?' checked':'')+'> DHCP (automatic IP)</label><div id="static"><div class="row"><div><label>Static IP</label><input type="text" name="ip" value="'+esc(s.ip)+'"></div><div><label>Gateway</label><input type="text" name="gw" value="'+esc(s.gw)+'"></div><div><label>Subnet mask</label><input type="text" name="mask" value="'+esc(s.mask)+'"></div></div><div class="row"><div><label>DNS 1</label><input type="text" name="dns1" value="'+esc(s.dns1)+'"></div><div><label>DNS 2</label><input type="text" name="dns2" value="'+esc(s.dns2)+'"></div></div></div><div style="margin-top:10px"><button class="btn p" type="submit">Save &amp; connect</button></div><p class="hint">If you are connected through the station interface, saving new credentials may drop this connection; reconnect via the new IP or the access point.</p>';
    var st=function(){$('#static').style.display=f.dhcp.checked?'none':'block';};f.dhcp.onchange=st;st();
    $('#scan').onclick=scan;
    f.onsubmit=async function(e){e.preventDefault();var b={en:f.en.checked,ssid:f.ssid.value,host:f.host.value.trim(),dhcp:f.dhcp.checked,ip:f.ip.value.trim(),gw:f.gw.value.trim(),mask:f.mask.value.trim(),dns1:f.dns1.value.trim(),dns2:f.dns2.value.trim(),connect:true};if(f.pass.value)b.pass=f.pass.value;
      try{await api('/api/net/sta',b);toast('Saved - applying station settings');f.pass.value='';setTimeout(form,800);}catch(e){fail(e);}};
  }catch(e){fail(e);}}
  var scanT;
  var scan=function(){var box=$('#nets');box.style.display='block';box.innerHTML='<div class="mut">scanning...</div>';clearTimeout(scanT);
    (async function poll(){try{var r=await api('/api/net/scan');if(r.st==='done'){var nets=r.nets.sort(function(a,b){return b.rssi-a.rssi;});box.innerHTML=nets.length?nets.map(function(n){return '<div data-s="'+esc(n.ssid)+'"><span>'+(n.ssid?esc(n.ssid):'<i>(hidden)</i>')+' '+(n.enc?'&#128274;':'')+'</span><span class="mut">ch '+n.ch+' &middot; '+n.rssi+' dBm</span></div>';}).join(''):'<div class="mut">no networks found</div>';
        $$('div[data-s]',box).forEach(function(d){d.onclick=function(){$('#sf').ssid.value=d.getAttribute('data-s');box.style.display='none';};});}
      else if(r.st==='busy'){box.innerHTML='<div class="mut">radio busy (connecting?) - try again</div>';}else scanT=setTimeout(poll,1200);}catch(e){box.innerHTML='<div class="mut">'+esc(e.message)+'</div>';}})();}
  stat();form();every(3000,stat);
}
var ap=function(){
  var m=$('#main');m.innerHTML='<h1>WiFi Access Point</h1><div class="grid"><div class="card" id="as"></div><div class="card"><h2>Settings</h2><form id="af"></form></div></div>';
  var TX=[[78,'19.5 dBm (max)'],[68,'17 dBm'],[60,'15 dBm'],[52,'13 dBm'],[44,'11 dBm'],[34,'8.5 dBm'],[28,'7 dBm'],[20,'5 dBm'],[8,'2 dBm']];
  var stat=async function(){try{var s=await api('/api/status');var a=s.ap;$('#as').innerHTML='<h2>Status</h2><div class="kv"><b>State</b><span>'+(a.on?'<span class="badge ok">RUNNING</span>':'<span class="badge off">OFF</span>')+'</span><b>SSID</b><span>'+esc(a.ssid)+(a.hidden?' <span class="badge">hidden</span>':'')+'</span><b>IP</b><span class="mono">'+esc(a.ip||'-')+'</span><b>Channel</b><span>'+(a.ch||'-')+(s.sta.up?' <span class="mut">(follows the station channel while connected)</span>':'')+'</span><b>Clients</b><span>'+a.clients+'</span><b>Captive portal</b><span>'+(a.captive?'active':'off')+'</span><b>AP MAC</b><span class="mono">'+esc(a.mac)+'</span><b>Mode</b><span>'+(a.mode==1?'fallback (only while station is down)':'always on')+'</span></div>';}catch(e){}};
  var form=async function(){try{var c=await api('/api/net');var a=c.ap;var f=$('#af');
    f.innerHTML='<label class="chk"><input type="checkbox" name="en"'+(a.en?' checked':'')+'> Access point enabled <span class="mut">(forced on while no station network is configured)</span></label><label>SSID</label><input type="text" name="ssid" maxlength="32" value="'+esc(a.ssid)+'"><label>Password '+(a.passSet?'<span class="mut">(set - leave blank to keep, type OPEN to remove)</span>':'<span class="mut">(empty = open network)</span>')+'</label><input type="password" name="pass" maxlength="63" autocomplete="new-password"><div class="row"><div><label>Channel (1-13)</label><input type="number" name="ch" min="1" max="13" value="'+a.ch+'"></div><div><label>Max clients (1-8)</label><input type="number" name="max" min="1" max="8" value="'+a.max+'"></div><div><label>TX power (radio-wide)</label><select name="tx">'+TX.map(function(t){return '<option value="'+t[0]+'"'+(t[0]===a.tx?' selected':'')+'>'+t[1]+'</option>';}).join('')+'</select></div></div><div class="row"><div><label>AP IP address</label><input type="text" name="ip" value="'+esc(a.ip)+'"></div><div><label>Mode</label><select name="mode"><option value="0"'+(a.mode==0?' selected':'')+'>Always on</option><option value="1"'+(a.mode==1?' selected':'')+'>Fallback: only while station is disconnected</option></select></div></div><label class="chk"><input type="checkbox" name="hid"'+(a.hidden?' checked':'')+'> Hidden SSID</label><label class="chk"><input type="checkbox" name="captive"'+(a.captive?' checked':'')+'> Captive portal (DNS hijack + OS sign-in detection on the AP network)</label><div style="margin-top:10px"><button class="btn p" type="submit">Save &amp; apply</button></div><p class="hint">Applying restarts the access point: clients connected to it will reconnect. While the station is connected, the AP channel is dictated by the router (single radio).</p>';
    f.onsubmit=async function(e){e.preventDefault();var b={en:f.en.checked,ssid:f.ssid.value,ch:+f.ch.value,max:+f.max.value,tx:+f.tx.value,ip:f.ip.value.trim(),mode:+f.mode.value,hidden:f.hid.checked,captive:f.captive.checked};var p=f.pass.value;if(p==='OPEN')b.pass='';else if(p)b.pass=p;
      try{var r=await api('/api/net/ap',b);toast(r.forced&&!b.en?'Saved - AP stays on until a station network is configured':'Saved - applying AP settings');f.pass.value='';setTimeout(form,800);}catch(e){fail(e);}};
  }catch(e){fail(e);}}
  stat();form();every(3000,stat);
}
var system=function(){
  var m=$('#main');m.innerHTML='<h1>System</h1><div class="grid" id="sg"></div><div class="card" style="margin-top:12px"><h2>Maintenance</h2><button class="btn" id="bk">Download config backup</button><button class="btn" id="rb">Reboot</button><button class="btn r" id="fr">Factory reset</button><p class="hint">Factory reset erases all settings (WiFi, time, GPIO map, relays) and reboots into the default access point. Holding the BOOT button (GPIO0) for 5 seconds does the same.</p><div id="bt"></div></div>';
  var load=async function(){try{var s=await api('/api/system');var t=s.time;
    var h='<div class="card"><h2>Device</h2><div class="kv"><b>Chip</b><span>'+esc(s.chip)+' rev '+s.rev+', '+s.cores+' core(s) @ '+s.mhz+' MHz</span><b>SDK</b><span>'+esc(s.sdk)+'</span><b>Firmware</b><span>'+esc(s.fw)+'</span><b>Flash</b><span>'+kb(s.flash)+'</span><b>Sketch</b><span>'+kb(s.sketch)+' used, '+kb(s.sketchFree)+' free</span><b>PSRAM</b><span>'+(s.psram?kb(s.psram):'none')+'</span><b>Reset reason</b><span>'+esc(s.reset)+'</span><b>Uptime</b><span>'+fmtUp(s.up)+'</span><b>Serial debug</b><span>'+(s.serialDebug?'on (GPIO1/3 reserved)':'off')+'</span><b>Clock range</b><span>'+(s.timeT64?'64-bit time_t: valid beyond 2038':'<span class="badge warn">32-bit time_t</span> valid until 2038-01-19 - build with core 3.x')+'</span><b>I2C</b><span>'+esc(s.i2c)+'</span></div></div>';
    var frag=s.maxBlock&&s.heap?Math.round(100-100*s.maxBlock/s.heap):0;
    h+='<div class="card"><h2>Memory</h2><div class="kv"><b>Heap free</b><span>'+kb(s.heap)+' of '+kb(s.heapTotal)+'</span><b>Min free ever</b><span>'+kb(s.minHeap)+'</span><b>Largest block</b><span>'+kb(s.maxBlock)+'</span><b>Fragmentation</b><span>'+frag+' %</span></div><div class="prog"><i style="width:'+Math.round(100*s.heap/s.heapTotal)+'%"></i></div></div>';
    h+='<div class="card"><h2>Storage (LittleFS)</h2><div class="kv"><b>Mounted</b><span>'+(s.fsOk?'yes':'<span class="badge bad">NO</span>')+'</span><b>Used</b><span>'+kb(s.fsUsed)+' / '+kb(s.fsTotal)+'</span><b>Pending writes</b><span>'+(s.dirty?'yes (debounced)':'none')+'</span></div><h3>Config files</h3><table>'+(s.files.length?s.files.map(function(f){return '<tr><td class="mono">'+esc(f.n)+'</td><td>'+f.s+' B</td></tr>';}).join(''):'<tr><td class="mut">no files yet (defaults in use)</td></tr>')+'</table></div>';
    h+='<div class="card"><h2>Time</h2><div class="kv"><b>Local</b><span class="mono">'+fmtEpoch(t.e)+'</span><b>Source</b><span>'+esc(t.src)+(t.v?'':' <span class="badge bad">NOT SET</span>')+'</span><b>TZ</b><span class="mono">'+esc(t.tz)+'</span><b>NTP</b><span>'+esc(t.ntp.st)+' &middot; ok '+t.ntp.ok+' &middot; failed cycles '+t.ntp.fails+'</span><b>DS3231</b><span>'+(t.rtc.en?(t.rtc.ok?'present, '+t.rtc.t.toFixed(1)+'&deg;C, drift '+t.rtc.drift+' s':'not found'):'disabled')+'</span><b>Schedule recalcs</b><span>'+s.recalcs+'</span></div></div>';
    h+='<div class="card"><h2>Network</h2><div class="kv"><b>Station</b><span>'+esc(s.sta.st)+' '+esc(s.sta.ip||'')+'</span><b>STA MAC</b><span class="mono">'+esc(s.sta.mac)+'</span><b>Access point</b><span>'+(s.ap.on?'on, '+esc(s.ap.ip)+', '+s.ap.clients+' client(s)':'off')+'</span><b>AP MAC</b><span class="mono">'+esc(s.ap.mac)+'</span><b>Hostname</b><span>'+esc(s.sta.host)+'.local</span></div></div>';
    $('#sg').innerHTML=h;$('#bt').innerHTML=s.btn?'<div class="warnbox">BOOT button held '+(s.btn/1000).toFixed(1)+' s</div>':'';
  }catch(e){}}
  $('#bk').onclick=function(){window.location='/api/backup';};
  $('#rb').onclick=function(){if(confirm('Reboot the device now?'))api('/api/system/reboot',{}).then(function(){toast('Rebooting...');}).catch(fail);};
  $('#fr').onclick=function(){if(confirm('FACTORY RESET: erase ALL settings and reboot?')&&confirm('Really erase everything?'))api('/api/system/factory',{}).then(function(){toast('Factory reset in progress...');}).catch(fail);};
  load();every(3000,load);
}
shell();
({dash:dash,relays:relays,gpio:gpio,time:timePage,sta:sta,ap:ap,system:system}[page]||dash)();
})();
)JS";

static void sendPage(const char* id) {
  if (captiveRedirect()) return;
  snprintf(jbuf, sizeof(jbuf), PAGE_TMPL, id);
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "text/html", jbuf);
}

static void setupRoutes() {
  server.on("/",        HTTP_GET, []() { sendPage("dash"); });
  server.on("/index.html", HTTP_GET, []() { sendPage("dash"); });
  server.on("/relays",  HTTP_GET, []() { sendPage("relays"); });
  server.on("/gpio",    HTTP_GET, []() { sendPage("gpio"); });
  server.on("/time",    HTTP_GET, []() { sendPage("time"); });
  server.on("/sta",     HTTP_GET, []() { sendPage("sta"); });
  server.on("/ap",      HTTP_GET, []() { sendPage("ap"); });
  server.on("/system",  HTTP_GET, []() { sendPage("system"); });
  server.on("/app.css", HTTP_GET, []() { server.sendHeader("Cache-Control", "max-age=86400"); server.send_P(200, "text/css", APP_CSS); });
  server.on("/app.js",  HTTP_GET, []() { server.sendHeader("Cache-Control", "max-age=86400"); server.send_P(200, "application/javascript", APP_JS); });
  server.on("/favicon.ico", HTTP_GET, []() { server.send(204, "text/plain", ""); });
  server.on("/api/status",        HTTP_GET,  apiStatus);
  server.on("/api/time",          HTTP_GET,  apiTimeGet);
  server.on("/api/time/cfg",      HTTP_GET,  apiTimeCfgGet);
  server.on("/api/time/cfg",      HTTP_POST, apiTimeCfgPost);
  server.on("/api/time/sync",     HTTP_POST, apiTimeSync);
  server.on("/api/time/set",      HTTP_POST, apiTimeSet);
  server.on("/api/time/tz",       HTTP_POST, apiTimeTz);
  server.on("/api/time/tzdetect", HTTP_POST, apiTimeTzDetect);
  server.on("/api/time/rtcread",  HTTP_POST, apiTimeRtcRead);
  server.on("/api/time/rtcwrite", HTTP_POST, apiTimeRtcWrite);
  server.on("/api/relays",                     HTTP_GET,  apiRelaysGet);
  server.on(UriBraces("/api/relay/{}/mode"),   HTTP_POST, apiRelayMode);
  server.on(UriBraces("/api/relay/{}"),        HTTP_GET,  apiRelayGet);
  server.on(UriBraces("/api/relay/{}"),        HTTP_POST, apiRelayPost);
  server.on("/api/gpio",                       HTTP_GET,  apiGpioGet);
  server.on("/api/gpio/all",                   HTTP_POST, apiGpioAll);
  server.on(UriBraces("/api/gpio/{}/level"),   HTTP_POST, apiGpioLevel);
  server.on(UriBraces("/api/gpio/{}/remove"),  HTTP_POST, apiGpioRemove);
  server.on(UriBraces("/api/gpio/{}"),         HTTP_POST, apiGpioSet);
  server.on("/api/net",      HTTP_GET,  apiNetGet);
  server.on("/api/net/sta",  HTTP_POST, apiNetStaPost);
  server.on("/api/net/ap",   HTTP_POST, apiNetApPost);
  server.on("/api/net/scan", HTTP_GET,  apiNetScan);
  server.on("/api/system",         HTTP_GET,  apiSystem);
  server.on("/api/system/reboot",  HTTP_POST, apiReboot);
  server.on("/api/system/factory", HTTP_POST, apiFactory);
  server.on("/api/backup",         HTTP_GET,  apiBackup);
  const char* probes[] = {"/generate_204", "/gen_204", "/hotspot-detect.html", "/library/test/success.html", "/connecttest.txt",
                          "/ncsi.txt", "/redirect", "/fwlink", "/success.txt", "/canonical.html", "/check_network_status.txt", "/mobile/status.php"};
  for (const char* p : probes) server.on(p, handleCaptiveProbe);
  server.onNotFound(handleNotFound);
}

void setup() {
  pinMode(PIN_BOOT_BTN, INPUT_PULLUP);
  checkTimeTWidth(std::conditional<(sizeof(time_t) >= 8), TimeTIs64, TimeTIs32>::type());

  fsOk = LittleFS.begin(false);
  if (!fsOk) { fsOk = LittleFS.begin(true); }
  if (fsOk && !LittleFS.exists(CFG_DIR)) LittleFS.mkdir(CFG_DIR);
  loadAllConfig();
  initAllPins();

  applyTimezone();
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  rtcProbe();
  if (rtcUsable()) {
    setSystemTime((double)rtcEpoch + 0.5, SRC_RTC);
    rtcRequestAlignSet();
  }

  WiFi.onEvent(onWiFiEvent);
  applyNetworkConfig();

  setupRoutes();
  server.begin();
}

void loop() {
  server.handleClient();
  dnsServer.processNextRequest();
  wifiTick();
  ntpTick();
  tzTick();
  rtcTick();
  schedTick();
  buttonTick();
  storageTick();

  if (restartAtMs && (int32_t)(millis() - restartAtMs) >= 0) {
    if (factoryPending) { dirtyMask = 0; factoryResetFiles(); }
    else saveDirtyNow();
    delay(100);
    ESP.restart();
  }
}
