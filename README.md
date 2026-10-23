# ESP32 16-Relay Controller — Firmware Documentation

**Firmware version:** 1.0.0 · **Asset version:** 5 (cache-busting key for `/app.css` and `/app.js`)

---

## 1. Overview

This firmware turns an ESP32 into a 16-channel relay controller with schedule automation, time synchronization (NTP + DS3231 RTC + browser fallback), automatic time-zone handling, WiFi station + access-point modes with a captive portal, and a fully self-contained web UI served from flash (no LittleFS assets required — the page template, CSS, and JS are compiled into the binary as PROGMEM strings).

### Key features

- **16 relays**, each with a configurable name, GPIO pin, polarity (active-LOW / active-HIGH), and a mode (`Auto`, `Force ON`, `Force OFF`).
- **8 schedule pairs per relay** — each pair has a START and STOP cron-style specification. A relay in Auto mode is ON while any enabled pair window is active.
- **Rich cron-style spec syntax** — `sec min hour dow dom month year`, supporting `*`, values, lists (`1,3,5`), ranges (`1-5`), steps (`*/10`, `8-17/2`), and names (`mon-fri`, `jan,jul`). `dow` 0=Sunday (7 also = Sunday). Year may be `*`, `2026`, or `2026-2030`.
- **Robust scheduling engine** — per-second evaluation plus a "find previous match" look-back (up to ~8 years) used for recomputation after time jumps, NTP syncs, DST changes, or config edits. Handles reboot catch-up correctly.
- **Time sources, in priority order for a given set:** DS3231 RTC (seeds clock at boot), NTP (4 configurable servers, rotation, backoff), and browser time (manual or automatic when the clock is invalid).
- **Time-zone engine** — manual offset + custom DST rules + arbitrary POSIX TZ strings, or automatic via IP geolocation (`ip-api.com`) with a built-in IANA→POSIX lookup table. Fixed-offset fallback for zones not in the table.
- **WiFi station** with DHCP or static IP, hostname/mDNS, exponential backoff reconnect, and detailed disconnect diagnostics.
- **WiFi access point** with configurable SSID/password/channel/hidden/ max-clients/ TX power/ IP, optional **captive portal** (DNS hijack + OS probe responses), and a "fallback" mode that only keeps the AP on while the station is down.
- **Atomic, CRC-verified config storage** on LittleFS with `.tmp`/`.bak` recovery and debounced writes (1.5 s).
- **Factory reset** via BOOT button (5 s hold) or web UI; also restores defaults for all config.
- **Backup/restore** via a single JSON download.
- Web UI is **responsive**, touch-friendly, with a live clock that continues ticking between polls and a server-side-only time model.

---

## 2. Hardware

| Item | Detail |
|---|---|
| MCU | ESP32 (any variant; `ESP.getChipModel()` reported) |
| Relays | Up to 16, GPIOs from the allow-list |
| Allowed relay GPIOs | `1, 2, 3, 4, 5, 13, 14, 15, 16, 17, 18, 19, 23, 25, 26, 27, 32, 33` |
| Default relay GPIO map | `23, 32, 33, 25, 26, 27, 14, 13, 1, 3, 19, 18, 5, 4, 2, 15` |
| Reserved GPIOs | `0` (BOOT button), `21` (I2C SDA), `22` (I2C SCL) |
| RTC | DS3231 on I2C (`SDA=21`, `SCL=22`, 400 kHz) — optional |
| BOOT button | GPIO 0, input-pullup; hold 5 s → factory reset |

> **Note:** GPIO1/3 are UART TX/RX. They are in the allow-list (the build has serial debug **off**), but avoid them if you attach a serial monitor.

Relay polarity is per-relay (`activeLow[i]`). The physical pin level is `HIGH` when `state XOR activeLow` is true — i.e. with `activeLow=true`, an ON relay drives the pin `LOW`.

Unassigned relays (`pin = -1`) are not driven and report `driven=false` / `level=-1`.

---

## 3. Build & flash

- Arduino-ESP32 core, board "ESP32 Dev Module" (or equivalent).
- **Core 3.x is strongly recommended** — with a 32-bit `time_t` the clock cannot represent dates after **2038-01-19**. The build detects this at compile time and emits a `[[deprecated]]` warning; `timeT64` is reported in `/api/system`.
- Partition scheme must include a LittleFS partition large enough for config files (a handful of KB).
- Flash the sketch; the web UI is built in, so no filesystem upload is needed. LittleFS is formatted automatically on first boot if mounting fails.

Libraries used: `WiFi`, `WiFiUdp`, `WebServer`, `DNSServer`, `ESPmDNS`, `LittleFS`, `Wire`, `ArduinoJson` (v7 API), `RTClib`. Plus ESP-IDF/lwIP headers for the custom DNS resolver and the non-blocking TCP client used for time-zone lookup.

---

## 4. Configuration storage

Files live in `/cfg` on LittleFS:

| File | Contents |
|---|---|
| `/cfg/net.json` | Hostname, station config, AP config |
| `/cfg/time.json` | NTP servers, intervals, RTC/browser flags, TZ mode, offsets, DST rules, POSIX string, last auto-TZ result |
| `/cfg/gpio.json` | Per-relay pin and polarity |
| `/cfg/relay_00.json` … `/cfg/relay_15.json` | Per-relay name, mode, and the 8 schedule pairs |

### Durability model

- **Atomic writes** (`saveJsonAtomic`): serialize to `<file>.tmp` through a CRC32-computing `Print`, flush/close, re-read and verify size+CRC, then rename old → `.bak` and tmp → final.
- **Recovery** (`loadJsonAtomic`): try the main file; if it fails, try `.tmp` (promote it); if that fails, try `.bak` (and mark for rewrite). Invalid content falls back to defaults per-file.
- **Debounced writes:** mutations call `markDirty(bits)`; `storageTick()` flushes 1.5 s after the last change. Explicit saves occur before reboot, backup, and factory reset.
- Config JSON files are size-capped at 16 KB and must be JSON objects.

---

## 5. Web UI

### Pages

| Path | Purpose |
|---|---|
| `/` | Dashboard: time, WiFi STA, AP, system cards + relay tiles with quick mode buttons |
| `/relays` | Per-relay name (double-tap to rename), pin/polarity summary, mode, and the 8-pair schedule editor with presets |
| `/gpio` | Pin/polarity assignment table, Force ON/OFF, Auto, Remove, All ON/OFF/Auto |
| `/time` | Time status, actions (NTP sync, browser sync, zone detect/apply, RTC read/write), full time settings form |
| `/sta` | Station status + settings (SSID scan, static IP, hostname) |
| `/ap` | AP status + settings (SSID/pass/channel/max/TX/IP/mode/captive) |
| `/system` | Device, memory, storage, time, network, maintenance (backup/reboot/factory) |

The stylesheet and script are served with `Cache-Control: max-age=86400`; the `?v=5` query string forces a refresh when `ASSET_VERSION` changes. HTML pages are `no-cache`.

### Behaviour notes

- The UI keeps a device-time anchor (`tm`) refreshed every 10 s and renders the header clock locally each second, so it stays accurate between polls without hammering the device.
- If the device clock is invalid and `autoBrowser` is enabled, **any open page** will push the browser's current time once (`autoSynced` guard).
- All mutating calls use `POST` with JSON bodies; errors come back as `{"ok":false,"err":"..."}` and are surfaced as toasts.
- Mobile-friendly CSS: the GPIO and schedule tables collapse into stacked cards under 700 px; inputs get 16 px font on coarse pointers to prevent iOS zoom.

---

## 6. HTTP API

All responses are `application/json` unless noted. Errors use HTTP status codes with `{"ok":false,"err":msg}`. Bodies must be JSON objects (`Content-Type` not strictly enforced, body read from `plain`).

### Status & time

| Method | Path | Description |
|---|---|---|
| GET | `/api/status` | Full status: firmware, uptime, heap, clock validity, time/STA/AP summaries, relay summary array |
| GET | `/api/time` | Clock snapshot: epoch `e`, ms `ms`, UTC offset `o`, valid `v`, source `src`, last-set epoch, TZ rule `tz`, TZ source `tzs`, auto-TZ info, NTP status object, RTC object |
| GET | `/api/time/cfg` | Time configuration (see §7) plus `tzNow` |
| POST | `/api/time/cfg` | Update time settings (validated; see §7). Re-applies TZ, restarts NTP if servers/interval changed, reprobes RTC if toggled. Returns updated config |
| POST | `/api/time/sync` | Trigger an NTP sync now. `409` if STA not connected, `400` if no servers |
| POST | `/api/time/set` | Set clock from `{"epoch":<unix seconds>,"ms":0..999}` (range `MIN_VALID_EPOCH .. 4102444800`). Source becomes `browser`; schedules recomputed; RTC write scheduled |
| POST | `/api/time/tz` | Apply `{"iana":"Europe/Paris","offset":<seconds east of UTC>}`. Uses table if found, else fixed offset. **Forces manual TZ mode.** Returns `{ok,found,posix,iana}` |
| POST | `/api/time/tzdetect` | Request an automatic zone lookup via ip-api.com. `409` if STA not connected |
| POST | `/api/time/rtcread` | Load the clock from the DS3231 (aligned-set procedure). `409` if RTC unusable |
| POST | `/api/time/rtcwrite` | Write the current system time to the DS3231. `409` if RTC absent or clock invalid |

### Relays

| Method | Path | Description |
|---|---|---|
| GET | `/api/relays` | Summary array of 16 relays: `n` name, `m` mode, `s` state, `p` pin, `al` polarity, `lv` physical level, `act`/`en` pair bitmasks |
| GET | `/api/relay/{i}` | Full relay: name, mode, live state/pin/polarity, `s[]` pairs with `en`,`a`,`b`,`act` |
| POST | `/api/relay/{i}` | Partial update: `name` (≤24 chars), `mode` (0/1/2), and/or `s[]` array of pair objects `{en,a,b}` (≤8). Pair strings are validated; enabled pairs with neither START nor STOP are rejected. Recomputes active state and applies outputs |
| POST | `/api/relay/{i}/mode` | Set mode only `{"mode":0|1|2}` |

Pair validation messages use 1-based indices: e.g. `Pair 3 START: hour: out of range`.

### GPIO

| Method | Path | Description |
|---|---|---|
| GET | `/api/gpio` | `pins[]` allowed list (excluding reserved), `serialDebug`, relay summary |
| POST | `/api/gpio/{i}` | Assign pin/polarity `{"pin":n|-1,"al":bool}`. Rejects disallowed, reserved, or already-used pins (409 with the other relay's name). Applies immediately and saves |
| POST | `/api/gpio/{i}/remove` | Unassign (`pin=-1`) |
| POST | `/api/gpio/{i}/level` | Force logical level `{"level":0|1}` — sets mode to 1 (ON) or 2 (OFF). `409` if pin unassigned or not drivable |
| POST | `/api/gpio/all` | `{"auto":true}` → all relays to Auto; `{"level":0|1}` → all assigned relays to manual OFF/ON |

### Network

| Method | Path | Description |
|---|---|---|
| GET | `/api/net` | Network config (passwords blanked, `passSet` flags present) |
| POST | `/api/net/sta` | Update station + hostname. Fields: `host`, `en`, `ssid`, `pass`, `dhcp`, `ip`, `gw`, `mask`, `dns1`, `dns2`, `connect`. Validates hostname format, SSID/password lengths (WPA min 8), static completeness. Schedules apply after ~600 ms; may restart STA |
| POST | `/api/net/ap` | Update AP. Fields: `en`, `ssid`, `pass`, `ch` 1–13, `hidden`, `max` 1–8, `tx` (allowed value), `ip`, `captive`, `mode` 0/1. Schedules apply |
| GET | `/api/net/scan` | Async scan: `{"st":"running"|"started"|"busy"|"done"}`; on `done` returns up to 40 networks sorted as found (`ssid`,`rssi`,`ch`,`enc`,`bssid`) |

> When the station is not configured at all, the AP is forced on (`forced:true` in the AP POST response).

### System

| Method | Path | Description |
|---|---|---|
| GET | `/api/system` | Chip/SDK/flash/heap/PSRAM/reset reason, filesystem usage + config file listing, clock range (`timeT64`), schedule recalc counter, full time/STA/AP objects |
| POST | `/api/system/reboot` | Save pending changes, restart after ~600 ms |
| POST | `/api/system/factory` | Erase all config files (incl. `.tmp`/`.bak`), restart after ~700 ms |
| GET | `/api/backup` | Download `esp32-relay-backup.json` (streamed, chunked) with `net` (secrets included), `time`, `gpio`, `relays` |

### Captive portal

When the AP is active, captive portal is enabled, and the request arrives on the AP interface, unknown non-`/api/` hosts are answered with a `302` to the AP root. OS probes are answered appropriately:

- `/generate_204`, `/gen_204` → `204`
- `/hotspot-detect.html`, `/library/test/success.html` → `200` "Success"
- `/connecttest.txt` → `200` "Microsoft Connect Test"
- `/ncsi.txt` → `200` "Microsoft NCSI"
- `/success.txt` → `200` "success"
- Any other unknown path → `302` to `/`

`/api/*` paths are never redirected.

---

## 7. Time subsystem

### 7.1 Sources

| Source | Token | When set |
|---|---|---|
| RTC | `rtc` | At boot if DS3231 usable; also via `rtcread` (aligned to the RTC second rollover) |
| NTP | `ntp` | After a successful NTP exchange |
| Browser | `browser` | `/api/time/set`, manual or auto |

`setSystemTime()` uses `settimeofday` (µs precision), records source and epoch, requests full schedule recalc, and — except for RTC-sourced sets — schedules an RTC write (at the next µs < 40 ms boundary, or after 1.5 s).

The **`rtcread` alignment procedure**: capture the RTC epoch, then poll until the RTC second changes (or 1.3 s), then set the system clock to that value + 1 ms. This removes the up-to-1 s offset from reading mid-second.

### 7.2 RTC (DS3231)

- Probed at boot and every 30 s if enabled but absent.
- `lostPower()` (oscillator stopped / battery) invalidates the RTC time.
- Polled once per second for drift display (`rtcEpoch - system time`); temperature every 10 s.
- `rtcUsable()` = enabled ∧ present ∧ !lostPower ∧ valid epoch.
- After every NTP/browser set, the RTC is written. After a STA disconnect, `rtcRequestAlignSet()` re-disciplines the system clock from the RTC.

### 7.3 NTP

- Up to 4 servers; empty entries skipped. Round-robin optional (`rotate`), otherwise always start at #1 and fall through.
- DNS resolution via a **non-blocking lwIP callback API** (`dns_gethostbyname` + `LOCK_TCPIP_CORE`), so the loop never blocks on DNS.
- UDP request: LI=0, VN=4, mode=3; transmit timestamp used as originate nonce for reply matching. Replies validated for mode (4 or 5), stratum ≠ 0 (kiss-of-death), LI ≠ 3, and plausible time (≥ `MIN_VALID_EPOCH`). RTT clamped to 0–10 s. Offset = `((T2−T1)+(T3−T4))/2`.
- **Backoff:** after a full failed cycle, delay doubles (20 s → 600 s cap). Success resets it and schedules the next sync at `interval` minutes (1–1440).
- Triggered on every STA connect.
- Timeout per server configurable 500–10000 ms (default 2500 ms).

### 7.4 Time zone

`applyTimezone()` chooses the active POSIX rule, in order:

1. **Auto mode (`tzMode=1`) with a valid result:** use the detected POSIX rule (`acPosix`), else a fixed offset (`acOffset`).
2. **Auto mode without a result yet:** fall back to the manual rule (source `auto-pending`).
3. **Manual mode with `posix` set:** use it verbatim (source `posix`).
4. **Manual mode otherwise:** build from `gmtOffset`/`dst`/`dstOffset`/DST boundary fields (`manual`).

Rules are installed with `setenv("TZ", …)` + `tzset()`, and a full schedule recalc is requested on change.

**Auto detection** (`ip-api.com/json/?fields=status,message,timezone,offset`):

- Non-blocking TCP socket (`lwip_socket` / `O_NONBLOCK` / `lwip_select`), ~1 KB response buffer, strict HTTP status check and JSON parse.
- A built-in IANA→POSIX table (`TZ_TABLE`, PROGMEM, hundreds of zones incl. US, CA, MX, SA, EU, RU, ME, Africa, Asia, Oceania) is consulted; unknown zones fall back to a fixed offset and are retried **hourly** (rule-backed results refresh every 6 h; failures retry after 2 min).
- The last result is persisted (`acName`, `acPosix`, `acOffset`, `acValid`) and shown in the UI.
- Auto detection runs only when the station has an IP and no NTP exchange is in flight; it shares the single lwIP DNS slot via an owner tag (`DNS_NTP`/`DNS_TZ`).

### 7.5 DST rules (manual mode)

Start: month 1–12, week 1–5 (`5` = last), weekday 0–6, hour 0–24. End likewise. The generated POSIX rule uses `STD<off>DST<off>,M.m.w/h,M.m.w/h`. Presets for US/Canada, EU, UK, AU-SE, NZ are offered in the UI.

### 7.6 POSIX string validation

Max 64 printable ASCII chars; must start with a letter or `<` and contain at least one digit (e.g. `EST5EDT,M3.2.0,M11.1.0`). This is a sanity check only — the actual rule is interpreted by the C library.

---

## 8. Schedule engine

### 8.1 Specification format

Fields: **`sec min hour dow dom month year`** (all optional — missing trailing fields default to `*`; a completely empty string means "never").

| Field | Range | Names |
|---|---|---|
| sec | 0–59 | — |
| min | 0–59 | — |
| hour | 0–23 | — |
| dow | 0–7 (7 ≡ 0 = Sunday) | `sun`…`sat` |
| dom | 1–31 | — |
| month | 1–12 | `jan`…`dec` |
| year | 1970–2199 or `*` | — |

Each field accepts `*`, a value, a list, a range, and a step (`/n`): `1,3,5`, `1-5`, `*/10`, `8-17/2`, `mon-fri`, `jan,jul`. The spec is compiled into bitmasks (64-bit for sec/min, 32-bit for hour/dow/dom/month) plus a year range.

All fields are ANDed — including **dow AND dom** (unlike Vixie cron which ORs them).

Errors are reported per field, e.g. `hour: out of range`, `year: out of range (1970-2199)`, `too many fields (max 7)`, `spec too long` (max 56 chars).

### 8.2 Pair semantics

A pair is **active** when `lastMatch(START) ≥ lastMatch(STOP)` (both evaluated in local time, looking back up to `DAY_LOOKBACK = 366·8+2` days). This gives natural behaviour for:

- Overnight windows (START 22:00, STOP 06:00).
- Leap-day and long-interval schedules (e.g. START `Feb 29`, STOP `Feb 30`-style boundaries).
- Catch-up after reboot, NTP jumps, and DST transitions.

A relay in Auto mode is ON if **any** enabled pair is active. Disabled pairs are inactive. An enabled pair with neither START nor STOP is rejected at save time.

### 8.3 Evaluation

- `schedTick()` runs every loop; it returns immediately if the system time hasn't changed since the last evaluation.
- On invalid time or a time jump >120 s / backwards, it does a **full recompute** for all relays (`findPrevMatch` per pair).
- Otherwise it **walks each elapsed second** with `localtime_r`; when the local pseudo-epoch is contiguous with the last one it does a cheap per-second match (`evalSecond`) that flips `active` when a START or STOP spec matches exactly; otherwise it recomputes that relay's pairs.
- `applyOutputs()` then drives pins only when the desired state differs (or `force`), preserving the physical level per polarity.
- The recalc counter is exposed as `recalcs` in `/api/system`.

---

## 9. Network subsystem

### 9.1 Modes

The effective mode is derived each time from config and state:

- STA wanted = `sta.enabled && ssid ≠ ""`.
- AP wanted = forced on if no STA is wanted; else `ap.enabled` and (mode 0 = always, or mode 1 = while STA is down or within 60 s of connecting).
- Mode = `WIFI_STA`, `WIFI_AP`, or `WIFI_AP_STA` accordingly.

WiFi runs with `persistent(false)`, `autoReconnect(false)`, `sleep(false)`, and the configured hostname. TX power is set radio-wide from the AP setting.

### 9.2 Station

- `staBegin()` applies DHCP (all-zero config → DHCP) or static IP/gw/mask/DNS, then `WiFi.begin`. A 20 s attempt watchdog forces a reconnect cycle.
- Exponential backoff between attempts: 2 s → 30 s (doubling).
- Every successful connect triggers an NTP sync, an auto-TZ detection (if in auto mode), mDNS start, and records the connect epoch.
- Disconnect reasons are decoded from the WiFi event reason code (`wifiReasonStr`).

### 9.3 Access point

- `WiFi.softAP(ssid, pass|null, channel, hidden, maxConn)` with a fallback SSID `ESP32-Relay-XXXX` if creation fails.
- `softAPConfig(ip, ip, 255.255.255.0)` using the configured AP IP (default `192.168.4.1`).
- Captive portal: `DNSServer` with `"*"` wildcard, TTL 0, `NoError` reply code, started only when `ap.captive` and the AP is active.

### 9.4 Configuration application

Changes are **debounced 600 ms** and applied in `wifiTick()`:

- `1` → stop STA, schedule restart after 400 ms, then `staBegin()` if still wanted.
- `2` → stop/start AP if the wanted state changed.
- `4` → re-apply hostname and restart mDNS.
- If nothing is wanted (no STA, no AP), the AP is turned on as a safety net.

Events are queued from the WiFi event callback (8-slot ring buffer with UP/DOWN and reason codes) and processed in the main loop; a 1 s reconciliation also compares driver state (`WL_CONNECTED` + non-zero IP) against the tracked state to catch missed events.

### 9.5 mDNS

`MDNS.begin(hostname)` + service `_http._tcp` on port 80. Restarted on hostname change; started in AP-only mode too.

---

## 10. mDNS / hostname

Hostname rules: 1–32 chars, alphanumerics and hyphens, cannot start/end with a hyphen. The UI shows `<host>.local`. The captive portal whitelist also accepts the hostname and `<host>.local` so it doesn't redirect mDNS-style requests made by IP-like host headers.

---

## 11. Factory reset

Triggered by either:

- **BOOT button** held for 5 s (`FACTORY_HOLD_MS`) — the UI shows the live held time; release before 5 s cancels.
- **`POST /api/system/factory`**.

Behaviour: cancel pending writes (`dirtyMask = 0`), delete all config files plus `.tmp`/`.bak` variants, remove the `/cfg` directory, restart after 700 ms. On next boot, defaults are generated (including a MAC-derived AP SSID).

Default AP: SSID `ESP32-Relay-XXYY` (last MAC octets), password `esp32relay`, channel 1, IP `192.168.4.1`, captive portal on. Station disabled.

---

## 12. Defaults reference

### Network (`defaultNetCfg`)

| Field | Default |
|---|---|
| hostname | `esp32-relay` |
| STA | disabled, DHCP, empty SSID/pass |
| AP | enabled, `ESP32-Relay-XXYY` / `esp32relay`, ch 1, not hidden, max 4 clients, TX `78`, IP `192.168.4.1`, captive on, mode 0 |

### Time (`defaultTimeCfg`)

| Field | Default |
|---|---|
| NTP | `pool.ntp.org`, `time.google.com`, `time.cloudflare.com`, `time.nist.gov` |
| interval / timeout | 60 min / 2500 ms |
| rotate | false |
| RTC / autoBrowser | enabled / enabled |
| TZ mode | 0 (manual), offset 0, DST off, DST offset 3600 s |
| DST rules | start M3.2.0 02:00, end M11.1.0 02:00 |

### GPIO (`defaultGpioCfg`)

Default pins listed in §2, all **active-LOW**.

### Relays (`defaultRelay`)

Name `Relay N`, mode 0 (Auto), no enabled pairs.

---

## 13. Error handling & resilience summary

| Area | Strategy |
|---|---|
| Config files | Atomic write + CRC + `.tmp`/`.bak` recovery; per-file fallback to defaults |
| DNS | Non-blocking lwIP callback, 6 s timeout, single-owner slot shared by NTP and TZ |
| NTP | Per-server rotation/backoff up to 10 min, reply validation, kiss-of-death detection |
| TZ lookup | Non-blocking TCP with explicit timeouts per phase; fixed-offset fallback; hourly retry |
| WiFi | Event queue + periodic reconciliation, 20 s attempt watchdog, 2–30 s backoff |
| Time jumps | Full schedule recompute; per-second contiguous walk otherwise |
| RTC | Probe retry every 30 s; lost-power detection; aligned read-set; post-disconnect discipline |
| RTC writes | Deferred to sub-40 ms boundary to avoid tearing mid-second |

---

## 14. Constants & limits (quick reference)

| Constant | Value |
|---|---|
| `NUM_RELAYS` / `NUM_PAIRS` | 16 / 8 |
| `NTP_SERVERS` | 4 |
| `SPEC_MAX` | 56 chars |
| `RELAY_NAME_MAX` | 24 chars |
| `DAY_LOOKBACK` | 366·8 + 2 days |
| `MIN_VALID_EPOCH` | 2020-01-01 00:00:00 UTC |
| `FACTORY_HOLD_MS` | 5000 |
| Config JSON size cap | 16 KB |
| Debounce before save | 1.5 s |
| `jbuf` (non-chunked JSON) | 4096 B — larger responses are sent chunked |
| Scan results cap | 40 |
| Backup filename | `esp32-relay-backup.json` |

---

## 15. Known limitations

- **32-bit `time_t`** (core < 3.x) cannot represent dates after **2038-01-19**; year specs up to 2199 are accepted but cannot be honoured on such builds. Use core 3.x.
- The AP channel follows the station channel while connected (single radio) — the configured AP channel applies only in AP-only mode.
- Auto-TZ uses a third-party service (`ip-api.com`); availability and rate limits are outside our control. The built-in IANA table is a snapshot and may lag recent zone changes.
- Only one lwIP DNS lookup is in flight at a time (shared slot), so NTP and TZ requests serialize.
- `GPIO1`/`GPIO3` are allowed but are the UART pins; serial debug is off in this build.
- The backup endpoint includes WiFi passwords in plaintext — handle the downloaded file accordingly.
