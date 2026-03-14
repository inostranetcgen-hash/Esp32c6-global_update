#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Update.h>
#include <FS.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

// ============================================================
// RLS-C6 v3.0
// Мини-РЛС: Radar + Servo + Battery + Rules + Zones + Files + System
// ------------------------------------------------------------
// Идея архитектуры:
//   - ядро прошивки остается в этом .ino;
//   - интерфейс, правила, зоны и профили хранятся в LittleFS;
//   - после первой прошивки большую часть логики можно менять через телефон.
// ============================================================

static const char* APP_VERSION = "RLS-C6 v3.0 (editable profiles + rules + zones + file manager + OTA)";
static const char* AP_SSID = "RLS-ESP32C6";
static const char* AP_PASS = "12345678";

WebServer server(80);
Preferences prefs;

// -------------------- ПИНЫ (совместимы с v2) --------------------
static const int SERVO_PIN     = 18;
static const int SERVO_PWR_PIN = -1;   // MOSFET на питание сервы, если появится
static const int RADAR_OUT_PIN = 0;    // цифровой OUT текущего радара
static const int RADAR_PWR_PIN = -1;   // ключ питания радара, если появится
static const int BAT_ADC_PIN   = -1;   // ADC батареи через делитель, если появится
static const float BAT_DIVIDER = 2.0f;

// -------------------- UTILS --------------------
static inline int clampi(int v, int lo, int hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }
static inline float clampf(float v, float lo, float hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }
static inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }
static inline float smoothstep(float x) { x = clampf(x, 0.0f, 1.0f); return x * x * (3.0f - 2.0f * x); }
static inline float cosineEase(float x) { x = clampf(x, 0.0f, 1.0f); return 0.5f - 0.5f * cosf((float)PI * x); }

static String jsonEscape(const String& s) {
  String out;
  out.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '\\') out += "\\\\";
    else if (c == '"') out += "\\\"";
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else out += c;
  }
  return out;
}

static const char* contentTypeFromPath(const String& path) {
  if (path.endsWith(".html")) return "text/html; charset=utf-8";
  if (path.endsWith(".css"))  return "text/css; charset=utf-8";
  if (path.endsWith(".js"))   return "application/javascript; charset=utf-8";
  if (path.endsWith(".json")) return "application/json; charset=utf-8";
  if (path.endsWith(".txt"))  return "text/plain; charset=utf-8";
  if (path.endsWith(".svg"))  return "image/svg+xml";
  if (path.endsWith(".png"))  return "image/png";
  if (path.endsWith(".jpg") || path.endsWith(".jpeg")) return "image/jpeg";
  return "application/octet-stream";
}

static String normPath(String p) {
  p.trim();
  if (p.length() == 0) return "/";
  if (!p.startsWith("/")) p = "/" + p;
  return p;
}

static bool isSafeFsPath(const String& p) {
  if (!p.startsWith("/")) return false;
  if (p.indexOf("..") >= 0) return false;
  if (p.length() > 200) return false;
  return true;
}

// -------------------- МОДЕЛИ --------------------
struct ServoTablePoint {
  float deg = 0;
  int us = 1500;
};

static const uint8_t SERVO_TABLE_MAX = 16;

struct Config {
  // Базовые настройки v2
  int center_us = 1500;
  int left_span_us = 100;
  int right_span_us = 100;
  int bias_right_us = 0;
  int bias_left_us = 0;
  int scan_period_ms = 9000;
  int tick_ms = 120;
  int ease_pct = 35;
  int end_hold_ms = 0;
  bool servo_hold_when_stop = false;
  bool failsafe_stop = false;
  int failsafe_sec = 60;
  bool radar_enabled_default = false;

  // Безопасность (опционально)
  bool auth_enabled = false;
  String auth_user = "admin";
  String auth_pass = "admin";
  String write_token = "";

  // Фразы для озвучки через телефон (Web Speech API)
  String phrase_motion_start = "Обнаружено движение";
  String phrase_motion_clear = "Пространство чисто";
  String phrase_radar_on = "Радар включен";
  String phrase_radar_off = "Радар выключен";
  String phrase_scan_start = "Сканирование запущено";
  String phrase_scan_stop = "Сканирование остановлено";
  String phrase_servo_center = "Центр";

  // Новые принципы сервы
  bool soft_edge_brake = true;
  bool backlash_comp = false;
  bool asym_correction = true;
  bool auto_center_before_scan = false;
  bool table_calibration = false;

  int soft_edge_pct = 60;
  int backlash_us = 0;
  int backlash_settle_ms = 120;
  int auto_center_ms = 600;
  int left_scale_pct = 100;
  int right_scale_pct = 100;

  uint8_t table_len = 0;
  ServoTablePoint table[SERVO_TABLE_MAX];
};

struct Hit {
  float angle_deg = 0;
  float range_cm = 0;
  float v_kmh = 0;
  int dir = 0;          // +1 к нам, -1 от нас, 0 неизвестно
  uint32_t ts = 0;      // millis
};

struct Zone {
  String id;
  bool enabled = true;
  float min_deg = 0;
  float max_deg = 180;
  float min_cm = 0;
  float max_cm = 9999;
  String severity = "info";
};

struct RuleAction {
  String type;          // speak, log, scan, radar, pwm, profile
  String text;          // speak/log/profile name
  int value = 0;        // numeric arg for scan/radar/pwm
};

struct Rule {
  String id;
  bool enabled = true;
  String event;         // motion_start, motion_clear
  String zone;
  bool use_speed_gt = false;
  float speed_gt = 0;
  bool use_speed_lt = false;
  float speed_lt = 0;
  int dir = 0;          // 0 ignore, +1 / -1
  uint32_t cooldown_ms = 0;
  uint32_t last_fire = 0;
  RuleAction actions[8];
  int actions_count = 0;
};

// -------------------- ГЛОБАЛЬНОЕ СОСТОЯНИЕ --------------------
static Config cfg;

static const int MAX_HITS = 32;
static Hit hits[MAX_HITS];
static int hit_head = 0;

static const int MAX_ZONES = 16;
static Zone zones[MAX_ZONES];
static int zones_count = 0;

static const int MAX_RULES = 24;
static Rule rules[MAX_RULES];
static int rules_count = 0;

static bool servo_scan_on = false;
static bool servo_pwm_on = false;
static bool radar_on = false;
static bool radar_powered = false;

static uint32_t last_client_ms = 0;
static uint32_t speak_seq = 0;
static String speak_text = "";

static float bat_v = 0.0f;
static int bat_pct = -1;
static uint32_t last_bat_ms = 0;

static bool motion_now = false;
static bool motion_prev = false;
static uint32_t last_motion_ms = 0;
static bool said_clear = false;

static String active_profile = "default";
static uint32_t cfg_revision = 0;

static int servo_us_cmd = 1500;
static char servo_dir = 'R';
static uint32_t scan_start_ms = 0;
static uint32_t next_pulse_ms = 0;

// runtime support for advanced servo modes
static uint32_t autocenter_until_ms = 0;
static uint32_t backlash_until_ms = 0;
static char last_dir = 'R';

// -------------------- PATHS --------------------
static String profileRoot() { return String("/profiles/") + active_profile; }
static String configPath()  { return profileRoot() + "/config.json"; }
static String rulesPath()   { return profileRoot() + "/rules.json"; }
static String zonesPath()   { return profileRoot() + "/zones.json"; }

// -------------------- ЛОГ --------------------
static void appendLog(const String& line) {
  if (!LittleFS.exists("/logs")) LittleFS.mkdir("/logs");

  // Простая защита от бесконечного роста
  if (LittleFS.exists("/logs/log.txt")) {
    File oldf = LittleFS.open("/logs/log.txt", FILE_READ);
    if (oldf && oldf.size() > 65536) {
      oldf.close();
      LittleFS.remove("/logs/log.txt");
    } else if (oldf) {
      oldf.close();
    }
  }

  File f = LittleFS.open("/logs/log.txt", FILE_APPEND);
  if (!f) return;
  f.print(millis());
  f.print(" ");
  f.println(line);
  f.close();
}

// -------------------- SPEAK --------------------
static void queueSpeak(const String& text) {
  speak_seq++;
  speak_text = text;
}

// -------------------- HITS --------------------
static void addHit(float angle, float range_cm, float v_kmh, int dir) {
  hits[hit_head] = {angle, range_cm, v_kmh, dir, (uint32_t)millis()};
  hit_head = (hit_head + 1) % MAX_HITS;
}

// -------------------- POWER --------------------
static void servoPower(bool on) {
  if (SERVO_PWR_PIN >= 0) {
    pinMode(SERVO_PWR_PIN, OUTPUT);
    digitalWrite(SERVO_PWR_PIN, on ? HIGH : LOW);
  }
}

static void radarPower(bool on) {
  if (RADAR_PWR_PIN >= 0) {
    pinMode(RADAR_PWR_PIN, OUTPUT);
    digitalWrite(RADAR_PWR_PIN, on ? HIGH : LOW);
  }
  radar_powered = on;
}

// -------------------- BATTERY --------------------
static int batteryPercentFromV(float v) {
  if (v <= 3.20f) return 0;
  if (v >= 4.20f) return 100;
  struct Pt { float v; int p; };
  static const Pt pts[] = {
    {3.20f,0}, {3.30f,3}, {3.40f,8}, {3.50f,15}, {3.60f,25},
    {3.70f,40}, {3.80f,55}, {3.90f,70}, {4.00f,85}, {4.10f,95}, {4.20f,100}
  };
  for (size_t i = 1; i < sizeof(pts) / sizeof(pts[0]); i++) {
    if (v <= pts[i].v) {
      float t = (v - pts[i - 1].v) / (pts[i].v - pts[i - 1].v);
      return (int)roundf(pts[i - 1].p + (pts[i].p - pts[i - 1].p) * t);
    }
  }
  return 100;
}

static void batteryService() {
  if (BAT_ADC_PIN < 0) {
    bat_pct = -1;
    return;
  }

  uint32_t now = millis();
  if (now - last_bat_ms < 2000) return;
  last_bat_ms = now;

  int raw = analogRead(BAT_ADC_PIN);
  if (raw <= 0) {
    bat_pct = -1;
    return;
  }
  float vadc = ((float)raw / 4095.0f) * 3.3f;
  bat_v = vadc * BAT_DIVIDER;
  bat_pct = batteryPercentFromV(bat_v);
}

// -------------------- SERVO --------------------
static void servoPWM(bool on) {
  servo_pwm_on = on;
  if (!on) {
    pinMode(SERVO_PIN, OUTPUT);
    digitalWrite(SERVO_PIN, LOW);
  } else {
    next_pulse_ms = millis();
  }
}

static void servoPulseUs(int us) {
  us = clampi(us, 500, 2500);
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(us);
  digitalWrite(SERVO_PIN, LOW);
}

static void servoService50Hz() {
  if (!servo_pwm_on) return;
  uint32_t now = millis();
  if ((int32_t)(now - next_pulse_ms) >= 0) {
    servoPulseUs(servo_us_cmd);
    next_pulse_ms = now + 20;
  }
}

static int mapAngleToUsTable(float angle_deg) {
  if (cfg.table_len < 2) return cfg.center_us;
  if (angle_deg <= cfg.table[0].deg) return cfg.table[0].us;
  if (angle_deg >= cfg.table[cfg.table_len - 1].deg) return cfg.table[cfg.table_len - 1].us;

  for (uint8_t i = 1; i < cfg.table_len; i++) {
    if (angle_deg <= cfg.table[i].deg) {
      float d0 = cfg.table[i - 1].deg;
      float d1 = cfg.table[i].deg;
      int u0 = cfg.table[i - 1].us;
      int u1 = cfg.table[i].us;
      float t = (angle_deg - d0) / (d1 - d0);
      return (int)roundf(lerpf((float)u0, (float)u1, t));
    }
  }
  return cfg.center_us;
}

static float servoAngleDeg() {
  int usL = cfg.center_us - cfg.left_span_us;
  int usR = cfg.center_us + cfg.right_span_us;
  if (usR <= usL) return 90.0f;
  float pos = (float)(servo_us_cmd - usL) / (float)(usR - usL);
  pos = clampf(pos, 0.0f, 1.0f);
  return pos * 180.0f;
}

static void startScanNow() {
  servoPower(true);
  servoPWM(true);
  servo_scan_on = true;
  if (cfg.auto_center_before_scan) {
    autocenter_until_ms = millis() + (uint32_t)clampi(cfg.auto_center_ms, 100, 2000);
    servo_us_cmd = cfg.center_us;
  } else {
    autocenter_until_ms = 0;
  }
  scan_start_ms = millis() - (uint32_t)clampi(cfg.scan_period_ms, 800, 60000) / 4;
}

static void stopScanNow() {
  servo_scan_on = false;
  if (!cfg.servo_hold_when_stop) servoPWM(false);
}

static void servoScanService() {
  if (!servo_scan_on) return;

  uint32_t now = millis();

  if (cfg.auto_center_before_scan && autocenter_until_ms != 0 && (int32_t)(now - autocenter_until_ms) < 0) {
    servo_us_cmd = cfg.center_us;
    return;
  }

  int period = clampi(cfg.scan_period_ms, 800, 60000);
  int hold = clampi(cfg.end_hold_ms, 0, period / 4);
  uint32_t t = (now - scan_start_ms) % (uint32_t)period;
  uint32_t half = (uint32_t)period / 2;

  float x = 0.0f;

  auto updateDir = [&](char d) {
    if (d != last_dir) {
      if (cfg.backlash_comp && cfg.backlash_us > 0) {
        backlash_until_ms = now + (uint32_t)clampi(cfg.backlash_settle_ms, 50, 500);
      }
      last_dir = d;
    }
    servo_dir = d;
  };

  if (t < (uint32_t)hold) {
    x = 0.0f;
    updateDir('R');
  } else if (t < (uint32_t)(half - hold)) {
    uint32_t tm = t - (uint32_t)hold;
    uint32_t denom = (uint32_t)(half - 2 * hold);
    float u = denom ? (float)tm / (float)denom : 0.0f;
    x = clampf(u, 0.0f, 1.0f);
    updateDir('R');
  } else if (t < (uint32_t)(half + hold)) {
    x = 1.0f;
    updateDir('L');
  } else if (t < (uint32_t)(period - hold)) {
    uint32_t tm = t - (uint32_t)(half + hold);
    uint32_t denom = (uint32_t)(half - 2 * hold);
    float u = denom ? (float)tm / (float)denom : 0.0f;
    x = 1.0f - clampf(u, 0.0f, 1.0f);
    updateDir('L');
  } else {
    x = 0.0f;
    updateDir('R');
  }

  float mix = clampf(cfg.ease_pct / 100.0f, 0.0f, 1.0f);
  float xe = lerpf(x, smoothstep(x), mix);

  // Принцип 1: мягкое торможение у краев
  if (cfg.soft_edge_brake) {
    float k = clampf(cfg.soft_edge_pct / 100.0f, 0.0f, 1.0f);
    xe = lerpf(xe, cosineEase(x), k);
  }

  float angle = xe * 180.0f;

  int us = cfg.center_us;
  if (cfg.table_calibration && cfg.table_len >= 2) {
    // Принцип 5: табличная калибровка
    us = mapAngleToUsTable(angle);
  } else {
    int usL = cfg.center_us - cfg.left_span_us;
    int usR = cfg.center_us + cfg.right_span_us;
    if (usR <= usL) usR = usL + 1;
    us = (int)roundf(lerpf((float)usL, (float)usR, xe));
  }

  // Принцип 3: раздельная коррекция левой/правой стороны
  if (cfg.asym_correction) {
    bool rightSide = (xe >= 0.5f);
    int scale = rightSide ? cfg.right_scale_pct : cfg.left_scale_pct;
    scale = clampi(scale, 50, 150);
    int du = us - cfg.center_us;
    us = cfg.center_us + (int)roundf((float)du * ((float)scale / 100.0f));
  }

  if (servo_dir == 'R') us += cfg.bias_right_us;
  else us += cfg.bias_left_us;

  // Принцип 2: компенсация люфта
  if (cfg.backlash_comp && cfg.backlash_us > 0) {
    if (backlash_until_ms != 0 && (int32_t)(now - backlash_until_ms) < 0) {
      int sgn = (servo_dir == 'R') ? 1 : -1;
      us += sgn * cfg.backlash_us;
    }
  }

  int baseL = cfg.center_us - cfg.left_span_us;
  int baseR = cfg.center_us + cfg.right_span_us;
  int minUs = min(baseL, baseR) - 400;
  int maxUs = max(baseL, baseR) + 400;
  servo_us_cmd = clampi(us, minUs, maxUs);
}

// -------------------- RADAR --------------------
static float last_hit_speed_kmh = 0.0f;
static int last_hit_dir = 0;

static void radarService() {
  if (!radar_on) return;

  bool m = false;
  if (RADAR_OUT_PIN >= 0) {
    m = (digitalRead(RADAR_OUT_PIN) == HIGH);
  }

  motion_prev = motion_now;
  motion_now = m;

  uint32_t now = millis();

  if (motion_now) {
    last_motion_ms = now;
    // Текущее железо дает цифровой факт движения, поэтому скорость здесь 0,
    // но движок правил уже умеет работать с speed_gt/speed_lt на будущее.
    addHit(servoAngleDeg(), 120.0f, 0.0f, 0);
    last_hit_speed_kmh = 0.0f;
    last_hit_dir = 0;
    said_clear = false;
  }

  if (motion_now && !motion_prev) {
    queueSpeak(cfg.phrase_motion_start);
    appendLog("motion_start");
  }

  if (!motion_now && !said_clear && (now - last_motion_ms) > 5000) {
    queueSpeak(cfg.phrase_motion_clear);
    appendLog("motion_clear");
    said_clear = true;
  }
}

// -------------------- LITTLEFS HELPERS --------------------
static bool writeIfMissing(const char* path, const char* content) {
  if (LittleFS.exists(path)) return true;
  File f = LittleFS.open(path, FILE_WRITE);
  if (!f) return false;
  size_t w = f.print(content);
  f.close();
  return w == strlen(content);
}

static bool ensureDir(const String& path) {
  if (LittleFS.exists(path)) return true;
  return LittleFS.mkdir(path);
}

static bool ensureProfileLayout(const String& profileName) {
  if (!ensureDir("/profiles")) return false;
  if (!ensureDir(String("/profiles/") + profileName)) return false;
  return true;
}

static bool saveTextAtomic(const String& path, const String& content, int keepBackups) {
  // ротация backup
  for (int i = keepBackups; i >= 1; i--) {
    String from = path + ".bak" + String(i);
    String to   = path + ".bak" + String(i + 1);
    if (LittleFS.exists(to)) LittleFS.remove(to);
    if (LittleFS.exists(from)) LittleFS.rename(from, to);
  }

  String bak1 = path + ".bak1";
  if (LittleFS.exists(bak1)) LittleFS.remove(bak1);
  if (LittleFS.exists(path)) LittleFS.rename(path, bak1);

  String tmp = path + ".tmp";
  if (LittleFS.exists(tmp)) LittleFS.remove(tmp);

  File f = LittleFS.open(tmp, FILE_WRITE);
  if (!f) return false;
  size_t w = f.print(content);
  f.flush();
  f.close();
  if (w != content.length()) {
    LittleFS.remove(tmp);
    return false;
  }

  if (LittleFS.exists(path)) LittleFS.remove(path);
  if (!LittleFS.rename(tmp, path)) {
    LittleFS.remove(tmp);
    if (LittleFS.exists(bak1)) LittleFS.rename(bak1, path);
    return false;
  }
  return true;
}

// -------------------- DEFAULT FILES --------------------
static const char DEFAULT_STYLE[] PROGMEM = R"CSS(
:root{
  --bg:#040804;
  --card:#07160b;
  --line:#145429;
  --fg:#8fffa8;
  --mut:#78c58d;
  --btn:#0a2212;
  --warn:#ffcc66;
}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial}
.top{display:flex;gap:8px;align-items:center;padding:10px;background:#020502;position:sticky;top:0;z-index:5;flex-wrap:wrap}
.tabbtn,.btn{border:1px solid var(--line);background:var(--btn);color:var(--fg);padding:10px 12px;border-radius:14px;font-weight:650}
.btn:active,.tabbtn:active{transform:scale(.99)}
.pill{border:1px solid var(--line);padding:8px 12px;border-radius:999px;background:#051009}
.wrap{max-width:1100px;margin:0 auto;padding:12px}
.tab{display:none}
.tab.on{display:block}
.card{background:var(--card);border:1px solid var(--line);border-radius:18px;padding:14px;margin-bottom:12px}
.big{font-size:20px;font-weight:800;margin-bottom:10px}
.small{font-size:12px;color:var(--mut)}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
.col{display:flex;flex-direction:column;gap:8px}
.inp,select,textarea{width:100%;background:#021008;color:var(--fg);border:1px solid var(--line);border-radius:12px;padding:10px}
textarea{font-family:ui-monospace,Consolas,monospace}
canvas{width:100%;max-width:480px;height:auto;border:1px solid var(--line);border-radius:16px;background:#001107}
label{display:block;margin-top:10px}
.hr{height:1px;background:var(--line);margin:14px 0}
.statgrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:10px}
.code{white-space:pre-wrap;font-family:ui-monospace,Consolas,monospace;font-size:12px;color:var(--mut)}
)CSS";

static const char DEFAULT_APPJS[] PROGMEM = R"JS(
let voiceOn = true;
let lastSpeakSeq = 0;
let scanOn = false;
let radarOn = false;
let pwmOn = false;
let currentProfile = 'default';
let stateCache = null;

function $(id){ return document.getElementById(id); }

function showTab(name){
  document.querySelectorAll('.tab').forEach(t=>t.classList.remove('on'));
  $('tab-'+name).classList.add('on');
}

function say(text){
  if(!voiceOn) return;
  if(!('speechSynthesis' in window)) return;
  const u = new SpeechSynthesisUtterance(text);
  u.lang = 'ru-RU';
  speechSynthesis.cancel();
  speechSynthesis.speak(u);
}

async function api(url, opt){
  const r = await fetch(url, opt);
  if(!r.ok) throw new Error('HTTP '+r.status);
  const ct = r.headers.get('content-type') || '';
  if(ct.includes('application/json')) return await r.json();
  return await r.text();
}

function syncVoiceBtn(){
  if($('btnVoice')) $('btnVoice').textContent = 'Озвучка: ' + (voiceOn ? 'ВКЛ' : 'ВЫКЛ');
}

function toggleVoice(){
  voiceOn = !voiceOn;
  syncVoiceBtn();
  if(voiceOn) say('Озвучка включена');
}

async function toggleScan(){
  scanOn = !scanOn;
  await api('/api/cmd?scan='+(scanOn?1:0));
}
async function toggleRadar(){
  radarOn = !radarOn;
  await api('/api/cmd?radar='+(radarOn?1:0));
}
async function togglePWM(){
  pwmOn = !pwmOn;
  await api('/api/cmd?pwm='+(pwmOn?1:0));
}
async function saveCfg(){
  await api('/api/save');
  say('Сохранено');
}
async function rollbackCfg(){
  await api('/api/cmd?rollback=1');
  say('Откат выполнен');
}
async function rebootDevice(){
  await api('/api/cmd?reboot=1');
}
async function servoCenter(){ await api('/api/cmd?center=1'); }
async function servoTest(dir){ await api('/api/cmd?test='+dir); }

async function applyServo(){
  const q = new URLSearchParams({
    center:$('center').value,
    lspan:$('lspan').value,
    rspan:$('rspan').value,
    br:$('br').value,
    bl:$('bl').value,
    period:$('period').value,
    ease:$('ease').value,
    tick:$('tick').value,

    soft:$('f_soft').checked ? 1 : 0,
    bkl:$('f_backlash').checked ? 1 : 0,
    asym:$('f_asym').checked ? 1 : 0,
    autoc:$('f_autoc').checked ? 1 : 0,
    tabl:$('f_table').checked ? 1 : 0,
    softPct:$('softEdge').value,
    backUs:$('backlash').value,
    backMs:$('backlashMs').value,
    autoMs:$('autocMs').value,
    lsc:$('lscale').value,
    rsc:$('rscale').value
  });
  await api('/api/set?'+q.toString());
  say('Применено');
}

function draw(state){
  const cv = $('cv');
  const ctx = cv.getContext('2d');
  ctx.clearRect(0,0,cv.width,cv.height);

  const cx = cv.width/2;
  const cy = cv.height - 10;
  const R = Math.min(cv.width * 0.46, cv.height * 0.94);

  ctx.fillStyle = '#000';
  ctx.fillRect(0,0,cv.width,cv.height);

  ctx.strokeStyle = 'rgba(80,255,140,0.18)';
  ctx.lineWidth = 2;

  if(state.showArcs !== false){
    for(let i=1;i<=4;i++){
      ctx.beginPath();
      ctx.arc(cx,cy,R*i/4,Math.PI,2*Math.PI);
      ctx.stroke();
    }
    for(let a=0;a<=180;a+=30){
      const rad = Math.PI + (a*Math.PI/180);
      ctx.beginPath();
      ctx.moveTo(cx,cy);
      ctx.lineTo(cx+Math.cos(rad)*R, cy+Math.sin(rad)*R);
      ctx.stroke();
    }
  }

  if(!state.radar_on){
    ctx.fillStyle = '#8fffa8';
    ctx.font = '28px system-ui';
    ctx.fillText('РАДАР ВЫКЛЮЧЕН', cv.width/2 - 150, cv.height/2);
    return;
  }

  const ang = state.servo_angle_deg || 0;
  const rad = Math.PI + (ang*Math.PI/180);
  ctx.strokeStyle = '#00ff66';
  ctx.lineWidth = 4;
  ctx.beginPath();
  ctx.moveTo(cx,cy);
  ctx.lineTo(cx+Math.cos(rad)*R, cy+Math.sin(rad)*R);
  ctx.stroke();

  const hits = state.hits || [];
  for(const h of hits){
    const age = h.age_ms || 0;
    if(age > 8000) continue;
    const alpha = Math.max(0, 1 - age/8000);
    let col = 'rgba(0,255,120,'+alpha+')';
    let size = 6;
    if(h.dir > 0){ col = 'rgba(255,90,90,'+alpha+')'; size = 8 + Math.min(10, Math.abs(h.v_kmh||0)); }
    else if(h.dir < 0){ col = 'rgba(80,180,255,'+alpha+')'; size = 7 + Math.min(8, Math.abs(h.v_kmh||0)); }

    const a = Math.PI + (h.angle_deg*Math.PI/180);
    const rr = R * Math.min(1.0, (h.range_cm||120)/400.0);
    const x = cx + Math.cos(a)*rr;
    const y = cy + Math.sin(a)*rr;

    ctx.fillStyle = col;
    ctx.beginPath();
    ctx.arc(x,y,size,0,Math.PI*2);
    ctx.fill();
  }
}

async function fmList(){
  const path = $('fmPath').value || '/';
  const j = await api('/api/files?op=list&path='+encodeURIComponent(path));
  let out = '';
  for(const e of (j.entries || [])){
    out += (e.is_dir ? '[D] ' : '[F] ') + e.name + (e.size ? (' ('+e.size+')') : '') + '\\n';
  }
  $('fmList').textContent = out || '(пусто)';
}
async function fmOpen(){
  const file = $('fmFile').value;
  const t = await api('/api/files?op=read&path='+encodeURIComponent(file)+'&ct=text/plain; charset=utf-8');
  $('fmText').value = t;
}
async function fmSave(){
  const file = $('fmFile').value;
  const body = $('fmText').value;
  await api('/api/files?op=write&path='+encodeURIComponent(file), {
    method:'POST',
    headers:{'content-type':'text/plain; charset=utf-8'},
    body
  });
  say('Файл сохранен');
  await fmList().catch(()=>{});
}
async function fmDelete(){
  const file = $('fmFile').value;
  await api('/api/files?op=del&path='+encodeURIComponent(file), {method:'POST'});
  say('Удалено');
  await fmList().catch(()=>{});
}
async function fmMkdir(){
  const dir = $('fmNewDir').value;
  if(!dir) return;
  await api('/api/files?op=mkdir&path='+encodeURIComponent(dir), {method:'POST'});
  $('fmNewDir').value = '';
  say('Папка создана');
  await fmList().catch(()=>{});
}
async function fmRename(){
  const from = $('fmFile').value;
  const to = $('fmRename').value;
  if(!from || !to) return;
  await api('/api/files?op=rename&path='+encodeURIComponent(from)+'&to='+encodeURIComponent(to), {method:'POST'});
  $('fmFile').value = to;
  $('fmRename').value = '';
  say('Переименовано');
  await fmList().catch(()=>{});
}

async function rulesLoad(){
  $('rulesText').value = await api('/api/rules');
}
async function rulesSave(){
  await api('/api/rules', {
    method:'POST',
    headers:{'content-type':'application/json; charset=utf-8'},
    body:$('rulesText').value
  });
  say('Правила сохранены');
}
async function zonesLoad(){
  $('zonesText').value = await api('/api/zones');
}
async function zonesSave(){
  await api('/api/zones', {
    method:'POST',
    headers:{'content-type':'application/json; charset=utf-8'},
    body:$('zonesText').value
  });
  say('Зоны сохранены');
}
async function logsLoad(){
  $('logsText').textContent = await api('/api/logs');
}

async function profilesLoad(){
  const j = await api('/api/profiles');
  const sel = $('profileSelect');
  sel.innerHTML = '';
  for(const p of (j.profiles || [])){
    const opt = document.createElement('option');
    opt.value = p;
    opt.textContent = p;
    if(p === j.active) opt.selected = true;
    sel.appendChild(opt);
  }
  currentProfile = j.active || 'default';
  $('profileCurrent').textContent = currentProfile;
}
async function profileSwitch(){
  const p = $('profileSelect').value;
  await api('/api/cmd?profile='+encodeURIComponent(p));
  currentProfile = p;
  $('profileCurrent').textContent = currentProfile;
  window._initServo = false;
  await rulesLoad().catch(()=>{});
  await zonesLoad().catch(()=>{});
  say('Профиль переключен');
}
async function profileCreate(){
  const p = ($('profileNew').value || '').trim();
  if(!p) return;
  await api('/api/profiles', {
    method:'POST',
    headers:{'content-type':'application/json; charset=utf-8'},
    body:JSON.stringify({name:p})
  });
  $('profileNew').value = '';
  await profilesLoad();
  say('Профиль создан');
}

async function loop(){
  try{
    const state = await api('/api/state');
    stateCache = state;

    $('ip').textContent = state.ip || '192.168.4.1';
    $('versionTop').textContent = state.version || '—';
    $('profileCurrent').textContent = state.profile || 'default';
    $('cfgRev').textContent = state.rev ?? '0';

    scanOn = !!state.scan_on;
    radarOn = !!state.radar_on;
    pwmOn = !!state.pwm_on;

    $('btnScan').textContent = 'Сканирование: ' + (scanOn ? 'СТОП' : 'СТАРТ');
    $('btnRadar').textContent = 'Радар: ' + (radarOn ? 'ВЫКЛ' : 'ВКЛ');
    syncVoiceBtn();

    $('status').textContent = state.motion ? 'Движение' : 'Тихо';
    $('dbg').textContent = `angle=${(state.servo_angle_deg||0).toFixed(1)}°, us=${state.servo_us}, dir=${state.servo_dir}, profile=${state.profile}`;

    if(state.bat_pct >= 0){
      $('batTop').textContent = state.bat_pct + '%';
      $('batV').textContent = (state.bat_v || 0).toFixed(2) + 'V';
      $('batP').textContent = state.bat_pct + '%';
    }else{
      $('batTop').textContent = '—';
      $('batV').textContent = '—';
      $('batP').textContent = '—';
    }

    if(!window._initServo && state.cfg && state.cfg_ext){
      window._initServo = true;
      $('center').value = state.cfg.center_us; $('cV').textContent = $('center').value;
      $('lspan').value = state.cfg.left_span_us; $('lV').textContent = $('lspan').value;
      $('rspan').value = state.cfg.right_span_us; $('rV').textContent = $('rspan').value;
      $('br').value = state.cfg.bias_right_us; $('brV').textContent = $('br').value;
      $('bl').value = state.cfg.bias_left_us; $('blV').textContent = $('bl').value;
      $('period').value = state.cfg.scan_period_ms; $('pV').textContent = $('period').value;
      $('ease').value = state.cfg.ease_pct; $('eV').textContent = $('ease').value;
      $('tick').value = state.cfg.tick_ms; $('tV').textContent = $('tick').value;

      $('f_soft').checked = !!state.cfg_ext.soft_edge_brake;
      $('f_backlash').checked = !!state.cfg_ext.backlash_comp;
      $('f_asym').checked = !!state.cfg_ext.asym_correction;
      $('f_autoc').checked = !!state.cfg_ext.auto_center_before_scan;
      $('f_table').checked = !!state.cfg_ext.table_calibration;
      $('softEdge').value = state.cfg_ext.soft_edge_pct; $('seV').textContent = $('softEdge').value;
      $('backlash').value = state.cfg_ext.backlash_us; $('bkV').textContent = $('backlash').value;
      $('backlashMs').value = state.cfg_ext.backlash_settle_ms; $('bkmV').textContent = $('backlashMs').value;
      $('autocMs').value = state.cfg_ext.auto_center_ms; $('acV').textContent = $('autocMs').value;
      $('lscale').value = state.cfg_ext.left_scale_pct; $('lsV').textContent = $('lscale').value;
      $('rscale').value = state.cfg_ext.right_scale_pct; $('rsV').textContent = $('rscale').value;

      await rulesLoad().catch(()=>{});
      await zonesLoad().catch(()=>{});
      await logsLoad().catch(()=>{});
      await fmList().catch(()=>{});
      await profilesLoad().catch(()=>{});
    }

    if(state.speak_seq && state.speak_seq !== lastSpeakSeq){
      lastSpeakSeq = state.speak_seq;
      if(state.speak_text) say(state.speak_text);
    }

    draw(state);
    setTimeout(loop, (state.cfg && state.cfg.tick_ms) ? state.cfg.tick_ms : 150);
  }catch(e){
    setTimeout(loop, 600);
  }
}
loop();
)JS";

static const char DEFAULT_INDEX[] PROGMEM = R"HTML(
<!doctype html>
<html lang="ru">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>RLS-C6</title>
  <link rel="stylesheet" href="/style.css">
</head>
<body>
  <div class="top">
    <button class="tabbtn" onclick="showTab('radar')">Радар</button>
    <button class="tabbtn" onclick="showTab('servo')">Сервопривод</button>
    <button class="tabbtn" onclick="showTab('battery')">Батарея</button>
    <button class="tabbtn" onclick="showTab('rules')">Правила</button>
    <button class="tabbtn" onclick="showTab('zones')">Зоны</button>
    <button class="tabbtn" onclick="showTab('files')">Файлы</button>
    <button class="tabbtn" onclick="showTab('system')">Система</button>
    <div class="pill">AP: <span id="ip">—</span></div>
    <div class="pill">Профиль: <span id="profileCurrent">default</span></div>
    <div class="pill">Заряд: <span id="batTop">—</span></div>
  </div>

  <div class="wrap">
    <div id="tab-radar" class="tab on">
      <div class="card">
        <div class="big">Радар</div>
        <canvas id="cv" width="900" height="420"></canvas>
      </div>
      <div class="card">
        <div class="statgrid">
          <div><div class="small">Статус</div><div class="big" id="status">—</div></div>
          <div><div class="small">Версия</div><div class="big" id="versionTop">—</div></div>
          <div><div class="small">Ревизия конфигов</div><div class="big" id="cfgRev">0</div></div>
        </div>
        <div class="small" id="dbg">—</div>
        <div class="row" style="margin-top:12px">
          <button class="btn" id="btnScan" onclick="toggleScan()">Сканирование: СТАРТ</button>
          <button class="btn" id="btnRadar" onclick="toggleRadar()">Радар: ВКЛ</button>
          <button class="btn" id="btnVoice" onclick="toggleVoice()">Озвучка: ВКЛ</button>
        </div>
      </div>
    </div>

    <div id="tab-servo" class="tab">
      <div class="card">
        <div class="big">Сервопривод — базовые настройки</div>

        <label>Center (us): <b id="cV">1500</b></label>
        <input id="center" type="range" min="1200" max="1800" value="1500" oninput="cV.textContent=this.value">

        <label>Left span (us): <b id="lV">100</b></label>
        <input id="lspan" type="range" min="10" max="500" value="100" oninput="lV.textContent=this.value">

        <label>Right span (us): <b id="rV">100</b></label>
        <input id="rspan" type="range" min="10" max="500" value="100" oninput="rV.textContent=this.value">

        <label>Bias right (us): <b id="brV">0</b></label>
        <input id="br" type="range" min="-120" max="120" value="0" oninput="brV.textContent=this.value">

        <label>Bias left (us): <b id="blV">0</b></label>
        <input id="bl" type="range" min="-120" max="120" value="0" oninput="blV.textContent=this.value">

        <label>Scan period (ms): <b id="pV">9000</b></label>
        <input id="period" type="range" min="800" max="60000" value="9000" oninput="pV.textContent=this.value">

        <label>Ease (%): <b id="eV">35</b></label>
        <input id="ease" type="range" min="0" max="100" value="35" oninput="eV.textContent=this.value">

        <label>Tick UI (ms): <b id="tV">120</b></label>
        <input id="tick" type="range" min="80" max="1200" value="120" oninput="tV.textContent=this.value">

        <div class="row" style="margin-top:12px">
          <button class="btn" onclick="applyServo()">Применить</button>
          <button class="btn" onclick="saveCfg()">Save</button>
          <button class="btn" onclick="togglePWM()">PWM: ON/OFF</button>
          <button class="btn" onclick="servoCenter()">Center</button>
          <button class="btn" onclick="servoTest('L')">Тест Лево</button>
          <button class="btn" onclick="servoTest('R')">Тест Право</button>
        </div>

        <div class="hr"></div>
        <div class="big">Принципы использования сервопривода</div>
        <label><input type="checkbox" id="f_soft"> Сканирование с мягким торможением у краёв</label>
        <label><input type="checkbox" id="f_backlash"> Сканирование с компенсацией люфта</label>
        <label><input type="checkbox" id="f_asym"> Сканирование с раздельной коррекцией левой и правой сторон</label>
        <label><input type="checkbox" id="f_autoc"> Автоцентрирование перед запуском</label>
        <label><input type="checkbox" id="f_table"> Табличная калибровка</label>

        <label>Soft edge %: <b id="seV">60</b></label>
        <input id="softEdge" type="range" min="0" max="100" value="60" oninput="seV.textContent=this.value">

        <label>Backlash us: <b id="bkV">0</b></label>
        <input id="backlash" type="range" min="0" max="200" value="0" oninput="bkV.textContent=this.value">

        <label>Backlash settle ms: <b id="bkmV">120</b></label>
        <input id="backlashMs" type="range" min="50" max="500" value="120" oninput="bkmV.textContent=this.value">

        <label>Auto center ms: <b id="acV">600</b></label>
        <input id="autocMs" type="range" min="100" max="2000" value="600" oninput="acV.textContent=this.value">

        <label>Left scale %: <b id="lsV">100</b></label>
        <input id="lscale" type="range" min="50" max="150" value="100" oninput="lsV.textContent=this.value">

        <label>Right scale %: <b id="rsV">100</b></label>
        <input id="rscale" type="range" min="50" max="150" value="100" oninput="rsV.textContent=this.value">

        <div class="small" style="margin-top:10px">
          Табличная калибровка задается в <b>/profiles/&lt;profile&gt;/config.json</b>, поле <b>table</b>.
        </div>
      </div>
    </div>

    <div id="tab-battery" class="tab">
      <div class="card">
        <div class="big">Батарея</div>
        <div>Напряжение: <b id="batV">—</b></div>
        <div>Процент: <b id="batP">—</b></div>
        <div class="small" style="margin-top:10px">
          Чтобы вкладка ожила, подключи батарею через делитель напряжения к ADC и задай BAT_ADC_PIN в ядре.
        </div>
      </div>
    </div>

    <div id="tab-rules" class="tab">
      <div class="card">
        <div class="big">Правила / сценарии (JSON)</div>
        <textarea id="rulesText" rows="18"></textarea>
        <div class="row" style="margin-top:12px">
          <button class="btn" onclick="rulesLoad()">Load</button>
          <button class="btn" onclick="rulesSave()">Save</button>
        </div>
        <div class="small">Поддерживаются события motion_start / motion_clear, фильтр по zone, speed_gt/speed_lt, dir и действия speak/log/scan/radar/pwm/profile.</div>
      </div>
    </div>

    <div id="tab-zones" class="tab">
      <div class="card">
        <div class="big">Зоны (JSON)</div>
        <textarea id="zonesText" rows="18"></textarea>
        <div class="row" style="margin-top:12px">
          <button class="btn" onclick="zonesLoad()">Load</button>
          <button class="btn" onclick="zonesSave()">Save</button>
        </div>
        <div class="small">Зоны задаются по углу и дальности. Их можно использовать в правилах.</div>
      </div>
    </div>

    <div id="tab-files" class="tab">
      <div class="card">
        <div class="big">Файловый менеджер</div>
        <div class="row">
          <input class="inp" id="fmPath" value="/ui">
          <button class="btn" onclick="fmList()">List</button>
        </div>
        <div class="code" id="fmList">(нажми List)</div>
        <div class="row" style="margin-top:10px">
          <input class="inp" id="fmNewDir" placeholder="/profiles/newprofile">
          <button class="btn" onclick="fmMkdir()">Создать папку</button>
        </div>
      </div>

      <div class="card">
        <div class="big">Редактор</div>
        <div class="row">
          <input class="inp" id="fmFile" value="/ui/app.js">
          <button class="btn" onclick="fmOpen()">Open</button>
          <button class="btn" onclick="fmSave()">Save</button>
          <button class="btn" onclick="fmDelete()">Delete</button>
        </div>
        <div class="row" style="margin-top:10px">
          <input class="inp" id="fmRename" placeholder="новый путь, например /ui/app2.js">
          <button class="btn" onclick="fmRename()">Rename</button>
        </div>
        <textarea id="fmText" rows="18"></textarea>
        <div class="small">Редактируй UI, конфиги, правила и зоны прямо с телефона.</div>
      </div>
    </div>

    <div id="tab-system" class="tab">
      <div class="card">
        <div class="big">Профили</div>
        <div class="row">
          <select id="profileSelect"></select>
          <button class="btn" onclick="profileSwitch()">Переключить</button>
        </div>
        <div class="row" style="margin-top:10px">
          <input class="inp" id="profileNew" placeholder="имя нового профиля">
          <button class="btn" onclick="profileCreate()">Создать профиль</button>
        </div>
      </div>

      <div class="card">
        <div class="big">Система</div>
        <div class="row">
          <button class="btn" onclick="saveCfg()">Save config</button>
          <button class="btn" onclick="rollbackCfg()">Rollback</button>
          <button class="btn" onclick="rebootDevice()">Reboot</button>
        </div>
        <div class="small" style="margin-top:10px">OTA: открой <b>/update</b> и загрузи новый .bin.</div>
      </div>

      <div class="card">
        <div class="big">Логи</div>
        <button class="btn" onclick="logsLoad()">Обновить</button>
        <div class="code" id="logsText">—</div>
      </div>
    </div>
  </div>

  <script src="/app.js"></script>
</body>
</html>
)HTML";

static const char DEFAULT_CONFIG_JSON[] PROGMEM = R"JSON(
{
  "schema": 3,
  "servo": {
    "center_us": 1500,
    "left_span_us": 100,
    "right_span_us": 100,
    "bias_right_us": 0,
    "bias_left_us": 0,
    "scan_period_ms": 9000,
    "tick_ms": 120,
    "ease_pct": 35,
    "end_hold_ms": 0,
    "hold_when_stop": false
  },
  "servo_principles": {
    "soft_edge_brake": true,
    "backlash_comp": false,
    "asym_correction": true,
    "auto_center_before_scan": false,
    "table_calibration": false,
    "soft_edge_pct": 60,
    "backlash_us": 0,
    "backlash_settle_ms": 120,
    "auto_center_ms": 600,
    "left_scale_pct": 100,
    "right_scale_pct": 100
  },
  "table": [],
  "system": {
    "failsafe_stop": false,
    "failsafe_sec": 60
  },
  "radar": {
    "enabled_default": false
  },
  "security": {
    "auth_enabled": false,
    "auth_user": "admin",
    "auth_pass": "admin",
    "write_token": ""
  },
  "phrases": {
    "motion_start": "Обнаружено движение",
    "motion_clear": "Пространство чисто",
    "radar_on": "Радар включен",
    "radar_off": "Радар выключен",
    "scan_start": "Сканирование запущено",
    "scan_stop": "Сканирование остановлено",
    "servo_center": "Центр"
  }
}
)JSON";

static const char DEFAULT_ZONES_JSON[] PROGMEM = R"JSON(
{
  "schema": 1,
  "zones": [
    {"id":"left","enabled":true,"min_deg":0,"max_deg":60,"min_cm":0,"max_cm":9999,"severity":"info"},
    {"id":"center","enabled":true,"min_deg":60,"max_deg":120,"min_cm":0,"max_cm":9999,"severity":"warn"},
    {"id":"right","enabled":true,"min_deg":120,"max_deg":180,"min_cm":0,"max_cm":9999,"severity":"info"}
  ]
}
)JSON";

static const char DEFAULT_RULES_JSON[] PROGMEM = R"JSON(
{
  "schema": 1,
  "rules": [
    {
      "id":"speak_motion_start",
      "enabled":true,
      "event":"motion_start",
      "cooldown_ms":1000,
      "actions":[
        {"type":"speak","text":"Обнаружено движение"},
        {"type":"log","text":"rule:speak_motion_start"}
      ]
    },
    {
      "id":"speak_motion_clear",
      "enabled":true,
      "event":"motion_clear",
      "cooldown_ms":2000,
      "actions":[
        {"type":"speak","text":"Пространство чисто"},
        {"type":"log","text":"rule:speak_motion_clear"}
      ]
    }
  ]
}
)JSON";

// -------------------- CONFIG LOAD/SAVE --------------------
static String exportConfigPretty() {
  DynamicJsonDocument doc(12288);

  JsonObject servo = doc.createNestedObject("servo");
  servo["center_us"] = cfg.center_us;
  servo["left_span_us"] = cfg.left_span_us;
  servo["right_span_us"] = cfg.right_span_us;
  servo["bias_right_us"] = cfg.bias_right_us;
  servo["bias_left_us"] = cfg.bias_left_us;
  servo["scan_period_ms"] = cfg.scan_period_ms;
  servo["tick_ms"] = cfg.tick_ms;
  servo["ease_pct"] = cfg.ease_pct;
  servo["end_hold_ms"] = cfg.end_hold_ms;
  servo["hold_when_stop"] = cfg.servo_hold_when_stop;

  JsonObject sp = doc.createNestedObject("servo_principles");
  sp["soft_edge_brake"] = cfg.soft_edge_brake;
  sp["backlash_comp"] = cfg.backlash_comp;
  sp["asym_correction"] = cfg.asym_correction;
  sp["auto_center_before_scan"] = cfg.auto_center_before_scan;
  sp["table_calibration"] = cfg.table_calibration;
  sp["soft_edge_pct"] = cfg.soft_edge_pct;
  sp["backlash_us"] = cfg.backlash_us;
  sp["backlash_settle_ms"] = cfg.backlash_settle_ms;
  sp["auto_center_ms"] = cfg.auto_center_ms;
  sp["left_scale_pct"] = cfg.left_scale_pct;
  sp["right_scale_pct"] = cfg.right_scale_pct;

  JsonArray ta = doc.createNestedArray("table");
  for (uint8_t i = 0; i < cfg.table_len && i < SERVO_TABLE_MAX; i++) {
    JsonObject p = ta.createNestedObject();
    p["deg"] = cfg.table[i].deg;
    p["us"] = cfg.table[i].us;
  }

  JsonObject system = doc.createNestedObject("system");
  system["failsafe_stop"] = cfg.failsafe_stop;
  system["failsafe_sec"] = cfg.failsafe_sec;

  JsonObject radar = doc.createNestedObject("radar");
  radar["enabled_default"] = cfg.radar_enabled_default;

  JsonObject sec = doc.createNestedObject("security");
  sec["auth_enabled"] = cfg.auth_enabled;
  sec["auth_user"] = cfg.auth_user;
  sec["auth_pass"] = cfg.auth_pass;
  sec["write_token"] = cfg.write_token;

  JsonObject ph = doc.createNestedObject("phrases");
  ph["motion_start"] = cfg.phrase_motion_start;
  ph["motion_clear"] = cfg.phrase_motion_clear;
  ph["radar_on"] = cfg.phrase_radar_on;
  ph["radar_off"] = cfg.phrase_radar_off;
  ph["scan_start"] = cfg.phrase_scan_start;
  ph["scan_stop"] = cfg.phrase_scan_stop;
  ph["servo_center"] = cfg.phrase_servo_center;

  String out;
  serializeJsonPretty(doc, out);
  return out;
}

static void clampConfig() {
  cfg.center_us = clampi(cfg.center_us, 1200, 1800);
  cfg.left_span_us = clampi(cfg.left_span_us, 10, 500);
  cfg.right_span_us = clampi(cfg.right_span_us, 10, 500);
  cfg.bias_right_us = clampi(cfg.bias_right_us, -120, 120);
  cfg.bias_left_us = clampi(cfg.bias_left_us, -120, 120);
  cfg.scan_period_ms = clampi(cfg.scan_period_ms, 800, 60000);
  cfg.tick_ms = clampi(cfg.tick_ms, 80, 1200);
  cfg.ease_pct = clampi(cfg.ease_pct, 0, 100);
  cfg.end_hold_ms = clampi(cfg.end_hold_ms, 0, cfg.scan_period_ms / 4);
  cfg.soft_edge_pct = clampi(cfg.soft_edge_pct, 0, 100);
  cfg.backlash_us = clampi(cfg.backlash_us, 0, 200);
  cfg.backlash_settle_ms = clampi(cfg.backlash_settle_ms, 50, 500);
  cfg.auto_center_ms = clampi(cfg.auto_center_ms, 100, 2000);
  cfg.left_scale_pct = clampi(cfg.left_scale_pct, 50, 150);
  cfg.right_scale_pct = clampi(cfg.right_scale_pct, 50, 150);
  cfg.failsafe_sec = clampi(cfg.failsafe_sec, 5, 3600);
  if (cfg.table_len > SERVO_TABLE_MAX) cfg.table_len = SERVO_TABLE_MAX;
}

static bool loadConfigFromFile() {
  File f = LittleFS.open(configPath(), FILE_READ);
  if (!f) return false;

  DynamicJsonDocument doc(12288);
  auto err = deserializeJson(doc, f);
  f.close();
  if (err) return false;

  JsonObject servo = doc["servo"].as<JsonObject>();
  if (!servo.isNull()) {
    if (servo["center_us"].is<int>()) cfg.center_us = servo["center_us"].as<int>();
    if (servo["left_span_us"].is<int>()) cfg.left_span_us = servo["left_span_us"].as<int>();
    if (servo["right_span_us"].is<int>()) cfg.right_span_us = servo["right_span_us"].as<int>();
    if (servo["bias_right_us"].is<int>()) cfg.bias_right_us = servo["bias_right_us"].as<int>();
    if (servo["bias_left_us"].is<int>()) cfg.bias_left_us = servo["bias_left_us"].as<int>();
    if (servo["scan_period_ms"].is<int>()) cfg.scan_period_ms = servo["scan_period_ms"].as<int>();
    if (servo["tick_ms"].is<int>()) cfg.tick_ms = servo["tick_ms"].as<int>();
    if (servo["ease_pct"].is<int>()) cfg.ease_pct = servo["ease_pct"].as<int>();
    if (servo["end_hold_ms"].is<int>()) cfg.end_hold_ms = servo["end_hold_ms"].as<int>();
    if (servo["hold_when_stop"].is<bool>()) cfg.servo_hold_when_stop = servo["hold_when_stop"].as<bool>();
  }

  JsonObject sp = doc["servo_principles"].as<JsonObject>();
  if (!sp.isNull()) {
    if (sp["soft_edge_brake"].is<bool>()) cfg.soft_edge_brake = sp["soft_edge_brake"].as<bool>();
    if (sp["backlash_comp"].is<bool>()) cfg.backlash_comp = sp["backlash_comp"].as<bool>();
    if (sp["asym_correction"].is<bool>()) cfg.asym_correction = sp["asym_correction"].as<bool>();
    if (sp["auto_center_before_scan"].is<bool>()) cfg.auto_center_before_scan = sp["auto_center_before_scan"].as<bool>();
    if (sp["table_calibration"].is<bool>()) cfg.table_calibration = sp["table_calibration"].as<bool>();
    if (sp["soft_edge_pct"].is<int>()) cfg.soft_edge_pct = sp["soft_edge_pct"].as<int>();
    if (sp["backlash_us"].is<int>()) cfg.backlash_us = sp["backlash_us"].as<int>();
    if (sp["backlash_settle_ms"].is<int>()) cfg.backlash_settle_ms = sp["backlash_settle_ms"].as<int>();
    if (sp["auto_center_ms"].is<int>()) cfg.auto_center_ms = sp["auto_center_ms"].as<int>();
    if (sp["left_scale_pct"].is<int>()) cfg.left_scale_pct = sp["left_scale_pct"].as<int>();
    if (sp["right_scale_pct"].is<int>()) cfg.right_scale_pct = sp["right_scale_pct"].as<int>();
  }

  cfg.table_len = 0;
  if (doc["table"].is<JsonArray>()) {
    JsonArray ta = doc["table"].as<JsonArray>();
    for (JsonVariant v : ta) {
      if (cfg.table_len >= SERVO_TABLE_MAX) break;
      if (!v.is<JsonObject>()) continue;
      JsonObject p = v.as<JsonObject>();
      if (!(p["deg"].is<float>() || p["deg"].is<int>())) continue;
      if (!p["us"].is<int>()) continue;
      cfg.table[cfg.table_len].deg = p["deg"].as<float>();
      cfg.table[cfg.table_len].us = p["us"].as<int>();
      cfg.table_len++;
    }
  }

  JsonObject system = doc["system"].as<JsonObject>();
  if (!system.isNull()) {
    if (system["failsafe_stop"].is<bool>()) cfg.failsafe_stop = system["failsafe_stop"].as<bool>();
    if (system["failsafe_sec"].is<int>()) cfg.failsafe_sec = system["failsafe_sec"].as<int>();
  }

  JsonObject radar = doc["radar"].as<JsonObject>();
  if (!radar.isNull()) {
    if (radar["enabled_default"].is<bool>()) cfg.radar_enabled_default = radar["enabled_default"].as<bool>();
  }

  JsonObject sec = doc["security"].as<JsonObject>();
  if (!sec.isNull()) {
    if (sec["auth_enabled"].is<bool>()) cfg.auth_enabled = sec["auth_enabled"].as<bool>();
    if (sec["auth_user"].is<const char*>()) cfg.auth_user = String(sec["auth_user"].as<const char*>());
    if (sec["auth_pass"].is<const char*>()) cfg.auth_pass = String(sec["auth_pass"].as<const char*>());
    if (sec["write_token"].is<const char*>()) cfg.write_token = String(sec["write_token"].as<const char*>());
  }

  JsonObject ph = doc["phrases"].as<JsonObject>();
  if (!ph.isNull()) {
    if (ph["motion_start"].is<const char*>()) cfg.phrase_motion_start = String(ph["motion_start"].as<const char*>());
    if (ph["motion_clear"].is<const char*>()) cfg.phrase_motion_clear = String(ph["motion_clear"].as<const char*>());
    if (ph["radar_on"].is<const char*>()) cfg.phrase_radar_on = String(ph["radar_on"].as<const char*>());
    if (ph["radar_off"].is<const char*>()) cfg.phrase_radar_off = String(ph["radar_off"].as<const char*>());
    if (ph["scan_start"].is<const char*>()) cfg.phrase_scan_start = String(ph["scan_start"].as<const char*>());
    if (ph["scan_stop"].is<const char*>()) cfg.phrase_scan_stop = String(ph["scan_stop"].as<const char*>());
    if (ph["servo_center"].is<const char*>()) cfg.phrase_servo_center = String(ph["servo_center"].as<const char*>());
  }

  clampConfig();
  return true;
}

static bool saveConfig() {
  clampConfig();
  bool ok = saveTextAtomic(configPath(), exportConfigPretty(), 3);
  if (!ok) return false;

  // Legacy NVS mirror (совместимость с v2)
  prefs.begin("rls", false);
  prefs.putInt("center", cfg.center_us);
  prefs.putInt("lspan", cfg.left_span_us);
  prefs.putInt("rspan", cfg.right_span_us);
  prefs.putInt("br", cfg.bias_right_us);
  prefs.putInt("bl", cfg.bias_left_us);
  prefs.putInt("period", cfg.scan_period_ms);
  prefs.putInt("tick", cfg.tick_ms);
  prefs.putInt("ease", cfg.ease_pct);
  prefs.end();

  cfg_revision++;
  prefs.begin("rlsmeta", false);
  prefs.putULong("rev", cfg_revision);
  prefs.putString("profile", active_profile);
  prefs.end();

  appendLog("config_saved profile=" + active_profile);
  return true;
}

static bool rollbackConfig() {
  String bak1 = configPath() + ".bak1";
  if (!LittleFS.exists(bak1)) return false;
  String cur = configPath();
  String bak2 = cur + ".bak2";
  if (LittleFS.exists(bak2)) LittleFS.remove(bak2);
  if (LittleFS.exists(cur)) LittleFS.rename(cur, bak2);
  LittleFS.rename(bak1, cur);
  bool ok = loadConfigFromFile();
  if (ok) appendLog("config_rollback profile=" + active_profile);
  return ok;
}

// -------------------- ZONES / RULES --------------------
static bool loadZones() {
  zones_count = 0;
  File f = LittleFS.open(zonesPath(), FILE_READ);
  if (!f) return false;

  DynamicJsonDocument doc(8192);
  auto err = deserializeJson(doc, f);
  f.close();
  if (err) return false;
  if (!doc["zones"].is<JsonArray>()) return false;

  for (JsonVariant v : doc["zones"].as<JsonArray>()) {
    if (zones_count >= MAX_ZONES) break;
    if (!v.is<JsonObject>()) continue;
    JsonObject o = v.as<JsonObject>();
    Zone z;
    z.id = o["id"].is<const char*>() ? String(o["id"].as<const char*>()) : String();
    if (z.id.length() == 0) continue;
    z.enabled = o["enabled"].is<bool>() ? o["enabled"].as<bool>() : true;
    z.min_deg = (o["min_deg"].is<int>() || o["min_deg"].is<float>()) ? o["min_deg"].as<float>() : 0.0f;
    z.max_deg = (o["max_deg"].is<int>() || o["max_deg"].is<float>()) ? o["max_deg"].as<float>() : 180.0f;
    z.min_cm = (o["min_cm"].is<int>() || o["min_cm"].is<float>()) ? o["min_cm"].as<float>() : 0.0f;
    z.max_cm = (o["max_cm"].is<int>() || o["max_cm"].is<float>()) ? o["max_cm"].as<float>() : 9999.0f;
    z.severity = o["severity"].is<const char*>() ? String(o["severity"].as<const char*>()) : String("info");
    zones[zones_count++] = z;
  }
  return true;
}

static bool loadRules() {
  rules_count = 0;
  File f = LittleFS.open(rulesPath(), FILE_READ);
  if (!f) return false;

  DynamicJsonDocument doc(12288);
  auto err = deserializeJson(doc, f);
  f.close();
  if (err) return false;
  if (!doc["rules"].is<JsonArray>()) return false;

  for (JsonVariant v : doc["rules"].as<JsonArray>()) {
    if (rules_count >= MAX_RULES) break;
    if (!v.is<JsonObject>()) continue;

    JsonObject o = v.as<JsonObject>();
    Rule r;
    r.id = o["id"].is<const char*>() ? String(o["id"].as<const char*>()) : String();
    if (r.id.length() == 0) continue;

    r.enabled = o["enabled"].is<bool>() ? o["enabled"].as<bool>() : true;
    r.event = o["event"].is<const char*>() ? String(o["event"].as<const char*>()) : String();
    r.zone  = o["zone"].is<const char*>() ? String(o["zone"].as<const char*>()) : String();
    r.cooldown_ms = o["cooldown_ms"].is<unsigned long>() ? o["cooldown_ms"].as<unsigned long>() : 0;
    r.use_speed_gt = o["speed_gt"].is<int>() || o["speed_gt"].is<float>();
    r.use_speed_lt = o["speed_lt"].is<int>() || o["speed_lt"].is<float>();
    if (r.use_speed_gt) r.speed_gt = o["speed_gt"].as<float>();
    if (r.use_speed_lt) r.speed_lt = o["speed_lt"].as<float>();
    if (o["dir"].is<int>()) r.dir = o["dir"].as<int>();
    else if (o["dir"].is<const char*>()) {
      String ds = String(o["dir"].as<const char*>());
      ds.toLowerCase();
      if (ds == "towards" || ds == "in" || ds == "+1") r.dir = 1;
      else if (ds == "away" || ds == "out" || ds == "-1") r.dir = -1;
    }

    r.actions_count = 0;
    if (o["actions"].is<JsonArray>()) {
      for (JsonVariant av : o["actions"].as<JsonArray>()) {
        if (r.actions_count >= 8) break;
        if (!av.is<JsonObject>()) continue;
        JsonObject ao = av.as<JsonObject>();
        RuleAction a;
        a.type = ao["type"].is<const char*>() ? String(ao["type"].as<const char*>()) : String();
        a.text = ao["text"].is<const char*>() ? String(ao["text"].as<const char*>()) : String();
        a.value = ao["value"].is<int>() ? ao["value"].as<int>() : 0;
        if (a.type.length() == 0) continue;
        r.actions[r.actions_count++] = a;
      }
    }
    rules[rules_count++] = r;
  }

  return true;
}

static bool hitInZone(const Zone& z, float ang, float range_cm) {
  if (!z.enabled) return false;
  if (ang < z.min_deg || ang > z.max_deg) return false;
  if (range_cm < z.min_cm || range_cm > z.max_cm) return false;
  return true;
}

static String detectZone(float ang, float range_cm) {
  for (int i = 0; i < zones_count; i++) {
    if (hitInZone(zones[i], ang, range_cm)) return zones[i].id;
  }
  return "";
}

static bool switchProfileInternal(const String& name, bool createIfMissing) {
  if (name.length() < 1 || name.length() > 32) return false;
  for (size_t i = 0; i < name.length(); i++) {
    char c = name[i];
    if (!(isalnum((int)c) || c == '_' || c == '-')) return false;
  }

  if (!ensureProfileLayout(name)) return false;
  String old = active_profile;
  active_profile = name;

  if (createIfMissing) {
    if (!LittleFS.exists(configPath())) writeIfMissing(configPath().c_str(), DEFAULT_CONFIG_JSON);
    if (!LittleFS.exists(rulesPath())) writeIfMissing(rulesPath().c_str(), DEFAULT_RULES_JSON);
    if (!LittleFS.exists(zonesPath())) writeIfMissing(zonesPath().c_str(), DEFAULT_ZONES_JSON);
  }

  if (!LittleFS.exists(configPath())) { active_profile = old; return false; }

  if (!loadConfigFromFile()) {
    active_profile = old;
    return false;
  }

  loadZones();
  loadRules();

  prefs.begin("rlsmeta", false);
  prefs.putString("profile", active_profile);
  prefs.end();

  appendLog("profile_switch " + active_profile);
  return true;
}

static void applyAction(const RuleAction& a) {
  if (a.type == "speak") {
    if (a.text.length()) queueSpeak(a.text);
  } else if (a.type == "log") {
    appendLog(a.text);
  } else if (a.type == "scan") {
    if (a.value) startScanNow();
    else stopScanNow();
  } else if (a.type == "radar") {
    radar_on = (a.value != 0);
    radarPower(radar_on);
  } else if (a.type == "pwm") {
    servoPWM(a.value != 0);
  } else if (a.type == "profile") {
    if (a.text.length()) switchProfileInternal(a.text, true);
  }
}

static void emitEvent(const String& event) {
  uint32_t now = millis();
  float ang = servoAngleDeg();
  float range_cm = 120.0f;
  float speed = last_hit_speed_kmh;
  int dir = last_hit_dir;

  String z = detectZone(ang, range_cm);

  for (int i = 0; i < rules_count; i++) {
    Rule& r = rules[i];
    if (!r.enabled) continue;
    if (r.event != event) continue;
    if (r.zone.length() && r.zone != z) continue;
    if (r.use_speed_gt && !(speed > r.speed_gt)) continue;
    if (r.use_speed_lt && !(speed < r.speed_lt)) continue;
    if (r.dir != 0 && r.dir != dir) continue;
    if (r.cooldown_ms && (now - r.last_fire) < r.cooldown_ms) continue;

    for (int k = 0; k < r.actions_count; k++) {
      applyAction(r.actions[k]);
    }
    r.last_fire = now;
  }
}

// -------------------- FAILSAFE --------------------
static void failsafeService() {
  if (!cfg.failsafe_stop) return;
  uint32_t now = millis();
  if ((now - last_client_ms) > (uint32_t)cfg.failsafe_sec * 1000UL) {
    stopScanNow();
    radar_on = false;
    radarPower(false);
  }
}

// -------------------- AUTH --------------------
static const char* HDR_KEYS[] = {"X-Token"};

static bool requireWriteAuth() {
  if (cfg.auth_enabled) {
    if (!server.authenticate(cfg.auth_user.c_str(), cfg.auth_pass.c_str())) {
      server.requestAuthentication();
      return false;
    }
  }
  if (cfg.write_token.length()) {
    String tok = server.header("X-Token");
    if (tok != cfg.write_token) {
      server.send(403, "application/json; charset=utf-8", "{\"ok\":0,\"error\":\"bad_token\"}");
      return false;
    }
  }
  return true;
}

// -------------------- HELPERS --------------------
static void markClient() {
  last_client_ms = millis();
}

static void sendJsonError(int code, const String& err) {
  server.send(code, "application/json; charset=utf-8",
              String("{\"ok\":0,\"error\":\"") + jsonEscape(err) + "\"}");
}

static void serveUiFile(String uri) {
  markClient();
  if (uri == "/") uri = "/index.html";
  String fsPath = String("/ui") + uri;
  if (!LittleFS.exists(fsPath)) {
    server.send(404, "text/plain; charset=utf-8", "Not found");
    return;
  }
  File f = LittleFS.open(fsPath, FILE_READ);
  if (!f) {
    server.send(500, "text/plain; charset=utf-8", "Open failed");
    return;
  }
  server.streamFile(f, String(contentTypeFromPath(uri)));
  f.close();
}

// -------------------- API --------------------
static void handleState() {
  markClient();

  uint32_t now = millis();
  DynamicJsonDocument doc(12288);

  doc["ip"] = WiFi.softAPIP().toString();
  doc["version"] = APP_VERSION;
  doc["profile"] = active_profile;
  doc["rev"] = cfg_revision;

  doc["scan_on"] = servo_scan_on ? 1 : 0;
  doc["pwm_on"] = servo_pwm_on ? 1 : 0;
  doc["radar_on"] = radar_on ? 1 : 0;
  doc["radar_powered"] = radar_powered ? 1 : 0;

  doc["servo_us"] = servo_us_cmd;
  doc["servo_dir"] = String(servo_dir);
  doc["servo_angle_deg"] = servoAngleDeg();
  doc["motion"] = motion_now ? 1 : 0;
  doc["last_speed_kmh"] = last_hit_speed_kmh;
  doc["last_dir"] = last_hit_dir;

  doc["bat_v"] = bat_v;
  doc["bat_pct"] = bat_pct;

  JsonArray arr = doc.createNestedArray("hits");
  for (int i = 0; i < MAX_HITS; i++) {
    const Hit& h = hits[i];
    if (h.ts == 0) continue;
    uint32_t age = now - h.ts;
    if (age > 8000) continue;
    JsonObject o = arr.createNestedObject();
    o["angle_deg"] = h.angle_deg;
    o["range_cm"] = h.range_cm;
    o["v_kmh"] = h.v_kmh;
    o["dir"] = h.dir;
    o["age_ms"] = age;
  }

  doc["speak_seq"] = speak_seq;
  doc["speak_text"] = speak_text;

  JsonObject c = doc.createNestedObject("cfg");
  c["center_us"] = cfg.center_us;
  c["left_span_us"] = cfg.left_span_us;
  c["right_span_us"] = cfg.right_span_us;
  c["bias_right_us"] = cfg.bias_right_us;
  c["bias_left_us"] = cfg.bias_left_us;
  c["scan_period_ms"] = cfg.scan_period_ms;
  c["tick_ms"] = cfg.tick_ms;
  c["ease_pct"] = cfg.ease_pct;

  JsonObject cx = doc.createNestedObject("cfg_ext");
  cx["soft_edge_brake"] = cfg.soft_edge_brake;
  cx["backlash_comp"] = cfg.backlash_comp;
  cx["asym_correction"] = cfg.asym_correction;
  cx["auto_center_before_scan"] = cfg.auto_center_before_scan;
  cx["table_calibration"] = cfg.table_calibration;
  cx["soft_edge_pct"] = cfg.soft_edge_pct;
  cx["backlash_us"] = cfg.backlash_us;
  cx["backlash_settle_ms"] = cfg.backlash_settle_ms;
  cx["auto_center_ms"] = cfg.auto_center_ms;
  cx["left_scale_pct"] = cfg.left_scale_pct;
  cx["right_scale_pct"] = cfg.right_scale_pct;

  String out;
  serializeJson(doc, out);
  server.send(200, "application/json; charset=utf-8", out);
}

static void handleVersion() {
  markClient();
  DynamicJsonDocument doc(512);
  doc["ok"] = 1;
  doc["version"] = APP_VERSION;
  doc["profile"] = active_profile;
  doc["rev"] = cfg_revision;
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json; charset=utf-8", out);
}

static void handleSet() {
  markClient();
  if (server.hasArg("center")) cfg.center_us = clampi(server.arg("center").toInt(), 1200, 1800);
  if (server.hasArg("lspan")) cfg.left_span_us = clampi(server.arg("lspan").toInt(), 10, 500);
  if (server.hasArg("rspan")) cfg.right_span_us = clampi(server.arg("rspan").toInt(), 10, 500);
  if (server.hasArg("br")) cfg.bias_right_us = clampi(server.arg("br").toInt(), -120, 120);
  if (server.hasArg("bl")) cfg.bias_left_us = clampi(server.arg("bl").toInt(), -120, 120);
  if (server.hasArg("period")) cfg.scan_period_ms = clampi(server.arg("period").toInt(), 800, 60000);
  if (server.hasArg("tick")) cfg.tick_ms = clampi(server.arg("tick").toInt(), 80, 1200);
  if (server.hasArg("ease")) cfg.ease_pct = clampi(server.arg("ease").toInt(), 0, 100);

  if (server.hasArg("soft")) cfg.soft_edge_brake = server.arg("soft").toInt() ? true : false;
  if (server.hasArg("bkl")) cfg.backlash_comp = server.arg("bkl").toInt() ? true : false;
  if (server.hasArg("asym")) cfg.asym_correction = server.arg("asym").toInt() ? true : false;
  if (server.hasArg("autoc")) cfg.auto_center_before_scan = server.arg("autoc").toInt() ? true : false;
  if (server.hasArg("tabl")) cfg.table_calibration = server.arg("tabl").toInt() ? true : false;
  if (server.hasArg("softPct")) cfg.soft_edge_pct = clampi(server.arg("softPct").toInt(), 0, 100);
  if (server.hasArg("backUs")) cfg.backlash_us = clampi(server.arg("backUs").toInt(), 0, 200);
  if (server.hasArg("backMs")) cfg.backlash_settle_ms = clampi(server.arg("backMs").toInt(), 50, 500);
  if (server.hasArg("autoMs")) cfg.auto_center_ms = clampi(server.arg("autoMs").toInt(), 100, 2000);
  if (server.hasArg("lsc")) cfg.left_scale_pct = clampi(server.arg("lsc").toInt(), 50, 150);
  if (server.hasArg("rsc")) cfg.right_scale_pct = clampi(server.arg("rsc").toInt(), 50, 150);

  clampConfig();
  server.send(200, "application/json; charset=utf-8", "{\"ok\":1}");
}

static void handleCmd() {
  markClient();

  if (server.hasArg("scan")) {
    int v = server.arg("scan").toInt();
    if (v == 1) {
      startScanNow();
      queueSpeak(cfg.phrase_scan_start);
      appendLog("scan_on");
    } else {
      stopScanNow();
      queueSpeak(cfg.phrase_scan_stop);
      appendLog("scan_off");
    }
  }

  if (server.hasArg("pwm")) {
    int v = server.arg("pwm").toInt();
    if (v == 1) {
      servoPower(true);
      servoPWM(true);
      appendLog("pwm_on");
    } else {
      servoPWM(false);
      appendLog("pwm_off");
    }
  }

  if (server.hasArg("radar")) {
    int v = server.arg("radar").toInt();
    radar_on = (v == 1);
    radarPower(radar_on);
    queueSpeak(radar_on ? cfg.phrase_radar_on : cfg.phrase_radar_off);
    appendLog(radar_on ? "radar_on" : "radar_off");
  }

  if (server.hasArg("center")) {
    stopScanNow();
    servoPower(true);
    servoPWM(true);
    servo_us_cmd = cfg.center_us;
    queueSpeak(cfg.phrase_servo_center);
    appendLog("servo_center");
  }

  if (server.hasArg("test")) {
    String d = server.arg("test");
    stopScanNow();
    servoPower(true);
    servoPWM(true);
    if (d == "L") servo_us_cmd = cfg.center_us - cfg.left_span_us;
    else if (d == "R") servo_us_cmd = cfg.center_us + cfg.right_span_us;
    appendLog("servo_test_" + d);
  }

  if (server.hasArg("profile")) {
    String p = server.arg("profile");
    if (!switchProfileInternal(p, true)) {
      sendJsonError(400, "bad_profile");
      return;
    }
  }

  if (server.hasArg("rollback")) {
    if (!requireWriteAuth()) return;
    if (!rollbackConfig()) {
      sendJsonError(500, "rollback_failed");
      return;
    }
    loadZones();
    loadRules();
  }

  if (server.hasArg("reboot")) {
    server.send(200, "application/json; charset=utf-8", "{\"ok\":1}");
    delay(200);
    ESP.restart();
    return;
  }

  server.send(200, "application/json; charset=utf-8", "{\"ok\":1}");
}

static void handleSave() {
  markClient();
  if (!requireWriteAuth()) return;
  bool ok = saveConfig();
  if (!ok) {
    sendJsonError(500, "save_failed");
    return;
  }
  loadZones();
  loadRules();
  server.send(200, "application/json; charset=utf-8", "{\"ok\":1}");
}

// -------------------- FILE MANAGER --------------------
static void handleFiles() {
  markClient();
  String op = server.hasArg("op") ? server.arg("op") : "list";
  String path = normPath(server.hasArg("path") ? server.arg("path") : "/");
  if (!isSafeFsPath(path)) { sendJsonError(400, "bad_path"); return; }

  if (op == "list") {
    File root = LittleFS.open(path, FILE_READ);
    if (!root) { sendJsonError(404, "not_found"); return; }
    if (!root.isDirectory()) { sendJsonError(400, "not_dir"); return; }

    DynamicJsonDocument doc(8192);
    doc["ok"] = 1;
    doc["path"] = path;
    JsonArray arr = doc.createNestedArray("entries");

    File f = root.openNextFile();
    while (f) {
      JsonObject e = arr.createNestedObject();
      e["name"] = String(f.name());
      e["is_dir"] = f.isDirectory();
      if (!f.isDirectory()) e["size"] = (uint32_t)f.size();
      f = root.openNextFile();
    }

    String out;
    serializeJson(doc, out);
    server.send(200, "application/json; charset=utf-8", out);
    return;
  }

  if (op == "read") {
    File f = LittleFS.open(path, FILE_READ);
    if (!f) { sendJsonError(404, "not_found"); return; }
    if (f.isDirectory()) { sendJsonError(400, "is_dir"); return; }
    String ct = server.hasArg("ct") ? server.arg("ct") : String(contentTypeFromPath(path));
    server.streamFile(f, ct);
    f.close();
    return;
  }

  if (!requireWriteAuth()) return;

  if (op == "write") {
    String body = server.arg("plain");
    String tmp = path + ".tmp";
    if (LittleFS.exists(tmp)) LittleFS.remove(tmp);
    File f = LittleFS.open(tmp, FILE_WRITE);
    if (!f) { sendJsonError(500, "open_failed"); return; }
    size_t w = f.print(body);
    f.flush();
    f.close();
    if (w != body.length()) { LittleFS.remove(tmp); sendJsonError(500, "write_failed"); return; }
    if (LittleFS.exists(path)) LittleFS.remove(path);
    if (!LittleFS.rename(tmp, path)) { LittleFS.remove(tmp); sendJsonError(500, "rename_failed"); return; }
    appendLog("file_write " + path);
    server.send(200, "application/json; charset=utf-8", "{\"ok\":1}");
    return;
  }

  if (op == "del") {
    if (!LittleFS.exists(path)) { sendJsonError(404, "not_found"); return; }
    if (!LittleFS.remove(path)) { sendJsonError(500, "delete_failed"); return; }
    appendLog("file_delete " + path);
    server.send(200, "application/json; charset=utf-8", "{\"ok\":1}");
    return;
  }

  if (op == "mkdir") {
    if (LittleFS.exists(path)) { sendJsonError(409, "exists"); return; }
    if (!LittleFS.mkdir(path)) { sendJsonError(500, "mkdir_failed"); return; }
    appendLog("mkdir " + path);
    server.send(200, "application/json; charset=utf-8", "{\"ok\":1}");
    return;
  }

  if (op == "rmdir") {
    if (!LittleFS.exists(path)) { sendJsonError(404, "not_found"); return; }
    if (!LittleFS.rmdir(path)) { sendJsonError(500, "rmdir_failed"); return; }
    appendLog("rmdir " + path);
    server.send(200, "application/json; charset=utf-8", "{\"ok\":1}");
    return;
  }

  if (op == "rename") {
    String to = normPath(server.hasArg("to") ? server.arg("to") : "");
    if (!isSafeFsPath(to)) { sendJsonError(400, "bad_to"); return; }
    if (!LittleFS.exists(path)) { sendJsonError(404, "not_found"); return; }
    if (LittleFS.exists(to)) { sendJsonError(409, "to_exists"); return; }
    if (!LittleFS.rename(path, to)) { sendJsonError(500, "rename_failed"); return; }
    appendLog("rename " + path + " -> " + to);
    server.send(200, "application/json; charset=utf-8", "{\"ok\":1}");
    return;
  }

  sendJsonError(400, "bad_op");
}

// -------------------- RULES / ZONES / LOGS / PROFILES --------------------
static void handleRulesGet() {
  markClient();
  File f = LittleFS.open(rulesPath(), FILE_READ);
  if (!f) { server.send(404, "text/plain; charset=utf-8", "rules.json missing"); return; }
  server.streamFile(f, "application/json; charset=utf-8");
  f.close();
}
static void handleRulesPost() {
  markClient();
  if (!requireWriteAuth()) return;
  String body = server.arg("plain");
  DynamicJsonDocument doc(12288);
  auto err = deserializeJson(doc, body);
  if (err) { sendJsonError(400, "bad_json"); return; }
  if (!saveTextAtomic(rulesPath(), body, 2)) { sendJsonError(500, "save_failed"); return; }
  loadRules();
  appendLog("rules_saved profile=" + active_profile);
  server.send(200, "application/json; charset=utf-8", "{\"ok\":1}");
}

static void handleZonesGet() {
  markClient();
  File f = LittleFS.open(zonesPath(), FILE_READ);
  if (!f) { server.send(404, "text/plain; charset=utf-8", "zones.json missing"); return; }
  server.streamFile(f, "application/json; charset=utf-8");
  f.close();
}
static void handleZonesPost() {
  markClient();
  if (!requireWriteAuth()) return;
  String body = server.arg("plain");
  DynamicJsonDocument doc(8192);
  auto err = deserializeJson(doc, body);
  if (err) { sendJsonError(400, "bad_json"); return; }
  if (!saveTextAtomic(zonesPath(), body, 2)) { sendJsonError(500, "save_failed"); return; }
  loadZones();
  appendLog("zones_saved profile=" + active_profile);
  server.send(200, "application/json; charset=utf-8", "{\"ok\":1}");
}

static void handleLogs() {
  markClient();
  File f = LittleFS.open("/logs/log.txt", FILE_READ);
  if (!f) {
    server.send(200, "text/plain; charset=utf-8", "(лог пуст)");
    return;
  }
  server.streamFile(f, "text/plain; charset=utf-8");
  f.close();
}

static void handleProfilesGet() {
  markClient();
  if (!LittleFS.exists("/profiles")) ensureDir("/profiles");
  File root = LittleFS.open("/profiles", FILE_READ);
  DynamicJsonDocument doc(4096);
  doc["ok"] = 1;
  doc["active"] = active_profile;
  JsonArray arr = doc.createNestedArray("profiles");

  if (root && root.isDirectory()) {
    File f = root.openNextFile();
    while (f) {
      if (f.isDirectory()) {
        String n = String(f.name());
        if (n.startsWith("/profiles/")) n = n.substring(String("/profiles/").length());
        arr.add(n);
      }
      f = root.openNextFile();
    }
  }

  if (arr.size() == 0) arr.add(active_profile);

  String out;
  serializeJson(doc, out);
  server.send(200, "application/json; charset=utf-8", out);
}

static void handleProfilesPost() {
  markClient();
  if (!requireWriteAuth()) return;
  String body = server.arg("plain");
  DynamicJsonDocument doc(512);
  if (deserializeJson(doc, body)) {
    sendJsonError(400, "bad_json");
    return;
  }
  String name = doc["name"].is<const char*>() ? String(doc["name"].as<const char*>()) : String();
  if (!switchProfileInternal(name, true)) {
    sendJsonError(400, "bad_profile");
    return;
  }
  appendLog("profile_created " + name);
  server.send(200, "application/json; charset=utf-8", "{\"ok\":1}");
}

// -------------------- OTA --------------------
static void handleUpdatePage() {
  markClient();
  if (cfg.auth_enabled) {
    if (!server.authenticate(cfg.auth_user.c_str(), cfg.auth_pass.c_str())) {
      server.requestAuthentication();
      return;
    }
  }

  String html =
    "<html><body style='background:#000;color:#8fffa8;font-family:system-ui'>"
    "<h2>OTA Update</h2>"
    "<p>Загружай файл <b>Esp32c6-radar.ino.bin</b> (не merged.bin).</p>"
    "<form method='POST' action='/update' enctype='multipart/form-data'>"
    "<input type='file' name='update' accept='.bin'>"
    "<input type='submit' value='Upload'>"
    "</form></body></html>";

  server.send(200, "text/html; charset=utf-8", html);
}

static void handleUpdateUpload() {
  if (cfg.auth_enabled) {
    if (!server.authenticate(cfg.auth_user.c_str(), cfg.auth_pass.c_str())) {
      server.requestAuthentication();
      return;
    }
  }

  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    Update.begin(UPDATE_SIZE_UNKNOWN);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    Update.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      appendLog("ota_success");
      server.send(200, "text/plain", "OK. Rebooting...");
      delay(300);
      ESP.restart();
    } else {
      appendLog("ota_failed");
      server.send(500, "text/plain", "Update failed");
    }
  }
}

// -------------------- FIRST BOOT INIT --------------------
static void seedDefaultFiles() {
  ensureDir("/ui");
  ensureDir("/profiles");
  ensureProfileLayout(active_profile);
  ensureDir("/logs");
  ensureDir("/cfg");

  writeIfMissing("/ui/index.html", DEFAULT_INDEX);
  writeIfMissing("/ui/style.css", DEFAULT_STYLE);
  writeIfMissing("/ui/app.js", DEFAULT_APPJS);

  if (!LittleFS.exists(configPath())) writeIfMissing(configPath().c_str(), DEFAULT_CONFIG_JSON);
  if (!LittleFS.exists(rulesPath())) writeIfMissing(rulesPath().c_str(), DEFAULT_RULES_JSON);
  if (!LittleFS.exists(zonesPath())) writeIfMissing(zonesPath().c_str(), DEFAULT_ZONES_JSON);
}

static void loadLegacyNVSIntoConfigIfNeeded() {
  prefs.begin("rls", true);
  cfg.center_us = prefs.getInt("center", cfg.center_us);
  cfg.left_span_us = prefs.getInt("lspan", cfg.left_span_us);
  cfg.right_span_us = prefs.getInt("rspan", cfg.right_span_us);
  cfg.bias_right_us = prefs.getInt("br", cfg.bias_right_us);
  cfg.bias_left_us = prefs.getInt("bl", cfg.bias_left_us);
  cfg.scan_period_ms = prefs.getInt("period", cfg.scan_period_ms);
  cfg.tick_ms = prefs.getInt("tick", cfg.tick_ms);
  cfg.ease_pct = prefs.getInt("ease", cfg.ease_pct);
  prefs.end();
}

// -------------------- SETUP / LOOP --------------------
void setup() {
  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);
  servoPWM(false);
  servoPower(false);

  if (RADAR_OUT_PIN >= 0) pinMode(RADAR_OUT_PIN, INPUT);
  if (RADAR_PWR_PIN >= 0) {
    pinMode(RADAR_PWR_PIN, OUTPUT);
    digitalWrite(RADAR_PWR_PIN, LOW);
  }

  Serial.begin(115200);
  delay(50);

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
  }

  prefs.begin("rlsmeta", true);
  active_profile = prefs.getString("profile", "default");
  cfg_revision = prefs.getULong("rev", 0);
  prefs.end();

  seedDefaultFiles();

  if (!LittleFS.exists(configPath())) {
    loadLegacyNVSIntoConfigIfNeeded();
    saveConfig();
  } else {
    loadConfigFromFile();
  }

  loadZones();
  loadRules();

  radar_on = cfg.radar_enabled_default;
  radarPower(radar_on);
  servo_us_cmd = cfg.center_us;

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(200);

  server.collectHeaders(HDR_KEYS, 1);

  // old endpoints retained
  server.on("/", HTTP_GET, [](){ serveUiFile("/"); });
  server.on("/api/state", HTTP_GET, handleState);
  server.on("/api/version", HTTP_GET, handleVersion);
  server.on("/api/set", HTTP_GET, handleSet);
  server.on("/api/cmd", HTTP_GET, handleCmd);
  server.on("/api/save", HTTP_GET, handleSave);

  // new endpoints
  server.on("/api/files", HTTP_ANY, handleFiles);
  server.on("/api/rules", HTTP_GET, handleRulesGet);
  server.on("/api/rules", HTTP_POST, handleRulesPost);
  server.on("/api/zones", HTTP_GET, handleZonesGet);
  server.on("/api/zones", HTTP_POST, handleZonesPost);
  server.on("/api/logs", HTTP_GET, handleLogs);
  server.on("/api/profiles", HTTP_GET, handleProfilesGet);
  server.on("/api/profiles", HTTP_POST, handleProfilesPost);

  server.on("/update", HTTP_GET, handleUpdatePage);
  server.on("/update", HTTP_POST, [](){}, handleUpdateUpload);

  server.onNotFound([]() {
    String uri = server.uri();
    // Отдаём статические файлы из /ui
    serveUiFile(uri);
  });

  server.begin();

  last_client_ms = millis();
  last_motion_ms = millis();
  appendLog("boot " + String(APP_VERSION));
}

void loop() {
  server.handleClient();

  servoScanService();
  servoService50Hz();
  radarService();
  batteryService();

  static bool prevMotion = false;
  if (motion_now && !prevMotion) emitEvent("motion_start");
  if (!motion_now && prevMotion) emitEvent("motion_clear");
  prevMotion = motion_now;

  failsafeService();
}
