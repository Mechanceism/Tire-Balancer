/*
  Web page - the screen makes its own open WiFi hotspot and serves a page
  that does everything the touch screen does, plus a console and diagnostics.

    WiFi "TireBalancer" (no password, never joins another network)
    http://192.168.4.1  (phones usually open it by themselves: every web
    address leads to the page, so the phone shows it as a sign-in page)

  How it works
  - The page (WebPage.h) asks for /api/state about 3 times a second: the
    machine state, the settings and any new log lines, as JSON.
  - Buttons POST to /api/do. Each action does exactly what the touch screen
    does (the do...() functions in ui.ino), and moves the same sliders and
    switches, so the touch screen and every phone always agree.
  - WebServer runs inside loop(), the same task as LVGL, so the actions can
    use LVGL directly.
  - Safety: anything that moves the platform or switches the laser on is
    refused unless the page sends ack=1, which it only does while "I can see
    the machine and it is clear" is ticked. Stop and Cancel always work.

  The Nano is not changed: it sees the same commands as from the touch screen.
*/

#include "WebPage.h"             // the page itself (HTML, CSS, JavaScript)

#define WIFI_SSID       "TireBalancer"
#define WEB_LOG_LINES   200          // console lines kept on the screen (the page keeps more)
#define WEB_LOG_LEN     100
#define WEB_LOG_MAX_OUT 80           // log lines per reply at most

static WebServer web(80);
static DNSServer dns;
static uint32_t  webBootId = 0;      // changes every restart, so the page knows to clear its log

static char     webLogBuf[WEB_LOG_LINES][WEB_LOG_LEN];
static uint32_t webLogSeq = 0;       // lines ever logged; line n is in webLogBuf[n % WEB_LOG_LINES]

static uint32_t noteId = 0, msgId = 0;
static char     noteText[48]  = "";
static char     msgTitle[32]  = "", msgText[200] = "";
static char     nanoVersion[16] = "";

static lv_obj_t *wifiLabel = NULL;   // Debug tab: how to open the page

// ------------------------------------------------------------ hooks called from ui.ino
// kind: '>' sent to the Nano, '<' heard from the Nano, '!' event
void webLog(char kind, const char *text) {
  char *d = webLogBuf[webLogSeq % WEB_LOG_LINES];
  d[0] = kind;
  strncpy(d + 1, text, WEB_LOG_LEN - 2);
  d[WEB_LOG_LEN - 1] = 0;
  webLogSeq++;
}

void webNote(const char *text) {
  strncpy(noteText, text, sizeof(noteText) - 1);
  noteId++;
}

void webMessage(const char *title, const char *text) {
  strncpy(msgTitle, title, sizeof(msgTitle) - 1);
  strncpy(msgText, text, sizeof(msgText) - 1);
  msgId++;
}

// Every valid line from the Nano: "Ready V0.7.0" (restart) or "# V0.7.0" (Status)
void webNanoLine(const char *line) {
  const char *v = NULL;
  if (!strncmp(line, "Ready V", 7)) v = line + 7;
  else if (!strncmp(line, "# V", 3) && line[3] >= '0' && line[3] <= '9') v = line + 3;
  if (v) strncpy(nanoVersion, v, sizeof(nanoVersion) - 1);
}

// ------------------------------------------------------------ JSON helpers
static void jStr(String &o, const char *s) {
  o += '"';
  for (; *s; s++) {
    char c = *s;
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if ((uint8_t)c < 0x20 || (uint8_t)c > 0x7E) o += ' ';
    else o += c;
  }
  o += '"';
}
static void jKey(String &o, const char *k) { o += ",\""; o += k; o += "\":"; }
static void jInt(String &o, const char *k, long v) { jKey(o, k); o += v; }
static void jNum(String &o, const char *k, float v, int dec) { jKey(o, k); o += String(v, dec); }
static void jTxt(String &o, const char *k, const char *v) { jKey(o, k); jStr(o, v); }

// Which number pad format a typed size is (0 metric, 1 inch, 2 flotation),
// so a size typed on the web also shows the right format on the touch screen.
static int tireFormatOf(const char *s) {
  char u[32];
  uint8_t n = 0;
  for (const char *q = s; *q && n < sizeof(u) - 1; q++) u[n++] = toupper(*q);
  u[n] = 0;
  const char *p = u;
  skipNonDigits(p);
  float a = readNum(p);
  if (a >= 60) return 0;
  skipSpaces(p);
  return (*p == 'X' || *p == '*') ? 2 : 1;
}

// "Width 122 mm, sidewall 116 mm ..." for a size, or "" if it is not valid
static void tireDescribe(const char *size, char *out, size_t len) {
  TireSize t;
  if (!parseTireSize(size, t)) { out[0] = 0; return; }
  snprintf(out, len, "Width %.0f mm, sidewall %.0f mm, rim %.0f mm, tire %.0f mm across",
           t.widthMm, t.sideMm, t.rimMm, t.rimMm + 2 * t.sideMm);
}

// ------------------------------------------------------------ GET /api/state?log=<next line wanted>
static void webState() {
  uint32_t now = millis();
  String o;
  o.reserve(3000);
  o += "{\"boot\":"; o += webBootId;
  jTxt(o, "fw", SCREEN_FW_VERSION);
  jTxt(o, "nv", nanoVersion);
  jInt(o, "up", now / 1000);
  jInt(o, "heap", ESP.getFreeHeap());
  jInt(o, "minHeap", ESP.getMinFreeHeap());
  jInt(o, "sta", WiFi.softAPgetStationNum());

  // link
  jInt(o, "link", linkUp);
  jInt(o, "quiet", quiet);
  jInt(o, "rx", rxBytes);
  jInt(o, "bad", rxBadLines);
  jInt(o, "heard", lastHeardMs ? (long)(now - lastHeardMs) : -1);
  jInt(o, "txq", txqCount);

  // what is running
  const char *busy = waiting == WAIT_LEVEL ? (reachRunning ? "reach" : "level")
                   : waiting == WAIT_BALANCE ? "balance" : waiting == WAIT_TARE ? "tare" : "";
  jTxt(o, "busy", busy);

  // last balance result (metric)
  jInt(o, "have", haveResult);
  jNum(o, "angle", resAngleDeg, 2);
  jNum(o, "massG", resMassG, 2);
  jNum(o, "totalKg", resTotalKg, 3);

  // tire
  jTxt(o, "box", tireText);
  jInt(o, "entered", tireEntered);
  jTxt(o, "tire", tireEnteredText);
  if (tireEntered) {
    char d[96];
    tireDescribe(tireEnteredText, d, sizeof(d));
    jTxt(o, "tireInfo", d);
    float offset = isChecked(weightPosSw) ? STICK_ON_OFFSET_MM : CLIP_ON_OFFSET_MM;
    jNum(o, "weightR", tire.rimMm / 2 + offset, 1);
  }
  jKey(o, "recents"); o += '[';
  for (uint8_t i = 0, first = 1; i < RECENT_N; i++) {
    const char *bar = strchr(recents[i], '|');
    if (!bar) continue;
    if (!first) o += ',';
    first = 0;
    jStr(o, bar + 1);
  }
  o += ']';

  // switches and sliders, as the touch screen shows them
  jInt(o, "stick", isChecked(weightPosSw));
  jInt(o, "autoStop", isChecked(ui_AutoStopCheckbox));
  jInt(o, "massComp", isChecked(ui_MassCompSwitch));
  jInt(o, "testMass", isChecked(ui_TestWeightSwitch));
  jInt(o, "manualLaser", isChecked(ui_ManualLaserSwitch));
  jInt(o, "grams", isChecked(ui_TestWeightSwitch1));
  jInt(o, "lbs", isChecked(ui_TestWeightSwitch2));
  jInt(o, "laserAngle", lv_slider_get_value(ui_ManualLaserAngleSlider));
  jInt(o, "pitchTrim", lv_slider_get_value(ui_PitchOffsetSlider));
  jInt(o, "rollTrim", lv_slider_get_value(ui_RollOffsetSlider));
  jInt(o, "minSpeed", enteredMin);
  jInt(o, "maxSpeed", enteredMax);
  jInt(o, "kp", enteredKp);

  // pop-ups
  jKey(o, "note"); o += '['; o += noteId; o += ','; jStr(o, noteText); o += ']';
  jKey(o, "msg");  o += '['; o += msgId;  o += ','; jStr(o, msgTitle); o += ','; jStr(o, msgText); o += ']';

  // new log lines
  uint32_t from = web.hasArg("log") ? strtoul(web.arg("log").c_str(), NULL, 10) : 0;
  uint32_t oldest = webLogSeq > WEB_LOG_LINES ? webLogSeq - WEB_LOG_LINES : 0;
  if (from > webLogSeq || from < oldest) from = oldest;
  uint32_t to = min(webLogSeq, from + WEB_LOG_MAX_OUT);
  jKey(o, "log"); o += '[';
  for (uint32_t n = from; n < to; n++) {
    if (n > from) o += ',';
    jStr(o, webLogBuf[n % WEB_LOG_LINES]);
  }
  o += ']';
  jInt(o, "seq", to);
  o += '}';

  web.sendHeader("Cache-Control", "no-store");
  web.send(200, "application/json", o);
}

// ------------------------------------------------------------ POST /api/do  a=<action> v=<value> ack=0|1
// Commands typed in the console that cannot move anything or switch the laser on
static const char *const SAFE_COMMANDS[] = {
  "Ping", "Status", "LevelDebugOn", "LevelDebugOff", "SensorDebugOn", "SensorDebugOff",
  "ScaleDebugOn", "ScaleDebugOff", "Stop", "LevelCancel", "BalanceCancel", "LaserOff",
  "ManualLaserOff", "Ack", NULL
};
static bool safeCommand(const char *c) {
  for (uint8_t i = 0; SAFE_COMMANDS[i]; i++) if (!strcmp(c, SAFE_COMMANDS[i])) return true;
  return false;
}

// Set a switch / checkbox and run its handler, exactly like a tap
static void tapSwitch(lv_obj_t *o, bool on) {
  if (isChecked(o) == on) return;
  setChecked(o, on);
  lv_event_send(o, LV_EVENT_VALUE_CHANGED, NULL);
}
// Set a slider and run its handlers (value label, send to the Nano, save)
static void dragSlider(lv_obj_t *s, int v) {
  v = constrain(v, (int)lv_slider_get_min_value(s), (int)lv_slider_get_max_value(s));
  lv_slider_set_value(s, v, LV_ANIM_OFF);
  lv_event_send(s, LV_EVENT_VALUE_CHANGED, NULL);
}

static void webReply(bool ok, const char *err, const char *info) {
  String o = "{\"ok\":";
  o += ok ? "true" : "false";
  jTxt(o, "err", err);
  jTxt(o, "info", info);
  o += '}';
  web.sendHeader("Cache-Control", "no-store");
  web.send(200, "application/json", o);
}

static void webDo() {
  String a = web.arg("a");
  String vs = web.arg("v");
  vs.trim();
  const char *v = vs.c_str();
  int vi = vs.toInt();
  bool on = vi != 0;
  bool ack = web.arg("ack") == "1";

  // Things that move the platform or switch the laser on need the safety tick
  bool moves = a == "level" || a == "reach" || a == "balance" || a == "laser" ||
               a == "laserAngle" || a == "laserStep" || a == "set90" ||
               (a == "manualLaser" && on) || (a == "cmd" && !safeCommand(v));
  if (moves && !ack) {
    webReply(false, "Tick \"I can see the machine and it is clear\" first.", "");
    return;
  }

  // ---- Level / Balance tabs
  if      (a == "stop")          { doStop(); }
  else if (a == "level")         { doLevel(); requestScreen(ui_LevelLoadingMenu); }
  else if (a == "levelCancel")   { doLevelCancel(); backToOptionMenu(); }
  else if (a == "reach")         { doFindReach(); }
  else if (a == "tare")          { doTare(); }
  else if (a == "balance")       { doBalance(); requestScreen(ui_BalanceLoadingMenu); }
  else if (a == "balanceCancel") { doBalanceCancel(); backToOptionMenu(); }
  else if (a == "laser")         { nanoSend("LaserToggle"); }
  else if (a == "autoStop")      { tapSwitch(ui_AutoStopCheckbox, on); }

  // ---- Tire Info tab
  else if (a == "tireCheck") {                        // what a size means, nothing is sent
    char d[96];
    tireDescribe(v, d, sizeof(d));
    webReply(d[0] != 0, d[0] ? "" : "Not a size this machine understands.", d);
    return;
  }
  else if (a == "tire") {
    TireSize t;
    if (vs.length() >= sizeof(tireText) || !parseTireSize(v, t)) {
      webReply(false, "Not a size this machine understands. Examples: 215/65R15, 4.80-12, 31x10.50R15", "");
      return;
    }
    lv_dropdown_set_selected(tireFmtDd, tireFormatOf(v));
    strcpy(tireText, v);
    showTireResult();
    doTireEnter();
  }
  else if (a == "stick")         { tapSwitch(weightPosSw, on); }

  // ---- Debug tab
  else if (a == "manualLaser")   { tapSwitch(ui_ManualLaserSwitch, on); }
  else if (a == "laserAngle")    { dragSlider(ui_ManualLaserAngleSlider, vi); }
  else if (a == "laserStep")     { doLaserStep(vi > 0 ? 1 : -1); }
  else if (a == "set90")         { if (!doLaserSet90()) { webReply(false, "Turn Manual Laser on first.", ""); return; } }
  else if (a == "pitchTrim")     { dragSlider(ui_PitchOffsetSlider, vi); }
  else if (a == "rollTrim")      { dragSlider(ui_RollOffsetSlider, vi); }
  else if (a == "massComp")      { tapSwitch(ui_MassCompSwitch, on); }
  else if (a == "testMass")      { tapSwitch(ui_TestWeightSwitch, on); }
  else if (a == "grams")         { tapSwitch(ui_TestWeightSwitch1, on); }
  else if (a == "lbs")           { tapSwitch(ui_TestWeightSwitch2, on); }
  else if (a == "speed") {                            // Enter Speed: min, max, kp
    dragSlider(ui_MinSpeedSlider, web.arg("min").toInt());
    dragSlider(ui_MaxSpeedSlider, web.arg("max").toInt());
    dragSlider(ui_SpeedCurveSlider, web.arg("kp").toInt());
    doSpeedEnter();
  }
  else if (a == "restore")       { doRestoreDefaults(); }

  // ---- Console
  else if (a == "cmd") {
    if (vs.length() == 0 || vs.length() > 40) { webReply(false, "Commands are 1 to 40 characters.", ""); return; }
    for (const char *p = v; *p; p++) {
      if (*p < 0x20 || *p > 0x7E || *p == '*') { webReply(false, "Only plain text, no '*'.", ""); return; }
    }
    nanoSend("%s", v);
  }
  else { webReply(false, "Unknown action.", ""); return; }

  webReply(true, "", "");
}

// ------------------------------------------------------------ setup / loop
static void webPage() {
  web.sendHeader("Cache-Control", "no-cache");
  web.send_P(200, "text/html", WEB_PAGE);
}

// Phones check for internet by fetching a known address. Sending every
// unknown address to the page makes the phone offer it as a "sign in" page.
static void webNotFound() {
  web.sendHeader("Location", "http://192.168.4.1/", true);
  web.send(302, "text/plain", "");
}

static void wifiLabelUpdate() {
  if (!wifiLabel) return;
  lv_label_set_text_fmt(wifiLabel, "Web page: join WiFi \"" WIFI_SSID "\"\nthen open 192.168.4.1   (%d connected)",
                        WiFi.softAPgetStationNum());
}

void webSetup() {
  webBootId = esp_random() & 0x7FFFFFFF;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID);                            // no password: open network
  dns.start(53, "*", WiFi.softAPIP());               // every name -> this screen

  web.on("/", HTTP_GET, webPage);
  web.on("/api/state", HTTP_GET, webState);
  web.on("/api/do", HTTP_POST, webDo);
  web.onNotFound(webNotFound);
  web.begin();

  // Debug tab, under Restore Factory Defaults
  wifiLabel = lv_label_create(ui_Debug_Tab);
  lv_obj_set_align(wifiLabel, LV_ALIGN_CENTER);
  lv_obj_set_y(wifiLabel, lv_pct(258));
  lv_obj_set_style_text_align(wifiLabel, LV_TEXT_ALIGN_CENTER, 0);
  wifiLabelUpdate();

  char b[64];
  snprintf(b, sizeof(b), "WiFi \"%s\" up, page at %s", WIFI_SSID, WiFi.softAPIP().toString().c_str());
  logEvent(b);
}

void webTick(uint32_t now) {
  dns.processNextRequest();
  web.handleClient();

  static uint32_t lastLabel = 0;
  if (now - lastLabel >= 2000) {
    lastLabel = now;
    wifiLabelUpdate();
  }
}
