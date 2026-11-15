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
static uint32_t webLogT[WEB_LOG_LINES];   // millis() of each line: the page's charts use it
static uint32_t webLogSeq = 0;       // lines ever logged; line n is in webLogBuf[n % WEB_LOG_LINES]

static uint32_t noteId = 0, msgId = 0;
static char     noteText[48]  = "";
static char     msgTitle[32]  = "", msgText[200] = "";
static char     nanoVersion[16] = "";

static lv_obj_t *wifiLabel = NULL;   // Debug tab: how to open the page

// ---- The Nano's Status lines, remembered so every phone can use them (reach
// map, level tolerance...). Key = how the line starts; must = a second word
// that tells a Status line from a command's echo ("# Kp 300.00").
#define ST_REACH 9
static const char *const STATUS_KEY[][2] = {
  {"Cal ", ""}, {"PitchOffset ", "RollOffset"}, {"MinSpeed ", "MaxSpeed"}, {"Kp ", " Ki "},
  {"At ", "kg:"}, {"Zero ", ""}, {"Alpha ", "LevelTol"}, {"Gyro ", "Ramp"}, {"Axes ", "base"},
  {"Reach ", ""}, {"LaserOffset ", "TestMass"}, {"Tire mm ", ""}, {"AutoStop ", "MassComp"},
};
#define STATUS_N (sizeof(STATUS_KEY) / sizeof(STATUS_KEY[0]))
static char     statusLine[STATUS_N][80];
static uint32_t statusAskedMs = 0;

// ---- Debug streams. The Nano's link (9600 baud) carries about one 10-per-
// second stream comfortably, so a stream is only on while some phone is
// showing it: each poll says what it shows (want=level,sensor,scale), and a
// stream nobody has asked for in STREAM_HOLD_MS is switched off again.
#define STREAM_N       3
#define STREAM_HOLD_MS 3000
static const char *const STREAM_CMD[STREAM_N] = {"LevelDebug", "SensorDebug", "ScaleDebug"};
static const char *const STREAM_WANT[STREAM_N] = {"level", "sensor", "scale"};
static uint32_t streamWantMs[STREAM_N] = {0, 0, 0};
static bool     streamOn[STREAM_N] = {false, false, false};   // switched on by the web page

// ------------------------------------------------------------ hooks called from ui.ino
// kind: '>' sent to the Nano, '<' heard from the Nano, '!' event
void webLog(char kind, const char *text) {
  uint32_t i = webLogSeq % WEB_LOG_LINES;
  char *d = webLogBuf[i];
  d[0] = kind;
  strncpy(d + 1, text, WEB_LOG_LEN - 2);
  d[WEB_LOG_LEN - 1] = 0;
  webLogT[i] = millis();
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

static bool statusLineOk(uint8_t k, const char *body) {
  const char *rest = body + strlen(STATUS_KEY[k][0]);
  if (STATUS_KEY[k][1][0] && !strstr(body, STATUS_KEY[k][1])) return false;
  if (k == ST_REACH) {                               // "Reach a,b c,d e,f g,h" (FindReach also prints single corners)
    uint8_t commas = 0;
    for (const char *p = rest; *p; p++) if (*p == ',') commas++;
    return commas == 4;
  }
  if (!STATUS_KEY[k][1][0] && k != 11) return (*rest >= '0' && *rest <= '9') || *rest == '-';   // "Zero saved" is not one
  return true;
}

// Every valid line from the Nano: "Ready V0.7.0" (restart), "# V0.7.0" and
// the other Status lines.
void webNanoLine(const char *line) {
  const char *v = NULL;
  if (!strncmp(line, "Ready V", 7)) {
    v = line + 7;
    for (uint8_t i = 0; i < STREAM_N; i++) streamOn[i] = false;   // a restarted Nano has its debug streams off
  }
  else if (!strncmp(line, "# V", 3) && line[3] >= '0' && line[3] <= '9') v = line + 3;
  if (v) { strncpy(nanoVersion, v, sizeof(nanoVersion) - 1); return; }

  if (strncmp(line, "# ", 2)) return;
  const char *body = line + 2;
  for (uint8_t k = 0; k < STATUS_N; k++) {
    if (strncmp(body, STATUS_KEY[k][0], strlen(STATUS_KEY[k][0])) || !statusLineOk(k, body)) continue;
    strncpy(statusLine[k], body, sizeof(statusLine[k]) - 1);
    return;
  }
}

// Ask the Nano for Status (it only answers when idle, and it is long: not
// more often than every 15 s unless forced).
static void webAskStatus(bool force) {
  uint32_t now = millis();
  if (!linkUp || waiting != WAIT_NONE) return;
  if (!force && statusAskedMs && now - statusAskedMs < 15000) return;
  statusAskedMs = now | 1;
  nanoSend("Status");
}

// The link has just come up. The Nano only says its version when it starts
// ("Ready V...") or in Status; if the screen started after the Nano it has
// heard neither, so ask (this also fills in the reach area etc.).
void webLinkUp() {
  if (!nanoVersion[0] || !statusLine[ST_REACH][0]) webAskStatus(true);
}

// FindReach has saved a new reach area: read it again.
void webReachChanged() {
  statusLine[ST_REACH][0] = 0;
  webAskStatus(true);
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

  // What this page is showing: keep those debug streams on (webStreamsTick)
  String want = web.arg("want");
  for (uint8_t i = 0; i < STREAM_N; i++) if (want.indexOf(STREAM_WANT[i]) >= 0) streamWantMs[i] = now | 1;
  if (want.indexOf("sensor") >= 0 && !statusLine[ST_REACH][0]) webAskStatus(false);   // reach map needs it

  String o;
  o.reserve(4000);
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
  jInt(o, "laserOffIn", laserAutoOffAt ? (long)((int32_t)(laserAutoOffAt - now) / 1000 + 1) : 0);
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
    jNum(o, "rimMm", tire.rimMm, 1);
    jNum(o, "sideMm", tire.sideMm, 1);
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
  jKey(o, "lt"); o += '[';                           // when each line was logged, ms since start
  for (uint32_t n = from; n < to; n++) {
    if (n > from) o += ',';
    o += webLogT[n % WEB_LOG_LINES];
  }
  o += ']';
  jInt(o, "seq", to);
  jInt(o, "now", now);

  // the Nano's Status lines and which debug streams are on
  jKey(o, "status"); o += '[';
  for (uint8_t k = 0; k < STATUS_N; k++) {
    if (k) o += ',';
    jStr(o, statusLine[k]);
  }
  o += ']';
  jKey(o, "streams"); o += '[';
  for (uint8_t i = 0; i < STREAM_N; i++) { if (i) o += ','; o += streamOn[i] ? '1' : '0'; }
  o += ']';
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
  else if (a == "laser")         { doLaserToggle(); }
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
  else if (a == "speed") {                            // Enter Speed: min, max
    dragSlider(ui_MinSpeedSlider, web.arg("min").toInt());
    dragSlider(ui_MaxSpeedSlider, web.arg("max").toInt());
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

  // Debug tab, just under Restore Factory Defaults. Placed from the button's
  // final position: setupExtraWidgets() moves the tab's widgets down after
  // creating them, so a fixed position would land on the buttons.
  wifiLabel = lv_label_create(ui_Debug_Tab);
  lv_obj_set_width(wifiLabel, lv_pct(100));          // full width, so a longer text stays centred
  lv_obj_set_style_text_align(wifiLabel, LV_TEXT_ALIGN_CENTER, 0);
  wifiLabelUpdate();
  lv_obj_update_layout(ui_Debug_Tab);
  lv_obj_align_to(wifiLabel, restoreBtn, LV_ALIGN_OUT_BOTTOM_MID, 0, 12);

  char b[64];
  snprintf(b, sizeof(b), "WiFi \"%s\" up, page at %s", WIFI_SSID, WiFi.softAPIP().toString().c_str());
  logEvent(b);
}

// Switch each debug stream on while a page shows it, off when none does
static void webStreamsTick(uint32_t now) {
  static uint32_t last = 0;
  if (now - last < 250) return;
  last = now;
  if (!linkUp || quiet) return;
  for (uint8_t i = 0; i < STREAM_N; i++) {
    bool want = streamWantMs[i] && now - streamWantMs[i] < STREAM_HOLD_MS;
    if (want == streamOn[i]) continue;
    streamOn[i] = want;
    nanoSend("%s%s", STREAM_CMD[i], want ? "On" : "Off");
  }
}

void webTick(uint32_t now) {
  dns.processNextRequest();
  web.handleClient();
  webStreamsTick(now);

  static uint32_t lastLabel = 0;
  if (now - lastLabel >= 2000) {
    lastLabel = now;
    wifiLabelUpdate();
  }
}
