#include <Arduino.h>
#include <math.h>
#include "esp_camera.h"
#include "img_converters.h"
#include <WiFi.h>
#include <WebServer.h>

// =====================================================
// AI THINKER ESP32-CAM / GC2145 CAMERA PINS
// =====================================================

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5

#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// =====================================================
// UART CONNECTION TO ATMEGA32
// =====================================================

#define ATMEGA_RX 13
#define ATMEGA_TX 14

HardwareSerial AtmegaSerial(2);

// =====================================================
// GLOBAL OBJECTS & VARIABLES
// =====================================================

const char* AP_SSID = "Scoutbot";
const char* AP_PASSWORD = "12345678";

WebServer server(80);

uint8_t* photoBuffer = NULL;
size_t photoLength = 0;

// True only for a short time after a fresh obstacle photo is captured.
bool obstacleDetected = false;
unsigned long obstacleDetectedAt = 0;

// Incremented only after a NEW photo has been successfully captured.
// The web UI uses this ID rather than a 2-second timer, preventing the
// previous obstacle image from being mistaken for the current one.
uint32_t photoSequence = 0;

// =====================================================
// GPS STATE
// -----------------------------------------------------
// The ATmega32 owns the actual NEO-6M GPS module (wired to its PC1
// pin) and forwards the last known fix to us over the same UART link
// used for motor commands, as a line: "G,<lat>,<lon>,<fixFlag>\n"
// sent right after each "O\n" obstacle notification.
// =====================================================

float lastLat = 0.0f;
float lastLon = 0.0f;
bool  lastFixValid = false; // true = the ATmega had a live satellite fix
bool  hasGpsData = false;   // true = at least one VALID fix has been received
unsigned long lastGpsFixAt = 0;
// A photo keeps its event location even when later live telemetry changes.
float photoLat = 0.0f, photoLon = 0.0f;
bool photoHasGps = false, photoFixValid = false;

// Live diagnostic telemetry, refreshed roughly once a second whenever
// the ATmega parses a GGA sentence (locked or not). This is what tells
// you WHY you might be seeing 0,0.000000 — e.g. satellites = 0 means
// the module just hasn't found a fix yet, while hasTelemetry staying
// false means no complete GGA telemetry has reached this ESP32.
uint8_t lastFixQuality = 0;
uint8_t lastSatellites = 0;
bool    hasTelemetry = false;
unsigned long lastTelemetryAt = 0;

// =====================================================
// CAMERA INITIALIZATION
// =====================================================

bool initCamera()
{
  camera_config_t config = {};

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;

  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;

  // Keep the known-working RGB565 path for GC2145, then convert the
  // freshly captured frame to JPEG for the browser.
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_LATEST;

  if (psramFound())
  {
    config.fb_location = CAMERA_FB_IN_PSRAM;
  }
  else
  {
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  Serial.println("Initializing camera...");

  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK)
  {
    Serial.printf("Camera initialization failed: 0x%x\n", err);
    return false;
  }

  sensor_t* sensor = esp_camera_sensor_get();
  Serial.printf("Camera initialized. PID = 0x%04X\n", sensor->id.PID);

  if (sensor->id.PID == 0x2145)
    Serial.println("Camera sensor = GC2145");
  else if (sensor->id.PID == 0x26)
    Serial.println("Camera sensor = OV2640");
  else
    Serial.println("Unexpected camera sensor detected - double check board type.");

  // Discard one warm-up frame after initialization.
  camera_fb_t* warmup = esp_camera_fb_get();
  if (warmup)
  {
    esp_camera_fb_return(warmup);
    Serial.println("Discarded warm-up frame");
  }

  return true;
}

// =====================================================
// CAPTURE PHOTO
// =====================================================
// The first frame acquired after an obstacle event is deliberately
// discarded. The second frame is encoded and published. This removes
// the common "one frame behind" behavior seen with a single camera
// framebuffer and ensures the event counter advances only for a fresh
// successfully encoded image.
// =====================================================

bool capturePhoto()
{
  Serial.println("Capturing fresh obstacle photo...");
  const float eventLat = lastLat, eventLon = lastLon;
  const bool eventHasGps = hasGpsData;
  const bool eventFixValid = lastFixValid && (millis() - lastGpsFixAt < 5000);

  camera_fb_t* staleFrame = esp_camera_fb_get();
  if (staleFrame)
  {
    esp_camera_fb_return(staleFrame);
    Serial.println("Discarded first post-trigger frame");
  }

  // Allow the sensor to advance to the next complete frame.
  delay(30);

  camera_fb_t* fb = esp_camera_fb_get();

  if (!fb)
  {
    Serial.println("Camera capture failed!");
    return false;
  }

  Serial.printf("Fresh frame: %dx%d, %d bytes\n", fb->width, fb->height, fb->len);

  uint8_t* jpgBuffer = NULL;
  size_t jpgLength = 0;

  bool converted = fmt2jpg(
    fb->buf,
    fb->len,
    fb->width,
    fb->height,
    PIXFORMAT_RGB565,
    70,
    &jpgBuffer,
    &jpgLength
  );

  esp_camera_fb_return(fb);

  if (!converted || jpgBuffer == NULL)
  {
    Serial.println("RGB565 -> JPEG conversion FAILED");
    if (jpgBuffer) free(jpgBuffer);
    return false;
  }

  // Replace the previous photo only after the NEW photo is fully ready.
  uint8_t* oldBuffer = photoBuffer;
  photoBuffer = jpgBuffer;
  photoLength = jpgLength;

  if (oldBuffer != NULL)
  {
    free(oldBuffer);
  }

  photoLat = eventLat;
  photoLon = eventLon;
  photoHasGps = eventHasGps;
  photoFixValid = eventFixValid;
  photoSequence++;
  obstacleDetected = true;
  obstacleDetectedAt = millis();

  Serial.printf("NEW JPEG ready: %d bytes | photo #%lu\n",
                photoLength, (unsigned long)photoSequence);

  return true;
}

// =====================================================
// ATMEGA LINK — LINE PROTOCOL
// -----------------------------------------------------
// The ATmega32 speaks two kinds of lines over this UART:
//   "O"                       -> obstacle detected, capture a photo
//   "G,<lat>,<lon>,<fixFlag>" -> latest GPS fix (fixFlag '1' or '0')
// =====================================================

// A no-fix message must never promote initial 0,0 into a real location.
// Keep the last verified position on loss of fix; 0,0 itself is legal
// when accompanied by a valid fix flag.
bool applyGpsUpdate(const String &latText, const String &lonText, const String &fixText)
{
  if (fixText != "0" && fixText != "1") return false;
  char *latEnd;
  char *lonEnd;
  double lat = strtod(latText.c_str(), &latEnd);
  double lon = strtod(lonText.c_str(), &lonEnd);
  if (latEnd == latText.c_str() || *latEnd != '\0' ||
      lonEnd == lonText.c_str() || *lonEnd != '\0' ||
      !isfinite(lat) || !isfinite(lon) ||
      lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) return false;

  lastFixValid = (fixText == "1");
  if (lastFixValid)
  {
    lastLat = lat;
    lastLon = lon;
    hasGpsData = true;
    lastGpsFixAt = millis();
  }
  return true;
}

void processAtmegaLine(const String &line)
{
  Serial.print("ATmega -> ESP32: ");
  Serial.println(line);

  if (line == "O")
  {
    Serial.println("OBSTACLE RECEIVED -> CAPTURING FRESH PHOTO");
    if (!capturePhoto())
    {
      Serial.println("Obstacle photo capture failed");
    }
  }
  else if (line.startsWith("G,"))
  {
    int firstComma  = line.indexOf(',', 2);
    int secondComma = (firstComma > 0) ? line.indexOf(',', firstComma + 1) : -1;

    if (firstComma > 0 && secondComma > firstComma)
    {
      String latStr = line.substring(2, firstComma);
      String lonStr = line.substring(firstComma + 1, secondComma);
      String fixStr = line.substring(secondComma + 1);

      if (!applyGpsUpdate(latStr, lonStr, fixStr))
      {
        Serial.println("Invalid GPS values, ignored");
        return;
      }

      Serial.printf("GPS update: lat=%.6f lon=%.6f fix=%d\n",
                    lastLat, lastLon, lastFixValid ? 1 : 0);
    }
    else
    {
      Serial.println("Malformed GPS line, ignored");
    }
  }
  else if (line.startsWith("T,"))
  {
    // T,<fixQuality>,<satellites>,<lat>,<lon>,<fixFlag>
    int c1 = line.indexOf(',', 2);
    int c2 = (c1 > 0) ? line.indexOf(',', c1 + 1) : -1;
    int c3 = (c2 > 0) ? line.indexOf(',', c2 + 1) : -1;
    int c4 = (c3 > 0) ? line.indexOf(',', c3 + 1) : -1;

    if (c1 > 0 && c2 > c1 && c3 > c2 && c4 > c3)
    {
      if (!applyGpsUpdate(line.substring(c2 + 1, c3),
                          line.substring(c3 + 1, c4), line.substring(c4 + 1)))
      {
        Serial.println("Invalid telemetry coordinates, ignored");
        return;
      }
      lastFixQuality  = (uint8_t)line.substring(2, c1).toInt();
      lastSatellites  = (uint8_t)line.substring(c1 + 1, c2).toInt();
      hasTelemetry    = true;
      lastTelemetryAt = millis();

      Serial.printf("GPS telemetry: fixQuality=%d satellites=%d lat=%.6f lon=%.6f valid=%d\n",
                    lastFixQuality, lastSatellites, lastLat, lastLon, lastFixValid ? 1 : 0);
    }
    else
    {
      Serial.println("Malformed telemetry line, ignored");
    }
  }
}

// =====================================================
// HOME PAGE
// =====================================================

void handleRoot()
{
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>Scoutbot Control</title>
<style>
:root {
  --bg: #07111f;
  --panel: rgba(15, 28, 49, 0.86);
  --panel-2: rgba(19, 36, 61, 0.82);
  --line: rgba(255,255,255,.09);
  --text: #eef6ff;
  --muted: #91a4bc;
  --accent: #55d6ff;
  --accent-2: #8b7cff;
  --danger: #ff5d73;
  --success: #45df9b;
  --shadow: 0 18px 50px rgba(0,0,0,.28);
}
* { box-sizing: border-box; }
body {
  margin: 0;
  min-height: 100vh;
  color: var(--text);
  font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
  background:
    radial-gradient(circle at 10% 0%, rgba(85,214,255,.14), transparent 32%),
    radial-gradient(circle at 90% 8%, rgba(139,124,255,.14), transparent 32%),
    var(--bg);
}
button { font: inherit; }
.app {
  width: min(1180px, calc(100% - 28px));
  margin: 0 auto;
  padding: 22px 0 34px;
}
.topbar {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 16px;
  margin-bottom: 18px;
}
.brand {
  display:flex; align-items:center; gap:12px;
}
.brandMark {
  width:44px; height:44px; border-radius:14px;
  display:grid; place-items:center;
  background: linear-gradient(135deg, rgba(85,214,255,.95), rgba(139,124,255,.95));
  color:#06101c; font-weight:900; box-shadow: var(--shadow);
}
.brand h1 { margin:0; font-size:22px; letter-spacing:.2px; }
.brand p { margin:3px 0 0; color:var(--muted); font-size:12px; }
.statusPill {
  display:flex; align-items:center; gap:8px;
  border:1px solid var(--line); border-radius:999px;
  padding:9px 13px; background:rgba(255,255,255,.035);
  color:#c9d8ea; font-size:13px;
}
.dot { width:8px; height:8px; border-radius:50%; background:var(--success); box-shadow:0 0 14px rgba(69,223,155,.7); }
.grid {
  display:grid;
  grid-template-columns: 1.05fr .95fr;
  gap:18px;
}
.panel {
  background: linear-gradient(180deg, rgba(17,32,55,.92), rgba(10,22,39,.86));
  border:1px solid var(--line);
  border-radius:24px;
  box-shadow:var(--shadow);
  overflow:hidden;
}
.panelHead {
  display:flex; align-items:center; justify-content:space-between; gap:10px;
  padding:18px 20px 14px; border-bottom:1px solid var(--line);
}
.panelTitle { margin:0; font-size:15px; }
.panelSub { margin:4px 0 0; color:var(--muted); font-size:12px; }
.panelBody { padding:20px; }
.heroStatus {
  display:flex; align-items:center; justify-content:space-between; gap:16px;
  padding:16px; border-radius:18px; background:rgba(255,255,255,.035);
  border:1px solid var(--line); margin-bottom:20px;
}
.heroStatus strong { display:block; font-size:18px; }
.heroStatus span { color:var(--muted); font-size:12px; }
.badge { padding:7px 10px; border-radius:999px; font-size:11px; font-weight:800; letter-spacing:.5px; }
.badge.ready { background:rgba(69,223,155,.1); color:var(--success); }
.badge.alert { background:rgba(255,93,115,.12); color:var(--danger); }
.controlPad {
  display:grid;
  grid-template-columns:repeat(3, 1fr);
  gap:12px; max-width:430px; margin:0 auto 24px;
}
.controlBtn {
  aspect-ratio:1.25; border:1px solid var(--line); border-radius:20px;
  color:var(--text); background:linear-gradient(180deg, rgba(255,255,255,.065), rgba(255,255,255,.025));
  cursor:pointer; transition:transform .12s ease, border-color .12s ease, background .12s ease;
  box-shadow: inset 0 1px 0 rgba(255,255,255,.03);
  user-select:none; -webkit-user-select:none; touch-action:manipulation;
}
.controlBtn:hover { border-color:rgba(85,214,255,.35); }
.controlBtn:active, .controlBtn.active { transform:translateY(2px) scale(.985); background:rgba(85,214,255,.12); border-color:rgba(85,214,255,.5); }
.controlBtn .icon { display:block; font-size:30px; line-height:1; margin-bottom:6px; }
.controlBtn small { color:var(--muted); font-size:10px; }
.stopBtn { background:linear-gradient(135deg, rgba(255,93,115,.96), rgba(209,58,80,.9)); border-color:transparent; }
.stopBtn small { color:#ffecef; }
.speedTitle { display:flex; justify-content:space-between; align-items:center; margin:0 0 10px; }
.speedTitle span { color:var(--muted); font-size:12px; }
.speedValue { color:var(--accent); font-weight:800; font-size:13px; }
.speedGrid { display:grid; grid-template-columns:repeat(5,1fr); gap:8px; }
.speedBtn {
  border:1px solid var(--line); background:rgba(255,255,255,.035); color:#cdd9e8;
  padding:11px 6px; border-radius:13px; cursor:pointer; font-weight:800; font-size:12px;
}
.speedBtn:hover, .speedBtn.active { color:#06101c; background:linear-gradient(135deg,var(--accent),#88e6ff); border-color:transparent; }
.photoWrap {
  background:#040a12; border-radius:18px; overflow:hidden; aspect-ratio:4/3;
  border:1px solid var(--line); position:relative; display:grid; place-items:center;
}
#photo { width:100%; height:100%; object-fit:cover; display:none; }
.emptyState { text-align:center; color:var(--muted); padding:30px; }
.emptyIcon { font-size:38px; margin-bottom:8px; opacity:.8; }
.photoMeta { display:flex; align-items:center; justify-content:space-between; gap:10px; margin-top:12px; flex-wrap:wrap; }
.photoMeta span { color:var(--muted); font-size:12px; }
.download {
  display:none; text-decoration:none; color:#06101c; font-weight:900; font-size:12px;
  padding:10px 13px; border-radius:12px; background:linear-gradient(135deg,var(--accent),#8ef0ff);
}
.gpsRow { margin-top:8px; }
.gpsRow #gpsText { font-size:12px; color:var(--muted); }
#mapLink { padding:8px 12px; font-size:11px; }
.eventLog { margin-top:16px; padding:13px 14px; border:1px solid var(--line); border-radius:15px; background:rgba(255,255,255,.025); }
.eventLog b { font-size:12px; }
.eventLog p { margin:5px 0 0; color:var(--muted); font-size:12px; }
.footer {
  margin-top:16px; color:#667b95; font-size:11px; display:flex; justify-content:space-between; gap:12px; flex-wrap:wrap;
}
.flash { animation: flash .65s ease; }
@keyframes flash { 0%{box-shadow:0 0 0 0 rgba(255,93,115,.7)} 100%{box-shadow:0 0 0 18px rgba(255,93,115,0)} }
@media (max-width: 820px) {
  .grid { grid-template-columns:1fr; }
}
@media (max-width: 520px) {
  .app { width:min(100% - 16px, 1180px); padding-top:12px; }
  .panelBody { padding:15px; }
  .controlPad { gap:8px; }
  .controlBtn { border-radius:17px; }
  .controlBtn .icon { font-size:25px; }
  .speedGrid { gap:5px; }
}
</style>
</head>
<body>
<div class="app">
  <header class="topbar">
    <div class="brand">
      <div class="brandMark">S</div>
      <div><h1>Scoutbot</h1><p>Obstacle-aware robotic rover</p></div>
    </div>
    <div class="statusPill"><span class="dot"></span><span id="connectionText">Local control online</span></div>
  </header>

  <main class="grid">
    <section class="panel">
      <div class="panelHead">
        <div><h2 class="panelTitle">Vehicle control</h2><p class="panelSub">Directional control and motor speed</p></div>
        <span class="badge ready" id="modeBadge">READY</span>
      </div>
      <div class="panelBody">
        <div class="heroStatus" id="statusCard">
          <div><strong id="status">Ready</strong><span id="statusSub">Awaiting movement command</span></div>
          <span class="badge ready" id="obstacleBadge">CLEAR</span>
        </div>

        <div class="controlPad" aria-label="Vehicle controls">
          <div></div>
          <button class="controlBtn" onclick="sendDrive('F', this, 'Moving forward')"><span class="icon">↑</span><small>FORWARD</small></button>
          <div></div>
          <!-- Left/right commands are intentionally swapped here to match the actual vehicle response. -->
          <button class="controlBtn" onclick="sendDrive('R', this, 'Turning left')"><span class="icon">←</span><small>LEFT</small></button>
          <button class="controlBtn stopBtn" onclick="sendCommand('S', 'Stopped')"><span class="icon">■</span><small>STOP</small></button>
          <button class="controlBtn" onclick="sendDrive('L', this, 'Turning right')"><span class="icon">→</span><small>RIGHT</small></button>
          <div></div>
          <button class="controlBtn" onclick="sendDrive('B', this, 'Moving backward')"><span class="icon">↓</span><small>BACKWARD</small></button>
          <div></div>
        </div>

        <div class="speedTitle"><span>Motor speed preset</span><span class="speedValue" id="speedValue">40%</span></div>
        <div class="speedGrid">
          <button class="speedBtn" data-speed="1" onclick="setSpeed('1', '20%', this)">20%</button>
          <button class="speedBtn active" data-speed="2" onclick="setSpeed('2', '40%', this)">40%</button>
          <button class="speedBtn" data-speed="3" onclick="setSpeed('3', '60%', this)">60%</button>
          <button class="speedBtn" data-speed="4" onclick="setSpeed('4', '80%', this)">80%</button>
          <button class="speedBtn" data-speed="5" onclick="setSpeed('5', '100%', this)">100%</button>
        </div>

        <div class="eventLog">
          <b>Control tip</b>
          <p>Use W/A/S/D or the arrow keys for quick control. Spacebar sends STOP.</p>
        </div>
      </div>
    </section>

    <section class="panel">
      <div class="panelHead">
        <div><h2 class="panelTitle">Obstacle snapshot</h2><p class="panelSub">Latest fresh image captured by the GC2145</p></div>
        <span class="badge ready" id="photoBadge">WAITING</span>
      </div>
      <div class="panelBody">
        <div class="photoWrap">
          <img id="photo" alt="Latest obstacle snapshot">
          <div class="emptyState" id="emptyState"><div class="emptyIcon">◌</div><div>No obstacle snapshot yet</div><div style="font-size:11px;margin-top:5px">A new image will appear automatically after detection.</div></div>
        </div>
        <div class="photoMeta">
          <span id="photoInfo">No new photo</span>
          <a class="download" id="downloadLink" download="scoutbot_obstacle.jpg">Download image</a>
        </div>
        <div class="photoMeta gpsRow">
          <span id="gpsText">GPS: waiting for fix…</span>
          <a class="download" id="mapLink" href="#" target="_blank" rel="noopener" style="display:none;">Open in Maps</a>
        </div>
        <div class="photoMeta gpsRow">
          <span id="gpsDiag">GPS module: not receiving data yet</span>
        </div>
      </div>
    </section>
  </main>

  <div class="footer"><span>Scoutbot local AP • 192.168.4.1</span><span>ESP32-CAM ↔ ATmega32</span></div>
</div>

<script>
let initialized = false;
let lastPhotoId = 0;
let pollBusy = false;
let statusResetTimer = null;

function setStatus(title, subtitle, alert=false) {
  document.getElementById('status').textContent = title;
  document.getElementById('statusSub').textContent = subtitle;
  const badge = document.getElementById('obstacleBadge');
  badge.textContent = alert ? 'OBSTACLE' : 'CLEAR';
  badge.className = 'badge ' + (alert ? 'alert' : 'ready');
  if (statusResetTimer) clearTimeout(statusResetTimer);
  if (alert) {
    document.getElementById('statusCard').classList.remove('flash');
    void document.getElementById('statusCard').offsetWidth;
    document.getElementById('statusCard').classList.add('flash');
    statusResetTimer = setTimeout(() => setStatus('Ready', 'Vehicle standing by', false), 3500);
  }
}

function sendCommand(cmd, label) {
  fetch('/cmd?c=' + encodeURIComponent(cmd), { cache: 'no-store' })
    .then(() => setStatus(label || ('Command ' + cmd), 'Command sent to ATmega32'))
    .catch(() => setStatus('Connection error', 'Could not send command', true));
}

function sendDrive(cmd, button, label) {
  document.querySelectorAll('.controlBtn').forEach(b => b.classList.remove('active'));
  if (button) button.classList.add('active');
  sendCommand(cmd, label);
}

function setSpeed(cmd, label, button) {
  document.querySelectorAll('.speedBtn').forEach(b => b.classList.remove('active'));
  button.classList.add('active');
  document.getElementById('speedValue').textContent = label;
  sendCommand(cmd, 'Speed set to ' + label);
}

function showPhoto(photoId) {
  const photoUrl = '/photo?id=' + photoId + '&t=' + Date.now();
  const img = document.getElementById('photo');
  const empty = document.getElementById('emptyState');
  const dl = document.getElementById('downloadLink');
  const badge = document.getElementById('photoBadge');

  img.onload = function() {
    img.style.display = 'block';
    empty.style.display = 'none';
    dl.href = photoUrl;
    dl.style.display = 'inline-block';
    badge.textContent = 'NEW';
    badge.className = 'badge alert';
    document.getElementById('photoInfo').textContent = 'Obstacle snapshot #' + photoId;
  };
  img.onerror = function() {
    document.getElementById('photoInfo').textContent = 'Photo fetch failed — retrying automatically';
  };
  img.src = photoUrl;
}

function updateGps(gps, isPhoto=false) {
  const text = document.getElementById('gpsText');
  const link = document.getElementById('mapLink');

  if (gps && gps.has) {
    const staleness = gps.valid ? '' : ' (last known fix)';
    text.textContent = (isPhoto ? 'Photo GPS: ' : 'GPS: ') + gps.lat.toFixed(6) + ', ' + gps.lon.toFixed(6) + staleness;
    link.href = 'https://www.google.com/maps?q=' + gps.lat + ',' + gps.lon;
    link.style.display = 'inline-block';
  } else {
    text.textContent = isPhoto ? 'Photo GPS: no location was available at capture' : 'GPS: waiting for fix…';
    link.style.display = 'none';
  }

}

// Tells you WHY the coordinates might still be 0,0:
//  - no telemetry -> no complete GGA has reached the ESP32; possible
//                    wiring, baud, software UART timing, or configuration issue
//  - receiving, 0 satellites -> reception is fine, module has no fix
//                       yet (needs clear sky + time)
//  - receiving, N satellites, locked -> working correctly
function updateGpsDiag(gps) {
  const diag = document.getElementById('gpsDiag');
  if (!diag) return;

  if (!gps || !gps.receiving) {
    diag.textContent = 'GPS: no recent valid GGA data — check wiring, baud and receiver timing';
    return;
  }

  if (gps.valid) {
    diag.textContent = 'GPS module: locked — ' + gps.satellites + ' satellite(s), fix quality ' + gps.fixQuality;
  } else {
    diag.textContent = 'GPS: receiving GGA, no usable current fix — ' + gps.satellites + ' satellite(s), quality ' + gps.fixQuality;
  }
}

async function checkObstacle() {
  if (pollBusy) return;
  pollBusy = true;
  try {
    const response = await fetch('/status?since=' + lastPhotoId, { cache: 'no-store' });
    const data = await response.json();

    updateGps(data.photoId ? data.photoGps : data.gps, !!data.photoId);
    updateGpsDiag(data.gps);

    if (!initialized) {
      // Do not treat an old image from before this page load as a new detection.
      lastPhotoId = data.photoId || 0;
      initialized = true;
      return;
    }

    if (data.newPhoto && data.photoId > lastPhotoId) {
      lastPhotoId = data.photoId;
      setStatus('Obstacle detected', 'Fresh image captured — automatic avoidance engaged', true);
      showPhoto(data.photoId);
    }
  } catch (e) {
    document.getElementById('connectionText').textContent = 'Connection issue';
  } finally {
    pollBusy = false;
  }
}

// Keyboard control for desktop testing.
const heldKeys = new Set();
document.addEventListener('keydown', function(e) {
  if (e.repeat) return;
  const k = e.key.toLowerCase();
  if (k === 'w' || e.key === 'ArrowUp') { e.preventDefault(); heldKeys.add(k); sendCommand('F', 'Moving forward'); }
  else if (k === 'a' || e.key === 'ArrowLeft') { e.preventDefault(); heldKeys.add(k); sendCommand('R', 'Turning left'); }
  else if (k === 'd' || e.key === 'ArrowRight') { e.preventDefault(); heldKeys.add(k); sendCommand('L', 'Turning right'); }
  else if (k === 's' || e.key === 'ArrowDown') { e.preventDefault(); heldKeys.add(k); sendCommand('B', 'Moving backward'); }
  else if (e.key === ' ') { e.preventDefault(); sendCommand('S', 'Stopped'); }
});

document.addEventListener('keyup', function(e) {
  const k = e.key.toLowerCase();
  if (heldKeys.has(k)) {
    heldKeys.delete(k);
    sendCommand('S', 'Stopped');
  }
});

checkObstacle();
setInterval(checkObstacle, 250);
</script>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", html);
}

// =====================================================
// PHONE COMMAND
// =====================================================

void handleCommand()
{
  if (!server.hasArg("c"))
  {
    server.send(400, "text/plain", "Missing command");
    return;
  }

  String cmd = server.arg("c");

  Serial.print("Phone command: ");
  Serial.println(cmd);

  if (cmd.length() > 0)
  {
    char c = cmd.charAt(0);
    AtmegaSerial.write(c);
  }

  server.send(200, "text/plain", "OK");
}

// =====================================================
// SEND PHOTO
// =====================================================

void handlePhoto()
{
  if (photoBuffer == NULL || photoLength == 0)
  {
    server.send(404, "text/plain", "No photo available");
    return;
  }

  // The id is deliberately accepted as a cache-busting/event identity.
  // The image buffer itself is always the most recently completed photo.
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
  server.sendHeader("Content-Disposition", "inline; filename=scoutbot_obstacle.jpg");

  server.send_P(
    200,
    "image/jpeg",
    (const char*)photoBuffer,
    photoLength
  );
}

// =====================================================
// STATUS
// =====================================================

void handleStatus()
{
  // Keep the obstacle indicator temporary; the photo sequence itself
  // remains monotonic and is the authoritative event identifier.
  if (obstacleDetected && (millis() - obstacleDetectedAt > 4000))
  {
    obstacleDetected = false;
  }

  uint32_t since = 0;
  if (server.hasArg("since"))
  {
    since = (uint32_t)strtoul(server.arg("since").c_str(), NULL, 10);
  }

  String json = "{\"obstacle\":";
  json += (obstacleDetected ? "true" : "false");
  json += ",\"photoId\":";
  json += String(photoSequence);
  json += ",\"newPhoto\":";
  json += (photoSequence != since ? "true" : "false");
  bool telemetryFresh = hasTelemetry && (millis() - lastTelemetryAt < 5000);

  json += ",\"gps\":{\"has\":";
  json += (hasGpsData ? "true" : "false");
  json += ",\"lat\":";
  json += String(lastLat, 6);
  json += ",\"lon\":";
  json += String(lastLon, 6);
  json += ",\"valid\":";
  bool fixFresh = lastFixValid && (millis() - lastGpsFixAt < 5000);
  json += (fixFresh ? "true" : "false");
  json += ",\"fixQuality\":";
  json += String(lastFixQuality);
  json += ",\"satellites\":";
  json += String(lastSatellites);
  json += ",\"receiving\":";
  json += (telemetryFresh ? "true" : "false");
  json += "},\"photoGps\":{\"has\":";
  json += (photoHasGps ? "true" : "false");
  json += ",\"lat\":";
  json += String(photoLat, 6);
  json += ",\"lon\":";
  json += String(photoLon, 6);
  json += ",\"valid\":";
  json += (photoFixValid ? "true" : "false");
  json += "}}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("==============================");
  Serial.println("SCOUTBOT ESP32-CAM");
  Serial.println("==============================");

  AtmegaSerial.setRxBufferSize(1024);
  AtmegaSerial.begin(4800, SERIAL_8N1, ATMEGA_RX, ATMEGA_TX);
  Serial.println("ATmega UART started");

  if (!initCamera())
  {
    Serial.println("Camera initialization failed!");
    return;
  }

  // ===================================================
  // WIFI ACCESS POINT SETUP WITH STABLE CONFIG
  // ===================================================

  WiFi.mode(WIFI_AP);

  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);

  WiFi.softAPConfig(local_IP, gateway, subnet);

  bool apStarted = WiFi.softAP(AP_SSID, AP_PASSWORD, 6, 0, 4);

  if (apStarted) {
    Serial.println("WiFi AP started successfully");
  } else {
    Serial.println("WiFi AP setup FAILED!");
  }

  IPAddress IP = WiFi.softAPIP();

  WiFi.setTxPower(WIFI_POWER_11dBm);
  WiFi.setSleep(false);

  Serial.println();
  Serial.print("SSID: ");
  Serial.println(AP_SSID);
  Serial.print("Password: ");
  Serial.println(AP_PASSWORD);
  Serial.print("IP address: ");
  Serial.println(IP);

  // ===================================================
  // HTTP SERVER
  // ===================================================

  server.on("/", HTTP_GET, handleRoot);
  server.on("/cmd", HTTP_GET, handleCommand);
  server.on("/photo", HTTP_GET, handlePhoto);
  server.on("/status", HTTP_GET, handleStatus);

  server.begin();
  Serial.println("HTTP server started");
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
  server.handleClient();

  static String rxLine = "";

  while (AtmegaSerial.available())
  {
    char c = AtmegaSerial.read();

    if (c == '\n')
    {
      processAtmegaLine(rxLine);
      rxLine = "";
    }
    else if (c != '\r')
    {
      rxLine += c;

      // Guard against a corrupted/never-terminated line eating memory.
      if (rxLine.length() > 96)
      {
        rxLine = "";
      }
    }
  }
}
