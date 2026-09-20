#include <Arduino.h>
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

#define Y9_GPIO_NUM       39
#define Y8_GPIO_NUM       36
#define Y7_GPIO_NUM       21
#define Y6_GPIO_NUM       19
#define Y5_GPIO_NUM       18
#define Y4_GPIO_NUM        5
#define Y3_GPIO_NUM        4
#define Y2_GPIO_NUM       34

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

bool obstacleDetected = false;

// =====================================================
// CAMERA INITIALIZATION
// =====================================================

bool initCamera()
{
  camera_config_t config;

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

  // GC2145 outputs RGB565 raw frame buffer
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;

  if (psramFound())
  {
    config.fb_location = CAMERA_FB_IN_PSRAM;
  }
  else
  {
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  Serial.println("Initializing camera...");

  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK)
  {
    Serial.printf("Camera initialization failed: 0x%x\n", err);
    return false;
  }

  sensor_t* sensor = esp_camera_sensor_get();
  Serial.printf("Camera initialized. PID = 0x%04X\n", sensor->id.PID);

  return true;
}

// =====================================================
// CAPTURE PHOTO
// =====================================================

bool capturePhoto()
{
  Serial.println("Capturing photo...");

  camera_fb_t* fb = esp_camera_fb_get();

  if (!fb)
  {
    Serial.println("Camera capture failed!");
    return false;
  }

  Serial.printf("Captured: %dx%d, %d bytes\n", fb->width, fb->height, fb->len);

  uint8_t* jpgBuffer = NULL;
  size_t jpgLength = 0;

  // Convert RGB565 -> JPEG
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

  // Double buffer swap
  uint8_t* oldBuffer = photoBuffer;

  photoBuffer = jpgBuffer;
  photoLength = jpgLength;

  if (oldBuffer != NULL)
  {
    free(oldBuffer);
  }

  Serial.printf("JPEG created: %d bytes\n", photoLength);

  obstacleDetected = true;

  return true;
}

// =====================================================
// HOME PAGE
// =====================================================

void handleRoot()
{
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Scoutbot</title>
<style>
body {
  font-family: Arial;
  text-align: center;
  background: #eeeeee;
}
button {
  width: 120px;
  height: 65px;
  margin: 7px;
  font-size: 22px;
  border-radius: 12px;
}
.stop {
  background: #ff5555;
}
.speed {
  width: 80px;
  height: 55px;
  font-size: 16px;
  background: #cfe8ff;
}
.download {
  background: #55cc77;
  color: white;
}
#photo {
  max-width: 95%;
  margin-top: 20px;
}
</style>
</head>
<body>

<h1>Scoutbot</h1>
<h2>Control</h2>

<div>
  <button onclick="sendCommand('F')">Forward</button>
</div>
<div>
  <button onclick="sendCommand('L')">Left</button>
  <button class="stop" onclick="sendCommand('S')">STOP</button>
  <button onclick="sendCommand('R')">Right</button>
</div>
<div>
  <button onclick="sendCommand('B')">Backward</button>
</div>

<h2>Speed</h2>
<div>
  <button class="speed" onclick="sendCommand('1')">20%</button>
  <button class="speed" onclick="sendCommand('2')">40%</button>
  <button class="speed" onclick="sendCommand('3')">60%</button>
  <button class="speed" onclick="sendCommand('4')">80%</button>
  <button class="speed" onclick="sendCommand('5')">100%</button>
</div>

<h2 id="status">Status: Ready</h2>
<img id="photo" style="display:none;">
<br>
<a id="downloadLink" style="display:none;" download="scoutbot_photo.jpg">
  <button class="download">Download Photo</button>
</a>

<script>
function sendCommand(cmd) {
  fetch("/cmd?c=" + cmd);
  document.getElementById("status").innerHTML = "Command: " + cmd;
}

function checkObstacle() {
  fetch("/status")
    .then(response => response.text())
    .then(data => {
      if(data == "OBSTACLE") {
        document.getElementById("status").innerHTML = "OBSTACLE DETECTED";

        // Show the warning for 2 seconds, then go back to normal
        setTimeout(function() {
          document.getElementById("status").innerHTML = "Status: Ready";
        }, 2000);

        let photoUrl = "/photo?t=" + new Date().getTime();

        let img = document.getElementById("photo");
        img.src = photoUrl;
        img.style.display = "block";

        let dl = document.getElementById("downloadLink");
        dl.href = photoUrl;
        dl.style.display = "inline-block";
      }
    });
}

setInterval(checkObstacle, 1000);
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

  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Content-Disposition", "inline; filename=scoutbot_photo.jpg");

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
  if (obstacleDetected)
  {
    server.send(200, "text/plain", "OBSTACLE");
    obstacleDetected = false;
  }
  else
  {
    server.send(200, "text/plain", "OK");
  }
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

  // HardwareSerial(2) to ATmega32
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

  // Configure IP routing before launching AP
  WiFi.softAPConfig(local_IP, gateway, subnet);

  // Channel 6, visible (0), max 4 clients
  bool apStarted = WiFi.softAP(AP_SSID, AP_PASSWORD, 6, 0, 4);

  if (apStarted) {
    Serial.println("WiFi AP started successfully");
  } else {
    Serial.println("WiFi AP setup FAILED!");
  }

  IPAddress IP = WiFi.softAPIP();

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

  while (AtmegaSerial.available())
  {
    char c = AtmegaSerial.read();

    Serial.print("ATmega -> ESP32: ");
    Serial.println(c);

    if (c == 'O')
    {
      Serial.println("OBSTACLE RECEIVED!");
      capturePhoto();
    }
  }
}