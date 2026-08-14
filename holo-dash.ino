#include <Wire.h>
#include <WiFiS3.h>
#include "Arduino_LED_Matrix.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// ============================================================
// WIFI & HTTP SERVER CONFIGURATION
// ============================================================

const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

WiFiServer server(80);

// ============================================================
// OLED DISPLAY SETUP (0.96" 128x64 I2C)
// ============================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ============================================================
// LED MATRIX FRAMES
// ============================================================
ArduinoLEDMatrix matrix;

uint8_t frame_WIFI1[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 }
};

uint8_t frame_WIFI2[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0 },
  { 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 }
};

uint8_t frame_WIFI3[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0 },
  { 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0 },
  { 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 }
};

uint8_t frame_ON[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0 },
  { 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0 },
  { 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0 },
  { 0, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0 },
  { 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

// ============================================================
// MPU
// ============================================================

#define MPU_ADDR 0x68

#define PWR_MGMT_1   0x6B
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B

int16_t rawAx, rawAy, rawAz;
int16_t rawGx, rawGy, rawGz;

float accelXOffset = 0;
float accelYOffset = 0;
float accelZOffset = 0;

float gyroXOffset = 0;
float gyroYOffset = 0;
float gyroZOffset = 0;

float roll = 0;
float pitch = 0;

unsigned long previousMicros = 0;

// ============================================================
// SETTINGS
// ============================================================

const float ACCEL_SCALE = 16384.0;
const float GYRO_SCALE = 131.0;
const float FILTER_ACCEL_WEIGHT = 0.04;
const float GYRO_DEADZONE = 0.02;

// ============================================================
// HTML PAGE
// ============================================================

const char MAIN_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Holo-Dash MPU</title>

<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>

<style>
* { box-sizing: border-box; }

body {
    margin: 0;
    background-color: #03070b;
    background-image:
        linear-gradient(rgba(0, 255, 255, 0.03) 1px, transparent 1px),
        linear-gradient(90deg, rgba(0, 255, 255, 0.03) 1px, transparent 1px);
    background-size: 30px 30px;
    color: #00ffff;
    font-family: 'Courier New', Courier, monospace;
    overflow: hidden;
}

#hud-container {
    position: fixed;
    top: 0;
    left: 0;
    width: 100vw;
    height: 100vh;
    pointer-events: none;
    z-index: 10;
    border: 2px solid rgba(0, 255, 255, 0.2);
    box-shadow: inset 0 0 50px rgba(0, 255, 255, 0.05);
}

.hud-corner {
    position: absolute;
    width: 30px;
    height: 30px;
    border: 2px solid #00ffff;
}
.tl { top: 10px; left: 10px; border-right: none; border-bottom: none; }
.tr { top: 10px; right: 10px; border-left: none; border-bottom: none; }
.bl { bottom: 10px; left: 10px; border-right: none; border-top: none; }
.br { bottom: 10px; right: 10px; border-left: none; border-top: none; }

#header {
    position: absolute;
    top: 20px;
    left: 50%;
    transform: translateX(-50%);
    text-align: center;
    background: rgba(0, 20, 40, 0.7);
    padding: 10px 30px;
    border: 1px solid #00ffff;
    box-shadow: 0 0 15px rgba(0, 255, 255, 0.3);
}

#title {
    font-size: 22px;
    font-weight: bold;
    letter-spacing: 4px;
    text-shadow: 0 0 8px #00ffff;
}

#status {
    margin-top: 5px;
    color: #00ff66;
    font-size: 14px;
    letter-spacing: 2px;
}

#telemetry-panel {
    position: absolute;
    top: 50%;
    right: 30px;
    transform: translateY(-50%);
    background: rgba(0, 15, 30, 0.8);
    border-left: 3px solid #00ffff;
    padding: 20px;
    width: 220px;
    box-shadow: -5px 0 15px rgba(0, 255, 255, 0.1);
}

.data-row {
    margin-bottom: 15px;
    font-size: 16px;
}
.data-label {
    color: #4488ff;
    font-size: 12px;
    margin-bottom: 4px;
    display: block;
}
.data-value {
    font-size: 24px;
    font-weight: bold;
    text-shadow: 0 0 10px #00ffff;
}
.data-unit {
    font-size: 14px;
    color: #66bbee;
}

#crosshair {
    position: absolute;
    top: 50%;
    left: 50%;
    transform: translate(-50%, -50%);
    width: 40px;
    height: 40px;
    opacity: 0.3;
}
#crosshair::before { content: ''; position: absolute; top: 19px; left: 0; width: 40px; height: 1px; background: #00ffff; }
#crosshair::after { content: ''; position: absolute; top: 0; left: 19px; width: 1px; height: 40px; background: #00ffff; }

#canvas-container {
    width: 100vw;
    height: 100vh;
}
</style>
</head>

<body>

<div id="hud-container">
    <div class="hud-corner tl"></div>
    <div class="hud-corner tr"></div>
    <div class="hud-corner bl"></div>
    <div class="hud-corner br"></div>

    <div id="header">
        <div id="title">SYS.ORIENTATION.HUD</div>
        <div id="status">[ LINK ACTIVE ]</div>
    </div>

    <div id="crosshair"></div>

    <div id="telemetry-panel">
        <div class="data-row">
            <span class="data-label">AXIS: ROLL (Z-ROT)</span>
            <span class="data-value" id="roll">0.00</span> <span class="data-unit">DEG</span>
        </div>
        <div class="data-row">
            <span class="data-label">AXIS: PITCH (X-ROT)</span>
            <span class="data-value" id="pitch">0.00</span> <span class="data-unit">DEG</span>
        </div>
        <div class="data-row" style="margin-top: 30px; border-top: 1px dashed #00ffff; padding-top: 15px;">
            <span class="data-label">SYS_REF</span>
            <span style="color:#ff3333; font-size:12px;">X: RED</span><br>
            <span style="color:#33ff33; font-size:12px;">Y: GREEN</span><br>
            <span style="color:#3333ff; font-size:12px;">Z: BLUE</span>
        </div>
    </div>
</div>

<div id="canvas-container"></div>

<script>
const scene = new THREE.Scene();
scene.fog = new THREE.FogExp2(0x03070b, 0.04);

const camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(0, 8, 10);
camera.lookAt(0, 0, 0);

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
document.getElementById("canvas-container").appendChild(renderer.domElement);

const ambient = new THREE.AmbientLight(0xffffff, 0.4);
scene.add(ambient);

const pointLight = new THREE.PointLight(0x00ffff, 2, 50);
pointLight.position.set(0, 10, 0);
scene.add(pointLight);

const blueLight = new THREE.DirectionalLight(0x0044ff, 1.5);
blueLight.position.set(-5, 5, 5);
scene.add(blueLight);

function createHoloObject(geometry, colorHex, wireHex, opacityVal) {
    const group = new THREE.Group();

    const coreMat = new THREE.MeshPhysicalMaterial({
        color: colorHex,
        metalness: 0.8,
        roughness: 0.1,
        transparent: true,
        opacity: opacityVal,
        side: THREE.DoubleSide
    });
    const core = new THREE.Mesh(geometry, coreMat);
    group.add(core);

    const wireMat = new THREE.MeshBasicMaterial({
        color: wireHex,
        wireframe: true,
        transparent: true,
        opacity: 0.9
    });
    const wire = new THREE.Mesh(geometry, wireMat);
    wire.scale.set(1.01, 1.01, 1.01);
    group.add(wire);

    return group;
}

const boardGroup = new THREE.Group();
scene.add(boardGroup);

const boardGeometry = new THREE.BoxGeometry(11, 0.4, 4.5);
const breadboard = createHoloObject(boardGeometry, 0x002244, 0x00ffff, 0.6);
boardGroup.add(breadboard);

const gridHelper = new THREE.GridHelper(30, 30, 0x003355, 0x000a14);
gridHelper.position.y = -3;
scene.add(gridHelper);

const r4Geometry = new THREE.BoxGeometry(2.6, 0.3, 3.6);
const r4 = createHoloObject(r4Geometry, 0x003355, 0x00aaff, 0.7);
r4.position.set(-3.5, 0.35, 0);
boardGroup.add(r4);

const r4Label = makeLabel("UNO R4", 0.7, "#00aaff");
r4Label.position.set(-3.5, 0.6, 0);
r4Label.rotation.x = -Math.PI / 2;
boardGroup.add(r4Label);

const mpuGeometry = new THREE.BoxGeometry(1.5, 0.2, 1.2);
const mpu = createHoloObject(mpuGeometry, 0x005533, 0x00ff88, 0.8);
mpu.position.set(0, 0.3, 0);
boardGroup.add(mpu);

const chipGeometry = new THREE.BoxGeometry(0.4, 0.1, 0.4);
const chip = createHoloObject(chipGeometry, 0x222222, 0xffffff, 0.9);
chip.position.set(0, 0.45, 0);
boardGroup.add(chip);

const mpuLabel = makeLabel("MPU-IMU", 0.5, "#00ff88");
mpuLabel.position.set(0, 0.6, 0.8);
mpuLabel.rotation.x = -Math.PI / 2;
boardGroup.add(mpuLabel);

const oledGeometry = new THREE.BoxGeometry(1.6, 0.20, 1.0);
const oledScreen = createHoloObject(oledGeometry, 0x001133, 0x00ffcc, 0.8);
oledScreen.position.set(3.5, 0.35, 0);
boardGroup.add(oledScreen);

const oledLabel = makeLabel("OLED", 0.5, "#00ffcc");
oledLabel.position.set(3.5, 0.6, 0.8);
oledLabel.rotation.x = -Math.PI / 2;
boardGroup.add(oledLabel);

const axesHelper = new THREE.AxesHelper(3);
axesHelper.position.set(3.5, 0.4, -1.2);
boardGroup.add(axesHelper);

function makeLabel(text, size, color) {
    const canvas = document.createElement("canvas");
    const ctx = canvas.getContext("2d");
    canvas.width = 512;
    canvas.height = 128;

    ctx.fillStyle = color;
    ctx.font = "bold 60px 'Courier New'";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.shadowColor = color;
    ctx.shadowBlur = 15;
    ctx.fillText(text, canvas.width / 2, canvas.height / 2);

    const texture = new THREE.CanvasTexture(canvas);
    const material = new THREE.SpriteMaterial({ map: texture, transparent: true });
    const sprite = new THREE.Sprite(material);
    sprite.scale.set(size * 3, size * 0.75, 1);
    return sprite;
}

let targetRoll = 0;
let targetPitch = 0;
let displayedRoll = 0;
let displayedPitch = 0;

function updateUI(data) {
    targetRoll = data.roll * Math.PI / 180;
    targetPitch = data.pitch * Math.PI / 180;

    let rStr = data.roll.toFixed(2);
    let pStr = data.pitch.toFixed(2);

    document.getElementById("roll").innerText = (data.roll >= 0 ? "+" : "") + rStr;
    document.getElementById("pitch").innerText = (data.pitch >= 0 ? "+" : "") + pStr;
}

async function getData() {
    try {
        const response = await fetch("/data?t=" + Date.now());
        const data = await response.json();
        updateUI(data);
        document.getElementById("status").innerText = "[ LINK ACTIVE ]";
        document.getElementById("status").style.color = "#00ff66";
    } catch (error) {
        document.getElementById("status").innerText = "[ LINK OFFLINE ]";
        document.getElementById("status").style.color = "#ff3333";
    }
    setTimeout(getData, 20);
}
getData();

function animate() {
    requestAnimationFrame(animate);

    displayedRoll += (targetRoll - displayedRoll) * 0.4;
    displayedPitch += (targetPitch - displayedPitch) * 0.4;

    boardGroup.rotation.z = displayedRoll;
    boardGroup.rotation.x = displayedPitch;

    renderer.render(scene, camera);
}
animate();

window.addEventListener("resize", () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});
</script>
</body>
</html>
)rawliteral";

// ============================================================
// MPU FUNCTIONS
// ============================================================

void writeMPU(byte reg, byte value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  rawAx = Wire.read() << 8 | Wire.read();
  rawAy = Wire.read() << 8 | Wire.read();
  rawAz = Wire.read() << 8 | Wire.read();
  Wire.read();
  Wire.read();
  rawGx = Wire.read() << 8 | Wire.read();
  rawGy = Wire.read() << 8 | Wire.read();
  rawGz = Wire.read() << 8 | Wire.read();
}

// ============================================================
// CALIBRATION
// ============================================================

void calibrateMPU() {
  Serial.println();
  Serial.println("================================");
  Serial.println("CALIBRATING MPU...");
  Serial.println("KEEP THE BREADBOARD COMPLETELY STILL");
  Serial.println("================================");

  const int samples = 1000;
  long axSum = 0, aySum = 0, azSum = 0;
  long gxSum = 0, gySum = 0, gzSum = 0;

  for (int i = 0; i < samples; i++) {
    readMPU();
    axSum += rawAx; aySum += rawAy; azSum += rawAz;
    gxSum += rawGx; gySum += rawGy; gzSum += rawGz;
    delay(3);
  }

  accelXOffset = (float)axSum / samples;
  accelYOffset = (float)aySum / samples;
  accelZOffset = ((float)azSum / samples) - ACCEL_SCALE;

  gyroXOffset = (float)gxSum / samples;
  gyroYOffset = (float)gySum / samples;
  gyroZOffset = (float)gzSum / samples;

  Serial.println("Calibration complete.");
  Serial.println();
}

// ============================================================
// UPDATE ORIENTATION
// ============================================================

void updateOrientation() {
  unsigned long now = micros();
  float dt = (now - previousMicros) / 1000000.0;
  previousMicros = now;

  if (dt <= 0 || dt > 1.0) { dt = 0.02; }

  readMPU();

  float ax = (rawAx - accelXOffset) / ACCEL_SCALE;
  float ay = (rawAy - accelYOffset) / ACCEL_SCALE;
  float az = (rawAz - accelZOffset) / ACCEL_SCALE;
  float gx = (rawGx - gyroXOffset) / GYRO_SCALE;
  float gy = (rawGy - gyroYOffset) / GYRO_SCALE;

  if (fabs(gx) < GYRO_DEADZONE) gx = 0;
  if (fabs(gy) < GYRO_DEADZONE) gy = 0;

  float accelRoll = atan2(ay, az) * 180.0 / PI;
  float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  float gyroRoll = roll + gx * dt;
  float gyroPitch = pitch + gy * dt;

  roll = gyroRoll * (1.0 - FILTER_ACCEL_WEIGHT) + accelRoll * FILTER_ACCEL_WEIGHT;
  pitch = gyroPitch * (1.0 - FILTER_ACCEL_WEIGHT) + accelPitch * FILTER_ACCEL_WEIGHT;

  if (fabs(roll) < 0.02) roll = 0;
  if (fabs(pitch) < 0.02) pitch = 0;
}

// ============================================================
// OLED 3D CUBOID RENDERER
// ============================================================

void draw3DCuboid(float rDeg, float pDeg) {
  float cx = 64;
  float cy = 32;

  float w = 36 * 2;
  float h = 12 * 2;
  float d = 20 * 2;

  float r = rDeg * PI / 180.0;
  float p = pDeg * PI / 180.0;

  float pts[8][3] = {
    {-w/2, -h/2, -d/2}, { w/2, -h/2, -d/2}, { w/2,  h/2, -d/2}, {-w/2,  h/2, -d/2},
    {-w/2, -h/2,  d/2}, { w/2, -h/2,  d/2}, { w/2,  h/2,  d/2}, {-w/2,  h/2,  d/2}
  };

  int proj[8][2];
  for (int i = 0; i < 8; i++) {
    float x = pts[i][0];
    float y = pts[i][1];
    float z = pts[i][2];

    float y1 = y * cos(p) - z * sin(p);
    float z1 = y * sin(p) + z * cos(p);

    float x2 = x * cos(r) - y1 * sin(r);
    float y2 = x * sin(r) + y1 * cos(r);

    proj[i][0] = (int)(cx + x2);
    proj[i][1] = (int)(cy + y2);
  }

  int edges[12][2] = {
    {0,1},{1,2},{2,3},{3,0},
    {4,5},{5,6},{6,7},{7,4},
    {0,4},{1,5},{2,6},{3,7}
  };

  for (int i = 0; i < 12; i++) {
    int x1 = proj[edges[i][0]][0];
    int y1 = proj[edges[i][0]][1];
    int x2 = proj[edges[i][1]][0];
    int y2 = proj[edges[i][1]][1];
    display.drawLine(x1, y1, x2, y2, SSD1306_WHITE);
  }
}

void drawOLED() {
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("R:"); display.print(roll, 1);
  display.setCursor(64, 0);
  display.print("P:"); display.print(pitch, 1);

  draw3DCuboid(roll, pitch);

  display.setTextSize(1);
  display.setCursor(0, 50);
  display.print("OLEN:");
  display.setCursor(40, 50);
  display.print(roll, 1);
  display.print(","); display.print(pitch, 1);

  display.display();
}

// ============================================================
// HTTP CLIENT REQUEST HANDLER
// ============================================================

void handleClient(WiFiClient client) {
  String request = "";
  unsigned long timeout = millis();

  while (client.available() || (millis() - timeout < 1000)) {
    if (client.available()) {
      char c = client.read();
      request += c;
      timeout = millis();
      if (request.endsWith("\r\n\r\n")) break;
    }
  }

  if (request.indexOf("GET /data") >= 0) {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.println("Cache-Control: no-cache, no-store, must-revalidate");
    client.println("Connection: close");
    client.println();
    client.print("{\"roll\":");
    client.print(roll, 2);
    client.print(",\"pitch\":");
    client.print(pitch, 2);
    client.println("}");
  } else {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Cache-Control: no-cache");
    client.println("Connection: close");
    client.println();
    client.print(MAIN_PAGE);
  }

  client.stop();
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(1500);

  matrix.begin();

  Serial.println();
  Serial.println("================================");
  Serial.println("MPU LIVE ORIENTATION SYSTEM");
  Serial.println("UNO R4 WiFi");
  Serial.println("================================");

  Wire.begin();

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 20);
  display.println(F("SYSTEM INITIALIZING"));
  display.display();

  writeMPU(PWR_MGMT_1, 0x00);
  delay(100);
  writeMPU(CONFIG, 0x03);
  writeMPU(GYRO_CONFIG, 0x00);
  writeMPU(ACCEL_CONFIG, 0x00);
  delay(100);

  calibrateMPU();
  previousMicros = micros();

  Serial.println("Connecting to WiFi...");

  int status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long wifiStart = millis();
  int animFrame = 0;

  while (status != WL_CONNECTED && millis() - wifiStart < 20000) {
    delay(500);
    Serial.print(".");
    status = WiFi.status();

    if (animFrame == 0) {
      matrix.renderBitmap(frame_WIFI1, 8, 12);
    } else if (animFrame == 1) {
      matrix.renderBitmap(frame_WIFI2, 8, 12);
    } else if (animFrame == 2) {
      matrix.renderBitmap(frame_WIFI3, 8, 12);
    }
    animFrame = (animFrame + 1) % 3;
  }
  Serial.println();

  if (status == WL_CONNECTED) {
    matrix.renderBitmap(frame_ON, 8, 12);

    Serial.println("WiFi connected!");
    delay(2000);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println();

    // Show dynamic IP on OLED for whoever runs the code
    display.clearDisplay();
    display.setCursor(0, 10);
    display.println(F("WIFI CONNECTED!"));
    display.setCursor(0, 30);
    display.print(F("IP: "));
    display.print(WiFi.localIP()); // Dynamically prints actual network IP
    display.display();
    delay(3000);

    server.begin();
    Serial.println("HTTP server started on port 80");
  } else {
    Serial.println("WiFi connection FAILED.");
    Serial.println("Check your SSID and password.");
    display.clearDisplay();
    display.setCursor(0, 20);
    display.println(F("WIFI FAILED!"));
    display.display();
  }
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  updateOrientation();
  drawOLED();

  WiFiClient client = server.available();
  if (client) {
    handleClient(client);
  }
}
