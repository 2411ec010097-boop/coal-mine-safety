#include <WiFi.h>
#include <WebServer.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* =========================================================
   WIFI
   ========================================================= */
const char* AP_SSID = "ESP32-MINE-ROBOT";
const char* AP_PASSWORD = "12345678";
WebServer server(80);

/* =========================================================
   L298N MOTOR PINS - KEEPING YOUR ORIGINAL PINS
   ========================================================= */
#define ENA 25
#define IN1 26
#define IN2 27

#define ENB 14
#define IN3 12
#define IN4 13

/* =========================================================
   SENSOR PINS
   ========================================================= */
#define DHTPIN 4
#define DHTTYPE DHT22

#define FLAME_PIN 19
#define BUZZER_PIN 23
#define GREEN_LED_PIN 16
#define YELLOW_LED_PIN 17
#define RED_LED_PIN 33

#define TRIG_PIN 5
#define ECHO_PIN 18

// MQ GAS SENSOR ANALOG OUTPUTS
// MQ-4  AO -> GPIO34
// MQ-7  AO -> GPIO35
// MQ-135 AO -> GPIO32
#define MQ4_PIN 34
#define MQ7_PIN 35
#define MQ135_PIN 32

/* =========================================================
   OLED I2C
   SDA = GPIO 21
   SCL = GPIO 22
   ========================================================= */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C

DHT dht(DHTPIN, DHTTYPE);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

/* =========================================================
   VARIABLES
   ========================================================= */
int speedPercent = 50;
String currentDirection = "STOPPED";

float temperature = 0.0;
float humidity = 0.0;
float distanceCM = 0.0;
bool flameDetected = false;

int mq4Raw = 0;
int mq7Raw = 0;
int mq135Raw = 0;
String gasStatus = "SAFE";
String mq4Status = "NORMAL";
String mq7Status = "NORMAL";
String mq135Status = "NORMAL";
bool gasDanger = false;

/* =========================================================
   MOTOR FUNCTIONS
   ========================================================= */
int pwmValue() {
  return map(speedPercent, 0, 100, 0, 255);
}

void leftMotor(int speedValue) {
  if (speedValue > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (speedValue < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  ledcWrite(ENA, abs(speedValue));
}

void rightMotor(int speedValue) {
  if (speedValue > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (speedValue < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  ledcWrite(ENB, abs(speedValue));
}

void stopMotors() {
  leftMotor(0);
  rightMotor(0);
  currentDirection = "STOPPED";
}

void forward() {
  int pwm = pwmValue();
  leftMotor(pwm);
  rightMotor(pwm);
  currentDirection = "FORWARD";
}

void backward() {
  int pwm = pwmValue();
  leftMotor(-pwm);
  rightMotor(-pwm);
  currentDirection = "BACKWARD";
}

void turnLeft() {
  int pwm = pwmValue();
  leftMotor(-pwm);
  rightMotor(pwm);
  currentDirection = "LEFT";
}

void turnRight() {
  int pwm = pwmValue();
  leftMotor(pwm);
  rightMotor(-pwm);
  currentDirection = "RIGHT";
}

/* =========================================================
   READ ULTRASONIC SENSOR
   HC-SR04 ECHO MUST BE REDUCED TO 3.3V
   USING A VOLTAGE DIVIDER
   ========================================================= */
float readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) {
    return 999.0;
  }

  return duration * 0.0343 / 2.0;
}

/* =========================================================
   READ ALL SENSORS
   ========================================================= */
void readSensors() {
  float newTemp = dht.readTemperature();
  float newHum = dht.readHumidity();

  if (!isnan(newTemp)) {
    temperature = newTemp;
  }

  if (!isnan(newHum)) {
    humidity = newHum;
  }

  distanceCM = readDistance();

  // Most common flame sensor modules:
  // LOW = flame detected
  flameDetected = (digitalRead(FLAME_PIN) == LOW);

  // Read MQ sensor analog outputs.
  // These are RAW ADC values, not calibrated ppm values.
  mq4Raw = analogRead(MQ4_PIN);
  mq7Raw = analogRead(MQ7_PIN);
  mq135Raw = analogRead(MQ135_PIN);

  // DEMO thresholds: raw ADC values, NOT ppm. Calibrate for your sensors.
  const int WARNING_LEVEL = 1800;
  const int DANGER_LEVEL  = 2800;

  mq4Status = (mq4Raw >= DANGER_LEVEL) ? "DANGEROUS" : (mq4Raw >= WARNING_LEVEL) ? "WARNING" : "NORMAL";
  mq7Status = (mq7Raw >= DANGER_LEVEL) ? "DANGEROUS" : (mq7Raw >= WARNING_LEVEL) ? "WARNING" : "NORMAL";
  mq135Status = (mq135Raw >= DANGER_LEVEL) ? "DANGEROUS" : (mq135Raw >= WARNING_LEVEL) ? "WARNING" : "NORMAL";

  gasDanger = (mq4Status == "DANGEROUS" || mq7Status == "DANGEROUS" || mq135Status == "DANGEROUS");
  bool gasWarning = (mq4Status == "WARNING" || mq7Status == "WARNING" || mq135Status == "WARNING");

  if (gasDanger || flameDetected) gasStatus = "DANGEROUS";
  else if (gasWarning) gasStatus = "WARNING";
  else gasStatus = "NORMAL";

  // Green = NORMAL, Yellow = WARNING, Red = DANGEROUS.
  digitalWrite(GREEN_LED_PIN, (!flameDetected && !gasDanger && !gasWarning) ? HIGH : LOW);
  digitalWrite(YELLOW_LED_PIN, (!flameDetected && !gasDanger && gasWarning) ? HIGH : LOW);
  digitalWrite(RED_LED_PIN, (flameDetected || gasDanger) ? HIGH : LOW);

  // Active-HIGH buzzer: OFF for NORMAL/WARNING, ON for DANGEROUS/flame.
  digitalWrite(BUZZER_PIN, (flameDetected || gasDanger) ? HIGH : LOW);

  // Safety stop if obstacle is too close or a dangerous gas/flame state is detected.
  if ((distanceCM < 15.0 && distanceCM < 999.0) || flameDetected || gasDanger) {
    stopMotors();
  }
}


/* =========================================================
   SERIAL MONITOR
   ========================================================= */
void printSerialData() {
  Serial.println();
  Serial.println("================================");
  Serial.println("     MINE ROBOT SENSOR DATA");
  Serial.println("================================");

  Serial.print("Temperature : ");
  Serial.print(temperature, 1);
  Serial.println(" C");

  Serial.print("Humidity    : ");
  Serial.print(humidity, 1);
  Serial.println(" %");

  Serial.print("Flame       : ");
  if (flameDetected) {
    Serial.println("DANGER");
  } else {
    Serial.println("SAFE");
  }

  Serial.print("MQ-4 CH4    : ");
  Serial.println(mq4Raw);

  Serial.print("MQ-7 CO     : ");
  Serial.println(mq7Raw);

  Serial.print("MQ-135 Air  : ");
  Serial.println(mq135Raw);

  Serial.print("MQ-4 Status : "); Serial.println(mq4Status);
  Serial.print("MQ-7 Status : "); Serial.println(mq7Status);
  Serial.print("MQ-135 Status: "); Serial.println(mq135Status);
  Serial.print("LEDs        : G="); Serial.print(digitalRead(GREEN_LED_PIN) ? "ON" : "OFF");
  Serial.print(" Y="); Serial.print(digitalRead(YELLOW_LED_PIN) ? "ON" : "OFF");
  Serial.print(" R="); Serial.println(digitalRead(RED_LED_PIN) ? "ON" : "OFF");
  Serial.print("Buzzer      : "); Serial.println(digitalRead(BUZZER_PIN) ? "ON" : "OFF");

  Serial.print("Gas Status  : ");
  Serial.println(gasStatus);

  Serial.print("Distance    : ");
  if (distanceCM >= 999.0) {
    Serial.println("OUT OF RANGE");
  } else {
    Serial.print(distanceCM, 1);
    Serial.println(" cm");
  }

  Serial.print("Direction   : ");
  Serial.println(currentDirection);

  Serial.print("Speed       : ");
  Serial.print(speedPercent);
  Serial.println(" %");

  Serial.println("================================");
}

/* =========================================================
   OLED DISPLAY
   ========================================================= */
void updateOLED() {
  static bool gasPage = false;
  gasPage = !gasPage;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("COAL MINE ROBOT");

  if (!gasPage) {
    display.setCursor(0, 13);
    display.print("Temp: ");
    display.print(temperature, 1);
    display.println(" C");

    display.setCursor(0, 25);
    display.print("Hum : ");
    display.print(humidity, 1);
    display.println(" %");

    display.setCursor(0, 37);
    display.print("Flame: ");
    display.println(flameDetected ? "DANGER" : "SAFE");

    display.setCursor(0, 49);
    display.print("Dist: ");
    if (distanceCM >= 999.0) display.println("OUT");
    else {
      display.print(distanceCM, 1);
      display.println(" cm");
    }
  } else {
    display.setCursor(0, 13);
    display.print("MQ4:"); display.print(mq4Raw); display.print(" "); display.println(mq4Status);

    display.setCursor(0, 25);
    display.print("MQ7:"); display.print(mq7Raw); display.print(" "); display.println(mq7Status);

    display.setCursor(0, 37);
    display.print("MQ135:"); display.print(mq135Raw);

    display.setCursor(0, 49);
    display.print("Gas:"); display.print(gasStatus);
    display.print(" B:"); display.println(digitalRead(BUZZER_PIN) ? "ON" : "OFF");
  }

  display.display();
}


/* =========================================================
   WEB PAGE
   ========================================================= */
const char MAIN_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32 Mine Rescue Robot</title>

<style>
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{
 margin:0;min-height:100vh;font-family:Arial;color:white;
 display:flex;justify-content:center;align-items:center;
 background:linear-gradient(135deg,#070711,#111126,#080812);
}
.dashboard{
 width:min(94vw,500px);padding:22px;border-radius:28px;
 background:rgba(255,255,255,.08);
 border:1px solid rgba(255,255,255,.15);
 box-shadow:0 20px 60px rgba(0,0,0,.5);
}
.header{display:flex;justify-content:space-between;align-items:center}
.title{font-size:25px;font-weight:bold}
.subtitle{font-size:11px;color:#aaa;margin-top:5px}
.dot{width:13px;height:13px;border-radius:50%;background:#43ffad}
.controls{
 display:grid;grid-template-columns:1fr 1fr 1fr;
 grid-template-rows:75px 75px 75px;gap:10px;
 max-width:350px;margin:22px auto;
}
button{
 border:1px solid rgba(255,255,255,.15);border-radius:20px;
 background:rgba(255,255,255,.07);color:white;font-size:27px;
 cursor:pointer;
}
button:active{transform:scale(.94)}
.fwd{grid-column:2;grid-row:1}
.left{grid-column:1;grid-row:2}
.stop{grid-column:2;grid-row:2;background:rgba(255,60,80,.25)}
.right{grid-column:3;grid-row:2}
.back{grid-column:2;grid-row:3}
.speedbox,.sensors{
 margin-top:15px;padding:17px;border-radius:20px;
 background:rgba(0,0,0,.18);
}
input{width:100%}
.speedrow{display:flex;justify-content:space-between}
.sensor{
 display:flex;justify-content:space-between;
 padding:10px 0;border-bottom:1px solid rgba(255,255,255,.08)
}
.sensor:last-child{border-bottom:0}
.label{color:#aaa}
.value{font-weight:bold}
.safe{color:#43ffad}.danger{color:#ff6478}.warning{color:#ffd166}

/* 2D DIGITAL TWIN MINE MAP */
.mineTwin{margin-top:16px;padding:15px;border-radius:20px;background:rgba(0,0,0,.24)}
.mineTwin h3{margin:0 0 8px}.mapLegend{font-size:12px;color:#bbb;margin-bottom:10px}
.mineMap{position:relative;height:230px;border:2px solid #687080;border-radius:18px;overflow:hidden;background:#202532;background-image:linear-gradient(90deg,transparent 48%,rgba(255,255,255,.05) 49%,rgba(255,255,255,.05) 51%,transparent 52%),linear-gradient(transparent 48%,rgba(255,255,255,.05) 49%,rgba(255,255,255,.05) 51%,transparent 52%);background-size:35px 35px}
.tunnel{position:absolute;background:#535b69;border:5px solid #343b47;border-radius:18px}
.tunnel.main{left:7%;top:42%;width:86%;height:24%}.tunnel.branch1{left:22%;top:12%;width:17%;height:40%}.tunnel.branch2{right:18%;top:48%;width:17%;height:39%}
.zone{position:absolute;z-index:2;border:2px solid #9aa2af;border-radius:12px;padding:7px 8px;font-size:12px;font-weight:bold;text-align:center;min-width:78px;background:#273142;box-shadow:0 2px 10px #0007}
.zone.safe{background:#174d36;border-color:#43ffad;color:#aaffd2}.zone.warning{background:#624b14;border-color:#ffd166;color:#ffe6a3}.zone.danger{background:#641e2c;border-color:#ff6478;color:#ffc0c8}
#zone1{left:4%;top:8%}#zone2{left:39%;top:48%}#zone3{right:3%;bottom:7%}
.robotDot{position:absolute;z-index:3;left:49%;top:39%;width:24px;height:24px;border-radius:50%;background:#55c8ff;border:3px solid white;box-shadow:0 0 12px #55c8ff;transition:left .4s,top .4s}
.mapNote{font-size:11px;color:#aaa;margin-top:8px;line-height:1.4}
@media(min-width:700px){.dashboard{width:min(96vw,1000px)}}

</style>
</head>

<body>
<div class="dashboard">

<div class="header">
 <div>
  <div class="title">Mine Rescue Robot</div>
  <div class="subtitle">ESP32 + L298N + SENSORS</div>
 </div>
 <div class="dot"></div>
</div>

<div class="mineTwin">
 <h3>2D Digital Twin - Virtual Coal Mine</h3>
 <div class="mapLegend">Live prototype status: <span id="overallStatus" class="safe">SAFE</span></div>
 <div class="mineMap" aria-label="Illustrative 2D mine map">
  <div class="tunnel main"></div><div class="tunnel branch1"></div><div class="tunnel branch2"></div>
  <div id="zone1" class="zone safe">ZONE 1<br><span id="zone1Text">NORMAL</span></div>
  <div id="zone2" class="zone safe">ZONE 2<br><span id="zone2Text">NORMAL</span></div>
  <div id="zone3" class="zone safe">ZONE 3<br><span id="zone3Text">CLEAR</span></div>
  <div class="robotDot" id="robotDot" title="Illustrative robot marker"></div>
 </div>
 <div class="mapNote">Schematic demo map only, not a surveyed mine plan. Zone colours follow connected prototype sensors. Robot marker is illustrative; no location sensor is installed.</div>
</div>

<div class="controls">
 <button class="fwd" onclick="moveCar('forward')">▲</button>
 <button class="left" onclick="moveCar('left')">◀</button>
 <button class="stop" onclick="stopCar()">■</button>
 <button class="right" onclick="moveCar('right')">▶</button>
 <button class="back" onclick="moveCar('backward')">▼</button>
</div>

<div class="speedbox">
 <div class="speedrow">
  <span>Motor Speed</span>
  <b><span id="speedValue">50</span>%</b>
 </div>
 <input id="speed" type="range" min="0" max="100" value="50"
        oninput="changeSpeed(this.value)">
 <p>Direction: <b id="direction">STOPPED</b></p>
</div>

<div class="sensors">
 <h3>Live Sensor Data</h3>

 <div class="sensor">
  <span class="label">Temperature</span>
  <span class="value"><span id="temp">--</span> °C</span>
 </div>

 <div class="sensor">
  <span class="label">Humidity</span>
  <span class="value"><span id="hum">--</span> %</span>
 </div>

 <div class="sensor">
  <span class="label">Flame</span>
  <span id="flame" class="value">--</span>
 </div>

 <div class="sensor">
  <span class="label">Distance</span>
  <span class="value"><span id="dist">--</span> cm</span>
 </div>

 <div class="sensor">
  <span class="label">MQ-4 Methane (ADC)</span>
  <span class="value" id="mq4">--</span>
 </div>

 <div class="sensor">
  <span class="label">MQ-7 CO (ADC)</span>
  <span class="value" id="mq7">--</span>
 </div>

 <div class="sensor">
  <span class="label">MQ-135 Air Quality (ADC)</span>
  <span class="value" id="mq135">--</span>
 </div>

 <div class="sensor"><span class="label">MQ-4 Status</span><span class="value safe" id="mq4Status">NORMAL</span></div>
 <div class="sensor"><span class="label">MQ-7 Status</span><span class="value safe" id="mq7Status">NORMAL</span></div>
 <div class="sensor"><span class="label">MQ-135 Status</span><span class="value safe" id="mq135Status">NORMAL</span></div>
 <div class="sensor">
  <span class="label">Gas Status</span>
  <span class="value safe" id="gasStatus">NORMAL</span>
 </div>
 <div class="sensor"><span class="label">Buzzer</span><span class="value safe" id="buzzerStatus">OFF</span></div>
</div>

</div>

<script>
let speed=50;

function changeSpeed(v){
 speed=v;
 document.getElementById("speedValue").innerText=v;
 fetch("/speed?value="+v).catch(()=>{});
}

function moveCar(d){
 document.getElementById("direction").innerText=d.toUpperCase();
 fetch("/move?direction="+d+"&speed="+speed).catch(()=>{});
}

function stopCar(){
 document.getElementById("direction").innerText="STOPPED";
 fetch("/stop").catch(()=>{});
}

function setStatus(id, status){
 const el=document.getElementById(id);
 el.innerText=status;
 el.className="value "+(status === "DANGEROUS" || status === "ON" ? "danger" : status === "WARNING" ? "warning" : "safe");
}

function setZone(zoneId, textId, text, status){
 const zone=document.getElementById(zoneId); zone.className="zone "+status;
 document.getElementById(textId).innerText=text;
}

function updateData(){
 fetch("/data")
 .then(r=>r.json())
 .then(d=>{
  document.getElementById("temp").innerText=d.temperature.toFixed(1);
  document.getElementById("hum").innerText=d.humidity.toFixed(1);

  let f=document.getElementById("flame");
  f.innerText=d.flame ? "DANGER" : "SAFE";
  f.className=d.flame ? "value danger" : "value safe";

  document.getElementById("dist").innerText=
    d.distance>=999 ? "OUT" : d.distance.toFixed(1);

  document.getElementById("mq4").innerText=d.mq4;
  document.getElementById("mq7").innerText=d.mq7;
  document.getElementById("mq135").innerText=d.mq135;
  setStatus("mq4Status", d.mq4Status);
  setStatus("mq7Status", d.mq7Status);
  setStatus("mq135Status", d.mq135Status);
  setStatus("buzzerStatus", d.buzzer ? "ON" : "OFF");

  let gs=document.getElementById("gasStatus");
  gs.innerText=d.gasStatus;
  gs.className="value " + (d.gasStatus==="DANGEROUS" ? "danger" : (d.gasStatus==="WARNING" ? "warning" : "safe"));

  document.getElementById("direction").innerText=d.direction;
  const danger = d.gasStatus === "DANGEROUS" || d.flame;
  const obstacle = d.distance < 15 && d.distance < 999;
  const warning = d.gasStatus === "WARNING" || d.temperature >= 35 || (d.distance < 30 && d.distance < 999);
  setZone("zone1", "zone1Text", danger ? "DANGEROUS" : warning ? "WARNING" : "NORMAL", danger ? "danger" : warning ? "warning" : "safe");
  setZone("zone2", "zone2Text", d.gasStatus, d.gasStatus === "DANGEROUS" ? "danger" : d.gasStatus === "WARNING" ? "warning" : "safe");
  setZone("zone3", "zone3Text", d.flame ? "FLAME" : obstacle ? "OBSTACLE" : "CLEAR", (d.flame || obstacle) ? "danger" : "safe");
  const overall = danger ? "DANGEROUS" : (warning || obstacle) ? "WARNING" : "SAFE";
  const os = document.getElementById("overallStatus"); os.innerText = overall; os.className = overall === "DANGEROUS" ? "danger" : overall === "WARNING" ? "warning" : "safe";
  // This is a command-based illustration, not measured robot localization.
  const marker = document.getElementById("robotDot");
  if (d.direction === "FORWARD") { marker.style.left = "57%"; marker.style.top = "39%"; }
  else if (d.direction === "BACKWARD") { marker.style.left = "41%"; marker.style.top = "55%"; }
  else if (d.direction === "LEFT") { marker.style.left = "43%"; marker.style.top = "35%"; }
  else if (d.direction === "RIGHT") { marker.style.left = "57%"; marker.style.top = "55%"; }
 })
 .catch(()=>{});
}

setInterval(updateData,2000);
updateData();
</script>
</body>
</html>
)rawliteral";

/* =========================================================
   WEB HANDLERS
   ========================================================= */
void handleRoot() {
  server.send_P(200, "text/html", MAIN_PAGE);
}

void handleSpeed() {
  if (server.hasArg("value")) {
    speedPercent = constrain(server.arg("value").toInt(), 0, 100);
  }

  server.send(200, "text/plain", "OK");
}

void handleMove() {
  if (!server.hasArg("direction")) {
    server.send(400, "text/plain", "Missing direction");
    return;
  }

  String direction = server.arg("direction");

  if (server.hasArg("speed")) {
    speedPercent = constrain(server.arg("speed").toInt(), 0, 100);
  }

  if (direction == "forward") {
    forward();
  } else if (direction == "backward") {
    backward();
  } else if (direction == "left") {
    turnLeft();
  } else if (direction == "right") {
    turnRight();
  } else {
    stopMotors();
  }

  server.send(200, "text/plain", "OK");
}

void handleStop() {
  stopMotors();
  server.send(200, "text/plain", "STOPPED");
}

void handleData() {
  String json = "{";
  json += "\"temperature\":" + String(temperature, 1) + ",";
  json += "\"humidity\":" + String(humidity, 1) + ",";
  json += "\"flame\":" + String(flameDetected ? "true" : "false") + ",";
  json += "\"distance\":" + String(distanceCM, 1) + ",";
  json += "\"mq4\":" + String(mq4Raw) + ",";
  json += "\"mq7\":" + String(mq7Raw) + ",";
  json += "\"mq135\":" + String(mq135Raw) + ",";
  json += "\"mq4Status\":\"" + mq4Status + "\",";
  json += "\"mq7Status\":\"" + mq7Status + "\",";
  json += "\"mq135Status\":\"" + mq135Status + "\",";
  json += "\"buzzer\":" + String(digitalRead(BUZZER_PIN) ? "true" : "false") + ",";
  json += "\"gasStatus\":\"" + gasStatus + "\",";
  json += "\"direction\":\"" + currentDirection + "\"";
  json += "}";

  server.send(200, "application/json", json);
}

/* =========================================================
   SETUP
   ========================================================= */
void setup() {
  Serial.begin(115200);
  delay(500);

  /* Motor pins */
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcAttach(ENA, 1000, 8);
  ledcAttach(ENB, 1000, 8);

  stopMotors();

  /* DHT22 */
  dht.begin();

  /* Flame sensor and buzzer */
  pinMode(FLAME_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);

  /* Ultrasonic */
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  /* MQ gas sensors: use ANALOG OUTPUT (AO) */
  pinMode(MQ4_PIN, INPUT);
  pinMode(MQ7_PIN, INPUT);
  pinMode(MQ135_PIN, INPUT);
  analogReadResolution(12);

  /* OLED */
  Wire.begin(21, 22);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED not found. Check wiring/address.");
  } else {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("COAL MINE ROBOT");
    display.println();
    display.println("Starting...");
    display.display();
  }

  /* WiFi Access Point */
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  Serial.println();
  Serial.println("================================");
  Serial.println("   ESP32 MINE RESCUE ROBOT");
  Serial.println("================================");
  Serial.print("WiFi Name : ");
  Serial.println(AP_SSID);
  Serial.print("Password  : ");
  Serial.println(AP_PASSWORD);
  Serial.print("Web IP    : http://");
  Serial.println(WiFi.softAPIP());

  /* Web routes */
  server.on("/", handleRoot);
  server.on("/speed", handleSpeed);
  server.on("/move", handleMove);
  server.on("/stop", handleStop);
  server.on("/data", handleData);

  server.begin();

  Serial.println("Web server started.");
  Serial.println("Open the IP address on your phone.");
}

/* =========================================================
   LOOP
   ========================================================= */
void loop() {
  server.handleClient();

  static unsigned long lastSensorRead = 0;

  if (millis() - lastSensorRead >= 2000) {
    lastSensorRead = millis();

    readSensors();
    printSerialData();
    updateOLED();
  }
}
