#include <ESPAsyncWebServer.h>
#include <Arduino.h>
#include <Wire.h>
#include <string>
#include <MPU6050.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "../include/home_wifi_multi.h"
#include "../include/control.h"
#include "../include/PID.h"

float Kp=2.5, Ki=0.05, Kd=0.01;

float pitch_setpoint, pitch_input, pitch_output;

float roll_setpoint, roll_input, roll_output;

struct pid_controller pitch_control;
struct_pid_t pitch_pid;

struct pid_controller roll_control;
struct_pid_t roll_pid;

float base_throttle = 4915;
float duty_1;
float duty_2;
float duty_3;
float duty_4;

static float yaw = 0;
MPU6050 mpu;

void calculate_angle(float* roll, float* pitch)
{
  //Start of calculate roll, pitch, yaw
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  
  // Get raw data from accelerometer and gyroscope
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert accelerometer values to g (9.81 m/s^2)
  float AccX = ax / 16384.0;
  float AccY = ay / 16384.0;
  float AccZ = az / 16384.0;

  // Calculate roll and pitch from the accelerometer data
  *roll = atan2(AccY, AccZ) * 180 / PI;
  *pitch = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180 / PI;
  
  // Integrate gyroscope data for yaw calculation
  float gyroZ = gz / 131.0;  // Convert gyro value to degrees/s
  yaw += gyroZ * 0.01;  // Assume loop runs every 10ms
  //End of calculate roll, pitch, yaw
}

// HTML cho giao diện web
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>Drone PID Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script>
    // Function to update duty values from server
    function fetchDutyValues() {
      fetch('/get_duty').then(response => response.json()).then(data => {
        document.getElementById('duty1').innerText = data.duty1;
        document.getElementById('duty2').innerText = data.duty2;
        document.getElementById('duty3').innerText = data.duty3;
        document.getElementById('duty4').innerText = data.duty4;
      });
    }
    setInterval(fetchDutyValues, 500);  // Fetch every 500ms
  </script>
</head>
<body>
  <h1>PID Tuning</h1>
  <form action="/set" method="get">
    <label for="Kp">Kp:</label>
    <input type="number" step="0.01" id="Kp" name="Kp" value="%Kp%">
    <br>
    <label for="Ki">Ki:</label>
    <input type="number" step="0.01" id="Ki" name="Ki" value="%Ki%">
    <br>
    <label for="Kd">Kd:</label>
    <input type="number" step="0.01" id="Kd" name="Kd" value="%Kd%">
    <br><br>
    <input type="submit" value="Update PID">
  </form>
  <h2>Motor Duty Cycles</h2>
  <p>Motor 1 Duty: <span id="duty1">0</span></p>
  <p>Motor 2 Duty: <span id="duty2">0</span></p>
  <p>Motor 3 Duty: <span id="duty3">0</span></p>
  <p>Motor 4 Duty: <span id="duty4">0</span></p>
</body>
</html>)rawliteral";

AsyncWebServer server(80);

// Hàm để cập nhật các giá trị PID từ trình duyệt
void updatePIDValues() {
  // Ở đây bạn cần cập nhật giá trị vào đối tượng PID
  pid_tune(pitch_pid, Kp, Ki, Kd);
  pid_tune(roll_pid, Kp, Ki, Kd);
}

// Hàm xử lý cho Web Server
String processor(const String& var) {
  if (var == "Kp") {
    return String(Kp);
  } else if (var == "Ki") {
    return String(Ki);
  } else if (var == "Kd") {
    return String(Kd);
  }
  return String();
}

void setup() {
  Serial.begin(115200);

  pitch_setpoint = 0;
  roll_setpoint = 0;

  pitch_pid = pid_create(&pitch_control, &pitch_input, &pitch_output, &pitch_setpoint, Kd, Ki, Kp);
  roll_pid = pid_create(&roll_control, &roll_input, &roll_output, &roll_setpoint, Kp, Ki, Kd);

  pid_limits(pitch_pid, -8191, 8191);
  pid_limits(roll_pid, -8191, 8191);

  pid_auto(pitch_pid);
  pid_auto(roll_pid);
  
  // Kết nối Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());
  // Thiết lập route chính cho web server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  // Nhận các giá trị PID từ client
  server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("Kp")) {
      Kp = request->getParam("Kp")->value().toDouble();
    }
    if (request->hasParam("Ki")) {
      Ki = request->getParam("Ki")->value().toDouble();
    }
    if (request->hasParam("Kd")) {
      Kd = request->getParam("Kd")->value().toDouble();
    }
    updatePIDValues(); // Cập nhật giá trị PID
    request->send(200, "text/plain", "PID Updated");
  });

  server.on("/get_duty", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "{\"duty1\":" + String(duty_1) + ",\"duty2\":" + String(duty_2) +
                  ",\"duty3\":" + String(duty_3) + ",\"duty4\":" + String(duty_4) + "}";
    request->send(200, "application/json", json);
  });

  // Bắt đầu web server
  server.begin();

  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  ledcSetup(CHN_1, FREQ, RES);
  ledcAttachPin(MOT1, CHN_1);
  ledcSetup(CHN_2, FREQ, RES);
  ledcAttachPin(MOT2, CHN_2);
  ledcSetup(CHN_3, FREQ, RES);
  ledcAttachPin(MOT3, CHN_3);
  ledcSetup(CHN_4, FREQ, RES);
  ledcAttachPin(MOT4, CHN_4);
}



void loop() {
  calculate_angle(&roll_input, &pitch_input);

  pid_compute(pitch_pid);
  pid_compute(roll_pid);
  duty_1 = base_throttle - pitch_output + roll_output;
  duty_2 = base_throttle - pitch_output - roll_output;
  duty_3 = base_throttle + pitch_output - roll_output;
  duty_4 = base_throttle + pitch_output + roll_output;
  duty_1 = constrain(duty_1, 0, 8191);
  duty_2 = constrain(duty_2, 0, 8191);
  duty_3 = constrain(duty_3, 0, 8191);
  duty_4 = constrain(duty_4, 0, 8191);
  
  Serial.print("Mot 1: "); Serial.print(duty_1);
  Serial.print(", Mot 2: "); Serial.print(duty_2);
  Serial.print(", Mot 3: "); Serial.print(duty_3);
  Serial.print(", Mot 4: "); Serial.println(duty_4);

  // ledcWrite(CHN_1, duty_1);
  // ledcWrite(CHN_2, duty_2);
  // ledcWrite(CHN_3, duty_3);
  // ledcWrite(CHN_4, duty_4);

  ledcWrite(CHN_1, 1000);
  ledcWrite(CHN_2, 1000);
  ledcWrite(CHN_3, 1000);
  ledcWrite(CHN_4, 1000);
  delay(10); // Delay for 10ms (adjust as needed
}

