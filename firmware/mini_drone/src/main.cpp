#include <Arduino.h>
#include <Wire.h>
#include <string>
#include <NimBLEDevice.h>
#include <MPU6050.h>
#include <WiFi.h>
#include "../include/control.h"
#include "../include/PID.h"
#include "esp_camera.h"

#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#define __run__

#include "../include/home_wifi_multi.h"
#include "camera_pins.h"

//Prototype 
void setupBLE();
void setupMPU6050();
void setupCamera();
void setupWiFi();
void updateMotors();
void captureAndHandleImage();

// UUID cho Service và Characteristic
#define SERVICE_UUID "00FF"
#define DRONE_CONTROL_CHAR_UUID "FF01"
#define DRONE_INFO_CHAR_UUID "FF02"

float Kp=2.5, Ki=0.05, Kd=0.01;

float pitch_setpoint, pitch_input, pitch_output;

float roll_setpoint, roll_input, roll_output;

struct pid_controller pitch_control;
struct_pid_t pitch_pid;

struct pid_controller roll_control;
struct_pid_t roll_pid;

float base_throttle = IDLE_THROTTLE;
float duty_1;
float duty_2;
float duty_3;
float duty_4;

bool button1 = false;
bool button2 = false;
bool button3 = false;
bool button4 = false;
bool button5 = false;
bool button6 = false;
bool button7 = false;

NimBLEServer* pServer = nullptr;
NimBLECharacteristic* pWriteCharacteristic = nullptr;
NimBLECharacteristic* pReadCharacteristic = nullptr;
bool deviceConnected = false;

static float yaw = 0;
MPU6050 mpu;

// Class để xử lý sự kiện kết nối và ngắt kết nối
class MyServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Device connected");
  }

  void onDisconnect(NimBLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected");
  }
};

// Class để xử lý việc nhận dữ liệu từ client
class MyCharacteristicCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pWriteCharacteristic) {
    Serial.println("Get data");
    
    std::string value = pWriteCharacteristic->getValue();
    Serial.print("Received raw data: ");
    for (int i = 0; i < value.length(); i++) {
      Serial.printf("%02X ", (uint8_t)value[i]);
    }
    Serial.println();

    uint8_t buttonState = value[0]; // Lấy giá trị đầu tiên dưới dạng uint8_t
    Serial.printf("Received button state: %d\n", buttonState);

    button1 = buttonState & (1 << 0);
    button2 = buttonState & (1 << 1);
    button3 = buttonState & (1 << 2);
    button4 = buttonState & (1 << 3);
    button5 = buttonState & (1 << 4);
    button6 = buttonState & (1 << 5);
    button7 = buttonState & (1 << 6);

    pWriteCharacteristic->notify(); // Thông báo client
  }

  void onRead(NimBLECharacteristic* pReadCharacteristic) {
    Serial.println("Client is reading data");

    // Tạo dữ liệu trả về với góc roll, pitch, và yaw
    char statusMessage[50];
    snprintf(statusMessage, sizeof(statusMessage), "Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll_input, pitch_input, yaw);

    // Gán dữ liệu cho pReadCharacteristic
    pReadCharacteristic->setValue(statusMessage);
  }
};

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

void setup() {
  Serial.begin(115200);
  setupBLE();
  // setupMPU6050();
  setupCamera();
  setupWiFi();
}

void setupBLE() {
  NimBLEDevice::init("TRUNG HIEU Drone");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  NimBLEService* pService = pServer->createService(SERVICE_UUID);

  pWriteCharacteristic = pService->createCharacteristic(
    DRONE_CONTROL_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE_NR
  );
  pWriteCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  pReadCharacteristic = pService->createCharacteristic(
    DRONE_INFO_CHAR_UUID,
    NIMBLE_PROPERTY::READ
  );
  pReadCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  pService->start();
  NimBLEDevice::getAdvertising()->addServiceUUID(SERVICE_UUID);
  NimBLEDevice::getAdvertising()->start();

  Serial.println("BLE Server is started and advertising");
  Serial.println(NimBLEDevice::getAddress().toString().c_str());
}
/*
void setupMPU6050() {
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  pitch_setpoint = roll_setpoint = 0;

  pitch_pid = pid_create(&pitch_control, &pitch_input, &pitch_output, &pitch_setpoint, Kd, Ki, Kp);
  roll_pid = pid_create(&roll_control, &roll_input, &roll_output, &roll_setpoint, Kp, Ki, Kd);

  pid_limits(pitch_pid, -8191, 8191);
  pid_limits(roll_pid, -8191, 8191);

  pid_auto(pitch_pid);
  pid_auto(roll_pid);
}
*/

void setupCamera() {
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
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 10;
  config.fb_count = 1;

  if (!psramFound()) {
    Serial.println("PSRAM not found");
    return;
  } else {
    Serial.println("PS Ram founded");
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }
  Serial.println("Camera init Successful");

  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID == OV2640_PID) {
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
}

void setupWiFi() {
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting...");
  }
  Serial.print("Connected to WiFi! IP Address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
/*
  calculate_angle(&roll_input, &pitch_input);
  
  pid_compute(pitch_pid);
  pid_compute(roll_pid);
  updateMotors();
*/
  captureAndHandleImage();
}

void updateMotors() {
  duty_1 = base_throttle + pitch_output - roll_output;
  duty_2 = base_throttle + pitch_output + roll_output;
  duty_3 = base_throttle - pitch_output + roll_output;
  duty_4 = base_throttle - pitch_output - roll_output;

  duty_1 = constrain(duty_1, 0, 8191);
  duty_2 = constrain(duty_2, 0, 8191);
  duty_3 = constrain(duty_3, 0, 8191);
  duty_4 = constrain(duty_4, 0, 8191);

  ledcWrite(CHN_1, duty_1);
  ledcWrite(CHN_2, duty_2);
  ledcWrite(CHN_3, duty_3);
  ledcWrite(CHN_4, duty_4);
}

void captureAndHandleImage() {
  WiFiServer server(80);
  server.begin();
  while (true) {
    WiFiClient client = server.available();
    if (client) {
      Serial.println("New Client");
      String request = client.readStringUntil('\r');
      Serial.println(request);
      client.flush();

      // Chụp ảnh từ camera
      camera_fb_t * fb = esp_camera_fb_get();
      if (!fb) {
        Serial.println("Camera capture failed");
        return;
      }

      // Gửi dữ liệu ảnh tới client
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: image/jpeg");
      client.println("Content-Length: " + String(fb->len));
      client.println();
      client.write(fb->buf, fb->len);

      esp_camera_fb_return(fb);
      delay(1000);
    }
  }
}