#include <Arduino.h>
#include <Wire.h>
#include <string>
#include <NimBLEDevice.h>
#include <MPU6050.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "../include/home_wifi_multi.h"
#include "../include/control.h"
#include "../include/PID.h"

//Prototype 

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

float base_throttle = 4915;
float duty_1;
float duty_2;
float duty_3;
float duty_4;

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

    bool button1 = buttonState & (1 << 0);
    bool button2 = buttonState & (1 << 1);
    bool button3 = buttonState & (1 << 2);
    bool button4 = buttonState & (1 << 3);
    bool button5 = buttonState & (1 << 4);
    bool button6 = buttonState & (1 << 5);
    bool button7 = buttonState & (1 << 6);

    switch (button1) {
      case 0: {
        Serial.println("Up Reset");
        break;
      }

      case 1: {
        Serial.println("Up Set");
        break;
      }
    }

    switch (button2) {
      case 0: {
        Serial.println("Down Reset");
        break;
      }

      case 1: {
        Serial.println("Down Set");
        break;
      }
    }

    switch (button3) {
      case 0: {
        Serial.println("Left Reset");
        break;
      }

      case 1: {
        Serial.println("Left Set");
        break;
      }
    }

    switch (button4) {
      case 0: {
        Serial.println("Right Reset");
        break;
      }

      case 1: {
        Serial.println("Right Set");
        break;
      }
    }

    switch (button5) {
      case 0: {
        Serial.println("High Reset");
        break;
      }

      case 1: {
        Serial.println("High Set");
        break;
      }
    }

    switch (button6) {
      case 0: {
        Serial.println("Low Reset");
        break;
      }

      case 1: {
        Serial.println("Low Set");
        break;
      }
    }

    switch (button7) {
      case 0: {
        Serial.println("Capture Reset");
        break;
      }

      case 1: {
        Serial.println("Capture Set");
        break;
      }
    }

    std::string notificationData = "Buttons: ";
    notificationData += (button1 ? "Up " : "");
    notificationData += (button2 ? "Down " : "");
    notificationData += (button3 ? "Left " : "");
    notificationData += (button4 ? "Right " : "");
    notificationData += (button5 ? "High " : "");
    notificationData += (button6 ? "Low " : "");

    Serial.println(notificationData.c_str());

    // Send notification to client
    pWriteCharacteristic->setValue(notificationData); // Gán giá trị thông báo
    pWriteCharacteristic->notify(); // Thông báo client
  }

  void onRead(NimBLECharacteristic* pReadCharacteristic) {
    Serial.println("Client is reading data");

    // Gán dữ liệu trả về khi client đọc
    std::string response = "Hello Client!";
    pReadCharacteristic->setValue(response);
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

// Thiết lập BLE Server
void setup() {
  Serial.begin(115200);

  // Khởi tạo BLE và đặt tên cho thiết bị
  NimBLEDevice::init("TRUNG HIEU Drone");

  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  // Tạo BLE server
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Tạo một Service mới
  NimBLEService* pService = pServer->createService(SERVICE_UUID);

  // Tạo một Characteristic mới với callback để xử lý việc ghi dữ liệu
  pWriteCharacteristic = pService->createCharacteristic(
    DRONE_CONTROL_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE_NR // Write Without Response để gửi gói tin nhỏ nhất
  );
  pWriteCharacteristic->setCallbacks(new MyCharacteristicCallbacks()); // Gán callback cho characteristic

  // Đặt giá trị ban đầu cho Characteristic
  pWriteCharacteristic->setValue("H"); // Giá trị ngắn để tối ưu gói tin

  pReadCharacteristic = pService->createCharacteristic(
    DRONE_INFO_CHAR_UUID,
    NIMBLE_PROPERTY::READ
  );
  pReadCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  pReadCharacteristic->setValue("H");
  // Bắt đầu Service
  pService->start();

  // Bắt đầu quảng bá BLE
  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();
  
  Serial.println("BLE Server is started and advertising");

  Serial.println("MAC Address of BLE Server: ");
  Serial.println(NimBLEDevice::getAddress().toString().c_str());

  // Wire.begin();
  // mpu.initialize();
  // if (!mpu.testConnection()) {
  //   Serial.println("MPU6050 connection failed");
  //   while (1);
  // }

  pitch_setpoint = 0;
  roll_setpoint = 0;

  pitch_pid = pid_create(&pitch_control, &pitch_input, &pitch_output, &pitch_setpoint, Kd, Ki, Kp);
  roll_pid = pid_create(&roll_control, &roll_input, &roll_output, &roll_setpoint, Kp, Ki, Kd);

  pid_limits(pitch_pid, -8191, 8191);
  pid_limits(roll_pid, -8191, 8191);

  pid_auto(pitch_pid);
  pid_auto(roll_pid);
  
  ledcSetup(CHN_1, FREQ, RES);
  ledcAttachPin(MOT1, CHN_1);
  ledcSetup(CHN_2, FREQ, RES);
  ledcAttachPin(MOT2, CHN_2);
  ledcSetup(CHN_3, FREQ, RES);
  ledcAttachPin(MOT3, CHN_3);
  ledcSetup(CHN_4, FREQ, RES);
  ledcAttachPin(MOT4, CHN_4);

  // Kết nối với Wi-Fi
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");

  // Chờ đến khi kết nối thành công
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting...");
  }

  // In địa chỉ IP khi đã kết nối thành công
  Serial.println("Connected to WiFi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // calculate_angle(&roll_input, &pitch_input);

  // pid_compute(pitch_pid);
  // pid_compute(roll_pid);
  // duty_1 = base_throttle + pitch_output - roll_output;
  // duty_2 = base_throttle + pitch_output + roll_output;
  // duty_3 = base_throttle - pitch_output + roll_output;
  // duty_4 = base_throttle - pitch_output - roll_output;
  // duty_1 = constrain(duty_1, 0, 8191);
  // duty_2 = constrain(duty_2, 0, 8191);
  // duty_3 = constrain(duty_3, 0, 8191);
  // duty_4 = constrain(duty_4, 0, 8191);
  // // Serial.print("Mot 1: "); Serial.print(duty_1);
  // // Serial.print(", Mot 2: "); Serial.print(duty_2);
  // // Serial.print(", Mot 3: "); Serial.print(duty_3);
  // // Serial.print(", Mot 4: "); Serial.println(duty_4);
  // ledcWrite(CHN_1, duty_1);
  // ledcWrite(CHN_2, duty_2);
  // ledcWrite(CHN_3, duty_3);
  // ledcWrite(CHN_4, duty_4);
  // delay(100); // Delay for 10ms (adjust as needed
}

/*
#include "../include/OV2640.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"


OV2640 cam;

WebServer server(80);

const char HEADER[] = "HTTP/1.1 200 OK\r\n" \
                      "Access-Control-Allow-Origin: *\r\n" \
                      "Content-Type: multipart/x-mixed-replace; boundary=123456789000000000000987654321\r\n";
const char BOUNDARY[] = "\r\n--123456789000000000000987654321\r\n";
const char CTNTTYPE[] = "Content-Type: image/jpeg\r\nContent-Length: ";
const int hdrLen = strlen(HEADER);
const int bdrLen = strlen(BOUNDARY);
const int cntLen = strlen(CTNTTYPE);

void handle_jpg_stream(void)
{
  char buf[32];
  int s;

  WiFiClient client = server.client();

  client.write(HEADER, hdrLen);
  client.write(BOUNDARY, bdrLen);

  while (true)
  {
    if (!client.connected()) break;
    cam.run();
    s = cam.getSize();
    client.write(CTNTTYPE, cntLen);
    sprintf( buf, "%d\r\n\r\n", s );
    client.write(buf, strlen(buf));
    client.write((char *)cam.getfb(), s);
    client.write(BOUNDARY, bdrLen);
  }
}

const char JHEADER[] = "HTTP/1.1 200 OK\r\n" \
                       "Content-disposition: inline; filename=capture.jpg\r\n" \
                       "Content-type: image/jpeg\r\n\r\n";
const int jhdLen = strlen(JHEADER);

void handle_jpg(void)
{
  WiFiClient client = server.client();

  cam.run();
  if (!client.connected()) return;

  client.write(JHEADER, jhdLen);
  client.write((char *)cam.getfb(), cam.getSize());
}

void handleNotFound()
{
  String message = "Server is running!\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  server.send(200, "text / plain", message);
}

void setup()
{

  Serial.begin(115200);
  //while (!Serial);            //wait for serial connection.

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
  config.pixel_format = PIXFORMAT_JPEG;

  // Frame parameters
  //  config.frame_size = FRAMESIZE_UXGA;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 2;

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  cam.init(config);

  IPAddress ip;

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID1, PWD1);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(F("."));
  }
  ip = WiFi.localIP();
  Serial.println(F("WiFi connected"));
  Serial.println("");
  Serial.println(ip);
  Serial.print("Stream Link: http://");
  Serial.print(ip);
  Serial.println("/mjpeg/1");
  server.on("/mjpeg/1", HTTP_GET, handle_jpg_stream);
  server.on("/jpg", HTTP_GET, handle_jpg);
  server.onNotFound(handleNotFound);
  server.begin();
}

void loop()
{
  server.handleClient();
}
*/