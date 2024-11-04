#include <Arduino.h>
#include <string>
#include <NimBLEDevice.h>
#include "../include/joystick.h"

bool button1 = 0;
bool button2 = 0;
bool button3 = 0;
bool button4 = 0;
bool button5 = 0;
bool button6 = 0;
bool button7 = 0;

uint8_t currentState;

NimBLEClient* pClient = nullptr;
NimBLERemoteCharacteristic* pRemoteCharacteristic = nullptr;  // Sửa thành NimBLERemoteCharacteristic
static NimBLEUUID serviceUUID("00FF");       // UUID của dịch vụ trên drone
static NimBLEUUID charUUID("FF01");   // UUID của characteristic nhận tín hiệu điều khiển

// Callback khi kết nối BLE thành công
class MyClientCallback : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pClient) {
    Serial.println("Connected to drone!");
  }

  void onDisconnect(NimBLEClient* pClient) {
    Serial.println("Disconnected from drone.");
  }
};

// Hàm thiết lập kết nối BLE Client
bool connectToServer() {
  pClient = NimBLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback(), false);
  
  if (pClient->connect(BLEAddress("e0:5a:1b:6b:b2:3e"))) {
    Serial.println("Connected to server");
    NimBLERemoteService* pRemoteService = pClient->getService(serviceUUID);  // Lấy dịch vụ từ drone
    if (pRemoteService) {
      pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);  // Lấy characteristic
      if (pRemoteCharacteristic) {
        Serial.println("Remote characteristic found");
        return true;
      }
    }
  }
  return false;
}

// Gửi tín hiệu điều khiển qua BLE
void sendControlSignal(uint8_t buttonState) {
  if (pRemoteCharacteristic != nullptr) {
    pRemoteCharacteristic->writeValue(&buttonState, 1, 1);  // Ghi giá trị tới drone
    Serial.printf("Sent button state: %d\n", buttonState);
  }
}

void setup() {
  Serial.begin(115200);
  NimBLEDevice::init("ESP32 Controller");
  
  // Kết nối đến drone
  if (!connectToServer()) {
    Serial.println("Failed to connect to drone.");
    while (1);
  }
  joystick_setup();
  analogReadResolution(10);
  Serial.println("Init done");
}

void loop() {
  joystick_getdata(&button1,
                   &button2,
                   &button3,
                   &button4,
                   &button5,
                   &button6,
                   &button7);

  // Mã hóa trạng thái các nút thành 1 byte
  uint8_t buttonState = 0;
  buttonState |= button1 << 0;
  buttonState |= button2 << 1;
  buttonState |= button3 << 2;
  buttonState |= button4 << 3;
  buttonState |= button5 << 4;
  buttonState |= button6 << 5;
  buttonState |= button7 << 6;

  uint8_t fixedState = ~buttonState &0b1111111;
  
  if (fixedState != currentState) {
    // Gửi tín hiệu điều khiển đến drone
    sendControlSignal(fixedState);
    currentState = fixedState;
    Serial.print("Send: ");
    Serial.println(fixedState);
  }
}
