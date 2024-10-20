#include <Arduino.h>
#include <string>
#include <NimBLEDevice.h>

#define UP_PIN        13 
#define DOWN_PIN      12  
#define LEFT_PIN      14
#define RIGHT_PIN     27
#define HIGH_PIN      26
#define LOW_PIN       25
#define CAP_PIN       33


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
  
  if (pClient->connect(BLEAddress("08:d1:f9:6b:00:1a"))) {
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

  pinMode(UP_PIN, INPUT_PULLUP);
  pinMode(DOWN_PIN, INPUT_PULLUP);
  pinMode(LEFT_PIN, INPUT_PULLUP);
  pinMode(RIGHT_PIN, INPUT_PULLUP);
  pinMode(HIGH_PIN, INPUT_PULLUP);
  pinMode(LOW_PIN, INPUT_PULLUP);
  pinMode(CAP_PIN, INPUT_PULLUP);
}

void loop() {
  // Giả sử bạn đọc trạng thái của các nút nhấn (button) qua các pin (ví dụ: GPIO pin)
  bool button1 = digitalRead(UP_PIN);
  bool button2 = digitalRead(DOWN_PIN);
  bool button3 = digitalRead(LEFT_PIN);
  bool button4 = digitalRead(RIGHT_PIN);
  bool button5 = digitalRead(HIGH_PIN);
  bool button6 = digitalRead(LOW_PIN);
  bool button7 = digitalRead(CAP_PIN);

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
  
  // Gửi tín hiệu điều khiển đến drone
  sendControlSignal(fixedState);

  delay(1000);  // Chu kỳ gửi dữ liệu
}
