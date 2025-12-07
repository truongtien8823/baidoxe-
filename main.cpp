#include <Arduino.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include <MFRC522.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "minhtien";
const char* password = "hongpeoi";
const char* mqtt_server = "mqtt.fuvitech.vn";
const int mqtt_port = 2883;

#define SERVO_IN_PIN   27
#define SERVO_OUT_PIN  14
#define RST_PIN        2
#define SS_IN_PIN      5
#define SS_OUT_PIN     17

#define IR_IN          26 
#define IR_MID         25  
#define IR_OUT         32  

#define BUZZER_PIN     4
#define NR_OF_READERS  2

#define CONTROL_PIN    13
#define BUTTON_PIN     36
#define CARD_SERVO_PIN 12

LiquidCrystal_I2C lcd(0x27, 16, 2);
byte ssPins[] = {SS_IN_PIN, SS_OUT_PIN};
MFRC522 mfrc522[NR_OF_READERS];

Servo servoIn;
Servo servoOut;
Servo cardServo;

WiFiClient espClient;
PubSubClient client(espClient);

struct Vehicle {
  String uid;
  String plate;
};
Vehicle vehicles[10];  
int vehicleCount = 0;  
String currentPlateVao = "";  
String currentPlateRa = "";  
bool hasVehicleVao = false;   
bool hasVehicleRa = false;   
bool isParkingFull = false;
int wrongAttempts = 0;
unsigned long servoOpenTime = 0;
const long SERVO_OPEN_DURATION = 3000;
unsigned long buzzerAlarmTime = 0;
const long BUZZER_ALARM_DURATION = 3000;
String lastDisplay = "";

bool lastButtonState = HIGH;  
unsigned long cardServoTime = 0;
const long CARD_SERVO_DURATION = 1000; 
const int CARD_HOME_ANGLE = 40;
const int CARD_PUSH_ANGLE = 140;

enum VaoState {
  WAITING_FOR_VEHICLE,    // Chờ xe vào (COXEVAO = 1)
  WAITING_FOR_PLATE,      // Chờ biển số
  WAITING_FOR_BUTTON,     // Chờ nhấn nút
  CARD_DISPENSING,        // Đang đẩy thẻ
  USER_SCANNING,          // Người dùng tự quét thẻ
  WAITING_VEHICLE_EXIT    // Chờ xe đi (COXEVAO = 0)
};
VaoState currentVaoState = WAITING_FOR_VEHICLE;
String currentVaoUID = "";     // UID đang xử lý
String currentVaoPlate = "";   // Biển số đang xử lý

bool parkingSlot[3] = {false, false, false}; 
bool lastParkingSlot[3] = {false, false, false};
unsigned long lastParkingUpdate = 0;
const long PARKING_UPDATE_INTERVAL = 2000; 
bool needUpdateParkingDisplay = true;

const char* TOPIC_CO_XE_VAO = "DACN/BAIXE/COXEVAO";
const char* TOPIC_BIEN_SO_VAO = "DACN/BAIXE/BIENSOVAO";
const char* TOPIC_CO_XE_RA = "DACN/BAIXE/COXERA";
const char* TOPIC_BIEN_SO_RA = "DACN/BAIXE/BIENSORA";

// hàm còi kêu khi sai 3 lần hoặc khi quét thẻ, ... 
void buzzerBeep(int duration = 150) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(duration);
  digitalWrite(BUZZER_PIN, LOW);
}
// hàm báo động khi sai quá 3 lần
void buzzerAlarm() {
  buzzerAlarmTime = millis();
  wrongAttempts = 0;
}

// Chuyển mảng byte thành chuỗi String ở định dạng HEX
String byteArrayToString(byte *buffer, byte bufferSize) {
  String str = "";
  for (byte i = 0; i < bufferSize; i++) {
    if (buffer[i] < 0x10) str += "0";
    str += String(buffer[i], HEX);
  }
  str.toUpperCase();
  return str;
}


// Thêm xe vào bãi
void addVehicle(String uid, String plate) {
  if (vehicleCount < 10) {
    vehicles[vehicleCount].uid = uid;
    vehicles[vehicleCount].plate = plate;
    vehicleCount++;
    Serial.print("Them xe vao bai: ");
    Serial.print(uid); Serial.print(" - "); Serial.println(plate);
    Serial.print("So xe trong bai: "); Serial.println(vehicleCount);
  }
}


// Xóa xe khỏi bãi
bool removeVehicle(String uid, String plate) {
  for (int i = 0; i < vehicleCount; i++) {
    if (vehicles[i].uid == uid && vehicles[i].plate == plate) {
      // Xóa xe bằng cách dịch chảy các xe phía sau lên trước
      for (int j = i; j < vehicleCount - 1; j++) {
        vehicles[j] = vehicles[j + 1];
      }
      vehicleCount--;
      Serial.print("Xoa xe khoi bai: ");
      Serial.print(uid); Serial.print(" - "); Serial.println(plate);
      Serial.print("So xe con lai: "); Serial.println(vehicleCount);
      return true;
    }
  }
  return false;
}

// Kiểm tra xe có trong bãi không
bool isVehicleInParking(String uid, String plate) {
  for (int i = 0; i < vehicleCount; i++) {
    if (vehicles[i].uid == uid && vehicles[i].plate == plate) {
      return true;
    }
  }
  return false;
}


// Mở servo cổng vào
void openServoIn() {
  servoIn.write(180);
  servoOpenTime = millis();
  lcd.clear();
  lcd.print("  VAO BAI XE");
  lcd.setCursor(0, 1);
  lcd.print("So xe: "); lcd.print(vehicleCount);
  buzzerBeep(300);
}

// Mở servo cổng ra
void openServoOut() {
  servoOut.write(180);
  servoOpenTime = millis();
  lcd.clear();
  lcd.print("  RA KHOI BAI");
  lcd.setCursor(0, 1);
  lcd.print("So xe: "); lcd.print(vehicleCount);
  buzzerBeep(300);
  delay(200);
  buzzerBeep(300);
}

// Đóng cả 2 servo
void closeServos() {
  servoIn.write(0);
  servoOut.write(0);
  servoOpenTime = 0;
}

// Đẩy thẻ ra cho người dùng
void pushCard() {
  cardServo.write(CARD_PUSH_ANGLE);
  cardServoTime = millis();
  lcd.clear();
  lcd.print("DANG CAP THE...");
  buzzerBeep(200);
}

// Trả servo thẻ về vị trí ban đầu
void returnCardServoHome() {
  cardServo.write(CARD_HOME_ANGLE);
  cardServoTime = 0;
}

// Kiểm tra trạng thái bãi xe từ cảm biến hồng ngoại
void checkParkingStatus() {
  bool ir1 = !digitalRead(IR_IN);
  bool ir2 = !digitalRead(IR_MID);
  bool ir3 = !digitalRead(IR_OUT);
  
  parkingSlot[0] = ir1;  // Vị trí 1 (gần cổng vào)
  parkingSlot[1] = ir2;  // Vị trí 2 (giữa)
  parkingSlot[2] = ir3;  // Vị trí 3 (gần cổng ra)
  
  for (int i = 0; i < 3; i++) {
    if (parkingSlot[i] != lastParkingSlot[i]) {
      needUpdateParkingDisplay = true;
      lastParkingSlot[i] = parkingSlot[i];
    }
  }
  
  isParkingFull = (ir1 && ir2 && ir3);

  if (isParkingFull && lastDisplay != "FULL") {
    lcd.clear();
    lcd.print("DAY XE");
    lcd.setCursor(0, 1);
    lcd.print(" Khong the vao");
    buzzerBeep(500);
    lastDisplay = "FULL";
  } else if (!isParkingFull && lastDisplay == "FULL") {
    lastDisplay = "";
    needUpdateParkingDisplay = true;
  }
}

// Hiển thị trạng thái bãi xe lên LCD
void displayParkingStatus() {
  if ((needUpdateParkingDisplay || (millis() - lastParkingUpdate >= PARKING_UPDATE_INTERVAL)) 
      && servoOpenTime == 0 && buzzerAlarmTime == 0 && !isParkingFull) {
    
    lastParkingUpdate = millis();
    needUpdateParkingDisplay = false;
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Vi tri: 1 2 3");
    lcd.setCursor(0, 1);
    lcd.print("Trang:  ");
    
    for (int i = 0; i < 3; i++) {
      lcd.setCursor(8 + i*2, 1);
      if (parkingSlot[i]) {
        lcd.print("X"); // Có xe
      } else {
        lcd.print("O"); // Trống
      }
    }
    
    lastDisplay = "PARKING_STATUS";
  }
}

// Xử lý tin nhắn MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  message.trim();
  // Xử lý các topic
  if (String(topic) == TOPIC_CO_XE_VAO) { // COXEVAO
    hasVehicleVao = (message == "1");
    Serial.print("COXEVAO: "); Serial.println(hasVehicleVao ? "CO" : "KHONG");
  }
  else if (String(topic) == TOPIC_BIEN_SO_VAO) {
    currentPlateVao = message;
    Serial.print("BIENSOVAO: "); Serial.println(currentPlateVao);
  }
  else if (String(topic) == TOPIC_CO_XE_RA) {
    hasVehicleRa = (message == "1");
    Serial.print("COXERA: "); Serial.println(hasVehicleRa ? "CO" : "KHONG");
  }
  else if (String(topic) == TOPIC_BIEN_SO_RA) {
    currentPlateRa = message;
    Serial.print("BIENSORA: "); Serial.println(currentPlateRa);
  }
}

// Kết nối lại MQTT nếu bị ngắt
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe(TOPIC_CO_XE_VAO);
      client.subscribe(TOPIC_BIEN_SO_VAO);
      client.subscribe(TOPIC_CO_XE_RA);
      client.subscribe(TOPIC_BIEN_SO_RA);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5s");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  pinMode(CONTROL_PIN, OUTPUT); 
  digitalWrite(CONTROL_PIN, HIGH);

  // WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  // MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // Chân
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(IR_IN, INPUT);
  pinMode(IR_MID, INPUT);
  pinMode(IR_OUT, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  servoIn.attach(SERVO_IN_PIN);
  servoOut.attach(SERVO_OUT_PIN);
  cardServo.attach(CARD_SERVO_PIN);
  servoIn.write(0);
  servoOut.write(0);
  cardServo.write(CARD_HOME_ANGLE);

  // LCD
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.print("  KHOI DONG...");
  delay(1000);
  lcd.clear();
  lcd.print("Cho xe den...");

  // RFID
  SPI.begin();
  for (uint8_t i = 0; i < NR_OF_READERS; i++) {
    mfrc522[i].PCD_Init(ssPins[i], RST_PIN);
  }

  Serial.println("He thong san sang!");
}

void loop() {
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();
  // Kiểm tra trạng thái bãi xe
  checkParkingStatus();


  if (servoOpenTime > 0 && millis() - servoOpenTime >= SERVO_OPEN_DURATION) {
    closeServos();
  }

  // Xử lý servo đẩy thẻ
  if (cardServoTime > 0 && millis() - cardServoTime >= CARD_SERVO_DURATION) {
    returnCardServoHome();
    
    if (currentVaoState == CARD_DISPENSING) {
      currentVaoState = USER_SCANNING;
      Serial.println("State: USER_SCANNING");
      
      lcd.clear();
      lcd.print("QUET THE VAO");
      lcd.setCursor(0, 1);
      lcd.print("READER!");
    }
  }

  // Xử lý nút nhấn
  bool buttonState = digitalRead(BUTTON_PIN);
  if (lastButtonState == HIGH && buttonState == LOW && currentVaoState == WAITING_FOR_BUTTON) {
    // Nút được nhấn
    pushCard(); // Đẩy thẻ ra
    currentVaoState = CARD_DISPENSING; 
    Serial.println("State: CARD_DISPENSING");
  }
  lastButtonState = buttonState;

  // Báo động
  if (buzzerAlarmTime > 0) {
    if (millis() - buzzerAlarmTime < BUZZER_ALARM_DURATION) {
      digitalWrite(BUZZER_PIN, HIGH);
    } else {
      digitalWrite(BUZZER_PIN, LOW);
      buzzerAlarmTime = 0;
      wrongAttempts = 0;
    }
    return;
  }
  // Xử lý trạng thái xe vào
  switch (currentVaoState) {
    case WAITING_FOR_VEHICLE: // Chờ xe vào
      if (hasVehicleVao) {
        currentVaoState = WAITING_FOR_PLATE; // Chờ biển số
        Serial.println("State: WAITING_FOR_PLATE");
      }
      break;
      
    case WAITING_FOR_PLATE: // Chờ biển số
      if (currentPlateVao != "" && currentPlateVao != "unknown") {
        if (isParkingFull || vehicleCount >= 10) {
          lcd.clear();
          lcd.print("BAI XE DAY!");
          buzzerBeep(800);
          currentVaoState = WAITING_VEHICLE_EXIT; // Chờ xe đi
        } else {
          currentVaoUID = "";
          currentVaoPlate = currentPlateVao; // Lưu biển số hiện tại
          currentVaoState = WAITING_FOR_BUTTON;
          
          lcd.clear();
          lcd.print("NHAN NUT DE");
          lcd.setCursor(0, 1);
          lcd.print("CAP THE");
          
          Serial.print("State: WAITING_FOR_BUTTON - Plate: ");
          Serial.println(currentVaoPlate);
        }
      }
      break;
      
    case WAITING_FOR_BUTTON:
      // Xử lý trong phần button handling
      break;
      
    case CARD_DISPENSING:
      // Xử lý trong phần servo handling
      break;
      
    case USER_SCANNING:
      if (millis() % 2000 < 1000) { // Nhấp nháy
        lcd.clear();
        lcd.print("QUET THE VAO");
        lcd.setCursor(0, 1);
        lcd.print("READER!");
      } else {
        lcd.clear(); // Xóa màn hình khi không nhấp nháy
      }
      break;
      
    case WAITING_VEHICLE_EXIT: // Chờ xe đi
      // Hiển thị thông báo đã vào thành công
      if (lastDisplay != "VEHICLE_EXIT") {
        lcd.clear();
        lcd.print("XE DA VAO!");
        lcd.setCursor(0, 1);
        lcd.print("Cho xe di het");
        lastDisplay = "VEHICLE_EXIT";
      }
      
      if (!hasVehicleVao) { // COXEVAO = 0
        currentVaoState = WAITING_FOR_VEHICLE;
        lastDisplay = ""; // Reset display
        Serial.println("Cycle complete, State: WAITING_FOR_VEHICLE");
      }
      break;
  }
  
  // Hiển thị trạng thái bãi xe khi chờ
  if (currentVaoState == WAITING_FOR_VEHICLE && 
      (!hasVehicleRa || currentPlateRa == "" || currentPlateRa == "unknown")) {
    displayParkingStatus(); // Hiển thị trạng thái bãi xe
    delay(100);
    return;
  }

  // CHỈ CHO ĐỌC THẺ RFID KHI CẦN THIẾT
  for (uint8_t reader = 0; reader < NR_OF_READERS; reader++) {
    // Reader 0 (VÀO): Chỉ đọc khi có xe vào + có biển số
    // Reader 1 (RA): Chỉ đọc khi có xe ra + có biển số
    bool shouldReadCard = false;
    
    if (reader == 0) {
      // Chỉ cho quét thẻ khi ở trạng thái USER_SCANNING
      shouldReadCard = (currentVaoState == USER_SCANNING);
      
      // Debug log - chỉ in khi state thay đổi
      static VaoState lastDebugState = WAITING_FOR_VEHICLE;
      if (!shouldReadCard && currentVaoState != lastDebugState) {
        Serial.print("DEBUG Reader 0 - State: ");
        Serial.println(currentVaoState);
        lastDebugState = currentVaoState;
      }
    } else if (reader == 1) {
      // Reader 1 (RA) - luôn cho phép quét khi có xe ra
      shouldReadCard = (hasVehicleRa && currentPlateRa != "" && currentPlateRa != "unknown");
    }
    
    if (shouldReadCard && mfrc522[reader].PICC_IsNewCardPresent() && mfrc522[reader].PICC_ReadCardSerial()) {
      String uid = byteArrayToString(mfrc522[reader].uid.uidByte, mfrc522[reader].uid.size);
      Serial.print("Reader "); Serial.print(reader == 0 ? "IN" : "OUT");
      Serial.print(" - Doc the cuoi cung: "); Serial.println(uid);

      buzzerBeep(150);

      // XỬ LÝ XE VÀO (reader 0)
      // XỬ LÝ XE VÀO (reader 0)
      if (reader == 0) {
        if (currentVaoState == USER_SCANNING) {
          // Người dùng quét thẻ - cập nhật biển số cuối cùng
          String finalPlate = (currentPlateVao != "" && currentPlateVao != "unknown") ? currentPlateVao : currentVaoPlate;
          addVehicle(uid, finalPlate);
          currentVaoState = WAITING_VEHICLE_EXIT;
          
          // Hiển thị ngay lập tức
          lcd.clear();
          lcd.print("XE DA VAO!");
          lcd.setCursor(0, 1);
          lcd.print("Cho xe di het");
          lastDisplay = "VEHICLE_EXIT";
          
          openServoIn(); // Mở cổng vào
          
          Serial.println("User scanned card successfully!");
          Serial.print("UID: "); Serial.println(uid);
          Serial.print("Saved with Plate: "); Serial.println(finalPlate);
        }
      }

      // XỬ LÝ XE RA (reader 1)
      else if (reader == 1) {
        // CHỈ XỬ LÝ KHI CÓ XE RA + CÓ BIỂN SỐ
        if (!hasVehicleRa || currentPlateRa == "" || currentPlateRa == "unknown") {
          lcd.clear();
          lcd.print("CHO XE DEN RA");
          lcd.setCursor(0, 1);
          lcd.print("DE DOC THE!");
          buzzerBeep(300);
        }
        else if (vehicleCount == 0) {
          lcd.clear();
          lcd.print(" Khong co xe!");
          buzzerBeep(600);
        }
        else { // Xử lý xe ra bình thường
          bool vehicleFound = isVehicleInParking(uid, currentPlateRa);
          
          Serial.print("DEBUG - Tim xe: UID="); Serial.print(uid);
          Serial.print(", Plate="); Serial.println(currentPlateRa);
          Serial.print("Danh sach xe trong bai:");
          for (int i = 0; i < vehicleCount; i++) {
            Serial.print(" ["); Serial.print(vehicles[i].uid);
            Serial.print("-"); Serial.print(vehicles[i].plate); Serial.print("]");
          }
          Serial.println();

          if (vehicleFound) {
            removeVehicle(uid, currentPlateRa);
            openServoOut();
            wrongAttempts = 0;
            Serial.println("Vehicle removed successfully");
          }
          else {
            wrongAttempts++;
            lcd.clear();
            lcd.print("   LOI XAC THUC");
            lcd.setCursor(0, 1);
            lcd.print("THE/BIEN SAI");

            buzzerBeep(800);
            delay(500);
            buzzerBeep(800);

            if (wrongAttempts >= 3) {
              lcd.clear();
              lcd.print("CANH BAO!");
              buzzerAlarm();
            }
          }
        }
      }

      mfrc522[reader].PICC_HaltA();
      mfrc522[reader].PCD_StopCrypto1();
    }
  }

  // Hiển thị trạng thái cuối chỉ khi không ở trong chu trình vào
  if (!isParkingFull && servoOpenTime == 0 && buzzerAlarmTime == 0 && 
      currentVaoState == WAITING_FOR_VEHICLE) {
    if (vehicleCount == 0) {
      displayParkingStatus();
    } else if (vehicleCount > 0 && lastDisplay != "CHO_RA") {
      lcd.clear();
      lcd.print("Co "); lcd.print(vehicleCount); lcd.print(" xe trong bai");
      lcd.setCursor(0, 1);
      lcd.print("Cho ra xe...");
      lastDisplay = "CHO_RA";
    }
  }

  delay(100);
}