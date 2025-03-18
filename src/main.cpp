#include <ESP8266WiFi.h>
#include <Servo.h>
#include <espnow.h>

typedef struct struct_message {
  bool L1Pressed;
  bool L2Pressed;
  bool L3Pressed;
  bool L4Pressed;
  bool R1Pressed;
  bool R2Pressed;
  bool R3Pressed;
  bool R4Pressed;
  int joy1_x;
  int joy1_y;
  int joy2_x;
  int joy2_y;    
} struct_message;

struct_message receivedData;

// Konfiguracja silników
const int PWMA = 5;  // Prawy silnik - kontrola prędkości
const int PWMB = 4;  // Lewy silnik - kontrola prędkości
const int DA = 0;    // Prawy silnik - kierunek
const int DB = 2;    // Lewy silnik - kierunek

const int SERVO_PIN = 14; // GPIO14 (D5) 
const int neutralPosition = 100;  // Pozycja neutralna serwa

const int JOYSTICK_DEADZONE = 10;

Servo myServo;

int mapJoystickToSpeed(int value) {
  if (abs(value) < JOYSTICK_DEADZONE) {
    return 0; 
  }
  
  int mappedSpeed = map(value, -100, 100, -255, 255); 
  return mappedSpeed;
}

void onDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));

  int leftMotorSpeed = mapJoystickToSpeed(receivedData.joy1_y);
  int rightMotorSpeed = mapJoystickToSpeed(receivedData.joy2_y);

  // Kontrola lewego silnika (joystick 1)
  if (leftMotorSpeed < 0) {
    digitalWrite(DB, LOW);  // Kierunek do przodu
    analogWrite(PWMB, -leftMotorSpeed);
  } else if (leftMotorSpeed > 0) {
    digitalWrite(DB, HIGH); // Kierunek do tyłu
    analogWrite(PWMB, leftMotorSpeed);
  } else {
    analogWrite(PWMB, 0);  // Zatrzymanie silnika
  }

  // Kontrola prawego silnika (joystick 2)
  if (rightMotorSpeed < 0) {
    digitalWrite(DA, LOW);  // Kierunek do przodu
    analogWrite(PWMA, -rightMotorSpeed);
  } else if (rightMotorSpeed > 0) {
    digitalWrite(DA, HIGH); // Kierunek do tyłu
    analogWrite(PWMA, rightMotorSpeed);
  } else {
    analogWrite(PWMA, 0);  // Zatrzymanie silnika
  }
  // Kontrola serwa
  if (receivedData.R1Pressed) {
    myServo.write(20);  
  } else if (receivedData.R2Pressed) {
    myServo.write(180); 
  } else {
    myServo.write(neutralPosition); // Powrót do pozycji neutralnej
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);  

  if (esp_now_init() != 0) {
    Serial.println("Błąd inicjalizacji ESP-NOW!");
    return;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(onDataRecv);

  myServo.attach(SERVO_PIN);
  myServo.write(neutralPosition);  // Ustaw serwo na pozycję neutralną

  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DA, OUTPUT);
  pinMode(DB, OUTPUT);

  Serial.print("Adres MAC ESP8266: ");
  Serial.println(WiFi.macAddress());
  
}

void loop() {
}
