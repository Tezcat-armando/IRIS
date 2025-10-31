#include <BluetoothSerial.h>

// Bluetooth
BluetoothSerial SerialBT;

// HC-SR04 Pins
const int trigPin = 4;
const int echoPin = 5;

// UART to STM32
#define STM32_UART Serial1
#define STM32_TX_PIN 8
#define STM32_RX_PIN 9

// Mode variables
int currentMode = 0; // 0: Bluetooth, 1: Distance
int bluetoothPWM = 0;

// HC-SR04 variables
long duration;
int distance;
int previousPWM = -1;

void setup() {
  Serial.begin(115200);
  
  // Initialize UART to STM32
  STM32_UART.begin(9600, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);
  
  // Initialize Bluetooth
  SerialBT.begin("ESP32_Motor_Control");
  
  // Initialize HC-SR04
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  Serial.println("🚀 ESP32 Started");
  Serial.println("Mode 0: Bluetooth Control");
  Serial.println("Mode 1: Distance Control");
  Serial.println("Send '0' or '1' via Bluetooth to switch modes");
}

void loop() {
  // Check for mode change via Bluetooth
  if (SerialBT.available()) {
    char modeChar = SerialBT.read();
    if (modeChar == '0') {
      currentMode = 0;
      Serial.println("🎮 Mode: Bluetooth Control");
      SerialBT.println("Mode: Bluetooth Control");
    } else if (modeChar == '1') {
      currentMode = 1;
      Serial.println("📏 Mode: Distance Control");
      SerialBT.println("Mode: Distance Control");
    } else if (currentMode == 0 && modeChar >= '0' && modeChar <= '9') {
      // Bluetooth PWM control (0-9)
      bluetoothPWM = map((modeChar - '0'), 0, 9, 0, 255);
      Serial.print("📱 Bluetooth PWM: ");
      Serial.println(bluetoothPWM);
    }
  }

  if (currentMode == 0) {
    // Bluetooth Mode
    if (bluetoothPWM != previousPWM) {
      sendPWMToSTM32(bluetoothPWM);
      previousPWM = bluetoothPWM;
    }
  } else {
    // Distance Mode
    int newPWM = calculatePWMFromDistance();
    if (newPWM != previousPWM) {
      sendPWMToSTM32(newPWM);
      previousPWM = newPWM;
    }
  }
  
  delay(100);
}

int calculatePWMFromDistance() {
  // Read distance from HC-SR04
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  
  // Map distance to PWM (Jauh → Cepat, Dekat → Lambat)
  int pwmValue;
  if (distance >= 50) {
    pwmValue = 255; // Max speed
  } else if (distance >= 30) {
    pwmValue = 200; // Fast
  } else if (distance >= 20) {
    pwmValue = 150; // Medium
  } else if (distance >= 10) {
    pwmValue = 100; // Slow
  } else if (distance >= 5) {
    pwmValue = 50;  // Very slow
  } else {
    pwmValue = 0;   // Stop
  }
  
  // Debug info
  Serial.print("📏 Distance: ");
  Serial.print(distance);
  Serial.print("cm | PWM: ");
  Serial.println(pwmValue);
  
  if (SerialBT.connected()) {
    SerialBT.print("Distance: ");
    SerialBT.print(distance);
    SerialBT.print("cm | PWM: ");
    SerialBT.println(pwmValue);
  }
  
  return pwmValue;
}

void sendPWMToSTM32(int pwmValue) {
  // Send PWM value to STM32 via UART
  STM32_UART.println(pwmValue);
  
  Serial.print("📤 Sent to STM32: ");
  Serial.println(pwmValue);
}
