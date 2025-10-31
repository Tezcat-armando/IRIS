#include <BluetoothSerial.h>

// Bluetooth Serial Object
BluetoothSerial SerialBT;

// HC-SR04 Pins
const int trigPin = 4;
const int echoPin = 5;

// UART to STM32 - ESP32 30 Pin Configuration
#define STM32_UART Serial2
#define STM32_TX_PIN 17  // GPIO 17 untuk TX ke STM32
#define STM32_RX_PIN 16  // GPIO 16 untuk RX dari STM32

// Mode Control Variables
int currentMode = 0;       // 0: Bluetooth Mode, 1: Distance Mode
int bluetoothPWM = 0;      // PWM value from Bluetooth (0-255)
int previousPWM = -1;      // Previous PWM value to avoid spam
bool systemActive = true;  // System enable/disable

// HC-SR04 Variables
long duration;
int distance;
unsigned long lastSensorRead = 0;
const int SENSOR_READ_INTERVAL = 100; // Read sensor every 100ms

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Initialize UART to STM32 with 115200 baud
  STM32_UART.begin(115200, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);
  
  // Initialize Bluetooth
  if (!SerialBT.begin("ESP32_Motor_Controller")) {
    Serial.println("❌ Bluetooth failed to initialize!");
    while (1); // Stop if Bluetooth fails
  }
  
  // Initialize HC-SR04 pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // Initial motor stop
  sendPWMToSTM32(0);
  
  Serial.println("🚀 ESP32 30-Pin Motor Controller Started");
  Serial.println("📡 Bluetooth Name: ESP32_Motor_Controller");
  Serial.println("🔧 Available Commands:");
  Serial.println("   '0' - Bluetooth Control Mode");
  Serial.println("   '1' - Distance Control Mode");
  Serial.println("   '0-9' - Speed control in Bluetooth mode");
  Serial.println("   's' - Stop motor");
  Serial.println("   'x' - Emergency stop");
  Serial.println("📏 Distance Mode: Far = Fast, Near = Slow");
}

void loop() {
  // Handle Bluetooth commands
  handleBluetoothCommands();
  
  // Execute current mode
  if (systemActive) {
    executeCurrentMode();
  }
  
  // Small delay to prevent overwhelming the system
  delay(50);
}

void handleBluetoothCommands() {
  if (SerialBT.available()) {
    char command = SerialBT.read();
    Serial.print("📱 Bluetooth Command: ");
    Serial.println(command);
    
    switch (command) {
      case '0': // Bluetooth Control Mode
        currentMode = 0;
        Serial.println("🎮 Mode: Bluetooth Control");
        SerialBT.println("Mode: Bluetooth Control");
        sendPWMToSTM32(0); // Stop when switching modes
        break;
        
      case '1': // Distance Control Mode
        currentMode = 1;
        Serial.println("📏 Mode: Distance Control");
        SerialBT.println("Mode: Distance Control");
        break;
        
      case 's': // Stop motor
      case 'S':
        bluetoothPWM = 0;
        sendPWMToSTM32(0);
        Serial.println("⏹️ Motor Stopped");
        SerialBT.println("Motor Stopped");
        break;
        
      case 'x': // Emergency stop
      case 'X':
        systemActive = false;
        sendPWMToSTM32(0);
        Serial.println("🛑 EMERGENCY STOP - System Disabled");
        SerialBT.println("EMERGENCY STOP - Send 'r' to resume");
        break;
        
      case 'r': // Resume system
      case 'R':
        systemActive = true;
        Serial.println("▶️ System Resumed");
        SerialBT.println("System Resumed");
        break;
