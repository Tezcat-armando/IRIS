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
        
      default:
        // Bluetooth PWM control (0-9)
        if (currentMode == 0 && command >= '0' && command <= '9') {
          bluetoothPWM = map((command - '0'), 0, 9, 0, 255);
          Serial.print("🎯 Bluetooth PWM: ");
          Serial.println(bluetoothPWM);
          SerialBT.print("PWM set to: ");
          SerialBT.println(bluetoothPWM);
        }
        break;
    }
  }
}

void executeCurrentMode() {
  int currentPWM = 0;
  
  if (currentMode == 0) {
    // Bluetooth Mode - Use value from Bluetooth
    currentPWM = bluetoothPWM;
  } else {
    // Distance Mode - Calculate PWM based on distance
    currentPWM = calculatePWMFromDistance();
  }
  
  // Send to STM32 only if PWM value changed
  if (currentPWM != previousPWM) {
    sendPWMToSTM32(currentPWM);
    previousPWM = currentPWM;
  }
}

int calculatePWMFromDistance() {
  // Read sensor only at specified intervals
  if (millis() - lastSensorRead < SENSOR_READ_INTERVAL) {
    return previousPWM; // Return previous value if not time to read yet
  }
  
  lastSensorRead = millis();
  
  // Read distance from HC-SR04
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH, 30000); // Timeout after 30ms
  distance = duration * 0.034 / 2;
  
  // Validate distance reading
  if (duration == 0 || distance > 200 || distance < 2) {
    Serial.println("❌ Invalid distance reading");
    return 0; // Stop motor on invalid reading
  }
  
  // Map distance to PWM (Jauh → Cepat, Dekat → Lambat)
  int pwmValue;
  if (distance >= 50) {
    pwmValue = 255; // Max speed - Far away
  } else if (distance >= 30) {
    pwmValue = 200; // Fast
  } else if (distance >= 20) {
    pwmValue = 150; // Medium
  } else if (distance >= 10) {
    pwmValue = 100; // Slow
  } else if (distance >= 5) {
    pwmValue = 50;  // Very slow
  } else {
    pwmValue = 0;   // Stop - Too close
  }
  
  // Debug info
  Serial.print("📏 Distance: ");
  Serial.print(distance);
  Serial.print("cm | PWM: ");
  Serial.println(pwmValue);
  
  // Send to Bluetooth if connected
  if (SerialBT.hasClient()) {
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

// Optional: Handle incoming data from STM32 (for encoder feedback, etc.)
void checkSTM32Response() {
  if (STM32_UART.available()) {
    String response = STM32_UART.readStringUntil('\n');
    response.trim();
    Serial.print("📥 STM32 Response: ");
    Serial.println(response);
    
    // Forward to Bluetooth if needed
    if (SerialBT.hasClient()) {
      SerialBT.print("STM32: ");
      SerialBT.println(response);
    }
  }
}

