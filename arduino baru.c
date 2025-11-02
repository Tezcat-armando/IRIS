/**
 * ESP32 Master Controller with HC-SR04 Mode
 * UART Communication with STM32F401
 * 
 * Commands:
 * - PWM:value    : Manual motor control
 * - READ         : Read encoder
 * - DIST         : Read distance
 * - MODE:0       : Manual mode
 * - MODE:1       : Auto distance mode
 */

HardwareSerial SerialSTM32(1);

// UART Pins
#define STM32_RX_PIN 16
#define STM32_TX_PIN 17

// System Variables
int current_mode = 0; // 0=Manual, 1=Auto Distance
int manual_pwm = 0;
int auto_pwm = 0;
float current_distance = 0;
int encoder_count = 0;
String received_data = "";
unsigned long last_auto_time = 0;

void setup() {
  Serial.begin(115200);
  SerialSTM32.begin(9600, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);
  
  delay(1000);
  
  Serial.println("==========================================");
  Serial.println("   ESP32 Master with HC-SR04 Mode");
  Serial.println("==========================================");
  Serial.println("Commands:");
  Serial.println("  PWM:value  - Set motor speed (-255 to 255)");
  Serial.println("  READ       - Read encoder value");
  Serial.println("  DIST       - Read distance");
  Serial.println("  MODE:0     - Manual mode");
  Serial.println("  MODE:1     - Auto distance mode");
  Serial.println("  STATUS     - System status");
  Serial.println("==========================================");
}

void loop() {
  handleSerialInput();
  handleSTM32Data();
  
  // Auto mode processing
  if (current_mode == 1) {
    if (millis() - last_auto_time > 200) { // Update every 200ms
      calculateAutoPWM();
      sendPWMCommand(auto_pwm);
      last_auto_time = millis();
    }
  }
  
  delay(10);
}

void handleSerialInput() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.startsWith("PWM:")) {
      manual_pwm = input.substring(4).toInt();
      if (manual_pwm >= -255 && manual_pwm <= 255) {
        sendPWMCommand(manual_pwm);
        Serial.println("Manual PWM: " + String(manual_pwm));
      }
    }
    else if (input.equalsIgnoreCase("READ")) {
      sendReadCommand();
    }
    else if (input.equalsIgnoreCase("DIST")) {
      sendDistanceCommand();
    }
    else if (input.startsWith("MODE:")) {
      int new_mode = input.substring(5).toInt();
      if (new_mode == 0 || new_mode == 1) {
        current_mode = new_mode;
        sendModeCommand(current_mode);
        Serial.println("Mode: " + String(current_mode == 0 ? "Manual" : "Auto Distance"));
        
        // Stop motor when switching modes
        sendPWMCommand(0);
      }
    }
    else if (input.equalsIgnoreCase("STATUS")) {
      printSystemStatus();
    }
    else if (input.equalsIgnoreCase("AUTO_TEST")) {
      runAutoTest();
    }
    else {
      Serial.println("Unknown command");
    }
  }
}

void handleSTM32Data() {
  if (SerialSTM32.available()) {
    received_data = SerialSTM32.readStringUntil('\n');
    received_data.trim();
    processSTM32Response(received_data);
  }
}

void processSTM32Response(String response) {
  if (response.startsWith("ACK:PWM:")) {
    // Format: "ACK:PWM:value,ENC:count,DIST:distance"
    parseSensorData(response);
  }
  else if (response.startsWith("ENC:")) {
    encoder_count = response.substring(4).toInt();
    Serial.println("Encoder: " + String(encoder_count));
  }
  else if (response.startsWith("DIST:")) {
    current_distance = response.substring(5).toFloat();
    Serial.println("Distance: " + String(current_distance) + " cm");
  }
  else if (response.startsWith("MODE:")) {
    int mode = response.substring(5).toInt();
    Serial.println("STM32 Mode: " + String(mode));
  }
  else if (response.startsWith("ERROR:")) {
    Serial.println("STM32 Error: " + response);
  }
  else {
    // Debug output from STM32
    Serial.println("STM32: " + response);
  }
}

void parseSensorData(String data) {
  int pwm_start = data.indexOf("PWM:") + 4;
  int enc_start = data.indexOf("ENC:") + 4;
  int dist_start = data.indexOf("DIST:") + 5;
  
  int comma1 = data.indexOf(',', pwm_start);
  int comma2 = data.indexOf(',', enc_start);
  
  if (pwm_start != -1 && enc_start != -1 && dist_start != -1) {
    String pwm_str = data.substring(pwm_start, comma1);
    String enc_str = data.substring(enc_start, comma2);
    String dist_str = data.substring(dist_start);
    
    manual_pwm = pwm_str.toInt();
    encoder_count = enc_str.toInt();
    current_distance = dist_str.toFloat();
    
    Serial.println("PWM:" + pwm_str + " ENC:" + enc_str + " DIST:" + dist_str + "cm");
  }
}

void calculateAutoPWM() {
  // Distance-based control logic
  if (current_distance > 50.0) {
    auto_pwm = 255; // Far distance - full speed
  }
  else if (current_distance > 30.0) {
    auto_pwm = 200; // Medium distance - medium speed
  }
  else if (current_distance > 20.0) {
    auto_pwm = 150; // Close distance - slow speed
  }
  else if (current_distance > 10.0) {
    auto_pwm = 100; // Very close - very slow
  }
  else if (current_distance > 5.0) {
    auto_pwm = 50;  // Too close - minimal speed
  }
  else {
    auto_pwm = 0;   // Stop - object too close
  }
  
  // Optional: Add dead zone to prevent constant switching
  static int last_auto_pwm = 0;
  if (abs(auto_pwm - last_auto_pwm) > 10) {
    last_auto_pwm = auto_pwm;
  } else {
    auto_pwm = last_auto_pwm;
  }
}

void sendPWMCommand(int pwm_value) {
  String command = "PWM:" + String(pwm_value) + "\n";
  SerialSTM32.print(command);
}

void sendReadCommand() {
  SerialSTM32.print("READ\n");
}

void sendDistanceCommand() {
  SerialSTM32.print("DIST\n");
}

void sendModeCommand(int mode) {
  String command = "MODE:" + String(mode) + "\n";
  SerialSTM32.print(command);
}

void printSystemStatus() {
  Serial.println("=== SYSTEM STATUS ===");
  Serial.println("Mode: " + String(current_mode == 0 ? "Manual" : "Auto Distance"));
  Serial.println("Distance: " + String(current_distance) + " cm");
  Serial.println("Encoder: " + String(encoder_count));
  Serial.println("Manual PWM: " + String(manual_pwm));
  Serial.println("Auto PWM: " + String(auto_pwm));
  Serial.println("=====================");
}

void runAutoTest() {
  Serial.println("Starting Auto Mode Test...");
  
  // Switch to auto mode
  sendModeCommand(1);
  current_mode = 1;
  delay(1000);
  
  // Test for 10 seconds
  unsigned long test_start = millis();
  while (millis() - test_start < 10000) {
    handleSTM32Data();
    calculateAutoPWM();
    sendPWMCommand(auto_pwm);
    
    Serial.println("Auto Test - Distance: " + String(current_distance) + 
                   "cm, PWM: " + String(auto_pwm));
    delay(200);
  }
  
  // Stop and return to manual
  sendPWMCommand(0);
  sendModeCommand(0);
  current_mode = 0;
  Serial.println("Auto Test Completed");
}
