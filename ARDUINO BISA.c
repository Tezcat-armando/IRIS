/**
 * ESP32 Master Controller for STM32 Communication
 * UART Communication with STM32F401 for Motor Control
 * 
 * Pin Connection:
 * ESP32 TX (GPIO17) -> STM32 RX (PA3)
 * ESP32 RX (GPIO16) -> STM32 TX (PA2)
 * GND -> GND
 */

// UART Configuration for STM32 Communication
HardwareSerial SerialSTM32(1); // Use UART1 on ESP32

// UART Pins
#define STM32_RX_PIN 16  // GPIO16 - ESP32 RX (connect to STM32 TX - PA2)
#define STM32_TX_PIN 17  // GPIO17 - ESP32 TX (connect to STM32 RX - PA3)

// Command Variables
int target_pwm = 0;
String received_data = "";
bool stm32_connected = false;
unsigned long last_heartbeat = 0;

void setup() {
  // Initialize Serial Monitor for debugging
  Serial.begin(115200);
  
  // Initialize UART communication with STM32
  SerialSTM32.begin(9600, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);
  
  // Wait for serial port to connect
  delay(1000);
  
  Serial.println("==========================================");
  Serial.println("   ESP32 Master Controller Started");
  Serial.println("   Communicating with STM32F401");
  Serial.println("==========================================");
  Serial.println("Commands:");
  Serial.println("  PWM:value  - Set motor speed (-255 to 255)");
  Serial.println("  READ       - Read encoder value");
  Serial.println("  STOP       - Stop motor");
  Serial.println("  STATUS     - Check system status");
  Serial.println("==========================================");
  
  // Send initial handshake to STM32
  sendHandshake();
}

void loop() {
  // Handle Serial Monitor input (user commands)
  handleSerialInput();
  
  // Handle incoming data from STM32
  handleSTM32Data();
  
  // Send heartbeat every 2 seconds to check connection
  if (millis() - last_heartbeat > 2000) {
    sendHeartbeat();
    last_heartbeat = millis();
  }
  
  delay(10); // Small delay for stability
}

/**
 * Handle commands from Serial Monitor
 */
void handleSerialInput() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.startsWith("PWM:")) {
      // Extract PWM value from command
      target_pwm = input.substring(4).toInt();
      
      // Validate PWM range
      if (target_pwm >= -255 && target_pwm <= 255) {
        sendPWMCommand(target_pwm);
        Serial.println("Sent PWM command: " + String(target_pwm));
      } else {
        Serial.println("Error: PWM value must be between -255 and 255");
      }
    }
    else if (input.equalsIgnoreCase("READ")) {
      sendReadCommand();
      Serial.println("Sent READ command");
    }
    else if (input.equalsIgnoreCase("STOP")) {
      sendPWMCommand(0);
      Serial.println("Sent STOP command");
    }
    else if (input.equalsIgnoreCase("STATUS")) {
      checkSystemStatus();
    }
    else if (input.equalsIgnoreCase("TEST")) {
      runMotorTest();
    }
    else {
      Serial.println("Unknown command. Available: PWM:value, READ, STOP, STATUS, TEST");
    }
  }
}

/**
 * Handle incoming data from STM32
 */
void handleSTM32Data() {
  if (SerialSTM32.available()) {
    received_data = SerialSTM32.readStringUntil('\n');
    received_data.trim();
    
    // Process the received data
    processSTM32Response(received_data);
  }
}

/**
 * Process responses from STM32
 */
void processSTM32Response(String response) {
  Serial.println("STM32: " + response);
  
  if (response.startsWith("ACK:PWM:")) {
    // Format: "ACK:PWM:value,ENC:count"
    int pwm_start = response.indexOf("PWM:") + 4;
    int enc_start = response.indexOf("ENC:") + 4;
    int comma_pos = response.indexOf(',');
    
    if (pwm_start != -1 && enc_start != -1 && comma_pos != -1) {
      String pwm_str = response.substring(pwm_start, comma_pos);
      String enc_str = response.substring(enc_start);
      
      int received_pwm = pwm_str.toInt();
      int encoder_count = enc_str.toInt();
      
      Serial.println("✓ Motor PWM: " + pwm_str + ", Encoder: " + enc_str);
    }
  }
  else if (response.startsWith("ENC:")) {
    // Format: "ENC:count"
    String enc_str = response.substring(4);
    int encoder_count = enc_str.toInt();
    Serial.println("✓ Encoder reading: " + enc_str);
  }
  else if (response.startsWith("STM32F401 Motor Controller Ready")) {
    stm32_connected = true;
    Serial.println("✓ STM32 successfully connected and ready!");
  }
  else if (response.startsWith("Encoder:")) {
    // Debug output from STM32
    // Just display it as is
  }
}

/**
 * Send PWM command to STM32
 */
void sendPWMCommand(int pwm_value) {
  String command = "PWM:" + String(pwm_value) + "\n";
  SerialSTM32.print(command);
}

/**
 * Send READ command to STM32
 */
void sendReadCommand() {
  SerialSTM32.print("READ\n");
}

/**
 * Send handshake to STM32
 */
void sendHandshake() {
  Serial.println("Attempting to connect to STM32...");
  SerialSTM32.print("\n"); // Send empty line to trigger response
}

/**
 * Send heartbeat to check connection
 */
void sendHeartbeat() {
  if (stm32_connected) {
    sendReadCommand(); // Use READ as heartbeat
  } else {
    sendHandshake(); // Try to reconnect
  }
}

/**
 * Check system status
 */
void checkSystemStatus() {
  Serial.println("=== SYSTEM STATUS ===");
  Serial.println("ESP32: OK");
  Serial.println("STM32: " + String(stm32_connected ? "CONNECTED" : "DISCONNECTED"));
  Serial.println("Current PWM: " + String(target_pwm));
  Serial.println("=====================");
  
  // Request current encoder reading
  sendReadCommand();
}

/**
 * Run motor test sequence
 */
void runMotorTest() {
  Serial.println("Starting motor test sequence...");
  
  // Stop motor first
  sendPWMCommand(0);
  delay(1000);
  
  // Test forward
  Serial.println("Testing FORWARD direction...");
  for (int pwm = 50; pwm <= 200; pwm += 50) {
    sendPWMCommand(pwm);
    Serial.println("PWM: " + String(pwm));
    delay(2000);
  }
  
  // Stop
  sendPWMCommand(0);
  Serial.println("Motor STOP");
  delay(1000);
  
  // Test reverse
  Serial.println("Testing REVERSE direction...");
  for (int pwm = -50; pwm >= -200; pwm -= 50) {
    sendPWMCommand(pwm);
    Serial.println("PWM: " + String(pwm));
    delay(2000);
  }
  
  // Stop
  sendPWMCommand(0);
  Serial.println("Motor test completed!");
}

/**
 * Emergency stop function
 */
void emergencyStop() {
  sendPWMCommand(0);
  Serial.println("!!! EMERGENCY STOP !!!");
}

/**
 * Calculate motor speed based on sensor data (for future use)
 */
int calculateAutoPWM(int sensor_value) {
  // Placeholder for automatic control logic
  // This can be expanded for sensor-based control
  if (sensor_value > 100) return 255;
  else if (sensor_value > 50) return 150;
  else if (sensor_value > 20) return 80;
  else return 0;
}
