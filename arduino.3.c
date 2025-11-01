// Pin definitions for HC-SR04 - MODIFIED VERSION
const int trigPin = 22;   // D22 - GPIO22
const int echoPin = 23;   // D23 - GPIO23

// Variables for distance measurement
long duration;
float distance;

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  Serial.println("HC-SR04 Sensor Ready - Using D22 & D23");
}

void loop() {
  // Clear trig pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Send 10us pulse to trigger
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read echo pulse duration
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate distance (cm)
  distance = duration * 0.034 / 2;
  
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  delay(500); // Wait 500ms between measurements
}
