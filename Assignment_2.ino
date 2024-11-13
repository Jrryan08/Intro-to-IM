
const int trigPin = 9;
const int echoPin = 10;
const int analogLedPin = 6;  // LED for distance indication
const int digitalLedPin = 5; // LED for alarm
const int switchPin = 4;     // switch
int distanceThreshold = 10; // Distance in cm for alarm
bool alarmEnabled = false;  // Alarm State

void setup() {
  Serial.begin(9600);

  // pin modes
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(analogLedPin, OUTPUT);
  pinMode(digitalLedPin, OUTPUT);
  pinMode(switchPin, INPUT_PULLUP); 

  // Initial LED state
  digitalWrite(digitalLedPin, LOW);
}

void loop() {
  // Distance from sensor
  int distance = getDistance();

  // State of the switch
  alarmEnabled = digitalRead(switchPin) == LOW; 

  // Analog LED brightness based on distance
  int brightness = map(distance, 5, 40, 255, 0); // distances between 5 cm and 40 cm
  brightness = constrain(brightness, 0, 255);     // Limit brightness to range
  analogWrite(analogLedPin, brightness);

  // If alarm is enabled and distance is below threshold, blink alarm LED
  if (alarmEnabled && distance < distanceThreshold) {
    digitalWrite(digitalLedPin, HIGH);
    delay(200);
    digitalWrite(digitalLedPin, LOW);
    delay(200);
  } else {
    digitalWrite(digitalLedPin, LOW); // Keep alarm LED off
  }

  //Debug
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm | Alarm: ");
  Serial.println(alarmEnabled ? "ON" : "OFF");

  delay(100); 
}

// Function to calculate distance 
int getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the pulse duration on echo pin
  long duration = pulseIn(echoPin, HIGH);

  // Calculate the distance in cm
  int distance = duration * 0.034 / 2;
  return distance;
}
