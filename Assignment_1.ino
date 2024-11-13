const int ldrPin = 2;    // Pin connected to the LDR
const int ledPin = 13;   // Pin connected to the LED

void setup() {
  pinMode(ldrPin, INPUT);   // LDR pin as input
  pinMode(ledPin, OUTPUT);  //LED pin as output
}

void loop() {
  int lightStatus = digitalRead(ldrPin); // LDR value

  if (lightStatus == LOW) {
    // It’s dark, turn on the LED
    digitalWrite(ledPin, HIGH);
  } else {
    // It’s bright, turn off the LED
    digitalWrite(ledPin, LOW);
  }
}
