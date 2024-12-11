#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1); // right motor
Adafruit_DCMotor *motor2 = AFMS.getMotor(2); // left motor

// Servo objects and control pins
Servo rightHandServo; 
Servo leftHandServo;  
const int rightHandPin = 9;
const int leftHandPin = 10;

// LED control pins for eyes
const int leftEyeLED = 6;  
const int rightEyeLED = 12; 

// Servo positions
const int handUpAngle = 90;   // Angle when hand is raised
const int handDownAngle = 0;  // Angle when hand is lowered

String command = ""; // Store incoming serial data
bool spinMode = false; // Continuous hand waving in spin mode
int speed = 100;       // Speed of the motors

// Timer variables for LED blinking
unsigned long lastBlinkTime = 0; 
const unsigned long blinkInterval = 1500; 

bool isLedOn = true;

void setup() {
  Serial.begin(9600); 
  //Serial.println("Robot ready with Adafruit Motor Shield!");

  // Initialize motor shield
  if (!AFMS.begin()) {
    Serial.println("Motor Shield not detected. Check connections!");
    while (1); // Stop if no motor shield
  }

  // Attach servos
  rightHandServo.attach(rightHandPin);
  leftHandServo.attach(leftHandPin);

  // Initialize servos to the down position
  rightHandServo.write(handDownAngle);
  leftHandServo.write(handDownAngle);

  // Initialize LEDs
  pinMode(leftEyeLED, OUTPUT);
  pinMode(rightEyeLED, OUTPUT);

  // Start LEDs off
  digitalWrite(leftEyeLED, LOW);
  digitalWrite(rightEyeLED, LOW);

  
  robotblink();

  // Stop all motors initially
  stopMotors();
}

void loop() {
  // Check for serial commands
  if (Serial.available() > 0) {
    char inChar = (char)Serial.read(); // Read the incoming byte
    if (inChar == '\n') { // End of command
      processCommand(command); // Process the complete command
      command = ""; 
    } else {
      command += inChar; 
    }
  }

  // If in spin mode, wave hands continuously
  if (spinMode) {
    waveHands();
  }

  // Blink LEDs periodically
  blinkLeds();
}

// Function to process received serial commands
void processCommand(String cmd) {
  cmd.trim(); // Remove any leading/trailing whitespace
  Serial.println("Received command: " + cmd);

  spinMode = false; // Disable spin mode by default

  if (cmd == "moves forward") {//Move robot Forward
    moveForward();
    lowerHands();
  } else if (cmd == "moves back") {//Move robot Backwards
    moveBackward();
    lowerHands();
  } else if (cmd == "turns left") {//Turn Robot left
    moveLeft();
    raiseLeftHand();
  } else if (cmd == "turns right") {//Turn Robot Right
    moveRight();
    raiseRightHand();
  } else if (cmd == "spins") {//Spin Robot
    spin();
    spinMode = true;
  } else if (cmd == "stops") {//Stop robot
    stopMotors();
    lowerHands();
  } else {
    Serial.println("Unknown command");
    stopMotors();
    lowerHands();
  }
}

// Functions to control robot movements
void moveForward() {
  Serial.println("Moving forward...");
  motor1->setSpeed(speed);
  motor2->setSpeed(speed);
  motor1->run(FORWARD);
  motor2->run(FORWARD);
}

void moveBackward() {
  Serial.println("Moving backward...");
  motor1->setSpeed(speed);
  motor2->setSpeed(speed);
  motor1->run(BACKWARD);
  motor2->run(BACKWARD);
}

void moveLeft() {
  Serial.println("Turning left...");
  motor1->setSpeed(80);
  motor2->setSpeed(speed);
  motor1->run(FORWARD);
  motor2->run(RELEASE);
}

void moveRight() {
  Serial.println("Turning right...");
  motor1->setSpeed(speed);
  motor2->setSpeed(80);
  motor1->run(RELEASE);
  motor2->run(FORWARD);
}

void spin() {
  Serial.println("Spinning...");
  motor1->setSpeed(speed);
  motor2->setSpeed(speed);
  motor1->run(FORWARD);
  motor2->run(BACKWARD);
}

// Function to stop both motors
void stopMotors() {
  Serial.println("Stopping motors...");
  motor1->run(RELEASE);
  motor2->run(RELEASE);
}

// Servo control functions
void raiseRightHand() {
  rightHandServo.write(handUpAngle); // Raise the right hand
  leftHandServo.write(handDownAngle); // Left hand down
}

void lowerHands() {
  rightHandServo.write(handDownAngle); // Lower the right hand
  leftHandServo.write(handDownAngle); // Lower the left hand
}

void raiseLeftHand() {
  leftHandServo.write(handUpAngle); // Raise the left hand
  rightHandServo.write(handDownAngle); // Right hand down
}

void waveHands() {
  static unsigned long lastWaveTime = 0;
  static bool handsUp = false;

  if (millis() - lastWaveTime > 500) { // Adjust speed of waving
    handsUp = !handsUp; // Toggle hands position
    if (handsUp) {
      rightHandServo.write(handUpAngle); // Right hand forward up
      leftHandServo.write(handUpAngle); // Left hand forward up
    } else {
      rightHandServo.write(handDownAngle); // Right hand forward down
      leftHandServo.write(handDownAngle); // Left hand forward down
    }
    lastWaveTime = millis();
  }
}

// Function for LED blinking
void blinkLeds() {
  if (millis() - lastBlinkTime > blinkInterval) {
    isLedOn = !isLedOn; // Toggle LED state
    digitalWrite(leftEyeLED, isLedOn ? HIGH : LOW);
    digitalWrite(rightEyeLED, isLedOn ? HIGH : LOW);
    lastBlinkTime = millis(); // Update last blink time
  }
}

//initial robot blinking
void robotblink() {
  for (int i = 0; i < 3; i++) { 
    digitalWrite(leftEyeLED, HIGH);
    digitalWrite(rightEyeLED, HIGH);
    delay(300);
    digitalWrite(leftEyeLED, LOW);
    digitalWrite(rightEyeLED, LOW);
    delay(300);
  }
  digitalWrite(leftEyeLED, HIGH);
  digitalWrite(rightEyeLED, HIGH);
  Serial.println("Robot has come to life!");
}
