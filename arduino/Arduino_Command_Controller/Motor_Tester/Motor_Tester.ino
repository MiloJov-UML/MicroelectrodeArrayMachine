#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address (0x60)
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select the port where your DC motor is connected (M1, M2, M3, or M4)
Adafruit_DCMotor *motor = AFMS.getMotor(2); // Motor on M1

Servo myservo1;
const int servo1Pin = 9; 

// Motor speed (0–255)
const uint8_t MOTOR_SPEED = 50;

// Delay time in milliseconds for each direction
const unsigned long MOVE_TIME = 500;

void setup() {
  Serial.begin(9600);
  Serial.println("Adafruit Motor Shield V2 - Back and Forth Example");

  // Initialize the motor shield
  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1); // Halt if shield not found
  }

  myservo1.attach(servo1Pin);
  // Set initial speed
  motor->setSpeed(MOTOR_SPEED);
}

void loop() {
  // Move forward
  Serial.println("Moving forward...");
  motor->run(FORWARD);
  delay(MOVE_TIME);

  // Stop briefly
  motor->run(RELEASE);
  delay(500);

  // Move backward
  Serial.println("Moving backward...");
  motor->run(BACKWARD);
  delay(MOVE_TIME);

  // Stop briefly
  motor->run(RELEASE);
  delay(500);
}
