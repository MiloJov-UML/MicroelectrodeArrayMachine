
#include <Servo.h>   // Include the standard Servo library

Servo myservo;  // Create servo object to control a servo
int pos = 90;   // Variable to store the servo position, start at 90 degrees
int val;        // Variable to read the value from serial input

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud
  myservo.attach(9); // Attaches the servo on pin 10 to the servo object (or pin 9)
  myservo.write(pos);
  Serial.println("Servo control ready. Press 'u' to move up, 'd' to move down.");
}

void loop() {
  if (Serial.available()) { // Check if data is available to read on serial port
    val = Serial.read();    // Read the incoming byte

    if (val == 'u') {
      pos += 5; // Increase position by 5 degrees
    } else if (val == 'd') {
      pos -= 5; // Decrease position by 5 degrees
    }

    // Constrain the position to typical servo limits (0-180 degrees)
    if (pos > 180) pos = 180;
    if (pos < 0) pos = 0;

    myservo.write(pos); // Tell servo to go to the new position
    delay(15);          // Wait for the servo to reach the position
  }
}
