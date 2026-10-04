
#include <Servo.h>   // Include the standard Servo library

Servo myservo;  // Create servo object to control a servo
int pos = 105;   // Variable to store the servo position, start at 90 degrees, 0 is 40 degrees
        // Variable to read the value from serial input

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud
  myservo.attach(9); // Attaches the servo on pin 10 to the servo object (or pin 9)
  myservo.write(pos);
  Serial.println("Servo control ready. Press 'u' to move up, 'd' to move down.");
}

void loop() {
  if (Serial.available()) { // Check if data is available to read on serial port
    int val = Serial.parseInt();    // Read the incoming byte

    if (val >= 0){
      myservo.write(val); // Tell servo to go to the new position
      Serial.println(val);
    }
    
    delay(15);          // Wait for the servo to reach the position
  }
}
