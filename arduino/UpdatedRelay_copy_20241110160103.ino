#include <Adafruit_MotorShield.h>

const int relayPin = 10; // Pin connected to relay

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Connect a stepper motor with 200 steps per revolution (1.8 degree) to motor port M1/M2
Adafruit_StepperMotor *myStepper = AFMS.getStepper(200, 1);

void setup() {
  pinMode(relayPin, OUTPUT);      // Set relay pin as output
  digitalWrite(relayPin, LOW);    // Start with the relay off for safety
  Serial.begin(9600);             // Initialize serial communication
  
  // Initialize the motor shield
  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  
  // Set default stepper speed (RPM)
  myStepper->setSpeed(100);  // Increased to 100 RPM for smoother motion
  
  Serial.println("Arduino is ready to receive commands."); // Debugging statements
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // Read command from serial
    command.trim(); // Remove any leading or trailing whitespace

    Serial.print("Received command: "); // Print received command for debugging
    Serial.println(command);

    // Relay control commands
    if (command.equalsIgnoreCase("Laser_Relay_On")) {
      digitalWrite(relayPin, HIGH); // Turn relay ON
      Serial.println("Relay turned ON"); // Confirmation message
    }
    else if (command.equalsIgnoreCase("Laser_Relay_Off")) {
      digitalWrite(relayPin, LOW);  // Turn relay OFF
      Serial.println("Relay turned OFF"); // Confirmation message
    }
    // Motor control commands
    else if (command.startsWith("Motor_Forward_")) {
      int steps = command.substring(14).toInt();  // Extract number after "Motor_Forward_"
      if (steps > 0 && steps <= 10000) {  // Limit to reasonable range
        Serial.print("Moving motor forward ");
        Serial.print(steps);
        Serial.println(" steps");
        myStepper->step(steps, FORWARD, MICROSTEP);  // MICROSTEP for smoother motion
        myStepper->release();  // Release motor coils to prevent holding torque pulsing
        Serial.println("Motor forward complete");
      } else {
        Serial.println("Invalid step count (must be 1-10000)");
      }
    }
    else if (command.startsWith("Motor_Backward_")) {
      int steps = command.substring(15).toInt();  // Extract number after "Motor_Backward_"
      if (steps > 0 && steps <= 10000) {  // Limit to reasonable range
        Serial.print("Moving motor backward ");
        Serial.print(steps);
        Serial.println(" steps");
        myStepper->step(steps, BACKWARD, MICROSTEP);  // MICROSTEP for smoother motion
        myStepper->release();  // Release motor coils to prevent holding torque pulsing
        Serial.println("Motor backward complete");
      } else {
        Serial.println("Invalid step count (must be 1-10000)");
      }
    }
    // Motor release command (to stop holding torque when idle)
    else if (command.equalsIgnoreCase("Motor_Release")) {
      myStepper->release();
      Serial.println("Motor released");
    }
    else {
      Serial.println("Unknown command"); // For unrecognized commands
    }
  }
}
