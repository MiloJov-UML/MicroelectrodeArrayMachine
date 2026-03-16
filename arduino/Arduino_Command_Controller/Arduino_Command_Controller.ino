#include <Adafruit_MotorShield.h>

const int relayPin = 10; // Pin connected to relay
const int solPin = 11; // Pin connected to relay2
const int nordPin = 7;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *myStepper = AFMS.getStepper(200, 1);

void setup() {
  pinMode(relayPin, OUTPUT);
  pinMode(solPin, OUTPUT);
  pinMode(nordPin, OUTPUT);
  digitalWrite(relayPin, LOW);    // Start with the relay off for safety
  digitalWrite(solPin, LOW);
  digitalWrite(nordPin, LOW);     // Start with the solenoid relay off for safety
  Serial.begin(9600);
  
  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  
  myStepper->setSpeed(1);  // lower speed = more torque
  
  Serial.println("Arduino is ready to receive commands.");
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    Serial.print("Received command: ");
    Serial.println(command);

    if (command.equalsIgnoreCase("Laser_Relay_On")) {
      digitalWrite(relayPin, HIGH);
      Serial.println("Relay turned ON");
    }
    else if (command.equalsIgnoreCase("Laser_Relay_Off")) {
      digitalWrite(relayPin, LOW);
      Serial.println("Relay turned OFF");
    }

    // Solenoid control commands
    if (command.equalsIgnoreCase("Solenoid_Relay_On")) {
      digitalWrite(solPin, HIGH);
      Serial.println("Solenoid relay turned ON");
    }
    else if (command.equalsIgnoreCase("Solenoid_Relay_Off")) {
      digitalWrite(solPin, LOW);
      Serial.println("Solenoid relay turned OFF");
    }

    // Nordson control commands
    if (command.equalsIgnoreCase("Nordson_On")) {
      digitalWrite(nordPin, HIGH);
      Serial.println("Nordson turned ON");
    }
    else if (command.equalsIgnoreCase("Nordson_Off")) {
      digitalWrite(nordPin, LOW);
      Serial.println("Nordson turned OFF");
    }

    else if (command.startsWith("Motor_Forward_")) {
      int steps = command.substring(14).toInt();
      if (steps > 0 && steps <= 10000) {
        Serial.print("Moving motor forward ");
        Serial.print(steps);
        Serial.println(" steps");
        myStepper->step(steps, FORWARD, MICROSTEP);  // MICROSTEP for more torque
        myStepper->release();
        Serial.println("Motor forward complete");
      } else {
        Serial.println("Invalid step count (must be 1-10000)");
      }
    }
    else if (command.startsWith("Motor_Backward_")) {
      int steps = command.substring(15).toInt();
      if (steps > 0 && steps <= 10000) {
        Serial.print("Moving motor backward ");
        Serial.print(steps);
        Serial.println(" steps");
        myStepper->step(steps, BACKWARD, MICROSTEP);  // MICROSTEP for more torque
        myStepper->release();
        Serial.println("Motor backward complete");
      } else {
        Serial.println("Invalid step count (must be 1-10000)");
      }
    }
    else if (command.equalsIgnoreCase("Motor_Release")) {
      myStepper->release();
      Serial.println("Motor released");
    }
    else {
      Serial.println("Unknown command");
    }
  }
}