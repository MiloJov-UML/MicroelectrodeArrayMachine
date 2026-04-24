#include <Adafruit_MotorShield.h>
#include <Wire.h>
#include <Servo.h>   // Include the standard Servo library

Servo myservo;  // Create servo object to control a servo
int pos = 40;   // Variable to store the servo position, start at 0 degrees
int val;        // Variable to read the value from serial input


const int hallPin = 2; // Pin connected to sensor Signal
const int ledPin = 13; // Built-in LED
int sensorValue;

const int relayPin = 8; // Pin connected to relay
const int solPin = 11; // Pin connected to relay2
const int nordPin = 7;
const int r_limit = 5;
const int z_probe = 4;
bool r_lim = false;
bool z_prb = false;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *myStepper = AFMS.getStepper(200, 1);
Adafruit_DCMotor *motor = AFMS.getMotor(3); // Motor on M1

void setup() {
  pinMode(relayPin, OUTPUT);
  pinMode(solPin, OUTPUT);
  pinMode(nordPin, OUTPUT);
  pinMode(r_limit, INPUT_PULLUP);
  pinMode(z_probe, INPUT_PULLUP);

  pinMode(hallPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);

  digitalWrite(relayPin, LOW);    // Start with the relay off for safety
  digitalWrite(solPin, LOW);
  digitalWrite(nordPin, LOW);     // Start with the solenoid relay off for safety
  
  Serial.begin(9600); // Start serial communication at 9600 baud
  myservo.attach(9); // Attaches the servo on pin 10 to the servo object (or pin 9)
  myservo.write(pos);
  

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

    // Servo control Command  
    if (command.startsWith("Servo_To_")) {
      int angle = command.substring(9).toInt();
      if (angle >= 0 && angle <= 270) {
        Serial.print("Moving Servo to Angle ");
        Serial.println(angle);

        myservo.write(angle); // Tell servo to go to the new position
        Serial.println("Motion complete");
      } else {
        Serial.println("Invalid angle (must be 0-270)");
      }
    }
    
    // Motor Control Commands
    if (command.startsWith("PNP_Forward_")) {
      int speed = command.substring(12).toInt();
      if (speed > 0 && speed <= 255) {
        Serial.print("Moving motor forward, ");
        Serial.print("Speed: ");
        Serial.println(speed);
        
        motor->setSpeed(speed);  // MICROSTEP for more torque
        motor->run(BACKWARD);

      } else {
        Serial.println("Invalid speed count (must be 0-255)");
      }
    }
    else if (command.startsWith("PNP_Backward_")) {
      int speed = command.substring(13).toInt();
      if (speed > 0 && speed <= 255) {
        Serial.print("Moving motor backward, ");
        Serial.print("Speed: ");
        Serial.println(speed);
        
        motor->setSpeed(speed);  
        motor->run(FORWARD);
        delay(500);

      } else {
        Serial.println("Invalid speed count (must be 0-255)");
      }
    }
    else if (command.equalsIgnoreCase("PNP_Release")) {
      motor->run(RELEASE);
      Serial.println("Motor released");
    }
    else {
      Serial.println("Unknown command");
    }

    // Stepper Motor Control Commands
    if (command.startsWith("Motor_Forward_")) {
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
  
    // R limit switch control commands
    if (command.equalsIgnoreCase("Start_R_Poll")) {
      r_lim = true;
      //Serial.println("Starting r axis Poll");
        
    }
    else if (command.equalsIgnoreCase("End_R_Poll")) {
      r_lim = false;
      //Serial.println("Ending r axis Poll");
    }

    /// Z probe control commands
    if (command.equalsIgnoreCase("Start_Z_Poll")) {
      z_prb = true;
      //Serial.println("Starting Z axis Poll");

    }
    else if (command.equalsIgnoreCase("End_Z_Poll")) {
      z_prb = false; 
        //Serial.println("Ending Z axis Poll");
    }
  } 
    if (10>0)
    {
      // R limit commands
      if (digitalRead(r_limit) == LOW)
      {
        Serial.println("R limit"); 
      }

      // Z limit commands
      if (digitalRead(z_probe) == LOW)
      {
        Serial.println("Z limit"); 
      }

      // Hall Pin
      if (digitalRead(sensorValue) == LOW)
      {
        Serial.println("Magnet Detected");
      }
    }
    
        
 
}