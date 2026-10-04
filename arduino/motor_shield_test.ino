#include <Adafruit_MotorShield.h>

const int relayPin = 10; // Pin connected to relay

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Connect a stepper motor with 200 steps per revolution (1.8 degree) to motor port M1/M2
Adafruit_StepperMotor *myStepper = AFMS.getStepper(200, 1);

void setup() 

{
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

void loop()
{
    myStepper->step(10, FORWARD, MICROSTEP);  // MICROSTEP for smoother motion
      myStepper->release();  // Release motor coils to prevent holding torque pulsing
      Serial.println("Motor forward complete");
    } else {
      Serial.println("Invalid step count (must be 1-10000)");
    }

    myStepper->step(10, BACKWARD,MICROSTEP);
      myStepper->release();  // Release motor coils to prevent holding torque pulsing
      Serial.println("Motor forward complete");
    } else {
      Serial.println("Invalid step count (must be 1-10000)");
    }
}