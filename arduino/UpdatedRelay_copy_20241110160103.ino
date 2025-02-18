const int relayPin = 10; // Pin connected to relay

void setup() {
  pinMode(relayPin, OUTPUT);      // Set relay pin as output
  digitalWrite(relayPin, LOW);    // Start with the relay off for safety
  Serial.begin(9600);             // Initialize serial communication
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
    else {
      Serial.println("Unknown command"); // For unrecognized commands
    }
  }
}
