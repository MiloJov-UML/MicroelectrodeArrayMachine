void setup() {
  pinMode(7, OUTPUT);  // Set pin 7 as an output
}

void loop() {
  digitalWrite(7, HIGH);  // Turn pin 7 ON
  delay(1000);            // Wait 1 second (1000 ms)

  digitalWrite(7, LOW);   // Turn pin 7 OFF
  delay(1000);            // Wait 1 second
}