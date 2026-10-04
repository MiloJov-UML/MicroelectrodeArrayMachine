

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
pinMode(5, INPUT_PULLUP); //R
pinMode(4, INPUT_PULLUP); //Z


}

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(5) == LOW)
  {
      
      Serial.println("Touch R");
  }
  
  if (digitalRead(4) == LOW)
  {
      
      Serial.println("Touch Z");
  }

}
