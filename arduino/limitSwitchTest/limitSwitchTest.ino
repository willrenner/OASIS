void setup() {
  pinMode(3, INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(3) == LOW) {
    Serial.println("hit");
  }
  delay(50);
}
