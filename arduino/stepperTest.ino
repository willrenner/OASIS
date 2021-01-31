int count = 0;
long myTime;
long prevTime;
long diff;
void setup()
{
    // initialize digital pin LED_BUILTIN as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(9600);
    prevTime = millis();
}

// the loop function runs over and over again forever
void loop()
{
    myTime = millis();
    diff = myTime - prevTime;
    prevTime = millis();
    // Serial.print("high");
    Serial.write(diff);
    digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
    delay(100);
    digitalWrite(LED_BUILTIN, LOW); // wait for a second
    // Serial.print("low ");
    // Serial.print("\n");
    // turn the LED off by making the voltage LOW
    delay(100); // wait for a second
    count++;
}
