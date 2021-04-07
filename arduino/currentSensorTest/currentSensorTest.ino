#define currPin A0
float currentVal = 0;
void setup()
{
    Serial.begin(9600);
}

void loop()
{
    currentVal = 0.1 * analogRead(currPin) + 0.9*currentVal;
    Serial.println(currentVal);
    delay(100);
}
