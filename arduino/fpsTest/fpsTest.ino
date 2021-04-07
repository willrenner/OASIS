int fpscount = 0;
unsigned long t1 = 0;
unsigned long t2 = 0;
void setup()
{
    Serial.begin(115200);
}

void loop()
{
    fpsCounter();
}
void fpsCounter() {
    t1 = millis();
    if ((t1 - t2) > 1000) {
        Serial.println(fpscount);
        fpscount = 0;
        t2 = millis();
    }
    else {
        fpscount++;
    }
}