#include <Adafruit_ADS1X15.h>

Adafruit_ADS1015 ads1115;



void setup() {
    Serial.begin(115200);
    Serial.println("ready");
}

void loop() {
    analogWrite(2, 127);
}
