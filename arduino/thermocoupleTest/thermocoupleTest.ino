#include <SPI.h>
#include "Adafruit_MAX31855.h"

#define thermocoupleCLK 28
#define thermocoupleDO 30
#define thermocoupleCS 29
Adafruit_MAX31855 thermocouple(thermocoupleCLK, thermocoupleCS, thermocoupleDO);

void setup()
{
    Serial.begin(115200);
    if (!thermocouple.begin()) {
        Serial.println("Setting up thermocouple");
        while (1) {
          delay(10);
          Serial.println("waiting");
        }
    }
    Serial.println("Thermcouple setup complete!");
    Serial.print("Int. Temp = ");
    Serial.println((float)thermocouple.readCelsius() * 1.8 + 32, 2);
}

void loop()
{
    Serial.print("Heater temp: ");
    Serial.println((float)thermocouple.readCelsius() * 1.8 + 32, 2);

    delay(100);
}
