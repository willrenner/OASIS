#include <SPI.h>
#include "Adafruit_MAX31855.h"

#define thermocoupleCLK 28 //GREEN
#define thermocoupleDO 30 //BLUE
#define thermocoupleCS 29 //WHITE
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
    Serial.println((float)thermocouple.readFahrenheit(),2);
    Serial.print("Error : ");

    Serial.println(thermocouple.readError());
}

void loop()
{
    Serial.print("Heater temp: ");
    Serial.println((float)thermocouple.readFahrenheit(), 2);
    Serial.print("Error : ");
    Serial.println(thermocouple.readError());

    delay(300);
}
