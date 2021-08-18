#include <AccelStepper.h>
uint8_t pinA = 4;
uint8_t pinB = 10;
AccelStepper stepper(1, pinA,pinB); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
void setup()
{  
   Serial.begin(9600);
   stepper.setMaxSpeed(400);
   stepper.setSpeed(400);
   Serial.println("setup");
}
void loop()
{  
  //computeNewSpeed() //look into this
   stepper.runSpeed(); //call as freq as possible
}
