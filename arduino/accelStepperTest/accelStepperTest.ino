#include <AccelStepper.h>
uint8_t pin2 = 2;
uint8_t pin3 = 3;
AccelStepper stepper(1, pin2,pin3); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
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
