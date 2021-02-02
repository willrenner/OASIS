// Include the AccelStepper library:
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Stepper.h>

int StepsPerRev = 400;
//This code will control the position of mirage
// should accept an input between 1-6 indicating
AccelStepper MirageStepper(1, 2, 3);
int stepCount = 0;  //Steps the motor has done
int DirectionMultiplier = -1; // allows for a direction to be chosen
char recieveCommand;
bool NewData, runallowed =false;
long StepsToNext = 60;



MultiStepper steppers;
// Motor setup
void setup() {
  // put your setup code here, to run once:
  Serial.begin (9600);
  Serial.println("The code has initialized");
  Serial.println("the steps of the motor is set to 400");
  MirageStepper.setMaxSpeed(100); // Steps per second
  MirageStepper.setSpeed(400);
  MirageStepper.setAcceleration(10); //Steps/sec^2
  steppers.addStepper(MirageStepper);
} 


// Main loop to read serial data and write the motor position
  void loop()
  {
   checkSerial();
  }
 // after the serial is checked the mirage states will call either a move forward or move back function
void checkSerial()
{
  if (Serial.available() > 0 ) //if something changes in the values
  {
  recieveCommand = Serial.read();
  NewData = true; //this indicates the new values
  runallowed == true;
  if (NewData == true)
  {
    switch (recieveCommand)// checking the command string or number
    {
      case '1':
      {
        MotorPositionOne();
        Serial.println("we are at state 1");
         //calls the function to move one position forward
        break;
      }
      case '2':
      {
        Serial.println("we are at state 2");
        MotorPositionTwo();
        break;
      }
      case '3':
      {
        Serial.println("we are at state 3");
        MotorPositionThree();
        break;
      }
      case '4':
      {
        Serial.println("we are at state 4");
        MotorPositionFour();
        break;
      }

    }
    NewData = false;
  }
  
  }
}
// Motor change
void MotorPositionOne() 
  {
    MirageStepper.moveTo(50);
    MirageStepper.runToPosition(); //steps motor once every iteration 
    Serial.println("The motor has stepped to the first position");
  }
void MotorPositionTwo()
{
  MirageStepper.moveTo(100);
  MirageStepper.runToPosition();
  Serial.println("The motor is in second position");
}
void MotorPositionThree()
{
  MirageStepper.moveTo(150);
  MirageStepper.runToPosition();
  Serial.println("The motor is in third position");
}
void MotorPositionFour()
{
  MirageStepper.moveTo(200);
  MirageStepper.runToPosition();
  Serial.println("The motor is in fourth position");
}

void MoveMotorBackOnePosition()
{
  MirageStepper.move(-50);
  MirageStepper.runToPosition();
  Serial.println("the motor has stepped back one position");
  
}
void ReturnHome()
{
  MirageStepper.moveTo(0);
  MirageStepper.run();
  Serial.println("the motor is returning home");
  
}
