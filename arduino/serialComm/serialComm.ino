#include <AccelStepper.h>

uint8_t pin2 = 2;
uint8_t pin3 = 3;
AccelStepper stepper(1, pin2, pin3);
AccelStepper MirageStepper(1, 8, 9);

struct controlCommands {
    int drillMovementDirection; // 1 for down
    double speed;
    int positioncommand;
};

controlCommands cmds = {
    .drillMovementDirection = 0,
    .speed = 0,
    .positioncommand = 0};

const int numCmds = 3;    //num of vars in struct above
const int sizeOfCmd = 40; //number of chars sent from matlab to arduino must be less than this

unsigned long currTime = 0;
unsigned long prevTime = 0;
const unsigned long sendRate = 100;
bool incomingStringComplete = false; // whether the string is complete
int currPosOfChar = 0; //
char cstring[sizeOfCmd];
char *arrayOfcstring[numCmds];
char *myPointer;

void sendDataOut();
void buildDataStruct();
void formatIncomingData();
void setup() {
    stepper.setMaxSpeed(400);
    MirageStepper.setMaxSpeed(400); // Steps per second
    MirageStepper.setSpeed(400);
    MirageStepper.setAcceleration(50); //Steps/sec^2


Serial.begin(115200);
    while (!Serial)
    {
        ; // wait for serial port to connect. Needed for native USB port only
    }
    Serial.flush();
}

void loop()
{
    if (incomingStringComplete)
    {
        formatIncomingData(); //formats cmds data
        buildDataStruct(); //formats cmds data struct
        stepper.setSpeed((int)(cmds.speed * cmds.drillMovementDirection)); //sets drill vertical speed
        chooseMotorPosition(cmds.positioncommand); //move mirage stepper to set pos
        incomingStringComplete = false;
    }

    currTime = millis();
    if (currTime - prevTime >= sendRate) //every (sendRate) ms
    {
        sendDataOut();
        prevTime = millis();
    }
    stepper.runSpeed();
    MirageStepper.run();
    
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent()
{
    while (Serial.available())
    {
        char inChar = (char)Serial.read();
        if (inChar == '\n')
        {
            cstring[currPosOfChar] = NULL; //end char array in null char
            incomingStringComplete = true;
            currPosOfChar = 0;
//            Serial.print("Arduino Recieved: " + String(cstring) + "\n");
            break;
        }
        cstring[currPosOfChar] = inChar;
        currPosOfChar++;
    }
}
void sendDataOut()
{
    //send data back to serial
    Serial.print(cmds.drillMovementDirection, DEC); //formated as int
    Serial.print(",");
    Serial.print(cmds.speed, 2); //formated as float to 2 decials
    Serial.print(",");
    Serial.print(cmds.positioncommand, DEC);
    
    

    Serial.print("\n");
}
void buildDataStruct()
{
    //direction
    int speed = atoi(arrayOfcstring[0]);
    if (speed == 1)
    {
        cmds.drillMovementDirection = 1;
    }
    else if (speed == -1)
    {
        cmds.drillMovementDirection = -1;
    }
    else
    {
        cmds.drillMovementDirection = 0;
    }
    //speed
    cmds.speed = atof(arrayOfcstring[1]);
    // Serial.println(cmds.speed, 2);
    int positioncommand = atoi(arrayOfcstring[2]);
    // Serial.println(positioncommand);
    cmds.positioncommand = positioncommand;
}
void formatIncomingData()
{
    //put stuff into correct array format
    int count = 0;
    myPointer = strtok(cstring, ",");
    while (myPointer != NULL)
    {
        arrayOfcstring[count] = myPointer;
        myPointer = strtok(NULL, ",");
        count++;
    }
    // Serial.println("here: " + String(arrayOfcstring[1]));
}
void chooseMotorPosition(int positioncommand) {
  if (positioncommand == 1){
      MotorPositionOne();
    }
    if (positioncommand == 2){
      MotorPositionTwo();
    }
    if (positioncommand == 3){
      MotorPositionThree();
    }
    if (positioncommand == 4){
      MotorPositionFour();
    }
    if (positioncommand == 5){
      MotorPositionFive();
    }
    if (positioncommand == 6){
      MotorPositionSix();
    }
}
// Mirage Position Code
// Motor change
void MotorPositionOne() 
{
  MirageStepper.moveTo(67);

  //steps motor once every iteration 
}
void MotorPositionTwo()
{
  Serial.println("here1");
  MirageStepper.moveTo(133);

  Serial.println("here2");
}
void MotorPositionThree()
{
  MirageStepper.moveTo(200);

}
void MotorPositionFour()
{
  MirageStepper.moveTo(267);

}
void MotorPositionFive()
{
  MirageStepper.moveTo(334);

}
void MotorPositionSix()
{
  MirageStepper.moveTo(400);

}
