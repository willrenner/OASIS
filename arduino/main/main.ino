#include "AccelStepper.h"
// automatically includes arduino.h??
uint8_t pin2 = 2;
uint8_t pin3 = 3;
AccelStepper stepper(1, pin2, pin3);
AccelStepper MirageStepper(1, 8, 9);

struct controlCommands {
    int controlMode; // 1 for manual rop control, 0 for automatic (pid)
    int drillMovementDirection; // 1 for down
    double speed;
    int miragePositionCmd;
};

controlCommands cmds = {
    .controlMode = 1,
    .drillMovementDirection = 0,
    .speed = 0,
    .miragePositionCmd = 0};

const int numCmds = 4;    //num of vars in struct above
const int sizeOfCmd = 40; //number of chars sent from matlab to arduino must be less than this

unsigned long currTime = 0;
unsigned long prevTime = 0;
const unsigned long sendRate = 100; //ms
bool incomingStringComplete = false; // whether the string is complete
int currPosOfChar = 0; //
char cstring[sizeOfCmd];
char *arrayOfcstring[numCmds];
char *myPointer;

// float augerArea =  PI * (0.02)^2;
float WOB = 0;
int currentStep = 0;
int leadScrewLead = 8; // mm/rev
int numSteps = 200; // (per rev) steps/rev, 1.8deg/step


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
        chooseMotorPosition(cmds.miragePositionCmd); //move mirage stepper to set pos
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

void sendDataOut()
{
    //send data back to serial
    Serial.print(cmds.controlMode, DEC); //formated as int
    Serial.print(",");
    Serial.print(cmds.drillMovementDirection, DEC); //formated as int
    Serial.print(",");
    Serial.print(cmds.speed, 2); //formated as float to 2 decials
    Serial.print(",");
    Serial.print(cmds.miragePositionCmd, DEC); //formated as int
    Serial.print("\n"); //serial terminator
}
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
void buildDataStruct()
{
    //mode
    cmds.controlMode = atoi(arrayOfcstring[0]);
    //direction
    int speed = atoi(arrayOfcstring[1]);
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
    cmds.speed = atof(arrayOfcstring[2]);
    //mirage pos
    cmds.miragePositionCmd = atoi(arrayOfcstring[3]);
}
void chooseMotorPosition(int pos) {
  if (pos == 1){
      MirageStepper.moveTo(67);
    }
    if (pos == 2){
      MirageStepper.moveTo(133);
    }
    if (pos == 3){
        MirageStepper.moveTo(200);
    }
    if (pos == 4){
        MirageStepper.moveTo(267);
    }
    if (pos == 5){
        MirageStepper.moveTo(334);
    }
    if (pos == 6){
        MirageStepper.moveTo(400);
    }
}

void activateHeater(int command) {
  if (command == 1) {
    //set arduino pins connected to relay high
  }
  else if (command == 0) {
    //set arduino pins connected to relay low
  }
}

void activatePump(int command) {
  if (command == 1) {
    //set arduino pins connected to relay high
  }
  else if (command == 0) {
    //set arduino pins connected to relay low
  }
  else if (command == -1) {
    //reverse pump ??
  }
}
void getMSE() {
  // float drillTorque = getDrillTorque();
  // float drillRPM = getDrillRPM();
  // float ROP = cmds.speed;
  // float MSE = WOB/augerArea + drillTorque*drillRPM/(augerArea*ROP); //MSE equation
}