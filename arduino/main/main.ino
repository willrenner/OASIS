#include "AccelStepper.h"
// automatically includes arduino.h??
uint8_t pin2 = 2;
uint8_t pin3 = 3;
AccelStepper DrillStepper(1, pin2, pin3);
AccelStepper MirageStepper(1, 8, 9);

struct controlCommands {
    int controlMode; // 1 for manual rop control, 0 for automatic (pid)
    int drillMovementDirection; // 1 for down
    double speed;
    int miragePositionCmd;
    int drillZeroCommand;
};

controlCommands cmds = {
    .controlMode = 1,
    .drillMovementDirection = 0,
    .speed = 0,
    .miragePositionCmd = 0,
    .drillZeroCommand = 0
};

const int numCmds = 4;    //num of vars in struct above
const int sizeOfCmd = 40; //number of chars sent from matlab to arduino must be less than this

unsigned long currTime = 0;
unsigned long prevTime = 0;
const unsigned long sendRate = 100; //ms
bool incomingStringComplete = false; // whether the string is complete
int currPosOfChar = 0; //
char cstring[sizeOfCmd];
char* arrayOfcstring[numCmds];
char* myPointer;

// float augerArea =  PI * (0.02)^2;
float WOB = 109.99;
float targetWOB = 100; //Newtons
float WOBerror = 0; //pid
float WOBcumulativeError = 0;
float WOBrateError = 0;
float WOBprevError = 0;
float PIDspeedCmd = 0;
float Kp = 0; //pid
float Kd = 0; //pid
float Ki = 0;
float WOBcurrTime = 0;
float WOBprevTime = 0;
double WOBelapsedTime = 0;

int leadScrewLead = 8; // mm/rev
int stepsPerRev = 200; // (per rev) steps/rev, 1.8deg/step
float drillStepperMaxSpeed = 800; //steps per sec, over 1000 makes setSpeed() unreliable
float PIDstepperMaxSpeed = 400; //steps per sec
float zeroingStepperMaxSpeed = 800; //steps per sec

int drillLimitSwitchActive = 0;
float drillRPM = 0;
float drillCurrent = 0;
float drillPos = 0; //mm from top
int limitSwitchReached = 1; //1 if reached


void setup() {
    DrillStepper.setMaxSpeed(drillStepperMaxSpeed);
    MirageStepper.setMaxSpeed(400); // Steps per second
    MirageStepper.setSpeed(400);
    MirageStepper.setAcceleration(50); //Steps/sec^2

    Serial.begin(115200);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }
    Serial.flush();
}

void loop() {
    checkLimitSwitches();
    //update wob
    //update current
    //update other stuff

    if (drillLimitSwitchActive == 1) { //limit switch reached
        DrillStepper.setCurrentPosition(0); //resets internal accellstepper position tracker, ALSO sets speed to 0
        DrillStepper.setSpeed(0); //repetitive actually (see ^)
        if (cmds.drillMovementDirection == 1) { //moving down
            DrillStepper.setSpeed(cmds.speed * cmds.drillMovementDirection); //sets drill vertical speed
        }
    }
    else {
        if (cmds.drillZeroCommand == 1) { //zeroing
            DrillStepper.setSpeed(zeroingStepperMaxSpeed * -1); // moveup
        }
        else if (cmds.controlMode == 0) { //automatic/limit WOB/pid control mode
            setPIDcmd();
            DrillStepper.setSpeed(PIDspeedCmd);
        }
    }
    MirageStepper.run();
    DrillStepper.runSpeed();

    chooseMotorPosition(cmds.miragePositionCmd); //move mirage stepper to set pos

    doHousekeeping();
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/

void sendDataOut() {
    //indicies =====> [WOB, drillRPM, drillCurrent, drillPos, limitSwitchReached] ... update as needed
    drillPos = DrillStepper.currentPosition() / stepsPerRev * leadScrewLead; //mm from limit switch, add offset to get pos of tip of drillbit
    Serial.print(WOB, 2);
    Serial.print(",");
    Serial.print(drillRPM, 2);
    Serial.print(",");
    Serial.print(drillCurrent, 2);
    Serial.print(",");
    Serial.print(drillPos, 2);
    Serial.print(",");
    Serial.print(limitSwitchReached, DEC);
    Serial.print(",");



    //sanity check from matlab below
    Serial.print(cmds.controlMode, DEC); //formated as int
    Serial.print(",");
    Serial.print(cmds.drillMovementDirection, DEC); //formated as int
    Serial.print(",");
    Serial.print(cmds.speed, 2); //formated as float to 2 decials
    Serial.print(",");
    Serial.print(cmds.miragePositionCmd, DEC); //formated as int

    Serial.print("\n"); //serial terminator
}
void serialEvent() {
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        if (inChar == '\n') {
            cstring[currPosOfChar] = NULL; //end char array in null char
            incomingStringComplete = true;
            currPosOfChar = 0;
            // Serial.print("Arduino Recieved: " + String(cstring) + "\n");
            break;
        }
        cstring[currPosOfChar] = inChar;
        currPosOfChar++;
    }
}
void formatIncomingData() {
    //put stuff into correct array format
    int count = 0;
    myPointer = strtok(cstring, ",");
    while (myPointer != NULL) {
        arrayOfcstring[count] = myPointer;
        myPointer = strtok(NULL, ",");
        count++;
    }
    // Serial.println("here: " + String(arrayOfcstring[1]));
}
void buildDataStruct() {
    //mode
    cmds.controlMode = atoi(arrayOfcstring[0]);
    //direction
    int speed = atoi(arrayOfcstring[1]);
    if (speed == 1) {
        cmds.drillMovementDirection = 1;
    }
    else if (speed == -1) {
        cmds.drillMovementDirection = -1;
    }
    else {
        cmds.drillMovementDirection = 0;
    }
    //speed
    cmds.speed = atof(arrayOfcstring[2]);
    //mirage pos
    cmds.miragePositionCmd = atoi(arrayOfcstring[3]);
}
void doHousekeeping() {
    if (incomingStringComplete) {
        formatIncomingData(); //formats cmds data
        buildDataStruct(); //formats cmds data struct
        incomingStringComplete = false;
    }
    currTime = millis();
    if (currTime - prevTime >= sendRate) //every (sendRate) ms
    {
        sendDataOut();
        prevTime = millis();
    }
}


void chooseMotorPosition(int pos) {
    if (pos == 1) {
        MirageStepper.moveTo(67);
    }
    if (pos == 2) {
        MirageStepper.moveTo(133);
    }
    if (pos == 3) {
        MirageStepper.moveTo(200);
    }
    if (pos == 4) {
        MirageStepper.moveTo(267);
    }
    if (pos == 5) {
        MirageStepper.moveTo(334);
    }
    if (pos == 6) {
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

void checkLimitSwitches() {
    // check the pin
    // if curr pin status != pin status (reduce number of writings)
        // if the pin is high,
        // drillLimitSwitchActive = 1;
        // else
        // if
        // drillLimitSwitchActive = 1;
}
void setPIDcmd() {
    WOBcurrTime = millis();
    WOBelapsedTime = (float)(WOBcurrTime - WOBprevTime);
    WOBerror = targetWOB - WOB; //proportional
    WOBcumulativeError += WOBerror * WOBelapsedTime; //integral
    WOBrateError = (WOBerror - WOBprevError) / WOBelapsedTime; //deriv
    PIDspeedCmd = WOBerror * Kd + WOBcumulativeError * Ki + WOBrateError * Kd; //trial error for values
    PIDspeedCmd = constrain(PIDspeedCmd, 0, PIDstepperMaxSpeed); //clamps to PIDstepperMaxSpeed (steps/sec)

    WOBprevError = WOBerror;
    WOBprevTime = millis();
}
