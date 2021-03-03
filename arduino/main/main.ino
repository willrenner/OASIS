#include "AccelStepper.h"
#include <HX711_ADC.h> // Include ADC Libraries

struct controlCommands {//[drillCmdMode, dir, speed, miragePosition, rpm, heater, pump]
    int drillControlMode; // 1 for manual rop control, 0 for automatic (pid)
    int drillMovementDirection; // 1 for down, 0 for stop, -1 for up
    double speed;
    int miragePositionCmd;
    int drillZeroCommand;
    int Drill_RPM;
    int heaterCmd;
    int pumpCmd;
};

controlCommands cmds = {//[drillCmdMode, dir, speed, miragePosition, rpm, heater, pump]
    .drillControlMode = 1,
    .drillMovementDirection = 0,
    .speed = 0,
    .miragePositionCmd = 0,
    .drillZeroCommand = 0, //tbd
    .Drill_RPM = 0,
    .heaterCmd = 0,
    .pumpCmd = 0
};

#define limitSwitchPin 5
#define AC_pin 999 //PWM pin for dimmer
#define drillStepPin 3
#define drillDirPin 4
#define mirageStepPin 6
#define mirageDirPin 7
#define HX711_data_1 999
#define HX711_clck_1 999
#define heaterRelayPin 999
#define pumpRelayPin 999


byte dim = 0; //Sets initial brightness to 0 out of 255
AccelStepper DrillStepper(1, drillStepPin, drillDirPin); //driver, step, dir pin
AccelStepper MirageStepper(1, mirageStepPin, mirageDirPin);
//Match pins to adc/sensor modules
HX711_ADC LoadCell(HX711_data_1, HX711_clck_1); // Module 1 for drilling system
unsigned long t = 0;

unsigned long currTime = 0;
unsigned long prevTime = 0;
const unsigned long checkRate = 50; //ms
const unsigned long sendRate = 200; //ms
bool incomingStringComplete = false; // whether the string is complete
int currPosOfChar = 0;
const int numCmds = 4;    //num of vars in struct above
const int sizeOfCmd = 40; //number of chars sent from matlab to arduino must be less than this
char cstring[sizeOfCmd];
char* arrayOfcstring[numCmds];
char* myPointer;

// float augerArea =  PI * (0.02)^2;
float WOB = 0;
float targetWOB = 100; //Newtons
float WOBerror = 0; //pid
float WOBcumulativeError = 0;
float WOBrateError = 0;
float WOBprevError = 0;
float PIDspeedCmd = 0;
float Kp = 0;
float Kd = 0;
float Ki = 0;
float WOBcurrTime = 0;
float WOBprevTime = 0;
double WOBelapsedTime = 0;

int leadScrewLead = 8; // mm/rev
int stepsPerRev = 200; // (per rev) steps/rev, 1.8deg/step
float drillStepperMaxSpeed = 800; //steps per sec, over 1000 makes setSpeed() unreliable says doc.
float PIDstepperMaxSpeed = 400; //steps per sec
float zeroingStepperMaxSpeed = 800; //steps per sec

int drillLimitSwitchActive = 0; //1 for active
unsigned long checkTime = 0;
unsigned long prevCheckTime = 0;
float drillRPM = 0;
float drillCurrent = 0;
float drillPos = 0; //mm from top
float mirageAngle = 0; //degrees from start


void setup() {
    pinMode(limitSwitchPin, INPUT);
    pinMode(heaterRelayPin, OUTPUT);
    pinMode(pumpRelayPin, OUTPUT);

    Serial.begin(115200);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }
    Serial.flush();

    // pinMode(AC_pin, OUTPUT);
    // attachInterrupt(0, Drill_RPM, FALLING); //actuator when pin is falling high to low to form square wave

    float calval_LoadCell;
    // Define calibration values given by the calibration script
    calval_LoadCell = 100;
    LoadCell.begin();
    unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
    boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
    byte LoadCell_ready = 0;
    //run startup, stabilization and tare, both modules simultaniously
    // while ((LoadCell_ready) < 1) {
    //     if (!LoadCell_ready) LoadCell_ready = LoadCell.startMultiple(stabilizingtime, _tare);
    //     Serial.println("in loop");
    // }
    if (LoadCell.getTareTimeoutFlag()) {
        Serial.println("Timeout, check wiring and pin designations for load cell module");
    }
    // Apply calibration value
    LoadCell.setCalFactor(calval_LoadCell);
    Serial.println("Startup is complete");

    DrillStepper.setMaxSpeed(drillStepperMaxSpeed);
    MirageStepper.setMaxSpeed(400); // Steps per second
    // MirageStepper.setSpeed(400);
    MirageStepper.setAcceleration(50); //Steps/sec^2


}

void loop() {
    checkTime = millis();
    if (checkTime - prevCheckTime > checkRate) {//20 times per sec
        checkLimitSwitches(); //may need to set timer for this to not check every loop
        checkRelayCmds();
        prevCheckTime = millis();
    }

    setDrillSpeed();
    setMiragePosition();

    MirageStepper.run();
    DrillStepper.runSpeed();


    doHousekeeping();
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/

void sendDataOut() {
    //indicies =====> [WOB, drillRPM, drillCurrent, drillPos, mirageAngle, drillLimitSwitchActive] ... update as needed
    drillPos = DrillStepper.currentPosition() / stepsPerRev * leadScrewLead; //mm from limit switch, add offset to get pos of tip of drillbit
    mirageAngle = MirageStepper.currentPosition();
    Serial.print(WOB, 2);
    Serial.print(",");
    Serial.print(cmds.Drill_RPM, DEC);
    Serial.print(",");
    Serial.print(drillCurrent, 2);
    Serial.print(",");
    Serial.print(drillPos, 2);
    Serial.print(",");
    Serial.print(mirageAngle, 2);
    Serial.print(",");
    Serial.print(drillLimitSwitchActive, DEC);
    Serial.print(",");
    //mode, dir, speed, miragePos
    //sanity check from matlab below
    Serial.print(cmds.drillControlMode, DEC); //formated as int
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
            // Serial.println("Arduino Recieved data");
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
void buildDataStruct() { //[drillCmdMode, dir, speed, miragePosition, rpm, heater, pump]
    //mode
    cmds.drillControlMode = atoi(arrayOfcstring[0]);
    //direction
    cmds.drillMovementDirection = atoi(arrayOfcstring[1]);
    //speed
    cmds.speed = atof(arrayOfcstring[2]);
    //mirage pos
    cmds.miragePositionCmd = atoi(arrayOfcstring[3]);
    //rpm
    cmds.Drill_RPM = atof(arrayOfcstring[4]);
    //heater
    cmds.heaterCmd = atoi(arrayOfcstring[5]);
    //pump
    cmds.pumpCmd = atoi(arrayOfcstring[6]);

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


void setMiragePosition() {
    MirageStepper.moveTo(cmds.miragePositionCmd);
}

void checkRelayCmds() {
    if (cmds.pumpCmd == 1) digitalWrite(pumpRelayPin, HIGH);
    else digitalWrite(pumpRelayPin, LOW);

    if (cmds.heaterCmd == 1) digitalWrite(heaterRelayPin, HIGH);
    else digitalWrite(heaterRelayPin, LOW);
}
// void activateHeater(int command) {
//     if (command == 1) {
//         //set arduino pins connected to relay high
//     }
//     else if (command == 0) {
//         //set arduino pins connected to relay low
//     }
// }

// void activatePump(int command) {
//     if (command == 1) {
//         //set arduino pins connected to relay high
//     }
//     else if (command == 0) {
//         //set arduino pins connected to relay low
//     }
//     else if (command == -1) {
//         //reverse pump ??
//     }
// }
void getMSE() {
    // float drillTorque = getDrillTorque();
    // float drillRPM = getDrillRPM();
    // float ROP = cmds.speed;
    // float MSE = WOB/augerArea + drillTorque*drillRPM/(augerArea*ROP); //MSE equation
}

void checkLimitSwitches() {
    if (digitalRead(limitSwitchPin) == LOW) {
        drillLimitSwitchActive = 1;
        // Serial.println("Limit switch activated!");
    }
    else {
        drillLimitSwitchActive = 0;
    }
}
// void setPIDcmd() {
//     WOBcurrTime = millis();
//     WOBelapsedTime = (float)(WOBcurrTime - WOBprevTime);
//     WOBerror = targetWOB - WOB; //proportional
//     WOBcumulativeError += WOBerror * WOBelapsedTime; //integral
//     WOBrateError = (WOBerror - WOBprevError) / WOBelapsedTime; //deriv
//     PIDspeedCmd = WOBerror * Kd + WOBcumulativeError * Ki + WOBrateError * Kd; //trial error for values
//     PIDspeedCmd = constrain(PIDspeedCmd, 0, PIDstepperMaxSpeed); //clamps to PIDstepperMaxSpeed (steps/sec)
//     WOBprevError = WOBerror;
//     WOBprevTime = millis();
// }
void setDrillSpeed() {
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
        else if (cmds.drillControlMode == 0) { //automatic/limit WOB/pid control mode
            // setPIDcmd();
            DrillStepper.setSpeed(PIDspeedCmd);
        }
        else {
            DrillStepper.setSpeed(cmds.speed * cmds.drillMovementDirection);
        }
    }
}


// possible control of dimmer module
// void Drill_RPM() {
//     if (Serial.available()) {
//         dim = Serial.read();
//         if (dim < 1) {
//             //Turn TRIAC completely OFF if dim is 0
//             digitalWrite(AC_pin, LOW);
//         }

//         if (dim > 254) { //Turn TRIAC completely ON if dim is 255
//             digitalWrite(AC_pin, HIGH);
//         }
//     }

//     if (dim > 0 && dim < 255) {
//         //Dimming part, if dim is not 0 and not 255
//         delayMicroseconds(34 * (255 - dim));
//         digitalWrite(AC_pin, HIGH);
//         delayMicroseconds(500);
//         digitalWrite(AC_pin, LOW);
//     }
// }


// #define ZeroCrPin   2// interrupt 0
// volatile unsigned long zcUsec = 0;  // must be set to volatile when used within an interrupt.
// attachInterrupt(0, [] {zcUsec = micros()}, FALLING); // used an anonomous function (lambda funciton) to set zcUsec to micros()

