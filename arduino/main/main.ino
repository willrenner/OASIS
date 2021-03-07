#include "AccelStepper.h"
#include <HX711_ADC.h> // Include ADC Libraries

struct controlCommands {//[drillCmdMode, dir, speed, miragePosition, rpm, heater, pump, tare, zeroCmd, fakeZeroAcitve]
    int drillControlMode; // 1 for manual rop control, 0 for automatic (pid)
    int drillMovementDirection; // 1 for down, 0 for stop, -1 for up
    double speed;
    int miragePositionCmd;
    int Drill_RPM;
    int heaterCmd;
    int pumpCmd;
    int tareCmd;
    int zeroCmd;
    int fakeZero;
};

controlCommands cmds = {//[drillCmdMode, dir, speed, miragePosition, rpm, heater, pump, tare, zeroCmd, fakeZeroAcitve]
    .drillControlMode = 1,
    .drillMovementDirection = 0,
    .speed = 0,
    .miragePositionCmd = 0,
    .Drill_RPM = 0,
    .heaterCmd = 0,
    .pumpCmd = 0,
    .tareCmd = 0,
    .zeroCmd = 0,
    .fakeZero = 0
};

#define numCmds 10 //num of vars in struct above
#define sizeOfCmd 100 //number of chars sent from matlab to arduino must be less than this
#define limitSwitchPin 5
#define AC_pin 999 //PWM pin for dimmer 
#define drillStepPin 3
#define drillDirPin 4
#define mirageStepPin 6
#define mirageDirPin 7
#define HX711_data_1 999 //digital pin (Jay used 1 and 2)
#define HX711_clck_1 999 //digital pin
#define heaterRelayPin 999
#define pumpRelayPin 999
#define sensor_interupt_pin 18 // interupt pin (On arduino Mega pins 2, 3, 18, 19, 20,& 21 can be used for interupts)
#define stepsPerRev 200 // (per rev) steps/rev, 1.8deg/step
#define leadScrewLead 8 // mm/rev
#define drillStepperMaxSpeed 800 //steps per sec, over 1000 makes setSpeed() unreliable says doc.
#define PIDstepperMaxSpeed 400 //steps per sec
#define zeroingStepperMaxSpeed 100 //steps per sec


unsigned long rotations         = 0;  
int      min_RPM_rotations      = 5;
unsigned long start_time        = 0;  // allocating start time value for t=0
byte     dim                    = 0;  //Sets initial brightness to 0 out of 255
float    mirageStepperGearRatio = 0;
float drillStepperRatio = leadScrewLead / stepsPerRev; //multiply by currentStep to get mm from limit switch, add offset to get pos of tip of drillbit
AccelStepper DrillStepper(1, drillStepPin, drillDirPin); //driver, step, dir pin
AccelStepper MirageStepper(1, mirageStepPin, mirageDirPin);
HX711_ADC LoadCell(HX711_data_1, HX711_clck_1); // Module 1 for drilling system

unsigned long currTime           = 0;
unsigned long prevTime           = 0;
// const    unsigned long checkRate = 20;     //ms
const    unsigned long sendRate  = 100;    //ms
bool     incomingStringComplete  = false;  // whether the string is complete
int      currPosOfChar           = 0;

char cstring[sizeOfCmd];
char* arrayOfcstring[numCmds];
char* myPointer;

// float augerArea =  PI * (0.02)^2;
float  WOB                = 0;
float  targetWOB          = 100;  //Newtons
float  WOBerror           = 0;    //pid
float  WOBcumulativeError = 0;
float  WOBrateError       = 0;
float  WOBprevError       = 0;
float  PIDspeedCmd        = 0;
float  Kp                 = 0;
float  Kd                 = 0;
float  Ki                 = 0;
float  WOBcurrTime        = 0;
float  WOBprevTime        = 0;
double WOBelapsedTime     = 0;

int      drillLimitSwitchActive = 0;  //1 for active
// unsigned long checkTime         = 0;
// unsigned long prevCheckTime     = 0;
float    drillRPM               = 0;
float    drillCurrent           = 0;
float    drillPos               = 0;  //mm from top
float    mirageAngle            = 0;  //degrees from start

// float voltages[60] = { 0 }; //60 points per full sine wave----- might can get from dimmer switch module
// float currents[60] = { 0 }; //60 points per full sine wave


unsigned long fpscount = 0;
unsigned long t1 = 0;
unsigned long t2 = 0;
void setup() {
    pinMode(sensor_interupt_pin, INPUT);
    pinMode(limitSwitchPin, INPUT);
    pinMode(heaterRelayPin, OUTPUT);
    pinMode(pumpRelayPin, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(sensor_interupt_pin), addValue, RISING);

    Serial.begin(115200);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }
    Serial.flush();

    // pinMode(AC_pin, OUTPUT);
    // attachInterrupt(0, Drill_RPM, FALLING); //actuator when pin is falling high to low to form square wave

    setupLoadCell();

    DrillStepper.setMaxSpeed(drillStepperMaxSpeed);
    MirageStepper.setMaxSpeed(400); // Steps per second
    MirageStepper.setAcceleration(50); //Steps/sec^2
}


void loop() {
    // fpsCounter();
    checkRelayCmds();
    checkLoadCellTare();
    // checkLimitSwitches(); can't bc not tied low (will bounce around if not actually connected)
    if (LoadCell.update()) WOB = LoadCell.getData();
    getdrillRPM();
    // getdrillVoltageCurrent();
    setDrillSpeed();
    setMiragePosition();


    // DrillStepper.setSpeed(cmds.speed * cmds.drillMovementDirection);
    // Serial.print("speed: ");
    // Serial.print(cmds.speed * cmds.drillMovementDirection);
    // Serial.print("      pos: ");
    // Serial.println(DrillStepper.currentPosition());

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
    drillPos = DrillStepper.currentPosition();
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
    //sanity check from matlab below
    //[drillCmdMode, dir, speed, miragePosition, rpm, heater, pump, tare]

    Serial.print(cmds.drillControlMode, DEC); //formated as int
    Serial.print(",");
    Serial.print(cmds.drillMovementDirection, DEC); //formated as int
    Serial.print(",");
    Serial.print(cmds.speed, 2); //formated as float to 2 decials
    Serial.print(",");
    Serial.print(cmds.miragePositionCmd, DEC); //formated as int
    Serial.print(",");
    Serial.print(cmds.Drill_RPM, DEC); //formated as int
    Serial.print(",");
    Serial.print(cmds.heaterCmd, DEC); //formated as int
    Serial.print(",");
    Serial.print(cmds.pumpCmd, DEC); //formated as int
    Serial.print(",");
    Serial.print(cmds.tareCmd, DEC); //formated as int
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
void buildDataStruct() { //[drillCmdMode, dir, speed, miragePosition, rpm, heater, pump, tare, zeroCmd, fakeZeroAcitve]
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
    //load cell
    cmds.tareCmd = atoi(arrayOfcstring[7]);
    //zero cmd
    cmds.zeroCmd = atoi(arrayOfcstring[8]);
    //fake zero
    cmds.fakeZero = atoi(arrayOfcstring[9]);
    if (cmds.fakeZero == 1) drillLimitSwitchActive = 1;
    else drillLimitSwitchActive = 0;
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
    if (cmds.pumpCmd == 1) {
        digitalWrite(pumpRelayPin, HIGH);
    }
    else digitalWrite(pumpRelayPin, LOW);

    if (cmds.heaterCmd == 1) digitalWrite(heaterRelayPin, HIGH);
    else digitalWrite(heaterRelayPin, LOW);
}

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
        if (cmds.zeroCmd == 1) { //zeroing
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

void checkLoadCellTare() {
    if (cmds.tareCmd == 1) {
        LoadCell.tareNoDelay();
        cmds.tareCmd = 0;
        Serial.println("Started Tare");
    }
    if (LoadCell.getTareStatus() == true) { //check if last tare operation is complete
        Serial.println("Tare Load Cell Complete");
    }
}

void setupLoadCell() {
    // Define calibration values given by the calibration script
    float calval_LoadCell;
    calval_LoadCell = 100;
    LoadCell.begin();
    unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
    boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
    byte LoadCell_ready = 0;

    //BLOCKING IF NOT ATTACHED?
    //run startup, stabilization and tare, both modules simultaniously
    // while ((LoadCell_ready) < 1) {
    //     if (!LoadCell_ready) LoadCell_ready = LoadCell.startMultiple(stabilizingtime, _tare);
    // }

    if (LoadCell.getTareTimeoutFlag()) {
        Serial.println("Timeout, check wiring and pin designations for load cell module");
    }
    // Apply calibration value
    LoadCell.setCalFactor(calval_LoadCell);
    Serial.println("Load Cell startup is complete");
}

void addValue() {
    rotations++; // counts
}
void getdrillRPM() {
    if (rotations >= min_RPM_rotations) { // doesn't allow loop to start until rotation value is met (basically stabilizing the measurement)
        unsigned long end_time = micros(); // assigns end time
        unsigned long elapsedtime = ((end_time - start_time) / 1000000.0); // How much time passed? (convert back to seconds from microseconds)
        drillRPM = (rotations / elapsedtime) * 60.0; // Convert to RPM
        rotations = 0;
        start_time = micros(); // reassigns start time to a new value
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

void fpsCounter() {
    t1 = millis();
    if ((t1 - t2) > 1000) {
        Serial.println(fpscount);
        fpscount = 0;
        t2 = millis();
    }
    else {
        fpscount++;
    }
}

void getdrillVoltageCurrent() {

}