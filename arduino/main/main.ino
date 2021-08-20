#include <AccelStepper.h>
#include <HX711_ADC.h> // Include ADC Libraries
#include <arduino-timer.h>
#include <Adafruit_MAX31855.h>



struct controlCommands {//[drillCmdMode, dir, speed, miragePosition, rpm, heater, pump, tare, zeroCmd, fakeZeroAcitve, drillCmd, WOBsetpoint]
    int drillControlMode; // 1 for manual rop control, 0 for automatic (pid)
    int drillMovementDirection; // 1 for down, 0 for stop, -1 for up
    double speed;
    int miragePositionCmd;
    int Drill_RPM;
    int heaterCmd;
    int pumpCmd;
    int drillCmd;
    int tareCmd;
    int zeroCmd;
    int fakeZero;
    int WOBsetpoint;
    int Kp_Drill;
    int Ki_Drill;
    int Kd_Drill;
    int Kp_Heater;
    int Ki_Heater;
    int Kd_Heater;
    int TemperatureSetpoint;
    int HeaterPowerSetpoint;
    int Extraction_ROP_Speed_Cmd;
    int Pump_ROP_Speed_Cmd;
    int Extraction_ROP_Dir_Cmd;
    int Pump_ROP_Dir_Cmd;
};

controlCommands cmds = {//[drillCmdMode, dir, speed, miragePosition, rpm, heater, pump, tare, zeroCmd, fakeZeroAcitve, drillCmd, WOBsetpoint]
    .drillControlMode = 1,
    .drillMovementDirection = 0,
    .speed = 0,
    .miragePositionCmd = 0,
    .Drill_RPM = 0,
    .heaterCmd = 0,
    .pumpCmd = 0,
    .drillCmd = 0,
    .tareCmd = 0,
    .zeroCmd = 0,
    .fakeZero = 0,
    .WOBsetpoint = 0,
    .Kp_Drill = 0,
    .Ki_Drill = 0,
    .Kd_Drill = 0,
    .Kp_Heater = 0,
    .Ki_Heater = 0,
    .Kd_Heater = 0,
    .TemperatureSetpoint = 0,
    .HeaterPowerSetpoint = 0,
    .Extraction_ROP_Speed_Cmd = 0,
    .Pump_ROP_Speed_Cmd = 0,
    .Extraction_ROP_Dir_Cmd = 0,
    .Pump_ROP_Dir_Cmd = 0

};


#define numCmds 25 //num of vars in struct above
#define sizeOfCmd 200 //number of chars sent from matlab to arduino must be less than this
#define limitSwitchPin 7
#define drillStepPin 2
#define drillDirPin 8
#define mirageStepPin 3 
#define mirageDirPin 9
#define extractionStepPin 5 
#define extractionDirPin 11 
#define pumpStepPin 4
#define pumpDirPin 10
const int HX711_data_1 = 10;
const int HX711_clck_1 = 11;
#define heaterRelayPin 6
#define pumpRelayPin 7
#define drillRelayPin 34
#define currentSensorPin A0
#define thermocoupleCLK 28 //GREEN
#define thermocoupleDO 30 //BLUE
#define thermocoupleCS 29 //WHITE
#define heaterModulePin A1

#define currentSensorRate 120
#define RPMsensor_interupt_pin 18 // interupt pin (On arduino Mega pins 2, 3, 18, 19, 20,& 21 can be used for interupts)
#define stepsPerRev 200 // (per rev) steps/rev, 1.8deg/step
#define leadScrewLead 8 // mm/rev
#define drillStepperMaxSpeed 1000 //steps per sec, over 1000 makes setSpeed() unreliable says doc.  (1875 is max per the linear rail, 4000 per the stepper motor)
#define mmPerSec_to_stepsPerSec 25 //mult mm/sec by this to get steps/sec
#define PIDstepperMaxSpeed 50 //steps per sec
#define zeroingStepperMaxSpeed 1000 //steps per sec

unsigned long rotations         = 0;  
int      min_RPM_rotations      = 5;
unsigned long start_time        = 0;  // allocating start time value for t=0
byte     dim                    = 0;  //Sets initial brightness to 0 out of 255
float    mirageStepperGearRatio = 0;
float drillStepperRatio = leadScrewLead / stepsPerRev; //multiply by currentStep to get mm from limit switch, add offset to get pos of tip of drillbit
AccelStepper DrillStepper(1, drillStepPin, drillDirPin); //driver, step, dir pin
AccelStepper MirageStepper(1, mirageStepPin, mirageDirPin);
AccelStepper ExtractionStepper(1, extractionStepPin, extractionDirPin);
AccelStepper PumpStepper(1, pumpStepPin, pumpDirPin);

HX711_ADC LoadCell(HX711_data_1, HX711_clck_1); // Module 1 for drilling system
Adafruit_MAX31855 thermocouple(thermocoupleCLK, thermocoupleCS, thermocoupleDO);


unsigned long currTime           = 0;
unsigned long prevTime           = 0;
// const    unsigned long checkRate = 20;     //ms
const    unsigned long sendRate  = 100;    //ms
bool     incomingStringComplete  = false;  // whether the string is complete
int      currPosOfChar           = 0;

char cstring[sizeOfCmd];
char* arrayOfcstring[numCmds];
char* myPointer;

//Amp array
#define ampArraySize 20
float ampArray[ampArraySize];
int ampCounter = 0;

float MSE = 0;
float augerArea = PI * pow(0.04, 2);
float  currentSensorValue = 0;
float  WOB                = 0;
float  targetWOB          = 50;  //Newtons
float  WOBerror           = 0;    //pid
float  WOBcumulativeError = 0;
float  WOBrateError       = 0;
float  WOBprevError       = 0;
float  PIDspeedCmd        = 0;

float  WOBcurrTime        = 0;
float  WOBprevTime        = 0;
double WOBelapsedTime     = 0;

float heaterTemperature = 0;
float heaterTemperatureError = 0;
float heaterTemperatureSetpoint = 0; //degrees C
float heaterKp = 0;
float heaterPower = 0;

int      drillLimitSwitchActive = 0;  //1 for active
// unsigned long checkTime         = 0;
// unsigned long prevCheckTime     = 0;
float    drillRPM               = 0;
float    drillCurrent           = 0;
float    drillPos               = 0;  //mm from top
float    mirageAngle            = 0;  //degrees from start

unsigned long fpscount = 0;
unsigned long t1 = 0;
unsigned long t2 = 0;
auto amptimer = timer_create_default();
auto WOBtimer = timer_create_default();
auto heatertimer = timer_create_default();
//
void setup() {
    delay(1000);
    amptimer.every(((float)1 / currentSensorRate) * 1000, getCurrentSensorValue); //calls func every set period, don't want to call every loop b/c analog read is slow
    WOBtimer.every(50, getCurrentSensorValue); //calls func every set period, don't want to call every loop b/c analog read is slow
    // heatertimer.every(50, setHeaterPower);
    analogWrite(heaterModulePin, 255);

    pinMode(RPMsensor_interupt_pin, INPUT);
    pinMode(limitSwitchPin, INPUT);
    pinMode(heaterRelayPin, OUTPUT);
    pinMode(pumpRelayPin, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(RPMsensor_interupt_pin), addValue, RISING);

    // pinMode(AC_pin, OUTPUT);
    // attachInterrupt(0, Drill_RPM, FALLING); //actuator when pin is falling high to low to form square wave

    setupLoadCell();
    setupThermocouple();

    DrillStepper.setMaxSpeed(drillStepperMaxSpeed);
    MirageStepper.setMaxSpeed(400); // Steps per second
    PumpStepper.setMaxSpeed(800); // Steps per second
    ExtractionStepper.setMaxSpeed(drillStepperMaxSpeed); // Steps per second

    MirageStepper.setAcceleration(50); //Steps/sec^2

    Serial.begin(115200);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }
    Serial.flush();
}


void loop() {
    amptimer.tick();
    WOBtimer.tick();

    fpsCounter();
    checkRelayCmds();
    checkLoadCellTare();
//  checkLimitSwitches(); //can't bc not tied low (will bounce around if not actually connected)
    getdrillRPM();
    getMSE();
    setStepperSpeeds();
    setMiragePosition();    
    MirageStepper.run();
    DrillStepper.runSpeed();   
    PumpStepper.runSpeed();
    ExtractionStepper.runSpeed();
    heaterTemperature = (float)thermocouple.readCelsius() * 1.8 + 32;
    doHousekeeping();
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/

void sendDataOut() {
    //indicies =====> [WOB, drillRPM, drillCurrent, drillPos, mirageAngle, drillLimitSwitchActive, MSE, heaterTemp,heaterPower] ... update as needed
    drillPos = DrillStepper.currentPosition() * drillStepperRatio; //to get mm
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
    Serial.print(MSE, 2);
    Serial.print(",");
    Serial.print(cmds.HeaterPowerSetpoint, 2);
    Serial.print(",");
    Serial.print(heaterPower, 2);
    Serial.print(",");
    //sanity check from matlab below
    //[drillCmdMode, dir, speed, miragePosition, rpm, heater, pump, tare]

    // Serial.print(cmds.drillControlMode, DEC); //formated as int
    // Serial.print(",");
    // Serial.print(cmds.drillMovementDirection, DEC); //formated as int
    // Serial.print(",");
    // Serial.print(cmds.speed, 2); //formated as float to 2 decials
    // Serial.print(",");
    // Serial.print(cmds.miragePositionCmd, DEC); //formated as int
    // Serial.print(",");
    // Serial.print(cmds.Drill_RPM, DEC); //formated as int
    // Serial.print(",");
    // Serial.print(cmds.heaterCmd, DEC); //formated as int
    // Serial.print(",");
    // Serial.print(cmds.pumpCmd, DEC); //formated as int
    // Serial.print(",");
    // Serial.print(cmds.tareCmd, DEC); //formated as int
    // Serial.print(",");
    // Serial.print(cmds.drillCmd, DEC); //formated as int
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
void buildDataStruct() { 
    // [drillCmdMode, dir, speed, miragePosition, rpm, heater, pump, tare, zeroCmd, 
    // fakeZeroAcitve, drillCmd, WOBsetpoint, Kp_Drill, Ki_Drill, Kd_Drill, Kp_Heater, 
    // Ki_Heater,Kd_Heater, TemperatureSetpoint, HeaterPowerSetpoint, Extraction_ROP_Speed_Cmd, Pump_ROP_Speed_Cmd]
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
    // if (cmds.fakeZero == 1) drillLimitSwitchActive = 1;
    // else drillLimitSwitchActive = 0;
    cmds.drillCmd = atoi(arrayOfcstring[10]);
    cmds.WOBsetpoint = atoi(arrayOfcstring[11]);
    cmds.Kp_Drill = atoi(arrayOfcstring[12]);
    cmds.Ki_Drill = atoi(arrayOfcstring[13]);
    cmds.Kd_Drill = atoi(arrayOfcstring[14]);
    cmds.Kp_Heater = atoi(arrayOfcstring[15]);
    cmds.Ki_Heater = atoi(arrayOfcstring[16]);
    cmds.Kd_Heater = atoi(arrayOfcstring[17]);
    cmds.TemperatureSetpoint = atoi(arrayOfcstring[18]);
    cmds.HeaterPowerSetpoint = atoi(arrayOfcstring[19]);
    cmds.Extraction_ROP_Speed_Cmd = atoi(arrayOfcstring[20]);
    cmds.Pump_ROP_Speed_Cmd = atoi(arrayOfcstring[21]);
    cmds.Extraction_ROP_Dir_Cmd = atoi(arrayOfcstring[22]);
    cmds.Pump_ROP_Dir_Cmd = atoi(arrayOfcstring[23]);
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
    if (cmds.pumpCmd == 0) {
        digitalWrite(pumpRelayPin, HIGH);
    }
    else digitalWrite(pumpRelayPin, LOW);

    if (cmds.heaterCmd == 0) {
        digitalWrite(heaterRelayPin, HIGH);
    }//
    else digitalWrite(heaterRelayPin, LOW);

    if (cmds.drillCmd == 0) {
        digitalWrite(drillRelayPin, HIGH);
    }
    else digitalWrite(drillRelayPin, LOW);
}

void getMSE() {
    // float ROP = cmds.speed / 1000; //m/sec
    // if (ROP < 0.0001) ROP = 0.0001;
    // MSE = WOB/augerArea + drillCurrent*drillRPM/(augerArea*ROP); //MSE equation, assuming torque is proportional to current
}

void checkLimitSwitches() {
    // if (digitalRead(limitSwitchPin) == LOW) {
    //     drillLimitSwitchActive = 1;
    //     // Serial.println("Limit switch activated!");
    // }
    // else {
    //     drillLimitSwitchActive = 0;
    // }
}
void setROP() {
    WOBcurrTime = micros();
    WOBelapsedTime = (float)(WOBcurrTime - WOBprevTime);
    WOBerror = cmds.WOBsetpoint - WOB; //proportional
    WOBcumulativeError += WOBerror * WOBelapsedTime * 1e6; //integral
    WOBrateError = (WOBerror - WOBprevError) / (WOBelapsedTime * 1e6); //deriv
    PIDspeedCmd = WOBerror * cmds.Kp_Drill + WOBcumulativeError * cmds.Ki_Drill + WOBrateError * cmds.Kd_Drill; //trial error for values
    PIDspeedCmd = constrain(PIDspeedCmd, 0, PIDstepperMaxSpeed); //clamps to PIDstepperMaxSpeed (steps/sec)
    WOBprevError = WOBerror;
    WOBprevTime = micros();
}
void setStepperSpeeds() {
    // if (drillLimitSwitchActive == 1) { //limit switch reached
    //     DrillStepper.setCurrentPosition(0); //resets internal accellstepper position tracker, ALSO sets speed to 0
    //     DrillStepper.setSpeed(0); //repetitive actually (see ^)
    //     if (cmds.drillMovementDirection == 1) { //moving down
    //         DrillStepper.setSpeed(cmds.speed * cmds.drillMovementDirection * mmPerSec_to_stepsPerSec); //sets drill vertical speed
    //     }
    // }
    // else {
    //     if (cmds.zeroCmd == 1) { //zeroing
    //         DrillStepper.setSpeed(zeroingStepperMaxSpeed * -1); // moveup
    //     }
    //     else if (cmds.drillControlMode == 0) { //automatic/limit WOB/pid control mode
    //         setROP();
    //         DrillStepper.setSpeed(PIDspeedCmd);
    //     }
    //     else {
            DrillStepper.setSpeed(cmds.speed * cmds.drillMovementDirection * mmPerSec_to_stepsPerSec); //for lead screw
        // }
    // }
            ExtractionStepper.setSpeed(cmds.Extraction_ROP_Speed_Cmd * cmds.Extraction_ROP_Dir_Cmd * mmPerSec_to_stepsPerSec); // for lead screw
            PumpStepper.setSpeed(-1 * cmds.Pump_ROP_Speed_Cmd * cmds.Pump_ROP_Dir_Cmd);

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
    calval_LoadCell = -7500;
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
void setupThermocouple() {
    
    Serial.println("Thermcouple setup complete!");
    Serial.print("Int. Temp = ");
    Serial.println((float)thermocouple.readCelsius() * 1.8 + 32, 2);
}

void addValue() {
    rotations++; // counts
}
void getdrillRPM() {
    // if (rotations >= min_RPM_rotations) { // doesn't allow loop to start until rotation value is met (basically stabilizing the measurement)
    //     unsigned long end_time = micros(); // assigns end time
    //     unsigned long elapsedtime = ((end_time - start_time) / 1000000.0); // How much time passed? (convert back to seconds from microseconds)
    //     drillRPM = (rotations / elapsedtime) * 60.0; // Convert to RPM
    //     rotations = 0;
    //     start_time = micros(); // reassigns start time to a new value
    // }
}

void fpsCounter() {
    t1 = millis();
    if ((t1 - t2) > 1000) {
        Serial.print("FPS: ");
        Serial.println(fpscount);
        Serial.print("Temp: ");
        Serial.println(heaterTemperature, 2);
        Serial.print("Heater power: ");
        Serial.println(cmds.HeaterPowerSetpoint);
        fpscount = 0;
        t2 = millis();
    }
    else {
        fpscount++;
    }
}

bool getCurrentSensorValue(void*) { //analog read is slow
    // //idk if any of this works
    // float val = -0.04757 * analogRead(currentSensorPin) + 24.36; //eqn to get amperage, y = mx + b by testing
    // // val = random(0,4);
    // if (ampCounter >= ampArraySize) ampCounter = 0; //loop back to start of array, essentially keep the last ampArraySize number of values
    // ampArray[ampCounter] = val;
    // ampCounter++;
    // float sumOfSquares = 0;
    // for(int i=0; i < ampArraySize; i++){
    //     sumOfSquares += pow(ampArray[i],2);
    // }
    // float RMScurrentVal = sqrt(sumOfSquares); //this rms value of last 10 moving measurements
    // drillCurrent = 0.1*drillCurrent + 0.9*RMScurrentVal; //filter the values with complementary filter
    // return true;
}
bool getWOB(void*) {
    if (LoadCell.update()) WOB = LoadCell.getData(); //this is slow culprit
    return true;
}
bool setHeaterPower(void*) {
    analogWrite(heaterModulePin, 250);
    // analogWrite(heaterModulePin, cmds.HeaterPowerSetpoint * 2.5);

    // heaterTemperature = (float)thermocouple.readCelsius();
    // Serial.print("Heater temp: ");
    // Serial.println((float)thermocouple.readInternal(), 2);

    // if (isnan(heaterTemperature)) {
    //     Serial.println("ERROR: Thermcouple temperature NAN!");
    // }
    // else {
    //     heaterTemperatureError = cmds.TemperatureSetpoint - heaterTemperature;
    //     heaterPower = heaterTemperatureError * cmds.Kp_Heater;
    //     if (heaterPower < 0) heaterPower = 0; //turn off heater
    //     analogWrite(heaterModulePin, heaterPower);
    //     // Serial.print("heater power: ");
    //     // Serial.println(heaterPower, 2);
    // }
    return true;
}
