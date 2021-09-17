#include <AccelStepper.h>
#include <HX711_ADC.h> // Include ADC Libraries
#include <arduino-timer.h>
#include <Adafruit_MAX31855.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#define PID_SERIAL_LOGGER true
#define PUMPCMDS_SERIAL_LOGGER false

struct controlCommands {
    int drillControlMode; // 1 for manual rop control, 0 for automatic (pid)
    int drillMovementDirection;
    double speed;
    int miragePositionCmd;
    int Drill_RPM;
    int heaterCmd;
    int pumpCmd;
    int drillCmd;
    int tareCmd;
    int drillZeroCmd;
    int fakeZero;
    int WOBsetpoint;
    int Kp_Drill;
    int Ki_Drill;
    int Kd_Drill;
    float Kp_Heater;
    float Ki_Heater;
    float Kd_Heater;
    int TemperatureSetpoint;
    int HeaterPowerSetpoint;
    int Extraction_ROP_Speed_Cmd;
    int Pump_ROP_Speed_Cmd;
    int Extraction_ROP_Dir_Cmd;
    int Pump_ROP_Dir_Cmd;
    int Mirage_Speed_Cmd;
    int Mirage_Direction_Cmd;
    int ExtractionZeroCmd;
    int DrillPower;
};

controlCommands cmds = {
    .drillControlMode = 1,
    .drillMovementDirection = 0,
    .speed = 0,
    .miragePositionCmd = 0,
    .Drill_RPM = 0,
    .heaterCmd = 0,
    .pumpCmd = 0,
    .drillCmd = 0,
    .tareCmd = 0,
    .drillZeroCmd = 0,
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
    .Pump_ROP_Dir_Cmd = 0,
    .Mirage_Speed_Cmd = 0,
    .Mirage_Direction_Cmd = 0,
    .ExtractionZeroCmd = 0,
    .DrillPower = 0
};


#define numCmds 27 //num of vars in struct above
#define sizeOfCmd 300 //number of chars sent from matlab to arduino must be less than this
#define limitSwitchPin 7
#define drillStepPin 5 //stepper driver 4
#define drillDirPin 11
#define mirageStepPin 3 
#define mirageDirPin 9
#define extractionStepPin 2 //stepper driver 1
#define extractionDirPin 8 
#define pumpStepPin 4
#define pumpDirPin 10
const int LoadCellLeftData = 36;
const int LoadCellLeftClock = 38;
const int LoadCellRightData = 40;
const int LoadCellRightClock = 42;
#define heaterRelayPin 6
#define pumpRelayPin 7
#define drillRelayPin 34
#define currentSensorPin A0
#define thermocoupleCLK 28 //GREEN
#define thermocoupleDO 30 //BLUE
#define thermocoupleCS 29 //WHITE
#define heaterModulePin 13
#define servoPin 12


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
Servo servo;

HX711_ADC LoadCellLeft(LoadCellLeftData, LoadCellLeftClock); // Module 1 for drilling system
HX711_ADC LoadCellRight(LoadCellRightData, LoadCellRightClock); // Module 1 for drilling system

Adafruit_MAX31855 thermocouple(thermocoupleCLK, thermocoupleCS, thermocoupleDO);

Adafruit_ADS1015 ads1115;

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
#define currentSensorRate 5//240
// float ampArray[currentSensorRate]; //1 sec per final reading
int ampCounter = 0;
int16_t ads1115Results;
float peakCurrent = 0;
float totalSystemCurrent = 0;

float MSE = 0;
float augerArea = PI * pow(0.04, 2);
float  currentSensorValue = 0;
float  LoadCellLeftValue       = 0;
float  LoadCellRightValue      = 0;
float  LoadCellCombined = 0;
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
float heaterPIDoutput = 0;
float heaterTemperatureErrorSum = 0;
float heaterDt = 100; //ms


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
auto LoadCellTimer = timer_create_default();
auto HeaterTimer = timer_create_default();
//
void setup() {
    delay(1000);
    servo.attach(servoPin);
    amptimer.every(200, getCurrentSensorValue); //calls func every set period, don't want to call every loop b/c analog read is slow
    LoadCellTimer.every(100, getLoadCells); //calls func every set period, don't want to call every loop b/c analog read is slow
    HeaterTimer.every(heaterDt, setHeaterPower);
    setupLoadCells();
    Serial.println("Getting differential reading from AIN0 (P) and AIN1 (N)");
    Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV)");
   ads1115.begin();

    pinMode(RPMsensor_interupt_pin, INPUT);
    pinMode(limitSwitchPin, INPUT);
    pinMode(heaterRelayPin, OUTPUT);
    pinMode(pumpRelayPin, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(RPMsensor_interupt_pin), addValue, RISING);

    // pinMode(AC_pin, OUTPUT);
    // attachInterrupt(0, Drill_RPM, FALLING); //actuator when pin is falling high to low to form square wave

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
    LoadCellTimer.tick();
    HeaterTimer.tick();

    // fpsCounter();
    checkRelayCmds();
    checkLoadCellTare();
//  checkLimitSwitches(); //can't bc not tied low (will bounce around if not actually connected)
    getdrillRPM();
    getMSE();
    setStepperSpeeds();
    // setMiragePosition();    
    // MirageStepper.run();
    MirageStepper.runSpeed();   
    DrillStepper.runSpeed();   
    PumpStepper.runSpeed();
    ExtractionStepper.runSpeed();
    doHousekeeping();
}

void sendDataOut() {
    //LoadCellLeftValue, LoadCellRightValue, totalSystemCurrent, HeaterPower, HeaterTemp, DrillPos, ExtractionPos, MiragePos, LoadCellCombined ----- ACTIVE

    // drillPos = DrillStepper.currentPosition() * drillStepperRatio; //to get mm
    // mirageAngle = MirageStepper.currentPosition();
    Serial.print(LoadCellLeftValue, 2);
    Serial.print(",");
    Serial.print(LoadCellRightValue, 2);
    Serial.print(",");
    Serial.print(totalSystemCurrent, 2);
    Serial.print(",");
    Serial.print(cmds.HeaterPowerSetpoint);
    Serial.print(",");
    Serial.print(heaterTemperature, 2);
    Serial.print(",");
    Serial.print(-1 * DrillStepper.currentPosition() * 8/400.0, 2); //  8/400 is ratio to get from steps to mm
    Serial.print(",");
    Serial.print(ExtractionStepper.currentPosition() * 8 / 400.0, 2);// 8 / 400 is ratio to get from steps to mm
    Serial.print(",");
    Serial.print(MirageStepper.currentPosition(), 2); // in # of steps
    Serial.print(",");
    Serial.print(LoadCellCombined, 2); // in # of steps
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
//     % [drillCmdMode, dir, speed, miragePosition, rpm, heater, pump, tare,
//         % drillZeroCmd, fakeZeroAcitve, drillCmd, WOBsetpoint, Kp_Drill, Ki_Drill,
//         % Kd_Drill, Kp_Heater, Ki_Heater, Kd_Heater, TemperatureSetpoint,
//         % HeaterPowerSetpoint, Extraction_ROP_Speed_Cmd, Pump_ROP_Speed_Cmd,
//         % Extraction_ROP_Direction_Cmd, Pump_ROP_Direction_Cmd, Mirage_Speed_Cmd, Mirage_Direction_Cmd, ExtractionZeroCmd]
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
    cmds.pumpCmd = atoi(arrayOfcstring[6]); //not used
    //load cell
    cmds.tareCmd = atoi(arrayOfcstring[7]);
    //zero cmd
    cmds.drillZeroCmd = atoi(arrayOfcstring[8]);
    //fake zero
    cmds.fakeZero = atoi(arrayOfcstring[9]);
    // if (cmds.fakeZero == 1) drillLimitSwitchActive = 1;
    // else drillLimitSwitchActive = 0;
    cmds.drillCmd = atoi(arrayOfcstring[10]);
    cmds.WOBsetpoint = atoi(arrayOfcstring[11]);
    cmds.Kp_Drill = atoi(arrayOfcstring[12]);
    cmds.Ki_Drill = atoi(arrayOfcstring[13]);
    cmds.Kd_Drill = atoi(arrayOfcstring[14]);
    cmds.Kp_Heater = atoi(arrayOfcstring[15]) / 100.0; //because we mult by 100 on matlab app side
    cmds.Ki_Heater = atoi(arrayOfcstring[16]) / 100.0;
    cmds.Kd_Heater = atoi(arrayOfcstring[17]) / 100.0;
    cmds.TemperatureSetpoint = atoi(arrayOfcstring[18]);
    cmds.HeaterPowerSetpoint = atoi(arrayOfcstring[19]);
    cmds.Extraction_ROP_Speed_Cmd = atoi(arrayOfcstring[20]);
    // Serial.print("cmds.Pump_ROP_speed_Cmd1: "); Serial.print(cmds.Pump_ROP_Speed_Cmd);
    cmds.Pump_ROP_Speed_Cmd = atoi(arrayOfcstring[21]);
    // Serial.print("|| cmds.Pump_ROP_speed_Cmd2: "); Serial.print(cmds.Pump_ROP_Speed_Cmd);
    cmds.Extraction_ROP_Dir_Cmd = atoi(arrayOfcstring[22]);
    // Serial.print(" || cmds.Pump_ROP_dir_Cmd1: "); Serial.print(cmds.Pump_ROP_Dir_Cmd);
    cmds.Pump_ROP_Dir_Cmd = atoi(arrayOfcstring[23]);
    // Serial.print(" || cmds.Pump_ROP_dir_Cmd2: "); Serial.println(cmds.Pump_ROP_Dir_Cmd);
    cmds.Mirage_Speed_Cmd = atoi(arrayOfcstring[24]);
    cmds.Mirage_Direction_Cmd = atoi(arrayOfcstring[25]);
    cmds.ExtractionZeroCmd = atoi(arrayOfcstring[26]);
    cmds.DrillPower = atoi(arrayOfcstring[27]);
}
// % [drillCmdMode, dir, speed, miragePosition, rpm, heater, pump, tare,
// % DrillZeroCmd, fakeZeroAcitve, drillCmd, WOBsetpoint, Kp_Drill, Ki_Drill,
// % Kd_Drill, Kp_Heater, Ki_Heater, Kd_Heater, TemperatureSetpoint,
// % HeaterPowerSetpoint, Extraction_ROP_Speed_Cmd, Pump_ROP_Speed_Cmd,
// % Extraction_ROP_Direction_Cmd, Pump_ROP_Direction_Cmd, Mirage_Speed_Cmd, Mirage_Direction_Cmd, ExtractionZeroCmd, DrillPower]
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


// void setMiragePosition() {
//     MirageStepper.moveTo(cmds.miragePositionCmd);
// }

void checkRelayCmds() {
    // if (cmds.pumpCmd == 0) {
    //     digitalWrite(pumpRelayPin, HIGH);
    // }
    // else digitalWrite(pumpRelayPin, LOW);

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
    // WOBcurrTime = micros();
    // WOBelapsedTime = (float)(WOBcurrTime - WOBprevTime);
    // WOBerror = cmds.WOBsetpoint - WOB; //proportional
    // WOBcumulativeError += WOBerror * WOBelapsedTime * 1e6; //integral
    // WOBrateError = (WOBerror - WOBprevError) / (WOBelapsedTime * 1e6); //deriv
    // PIDspeedCmd = WOBerror * cmds.Kp_Drill + WOBcumulativeError * cmds.Ki_Drill + WOBrateError * cmds.Kd_Drill; //trial error for values
    // PIDspeedCmd = constrain(PIDspeedCmd, 0, PIDstepperMaxSpeed); //clamps to PIDstepperMaxSpeed (steps/sec)
    // WOBprevError = WOBerror;
    // WOBprevTime = micros();
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
    //     if (cmds.drillZeroCmd == 1) { //zeroing
    //         DrillStepper.setSpeed(zeroingStepperMaxSpeed * -1); // moveup
    //     }
    //     else if (cmds.drillControlMode == 0) { //automatic/limit WOB/pid control mode
    //         setROP();
    //         DrillStepper.setSpeed(PIDspeedCmd);
    //     }
    //     else {
            DrillStepper.setSpeed(-1 * cmds.speed * cmds.drillMovementDirection * mmPerSec_to_stepsPerSec); //for lead screw
        // }
    // }
            ExtractionStepper.setSpeed(cmds.Extraction_ROP_Speed_Cmd * cmds.Extraction_ROP_Dir_Cmd * mmPerSec_to_stepsPerSec); // for lead screw
            PumpStepper.setSpeed(cmds.Pump_ROP_Speed_Cmd * cmds.Pump_ROP_Dir_Cmd);
            MirageStepper.setSpeed(cmds.Mirage_Speed_Cmd * cmds.Mirage_Direction_Cmd);
}

void checkLoadCellTare() {
    if (cmds.tareCmd == 1) {
        LoadCellLeft.tareNoDelay();
        LoadCellRight.tareNoDelay();
        cmds.tareCmd = 0;
        Serial.println("Started Tare");
    }
    if ((LoadCellLeft.getTareStatus() == true) && (LoadCellRight.getTareStatus() == true)) { //check if last tare operation is complete
        Serial.println("Tare Load Cell Complete");
    }
}
void checkZeroCommands() {
    if (cmds.drillZeroCmd == 1) {
        DrillStepper.setCurrentPosition(0);
        cmds.drillZeroCmd = 0;
    }
    if (cmds.ExtractionZeroCmd == 1) {
        ExtractionStepper.setCurrentPosition(0);
        cmds.ExtractionZeroCmd = 0;
    }
}

void setupLoadCells() {
    // Define calibration values given by the calibration script
    float calval_LoadCell;
    calval_LoadCell = 59900;//-62390;//-54728; //7.3
    LoadCellLeft.begin();
    LoadCellRight.begin();
    unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
    boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
    byte LoadCell_ready = 0;

    //BLOCKING IF NOT ATTACHED?
    //run startup, stabilization and tare, both modules simultaniously
    // while ((LoadCell_ready) < 1) {
    //     if (!LoadCell_ready) LoadCell_ready = LoadCell.startMultiple(stabilizingtime, _tare);
    // }

    if (LoadCellLeft.getTareTimeoutFlag()) {
        Serial.println("Timeout on Left, check wiring and pin designations for load cell module");
    }
        if (LoadCellRight.getTareTimeoutFlag()) {
        Serial.println("Timeout on Right, check wiring and pin designations for load cell module");
    }
    // Apply calibration value
    LoadCellLeft.setCalFactor(calval_LoadCell);
    LoadCellRight.setCalFactor(calval_LoadCell);
    Serial.println("Load Cell startup is complete");
}
void setupThermocouple() {
    
    Serial.println("Thermcouple setup complete!");
    Serial.print("Int. Temp = ");
    Serial.println((float)thermocouple.readFahrenheit(), 2);
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
        fpscount = 0;
        t2 = millis();
    }
    else {
        fpscount++;
    }
}

bool getCurrentSensorValue(void*) { //analog read is slow
    if (ampCounter > currentSensorRate) {
        totalSystemCurrent = 0.707 * peakCurrent;
        ampCounter = 0;
        peakCurrent = 0;
    }

    ads1115Results = ads1115.readADC_Differential_0_1();
    drillCurrent = (float)ads1115Results * 3 * 30 / 1000; //convert mV to Amps, 30A = 1V on ads1115
    if (drillCurrent > peakCurrent) {
        peakCurrent = drillCurrent;
    }
    ampCounter++;
//  Serial.print("Differential: "); Serial.print(results); Serial.print("("); Serial.print(results * 3); Serial.print("mV)");
//  Serial.print(" Amps: ");
//  Serial.println(drillCurrent,2);
  return true;
}
// #define currentSensorRate 240
// float ampArray[currentSensorRate]; //1 sec per final reading
// int ampCounter = 0;

bool getLoadCells(void*) {
    if (LoadCellLeft.update()) LoadCellLeftValue = LoadCellLeft.getData(); //this is slow culprit
    if (LoadCellRight.update()) LoadCellRightValue = LoadCellRight.getData(); //this is slow culprit
    LoadCellCombined = LoadCellLeftValue + LoadCellRightValue;
    // Serial.println(LoadCellCombined, 2);

    return true;
}
bool setHeaterPower(void*) {
    //ignore next line
    servo.write(cmds.DrillPower * -1 + 180); //switch rotation direction

    // analogWrite(heaterModulePin, cmds.HeaterPowerSetpoint * 2.55);
    heaterTemperature = (float)thermocouple.readFahrenheit();

    if (isnan(heaterTemperature)) {
        Serial.println("ERROR: Thermcouple temperature NAN!");
    }
    else {
        heaterTemperatureError = cmds.HeaterPowerSetpoint - heaterTemperature;

        heaterTemperatureErrorSum += heaterTemperatureError * heaterDt / 1000.0;

        
        heaterPIDoutput = heaterTemperatureError * cmds.Kp_Heater + heaterTemperatureErrorSum * cmds.Ki_Heater;
        heaterPIDoutput = constrain(heaterPIDoutput, 0.0, 100.0);
        analogWrite(heaterModulePin, heaterPIDoutput * 2.55); //heater power from 0 to 100 percent
        // analogWrite(heaterModulePin, cmds.HeaterPowerSetpoint); //heater power from 0 to 100 percent
        
        #if PID_SERIAL_LOGGER == true
            Serial.print(" || kp: ");
            Serial.print(cmds.Kp_Heater);
            Serial.print(" || DrillPower: ");
            Serial.print(cmds.DrillPower);
            Serial.print(" || heaterTemperatureError: ");
            Serial.print(heaterTemperatureError);
            Serial.print(" || heaterTemperatureErrorSum: ");
            Serial.print(heaterTemperatureErrorSum);

            Serial.print(" || heaterTemperatureError * kp: ");
            Serial.print(heaterTemperatureError * cmds.Kp_Heater);
            Serial.print(" || heaterTemperatureErrorSum * ki: ");
            Serial.print(heaterTemperatureErrorSum * cmds.Ki_Heater);

            Serial.print(" || heaterPIDoutput: ");
            Serial.println(heaterPIDoutput);
        #endif
    }
    return true;
}
