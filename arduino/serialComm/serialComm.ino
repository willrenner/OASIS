#include <AccelStepper.h>

uint8_t pin2 = 2;
uint8_t pin3 = 3;
AccelStepper stepper(1, pin2,pin3);

struct controlCommands
{
    int drillMovementDirection; // 1 for down
    double speed;
};
controlCommands cmds = {
        .drillMovementDirection = 0,
        .speed = 0
    };

const int numCmds = 2;
const int sizeOfCmd = 40;


long int printCount = 0;
unsigned long currTime = 0;
unsigned long prevTime = 0;
const unsigned long sendRate = 100;
String inputString = "";             // a String to hold incoming data
bool incomingStringComplete = false; // whether the string is complete
bool buildingIncomingData = false;
int index;
int currPosOfChar = 0;
char cstring[sizeOfCmd];
char *arrayOfcstring[numCmds];
char *myPointer;

void sendDataOut();
void buildDataStruct();
void formatIncomingData();
void setup()
{
    stepper.setMaxSpeed(400);
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
        formatIncomingData();
        buildDataStruct();
        stepper.setSpeed((int)(cmds.speed * cmds.drillMovementDirection));
        incomingStringComplete = false;
    }

    currTime = millis();
    if (currTime - prevTime >= sendRate) //every 1 second
    {
        sendDataOut();
        prevTime = millis();
    }
    stepper.runSpeed();
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
            // Serial.print("Arduino Recieved: " + String(cstring) + "\n");
            break;
        }
        cstring[currPosOfChar] = inChar;
        currPosOfChar++;
    }
}
void sendDataOut()
{
    //send data back to serial

    Serial.print(cmds.drillMovementDirection, DEC);
    Serial.print(",");
    Serial.print(cmds.speed, 2);
    Serial.print("\n");
}
void buildDataStruct()
{
    // Serial.println(arrayOfcstring[0]);
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
