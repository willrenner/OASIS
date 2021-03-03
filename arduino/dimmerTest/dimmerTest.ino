#include <RBDdimmer.h>

#define zcPin 2 
#define outputPin 3 
//(pwm, zc)
dimmerLamp dimmer(outputPin, zcPin); //initialase port for dimmer for MEGA, Leonardo, UNO, Arduino M0, Arduino Zero

void setup() {
    dimmer.begin(NORMAL_MODE, ON); //dimmer initialisation: name.begin(MODE, STATE) 
    dimmer.setPower(10); //10%
}
void loop() {}

