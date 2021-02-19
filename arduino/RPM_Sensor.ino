/* AARC Sensors & Prospecting ---> Author: John Stanfield _-_-_-_-_-_-
 RPM Sensor Code - Testing File -_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-
-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-
 Notation: _______________________________________________________________________________________________________
 warmup = Number of hall effect sensor trips before begins RPM reading
 rotations = number of rotations that have occured
_______________________________________________________________________________________________________________
*/
int sensor_interupt_pin = 3; // interupt pin (On arduino Mega pins 2, 3, 18, 19, 20,& 21 can be used for interupts)
unsigned long rotations = 0; //
int warmup = 10;

unsigned long start_time = 0; // allocating start time value for t=0
void setup() {
  Serial.begin(115200); // nits per second
  pinMode(sensor_interupt_pin, INPUT);

  attachInterrupt(digitalPinToInterrupt(sensor_interupt_pin), addValue, RISING);
}
void loop() {
  if (rotations >= warmup) { // doesn't allow loop to start until rotation value is met (basically stabilizing the measurement)
    unsigned long end_time = micros(); // assigns end time
    unsigned long elapsedtime = ((end_time - start_time) / 1000000.0); // How much time passed? (convert back to seconds from microseconds)
    float RPM = (rotations / elapsedtime) * 60.0; // Convert to RPM
    Serial.print(RPM); // Prints to serial monitor
    Serial.println(" RPM: "); // displays RPM: before data
    rotations = 0;
    start_time = micros(); // reassigns start time to a new value
  }
}
void addValue() {
  rotations++; // counts
}
