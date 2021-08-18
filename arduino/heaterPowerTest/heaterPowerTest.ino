
void setup() {
    Serial.begin(115200);
    Serial.println("ready");
}

void loop() {
    analogWrite(2, 127);
}
