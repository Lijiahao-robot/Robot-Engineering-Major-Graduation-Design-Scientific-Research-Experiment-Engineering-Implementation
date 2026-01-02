int sensor = 0;

void loop() {
    sensor = analogRead(A0);
    if(sensor < 300) {
        digitalWrite(9, HIGH);
    } else {
        digitalWrite(9, LOW);
    }
}
