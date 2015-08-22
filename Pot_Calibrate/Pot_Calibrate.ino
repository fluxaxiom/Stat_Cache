int potPin = 14;    // select the input pin for the potentiometer
int val = 0;       // variable to store the value coming from the sensor

void setup() {
  	Serial.begin(115200);
}

void loop() {
  val = analogRead(potPin);    // read the value from the sensor
  val = map(val, 0, 1023, 0, 190);
  val -=95;
  Serial.print(" Angle: ");
  Serial.println(val);

}
