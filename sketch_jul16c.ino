int sure;
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
sure=millis();
Serial.println(sure/1000);
delay(1000);

}
