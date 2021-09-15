#define PIN_LED 7

void setup() {
  pinMode(PIN_LED, OUTPUT);
}

void loop() {
  digitalWrite(PIN_LED, LOW); //on
  delay(900);
  for (int i = 0; i < 5; i++) {
    delay(100);
    digitalWrite(PIN_LED, HIGH); //off
    delay(100);
    digitalWrite(PIN_LED, LOW);
  }
  digitalWrite(PIN_LED, HIGH); //off
  while(1){}
}
