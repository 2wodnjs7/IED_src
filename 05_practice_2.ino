#define PIN7 7

void setup() {
  pinMode(PIN7, OUTPUT);
}

void loop() {
  digitalWrite(PIN7, LOW);
  delay(1000);
  digitalWrite(PIN7, HIGH);
  delay(100);
  digitalWrite(PIN7, LOW);
  delay(100);
  digitalWrite(PIN7, HIGH);
  delay(100);
  digitalWrite(PIN7, LOW);
  delay(100);
  digitalWrite(PIN7, HIGH);
  delay(100);
  digitalWrite(PIN7, LOW);
  delay(100);
  digitalWrite(PIN7, HIGH);
  delay(100);
  digitalWrite(PIN7, LOW);
  delay(100);
  digitalWrite(PIN7, HIGH);
  delay(100);
  digitalWrite(PIN7, LOW);
  delay(100);
  digitalWrite(PIN7, HIGH);
  delay(100);
  while(1) {}
}
