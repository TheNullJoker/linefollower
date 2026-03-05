#define TRIG_PIN 12
#define ECHO_PIN 13

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {

  // Zorg dat TRIG laag is
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Stuur 10 microseconden puls
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Meet hoe lang ECHO HIGH blijft
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Bereken afstand in cm
  float distance = duration / 58.3;

  Serial.print("Afstand: ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(500);
}