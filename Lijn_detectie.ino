#define S1 34
#define S2 35
#define S3 32
#define S4 33
#define S5 25

void setup() {
  Serial.begin(9600);

  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);
}

void loop() {

  int s1 = digitalRead(S1);
  int s2 = digitalRead(S2);
  int s3 = digitalRead(S3);
  int s4 = digitalRead(S4);
  int s5 = digitalRead(S5);

  // Toon ruwe waarden
  Serial.print("Ruwe data: ");
  Serial.print(s1); Serial.print(" ");
  Serial.print(s2); Serial.print(" ");
  Serial.print(s3); Serial.print(" ");
  Serial.print(s4); Serial.print(" ");
  Serial.println(s5);

  // Toon visueel zwart/wit
  Serial.print("Lijn patroon: ");

  printSensor(s1);
  printSensor(s2);
  printSensor(s3);
  printSensor(s4);
  printSensor(s5);

  Serial.println();
  Serial.println("----------------------");

  delay(800);
}

void printSensor(int value) {
  if (value == LOW) {
    Serial.print("█");   // zwart gedetecteerd
  } else {
    Serial.print("_");   // wit
  }
}