// ----- Motor Encoder Pins -----
#define ENC_A_L 18
#define ENC_B_L 19
#define ENC_A_R 21
#define ENC_B_R 22

// Variabelen moeten 'volatile' zijn omdat ze in een ISR veranderen
volatile long ticksL = 0;
volatile long ticksR = 0;

// Interrupt Service Routines (ISR)
void IRAM_ATTR isrEncL() {
  if (digitalRead(ENC_A_L) == digitalRead(ENC_B_L)) ticksL--;
  else ticksL++;
}

void IRAM_ATTR isrEncR() {
  if (digitalRead(ENC_A_R) == digitalRead(ENC_B_R)) ticksR++;
  else ticksR--;
}

void setup() {
  Serial.begin(9600);

  // Encoders instellen
  pinMode(ENC_A_L, INPUT_PULLUP);
  pinMode(ENC_B_L, INPUT_PULLUP);
  pinMode(ENC_A_R, INPUT_PULLUP);
  pinMode(ENC_B_R, INPUT_PULLUP);

  // Koppel de interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_A_L), isrEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_R), isrEncR, CHANGE);

  Serial.println("Encoder Test Gestart.");
  Serial.println("Draai het wiel handmatig 1 volledige rotatie...");
}

void loop() {
  // Print elke 100ms de huidige stand
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    Serial.print("Links: ");
    Serial.print(ticksL);
    Serial.print(" | Rechts: ");
    Serial.println(ticksR);
    lastPrint = millis();
  }
}
