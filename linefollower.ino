#define S1_PIN 34  // Meest links (Far Left)
#define S2_PIN 35   // Links (Near Left)
#define S3_PIN 32   // Midden (Center)
#define S4_PIN 33   // Rechts (Near Right)
#define S5_PIN 25   // Meest rechts (Far Right)

// ------------------------------------------------------------
// Pin-definities – Motor Driver TB6612FNG
// ------------------------------------------------------------
#define STBY_PIN  4   // Standby (HIGH = actief)

// Motor A – Links
#define PWMA_PIN 14
#define AIN1_PIN 26
#define AIN2_PIN 27

// Motor B – Rechts
#define PWMB_PIN 18
#define BIN1_PIN  2
#define BIN2_PIN 15

// ------------------------------------------------------------
// Pin-definities – Ultrasone sensor HC-SR04
// ------------------------------------------------------------
#define TRIG_PIN 12
#define ECHO_PIN 13

// ------------------------------------------------------------
// Pin-definities – Encoders
// ------------------------------------------------------------
// ENC_L_A is gewijzigd naar pin 23 om het conflict met
// PWMB_PIN (18) te vermijden. Pas aan als je hardware
// een andere vrije pin gebruikt.
#define ENC_L_A 23
#define ENC_L_B 22
#define ENC_R_A 21
#define ENC_R_B 19

// ------------------------------------------------------------
// PWM-kanalen (ESP32 ledc API)
// ------------------------------------------------------------
#define PWM_CHANNEL_A    0
#define PWM_CHANNEL_B    1
#define PWM_FREQ       1000   // Hz
#define PWM_RESOLUTION    8   // bits (0-255)

// ------------------------------------------------------------
// PID-parameters – pas Kp, Ki en Kd aan voor je robot
// ------------------------------------------------------------
float Kp = 25.0;
float Ki =  0.0;
float Kd = 15.0;

// ------------------------------------------------------------
// Rijsnelheid-instellingen (0-255)
// ------------------------------------------------------------
const int BASISSNELHEID      = 140;  // Normale rijsnelheid
const int MAX_SNELHEID       = 200;  // Maximale motorsnelheid
const int DRAAI_SNELHEID     = 120;  // Snelheid bij zoeken na lijnverlies
const int OBSTAKEL_AFSTAND   =  30;  // cm – trigger obstakelontwijking

// ------------------------------------------------------------
// Sensorposities in cm (offset t.o.v. midden)
// ------------------------------------------------------------
const float SENSOR_POS[5] = { -4.0, -2.0, 0.0, 2.0, 4.0 };

// ------------------------------------------------------------
// Globale variabelen
// ------------------------------------------------------------
int   sensorWaarden[5];          // Ruwe sensorlezingen (0=zwart, 1=wit)
bool  sensorOpLijn[5];           // true als sensor op zwarte lijn staat

float fout         = 0.0;        // Huidige positiefout (cm)
float vorigeFout   = 0.0;        // Fout vorige iteratie (voor D-term)
float integraal    = 0.0;        // Gecumuleerde fout (voor I-term)
float pidUitvoer   = 0.0;        // PID-correctiewaarde

int   snelheidLinks  = 0;
int   snelheidRechts = 0;

// Richting van laatste bekende lijnpositie: -1 = links, +1 = rechts, 0 = midden
int   laatsteBekendeRichting = 0;

// Teller voor "zoek"-iteraties na lijnverlies
int   zoekTeller = 0;
const int MAX_ZOEK_ITERATIES = 80;  // ~800 ms bij 10 ms lus

// Encoder-tellers (vluchtig wegens interrupt)
volatile long encoderLinks  = 0;
volatile long encoderRechts = 0;

// Toestandsmachine
enum RijToestand {
  WACHT_OP_START,
  LIJN_VOLGEN,
  LIJN_ZOEKEN,
  OBSTAKEL_ONTWIJKEN,
  FINISH_BEREIKT
};
RijToestand toestand = WACHT_OP_START;

// Tijdbeheer
unsigned long vorigeTijd = 0;
const unsigned long LUS_INTERVAL = 10;  // ms

// ------------------------------------------------------------
// Interrupt-service-routines voor encoders
// ------------------------------------------------------------
void IRAM_ATTR isrEncoderLinksA() {
  encoderLinks++;
}
void IRAM_ATTR isrEncoderRechtsA() {
  encoderRechts++;
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  // IR-sensoren
  pinMode(S1_PIN, INPUT);
  pinMode(S2_PIN, INPUT);
  pinMode(S3_PIN, INPUT);
  pinMode(S4_PIN, INPUT);
  pinMode(S5_PIN, INPUT);

  // Motor-standby
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, LOW);  // Motoren uitgeschakeld tot start

  // Motor-richtingspinnen
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);

  // PWM-kanalen instellen (ESP32 ledc)
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWMA_PIN, PWM_CHANNEL_A);
  ledcAttachPin(PWMB_PIN, PWM_CHANNEL_B);

  // Ultrasone sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Encoders
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrEncoderLinksA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrEncoderRechtsA, RISING);

  // Motoren stoppen
  motorStop();

  Serial.println("Robot gereed. Wacht op startsignaal (alle sensoren op lijn)...");
}

// ============================================================
// HOOFD-LUS
// ============================================================
void loop() {
  unsigned long nu = millis();
  if (nu - vorigeTijd < LUS_INTERVAL) return;
  vorigeTijd = nu;

  readSensors();

  switch (toestand) {
    case WACHT_OP_START:
      behandelWachtOpStart();
      break;

    case LIJN_VOLGEN:
      behandelLijnVolgen();
      break;

    case LIJN_ZOEKEN:
      behandelLijnZoeken();
      break;

    case OBSTAKEL_ONTWIJKEN:
      avoidObstacle();
      break;

    case FINISH_BEREIKT:
      motorStop();
      break;
  }
}

// ============================================================
// readSensors()
// Leest alle 5 IR-sensoren.
// LOW (0) = zwart (lijn), HIGH (1) = wit (achtergrond)
// ============================================================
void readSensors() {
  int pinnen[5] = { S1_PIN, S2_PIN, S3_PIN, S4_PIN, S5_PIN };
  for (int i = 0; i < 5; i++) {
    sensorWaarden[i] = digitalRead(pinnen[i]);
    sensorOpLijn[i]  = (sensorWaarden[i] == LOW);  // LOW = zwart = op lijn
  }
}

// ============================================================
// allesSensorsBuit()  →  true als alle sensoren wit zien (lijn kwijt)
// ============================================================
bool alleSensorsWit() {
  for (int i = 0; i < 5; i++) {
    if (sensorOpLijn[i]) return false;
  }
  return true;
}

// ============================================================
// alleSensorsZwart()  →  true als alle sensoren zwart zien (finish/startbox)
// ============================================================
bool alleSensorsZwart() {
  for (int i = 0; i < 5; i++) {
    if (!sensorOpLijn[i]) return false;
  }
  return true;
}

// ============================================================
// berekenLijnPositie()
// Geeft gewogen gemiddelde positie terug in cm (-4..+4).
// Geeft 999 terug als er geen sensor op de lijn staat.
// ============================================================
float berekenLijnPositie() {
  float som = 0.0;
  int  aantalOpLijn = 0;
  for (int i = 0; i < 5; i++) {
    if (sensorOpLijn[i]) {
      som += SENSOR_POS[i];
      aantalOpLijn++;
    }
  }
  if (aantalOpLijn == 0) return 999.0;
  return som / aantalOpLijn;
}

// ============================================================
// calculatePID()
// Berekent de PID-correctie op basis van de lijnpositie.
// ============================================================
float calculatePID(float positie) {
  float huidigeFout = positie;  // Fout = afwijking van midden (0)

  // Integraal – begrens om windup te voorkomen
  integraal += huidigeFout;
  integraal = constrain(integraal, -100.0, 100.0);

  float afgeleide = huidigeFout - vorigeFout;

  pidUitvoer = (Kp * huidigeFout) + (Ki * integraal) + (Kd * afgeleide);

  vorigeFout = huidigeFout;
  return pidUitvoer;
}

// ============================================================
// motorControl()
// Stuurt de motoren aan met de gegeven snelheden (-255..+255).
// Positief = vooruit, negatief = achteruit.
// ============================================================
void motorControl(int links, int rechts) {
  // Begrenzen
  links  = constrain(links,  -MAX_SNELHEID, MAX_SNELHEID);
  rechts = constrain(rechts, -MAX_SNELHEID, MAX_SNELHEID);

  // Motor A – Links
  if (links >= 0) {
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, LOW);
  } else {
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, HIGH);
    links = -links;
  }
  ledcWrite(PWM_CHANNEL_A, links);

  // Motor B – Rechts
  if (rechts >= 0) {
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, LOW);
  } else {
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, HIGH);
    rechts = -rechts;
  }
  ledcWrite(PWM_CHANNEL_B, rechts);

  // Standby activeren (motoren rijden)
  digitalWrite(STBY_PIN, HIGH);
}

// ============================================================
// motorStop()
// Stopt beide motoren en schakelt standby in.
// ============================================================
void motorStop() {
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 0);
  digitalWrite(AIN1_PIN, LOW);
  digitalWrite(AIN2_PIN, LOW);
  digitalWrite(BIN1_PIN, LOW);
  digitalWrite(BIN2_PIN, LOW);
  digitalWrite(STBY_PIN, LOW);
}

// ============================================================
// meetAfstand()
// Geeft de gemeten afstand in cm terug via de HC-SR04.
// ============================================================
float meetAfstand() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duur = pulseIn(ECHO_PIN, HIGH, 25000);  // Timeout: 25 ms (~4 m max)
  if (duur == 0) return 999.0;  // Geen echo = geen obstakel
  return duur / 58.3;
}

// ============================================================
// behandelWachtOpStart()
// De robot staat stil en wacht tot hij op de lijn staat.
// Zodra minstens één sensor zwart detecteert, start de robot.
// ============================================================
void behandelWachtOpStart() {
  bool eenOpLijn = false;
  for (int i = 0; i < 5; i++) {
    if (sensorOpLijn[i]) { eenOpLijn = true; break; }
  }
  if (eenOpLijn) {
    Serial.println("Lijn gedetecteerd – robot start!");
    integraal  = 0.0;
    vorigeFout = 0.0;
    toestand   = LIJN_VOLGEN;
  }
}

// ============================================================
// behandelLijnVolgen()
// Normaal rijgedrag: PID + obstakel- en finishcontrole.
// ============================================================
void behandelLijnVolgen() {
  // --- Finish: alle sensoren zwart ---
  if (alleSensorsZwart()) {
    Serial.println("FINISH gedetecteerd – robot stopt!");
    motorStop();
    toestand = FINISH_BEREIKT;
    return;
  }

  // --- Obstakelcontrole ---
  float afstand = meetAfstand();
  if (afstand < OBSTAKEL_AFSTAND) {
    Serial.print("Obstakel gedetecteerd op ");
    Serial.print(afstand);
    Serial.println(" cm – ontwijken...");
    toestand = OBSTAKEL_ONTWIJKEN;
    return;
  }

  // --- Lijn kwijt → overschakelen naar zoeken ---
  if (alleSensorsWit()) {
    zoekTeller = 0;
    toestand   = LIJN_ZOEKEN;
    return;
  }

  // --- Normaal lijnvolgen via PID ---
  float positie  = berekenLijnPositie();
  float correctie = calculatePID(positie);

  // Sla richting op voor het geval lijn kwijtraakt
  if (positie < -0.5)      laatsteBekendeRichting = -1;
  else if (positie > 0.5) laatsteBekendeRichting =  1;
  else                     laatsteBekendeRichting =  0;

  snelheidLinks  = BASISSNELHEID + (int)correctie;
  snelheidRechts = BASISSNELHEID - (int)correctie;

  motorControl(snelheidLinks, snelheidRechts);
}

// ============================================================
// behandelLijnZoeken()
// Draait naar de laatste bekende zijde om de lijn terug te
// vinden. Geldt ook als "rechtdoor"-logica voor driehoek-,
// zaagtand- en golfhindernissen: de robot rijdt eerst kort
// rechtdoor (afsnijden) voordat hij gaat zoeken.
// ============================================================
void behandelLijnZoeken() {
  // Lijn teruggevonden
  if (!alleSensorsWit()) {
    integraal  = 0.0;  // Reset integraal om windup te vermijden
    zoekTeller = 0;
    toestand   = LIJN_VOLGEN;
    return;
  }

  // Fase 1: Eerste ~200 ms rechtdoor rijden (hindernissen afsnijden)
  if (zoekTeller < 20) {
    motorControl(BASISSNELHEID, BASISSNELHEID);
  }
  // Fase 2: Draaien naar laatste bekende richting
  else {
    if (laatsteBekendeRichting <= 0) {
      // Draai links
      motorControl(-DRAAI_SNELHEID, DRAAI_SNELHEID);
    } else {
      // Draai rechts
      motorControl(DRAAI_SNELHEID, -DRAAI_SNELHEID);
    }
  }

  zoekTeller++;

  // Maximale zoektijd overschreden → stop (veiligheid)
  if (zoekTeller > MAX_ZOEK_ITERATIES) {
    Serial.println("Lijn niet gevonden na zoeken – robot stopt.");
    motorStop();
    toestand = FINISH_BEREIKT;  // Gebruik FINISH als veilige stop-toestand
  }
}

// ============================================================
// avoidObstacle()
// Rijdt om een cilindrisch obstakel (Ø 20 cm) heen via een
// eenvoudige links-bypass en pikt daarna de lijn weer op.
//
// Volgorde:
//   1. Stap achteruit
//   2. Draai 90° links (weg van lijn)
//   3. Rij langs obstakel (rechte lijn)
//   4. Draai 90° rechts (parallel aan originele lijn)
//   5. Rij over de breedte van het obstakel
//   6. Draai 90° rechts (terug naar lijn)
//   7. Rij naar lijn toe
//   8. Draai 90° links om op lijn te richten
//   9. Hervat lijnvolgen
//
// De tijden zijn afhankelijk van rijsnelheid en motortype –
// pas ONTWIJKING_MS_* aan voor jouw robot.
// ============================================================

// Tijdsduren voor ontwijkingsmanoeuvre (ms) – aan te passen per robot.
// Kalibratie-tips:
//   ONTWIJKING_ACHTERUIT_MS : rijd ~5 cm achteruit bij DRAAI_SNELHEID.
//   ONTWIJKING_DRAAI90_MS   : draai precies 90° op de as bij DRAAI_SNELHEID.
//                             Meet de draaitijd op een vlakke ondergrond.
//   ONTWIJKING_ZIJWAARTS_MS : rijd minstens 25 cm zijwaarts (obstakeldiameter
//                             20 cm + marge) bij BASISSNELHEID.
//   ONTWIJKING_OVERKANT_MS  : rijd de lijn voorbij (~30 cm) bij BASISSNELHEID.
const int ONTWIJKING_ACHTERUIT_MS  =  300;
const int ONTWIJKING_DRAAI90_MS    =  400;
const int ONTWIJKING_ZIJWAARTS_MS  =  600;
const int ONTWIJKING_OVERKANT_MS   =  700;

// Time-out voor het terugzoeken naar de lijn na obstakelontwijking
const int LIJN_ZOEK_TIMEOUT_MS     = 3000;

void avoidObstacle() {
  Serial.println("Obstakelontwijking: start");

  // 1. Stap achteruit
  motorControl(-DRAAI_SNELHEID, -DRAAI_SNELHEID);
  delay(ONTWIJKING_ACHTERUIT_MS);

  // 2. Draai 90° links
  motorControl(-DRAAI_SNELHEID, DRAAI_SNELHEID);
  delay(ONTWIJKING_DRAAI90_MS);

  // 3. Rij langs het obstakel (links erlangs)
  motorControl(BASISSNELHEID, BASISSNELHEID);
  delay(ONTWIJKING_ZIJWAARTS_MS);

  // 4. Draai 90° rechts
  motorControl(DRAAI_SNELHEID, -DRAAI_SNELHEID);
  delay(ONTWIJKING_DRAAI90_MS);

  // 5. Rij over de breedte van het obstakel heen
  motorControl(BASISSNELHEID, BASISSNELHEID);
  delay(ONTWIJKING_OVERKANT_MS);

  // 6. Draai 90° rechts
  motorControl(DRAAI_SNELHEID, -DRAAI_SNELHEID);
  delay(ONTWIJKING_DRAAI90_MS);

  // 7. Rij naar de lijn toe totdat een sensor de lijn vindt
  //    of tot maximale tijd verstreken is
  Serial.println("Obstakelontwijking: zoeken naar lijn...");
  unsigned long startTijd = millis();
  motorControl(BASISSNELHEID, BASISSNELHEID);
  while (alleSensorsWit()) {
    readSensors();
    if (millis() - startTijd > LIJN_ZOEK_TIMEOUT_MS) break;  // Veiligheidstime-out
    delay(10);
  }

  // 8. Draai 90° links om op de lijn te richten
  motorControl(-DRAAI_SNELHEID, DRAAI_SNELHEID);
  delay(ONTWIJKING_DRAAI90_MS);

  // Reset PID en hervat lijnvolgen
  integraal  = 0.0;
  vorigeFout = 0.0;
  Serial.println("Obstakelontwijking: voltooid – lijnvolgen hervat");
  toestand = LIJN_VOLGEN;
}
