#include <Preferences.h>

// ============================================================
// Pin-definities – IR-sensoren (LOW = lijn/zwart)
// ============================================================
#define S1_PIN 34  // Meest links (Far Left)
#define S2_PIN 35  // Links (Near Left)
#define S3_PIN 32  // Midden (Center) – iets verder vooruit gemonteerd
#define S4_PIN 33  // Rechts (Near Right)
#define S5_PIN 25  // Meest rechts (Far Right)

// ------------------------------------------------------------
// Pin-definities – Knop (start / later mode)
// ------------------------------------------------------------
// DOIT ESP32 DevKit v1: D5 = GPIO5
// Sluit knop aan tussen GPIO5 en GND.
// We gebruiken INPUT_PULLUP => ingedrukt = LOW.
// Let op: GPIO5 is een strapping pin; vermijd reboot/flash terwijl je de knop ingedrukt houdt.
#define BTN_PIN 5

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
// PWM-instellingen (ESP32 LEDC)
// ------------------------------------------------------------
// Let op: Nieuwere Arduino-ESP32 cores gebruiken een pin-gebaseerde LEDC API:
//   ledcAttach(pin, freq, resolutionBits)
//   ledcWrite(pin, duty)
// Daarom gebruiken we hier geen kanaal-IDs meer.
#define PWM_FREQ       1000   // Hz
#define PWM_RESOLUTION    8   // bits (0-255)

// ------------------------------------------------------------
// PID-parameters – pas Kp, Ki en Kd aan voor je robot
// ------------------------------------------------------------
float Kp = 25.0;
float Ki =  0.0;
float Kd = 15.0;
const float KP_ZIGZAG = 50.0;  // Verhoogde Kp bij zigzag/scherpe bocht (S3 vroegwaarschuwing)

// ------------------------------------------------------------
// Rijsnelheid-instellingen (0-255)
// ------------------------------------------------------------
const int BASISSNELHEID      = 60;  // Normale rijsnelheid
const int MAX_SNELHEID       = 90;  // Maximale motorsnelheid
const int MIN_SNELHEID       = 40;  // Minimale PWM om motor-stall te voorkomen (afstemmen!)
const int DRAAI_SNELHEID     = 50;  // Snelheid bij zoeken na lijnverlies
const int OBSTAKEL_AFSTAND   = 20;  // cm – trigger obstakelontwijking

// ------------------------------------------------------------
// Doodlopend einde / labyrinth-constanten
// ------------------------------------------------------------
const unsigned long LIJN_KWIJT_DOODLOPEND_MS = 1000;  // ms lijn kwijt → doodlopend einde
const long   TICKS_VOOR_180_GRADEN           = 120;   // Encoderticks voor 180° (kalibreren!)
const int    MAX_LABYRINTH_BESLISSINGEN       = 50;    // Max. opgeslagen kruispuntbeslissingen

// ------------------------------------------------------------
// Kruispunt-tijden (ms) – kalibreren per robot
// ------------------------------------------------------------
const unsigned long KRUISPUNT_CENTERING_MS = 250;   // Vooruit rijden om op kruispunt te centreren
const unsigned long KRUISPUNT_DRAAI_MS     = 400;   // Tijd voor 90°-draai bij kruispunt

// ------------------------------------------------------------
// Sensorposities in cm (offset t.o.v. midden)
// ------------------------------------------------------------
const float SENSOR_POS[5] = { -5.0, -1.9, 0.0, 1.9, 5.0 };

// ------------------------------------------------------------
// NVS-sleutels voor labyrinth-opslag (Preferences.h)
// ------------------------------------------------------------
const char* NVS_NAMESPACE      = "labyrinth";
const char* NVS_SLEUTEL_PAD    = "pad";
const char* NVS_SLEUTEL_AANTAL = "aantal";

// ------------------------------------------------------------
// Globale variabelen – sensoren
// ------------------------------------------------------------
int   sensorWaarden[5];
bool  sensorOpLijn[5];
bool  vorigeSensorOpLijn[5] = {false, false, false, false, false};  // Voor zigzag-detectie

// ------------------------------------------------------------
// Globale variabelen – PID
// ------------------------------------------------------------
float fout         = 0.0;
float vorigeFout   = 0.0;
float integraal    = 0.0;
float pidUitvoer   = 0.0;
bool  scherpeBochtModus = false;  // true = S3 vroegwaarschuwing actief

// ------------------------------------------------------------
// Globale variabelen – motorsnelheden & richting
// ------------------------------------------------------------
int   snelheidLinks  = 0;
int   snelheidRechts = 0;
int   laatsteBekendeRichting = 0;  // -1=links, +1=rechts, 0=midden

// ------------------------------------------------------------
// Globale variabelen – lijnzoeken
// ------------------------------------------------------------
int   zoekTeller = 0;
const int MAX_ZOEK_ITERATIES = 80;  // ~800 ms bij 10 ms lus

// ------------------------------------------------------------
// Globale variabelen – encoders
// ------------------------------------------------------------
volatile long encoderLinks  = 0;
volatile long encoderRechts = 0;

// ------------------------------------------------------------
// Globale variabelen – knop (debounce + edge detect)
// ------------------------------------------------------------
bool          knopStaatIngedrukt      = false;
bool          vorigeKnopStaatIngedrukt = false;
unsigned long laatsteDebounceTijd     = 0;
const unsigned long DEBOUNCE_MS       = 30;
unsigned long knopIngedruktSinds      = 0;   // Voor lang-indruk detectie

// ------------------------------------------------------------
// Globale variabelen – doodlopend einde (dead end)
// ------------------------------------------------------------
unsigned long lijnKwijtTijd          = 0;
bool          lijnKwijtTimerGestart  = false;

// ------------------------------------------------------------
// Globale variabelen – kruispunt-verwerking
// ------------------------------------------------------------
long encoderLinksOpKruispunt  = 0;
long encoderRechtsOpKruispunt = 0;
bool kruispuntVerwerkt        = false;  // Debounce: voorkom dubbele detectie

// Fase binnen de KRUISPUNT-toestand
enum KruispuntFase {
  KF_VOORUIT_CENTEREN,  // Rij vooruit om te centreren op kruispunt
  KF_DRAAIEN,           // Draai naar gekozen richting
  KF_WACHT_OP_LIJN      // Wacht tot lijn gevonden is
};
KruispuntFase kruispuntFase      = KF_VOORUIT_CENTEREN;
unsigned long kruispuntStartTijd = 0;

bool kruispuntLinksOptie     = false;
bool kruispuntRechtdoorOptie = false;
bool kruispuntRechtsOptie    = false;
char gekozenKruispuntRichting = 'S';  // 'L'=links, 'S'=rechtdoor, 'R'=rechts, 'U'=U-bocht

// ------------------------------------------------------------
// Globale variabelen – labyrinth / maze mapping
// ------------------------------------------------------------
char labyrinthPad[MAX_LABYRINTH_BESLISSINGEN];
int  aantalLabyrinthBeslissingen = 0;
int  huidigLabyrinthIndex        = 0;
bool herhaalmodus                = false;  // true = volg opgeslagen pad

Preferences voorkeuringen;

// ------------------------------------------------------------
// Hoofd-toestandsmachine
// ------------------------------------------------------------
enum RijToestand {
  WACHT_OP_START,
  LIJN_VOLGEN,
  LIJN_ZOEKEN,
  KRUISPUNT,
  DOODLOPEND_EINDE,
  TERUGRIJDEN,
  OBSTAKEL_ONTWIJKEN,
  FINISH_BEREIKT
};
RijToestand toestand = WACHT_OP_START;

// Tijdbeheer
unsigned long vorigeTijd = 0;
const unsigned long LUS_INTERVAL = 10;  // ms

// ============================================================
// Voorwaartse declaraties
// ============================================================
void behandelKruispunt();
void behandelDoodlopendEinde();
void behandelTerugrijden();
char bepaalKruispuntRichtingLinkseHand();
bool isKruispuntGedetecteerd();
void slaOpLabyrinthPad();
bool laadLabyrinthPad();
void wisLabyrinthPad();

// ============================================================
// Interrupt-service-routines voor encoders
// ============================================================
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

  // Knop
  pinMode(BTN_PIN, INPUT_PULLUP);

  // Motor-standby
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, LOW);  // Motoren uitgeschakeld tot start

  // Motor-richtingspinnen
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);

  // PWM instellen (ESP32 LEDC - nieuwe API)
  ledcAttach(PWMA_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PWMB_PIN, PWM_FREQ, PWM_RESOLUTION);

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

  // Laad opgeslagen labyrinthpad uit NVS
  herhaalmodus = laadLabyrinthPad();
  if (herhaalmodus) {
    Serial.println("Opgeslagen labyrinthpad gevonden – herhaalstand actief.");
    Serial.print("Opgeslagen beslissingen: ");
    Serial.println(aantalLabyrinthBeslissingen);
  } else {
    Serial.println("Geen opgeslagen pad – verkenstand actief (linkse-handregel).");
  }

  Serial.println("Robot gereed. Druk kort om te starten, houd 3 s in om pad te wissen.");
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

    case KRUISPUNT:
      behandelKruispunt();
      break;

    case DOODLOPEND_EINDE:
      behandelDoodlopendEinde();
      break;

    case TERUGRIJDEN:
      behandelTerugrijden();
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
// Slaat ook de vorige sensorstaat op voor zigzag-detectie.
// ============================================================
void readSensors() {
  int pinnen[5] = { S1_PIN, S2_PIN, S3_PIN, S4_PIN, S5_PIN };
  for (int i = 0; i < 5; i++) {
    vorigeSensorOpLijn[i] = sensorOpLijn[i];          // Sla vorige staat op
    sensorWaarden[i]      = digitalRead(pinnen[i]);
    sensorOpLijn[i]       = (sensorWaarden[i] == LOW); // LOW = zwart = op lijn
  }
}

// ============================================================
// alleSensorsWit()  →  true als alle sensoren wit zien (lijn kwijt)
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
  int   aantalOpLijn = 0;
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
// Gebruikt een verhoogde Kp in zigzag/scherpe-bocht-modus.
// ============================================================
float calculatePID(float positie) {
  float huidigeFout = positie;  // Fout = afwijking van midden (0)

  // Integraal – begrens om windup te voorkomen
  integraal += huidigeFout;
  integraal = constrain(integraal, -100.0, 100.0);

  float afgeleide = huidigeFout - vorigeFout;

  // Verhoog Kp tijdelijk als zigzag/scherpe bocht gedetecteerd (S3 vroegwaarschuwing)
  float effectieveKp = scherpeBochtModus ? KP_ZIGZAG : Kp;
  pidUitvoer = (effectieveKp * huidigeFout) + (Ki * integraal) + (Kd * afgeleide);

  vorigeFout = huidigeFout;
  return pidUitvoer;
}

// ============================================================
// motorControl()
// Stuurt de motoren aan met de gegeven snelheden (-255..+255).
// Positief = vooruit, negatief = achteruit.
// Niet-nul waarden worden begrensd tot minstens MIN_SNELHEID
// om motor-stall te voorkomen.
// ============================================================
void motorControl(int links, int rechts) {
  // Begrenzen op maximale snelheid
  links  = constrain(links,  -MAX_SNELHEID, MAX_SNELHEID);
  rechts = constrain(rechts, -MAX_SNELHEID, MAX_SNELHEID);

  // MIN_SNELHEID toepassen: niet-nul waarden moeten minstens MIN_SNELHEID zijn
  if (links  > 0 && links  < MIN_SNELHEID)  links  = MIN_SNELHEID;
  if (links  < 0 && links  > -MIN_SNELHEID) links  = -MIN_SNELHEID;
  if (rechts > 0 && rechts < MIN_SNELHEID)  rechts = MIN_SNELHEID;
  if (rechts < 0 && rechts > -MIN_SNELHEID) rechts = -MIN_SNELHEID;

  // Motor A – Links
  if (links >= 0) {
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, LOW);
  } else {
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, HIGH);
    links = -links;
  }
  ledcWrite(PWMA_PIN, links);

  // Motor B – Rechts
  if (rechts >= 0) {
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, LOW);
  } else {
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, HIGH);
    rechts = -rechts;
  }
  ledcWrite(PWMB_PIN, rechts);

  // Standby activeren (motoren rijden)
  digitalWrite(STBY_PIN, HIGH);
}

// ============================================================
// motorStop()
// Stopt beide motoren en schakelt standby in.
// ============================================================
void motorStop() {
  ledcWrite(PWMA_PIN, 0);
  ledcWrite(PWMB_PIN, 0);
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
// knopPressedEvent()
// Geeft true terug bij een "nieuwe druk" (edge: niet -> ingedrukt).
// ============================================================
bool knopPressedEvent() {
  bool rawPressed = (digitalRead(BTN_PIN) == LOW);

  if (rawPressed != knopStaatIngedrukt) {
    laatsteDebounceTijd = millis();
    if (rawPressed) knopIngedruktSinds = millis();
    knopStaatIngedrukt = rawPressed;
  }

  if (millis() - laatsteDebounceTijd < DEBOUNCE_MS) {
    return false;
  }

  bool event = (!vorigeKnopStaatIngedrukt && knopStaatIngedrukt);
  vorigeKnopStaatIngedrukt = knopStaatIngedrukt;
  return event;
}

// Geeft true als knop nu langer dan drempelTijd ms ingedrukt is
bool knopLangIngedrukt(unsigned long drempelTijd) {
  bool rawPressed = (digitalRead(BTN_PIN) == LOW);
  if (rawPressed && knopStaatIngedrukt) {
    return (millis() - knopIngedruktSinds >= drempelTijd);
  }
  return false;
}

// ============================================================
// behandelWachtOpStart()
// Robot staat stil en wacht op knopdruk om te starten.
// Lang indrukken (≥ 3 s) wist het opgeslagen labyrinthpad.
// ============================================================
void behandelWachtOpStart() {
  motorStop();

  // Lang indrukken: wis opgeslagen pad en start in verkenstand
  if (knopLangIngedrukt(3000)) {
    wisLabyrinthPad();
    Serial.println("Pad gewist – verkenstand gestart!");
    integraal  = 0.0;
    vorigeFout = 0.0;
    zoekTeller = 0;
    // Consumeer de knopstaat zodat een volgende kortere druk geen spurious event geeft
    vorigeKnopStaatIngedrukt = knopStaatIngedrukt;
    toestand   = LIJN_VOLGEN;
    return;
  }

  if (knopPressedEvent()) {
    Serial.println("Knop ingedrukt – robot start!");
    integraal  = 0.0;
    vorigeFout = 0.0;
    zoekTeller = 0;
    toestand   = LIJN_VOLGEN;
  }
}

// ============================================================
// behandelLijnVolgen()
// Normaal rijgedrag: PID + zigzag-detectie + kruispunt- en
// obstakel- en finishcontrole.
// ============================================================
void behandelLijnVolgen() {
  // Knop = pauze/stop
  if (knopPressedEvent()) {
    Serial.println("Knop ingedrukt – robot pauze/stop (terug naar WACHT_OP_START)");
    motorStop();
    toestand = WACHT_OP_START;
    return;
  }

  // --- Finish: alle sensoren zwart ---
  if (alleSensorsZwart()) {
    slaOpLabyrinthPad();  // Sla het gevolgde pad op voor de volgende run
    Serial.println("FINISH gedetecteerd – pad opgeslagen, robot stopt!");
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
    lijnKwijtTijd         = millis();
    lijnKwijtTimerGestart = true;
    zoekTeller = 0;
    toestand   = LIJN_ZOEKEN;
    return;
  }

  // --- Kruispunt-detectie: buitenste sensoren beide op lijn ---
  // (Y-kruispunt, T-kruising of volledige kruising)
  if (isKruispuntGedetecteerd() && !kruispuntVerwerkt) {
    Serial.println("Kruispunt gedetecteerd!");
    encoderLinksOpKruispunt  = encoderLinks;
    encoderRechtsOpKruispunt = encoderRechts;
    kruispuntFase      = KF_VOORUIT_CENTEREN;
    kruispuntStartTijd = millis();
    kruispuntVerwerkt  = true;
    toestand = KRUISPUNT;
    return;
  }

  // Reset kruispunt-debounce zodra buitenste sensoren beide wit zijn
  if (!sensorOpLijn[0] && !sensorOpLijn[4]) {
    kruispuntVerwerkt = false;
  }

  // --- Zigzag / scherpe-bocht-detectie via S3 (verder vooruit gemonteerd) ---
  // Trigger als S3 op lijn staat EN een buitensensor zojuist van de lijn afviel.
  bool s1Uitgevallen = vorigeSensorOpLijn[0] && !sensorOpLijn[0];
  bool s5Uitgevallen = vorigeSensorOpLijn[4] && !sensorOpLijn[4];
  if (sensorOpLijn[2] && (s1Uitgevallen || s5Uitgevallen)) {
    scherpeBochtModus = true;
  } else {
    scherpeBochtModus = false;  // Expliciet resetten als triggerconditie niet meer geldt
  }

  // --- Normaal lijnvolgen via PID ---
  float positie   = berekenLijnPositie();
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
// vinden. Als de lijn langer dan LIJN_KWIJT_DOODLOPEND_MS
// kwijt is, wordt een doodlopend einde aangenomen.
// ============================================================
void behandelLijnZoeken() {
  // Lijn teruggevonden
  if (!alleSensorsWit()) {
    integraal             = 0.0;
    zoekTeller            = 0;
    lijnKwijtTimerGestart = false;
    toestand              = LIJN_VOLGEN;
    return;
  }

  // Lijn kwijt > 1 seconde → doodlopend einde
  if (lijnKwijtTimerGestart && (millis() - lijnKwijtTijd > LIJN_KWIJT_DOODLOPEND_MS)) {
    Serial.println("Lijn > 1 s kwijt – doodlopend einde gedetecteerd!");
    lijnKwijtTimerGestart = false;
    motorStop();
    delay(100);
    toestand = DOODLOPEND_EINDE;
    return;
  }

  // Fase 1: Eerste ~200 ms rechtdoor rijden (hindernissen afsnijden)
  if (zoekTeller < 20) {
    motorControl(BASISSNELHEID, BASISSNELHEID);
  }
  // Fase 2: Draaien naar laatste bekende richting
  else {
    if (laatsteBekendeRichting <= 0) {
      motorControl(-DRAAI_SNELHEID, DRAAI_SNELHEID);  // Draai links
    } else {
      motorControl(DRAAI_SNELHEID, -DRAAI_SNELHEID);  // Draai rechts
    }
  }

  zoekTeller++;

  // Maximale zoektijd overschreden → stop (veiligheid)
  if (zoekTeller > MAX_ZOEK_ITERATIES) {
    Serial.println("Lijn niet gevonden na zoeken – robot stopt.");
    motorStop();
    toestand = FINISH_BEREIKT;
  }
}

// ============================================================
// isKruispuntGedetecteerd()
// true als buitenste sensoren beide op lijn staan maar het
// geen finish/startbox is (niet alleSensorsZwart).
// ============================================================
bool isKruispuntGedetecteerd() {
  return sensorOpLijn[0] && sensorOpLijn[4] && !alleSensorsZwart();
}

// ============================================================
// bepaalKruispuntRichtingLinkseHand()
// Past de linkse-handregel toe: L > S > R > U (U-bocht).
// ============================================================
char bepaalKruispuntRichtingLinkseHand() {
  if (kruispuntLinksOptie)     return 'L';
  if (kruispuntRechtdoorOptie) return 'S';
  if (kruispuntRechtsOptie)    return 'R';
  return 'U';  // Geen uitweg gevonden → U-bocht
}

// ============================================================
// behandelKruispunt()
// Beheert het drie-fasen kruispunt-protocol:
//   1. KF_VOORUIT_CENTEREN – rij vooruit om te centreren
//   2. KF_DRAAIEN          – draai naar gekozen richting
//   3. KF_WACHT_OP_LIJN    – wacht tot een sensor de lijn vindt
// In verkenstand: gebruik linkse-handregel en sla op.
// In herhaalstand: lees de opgeslagen beslissing.
// ============================================================
void behandelKruispunt() {
  switch (kruispuntFase) {

    case KF_VOORUIT_CENTEREN: {
      motorControl(BASISSNELHEID, BASISSNELHEID);
      if (millis() - kruispuntStartTijd >= KRUISPUNT_CENTERING_MS) {
        // Lees sensoren voor beschikbare richtingen
        readSensors();
        kruispuntLinksOptie     = sensorOpLijn[0] || sensorOpLijn[1];
        kruispuntRechtdoorOptie = sensorOpLijn[2];
        kruispuntRechtsOptie    = sensorOpLijn[3] || sensorOpLijn[4];

        // Kies richting: herhaalstand of verkenstand (linkse-handregel)
        if (herhaalmodus && huidigLabyrinthIndex < aantalLabyrinthBeslissingen) {
          gekozenKruispuntRichting = labyrinthPad[huidigLabyrinthIndex++];
          Serial.print("Kruispunt (herhaal) – richting: ");
        } else {
          gekozenKruispuntRichting = bepaalKruispuntRichtingLinkseHand();
          Serial.print("Kruispunt (verken) – richting: ");
        }
        Serial.println(gekozenKruispuntRichting);

        // Sla beslissing op in verkenstand
        if (!herhaalmodus && aantalLabyrinthBeslissingen < MAX_LABYRINTH_BESLISSINGEN) {
          labyrinthPad[aantalLabyrinthBeslissingen++] = gekozenKruispuntRichting;
        }

        kruispuntFase      = KF_DRAAIEN;
        kruispuntStartTijd = millis();
      }
      break;
    }

    case KF_DRAAIEN: {
      bool draaiKlaar = false;

      if (gekozenKruispuntRichting == 'L') {
        motorControl(-DRAAI_SNELHEID, DRAAI_SNELHEID);
        draaiKlaar = (millis() - kruispuntStartTijd >= KRUISPUNT_DRAAI_MS);
      } else if (gekozenKruispuntRichting == 'R') {
        motorControl(DRAAI_SNELHEID, -DRAAI_SNELHEID);
        draaiKlaar = (millis() - kruispuntStartTijd >= KRUISPUNT_DRAAI_MS);
      } else if (gekozenKruispuntRichting == 'U') {
        // U-bocht: gebruik gemiddelde van beide encoders voor nauwkeurige 180°
        motorControl(-DRAAI_SNELHEID, DRAAI_SNELHEID);
        long deltaLinks  = encoderLinks  - encoderLinksOpKruispunt;
        long deltaRechts = encoderRechts - encoderRechtsOpKruispunt;
        long gemGedraaid = (deltaLinks + deltaRechts) / 2;
        draaiKlaar = (gemGedraaid >= TICKS_VOOR_180_GRADEN) ||
                     (millis() - kruispuntStartTijd >= KRUISPUNT_DRAAI_MS * 2);
      } else {
        // Rechtdoor ('S'): kort doorrijden
        motorControl(BASISSNELHEID, BASISSNELHEID);
        draaiKlaar = (millis() - kruispuntStartTijd >= 150);
      }

      if (draaiKlaar) {
        kruispuntFase = KF_WACHT_OP_LIJN;
      }
      break;
    }

    case KF_WACHT_OP_LIJN: {
      if (!alleSensorsWit()) {
        integraal  = 0.0;
        vorigeFout = 0.0;
        toestand   = LIJN_VOLGEN;
      } else {
        // Blijf iets draaien/rijden om de lijn te vinden
        if (gekozenKruispuntRichting == 'L' || gekozenKruispuntRichting == 'U') {
          motorControl(-DRAAI_SNELHEID, DRAAI_SNELHEID);
        } else if (gekozenKruispuntRichting == 'R') {
          motorControl(DRAAI_SNELHEID, -DRAAI_SNELHEID);
        } else {
          motorControl(BASISSNELHEID, BASISSNELHEID);
        }
      }
      break;
    }
  }
}

// ============================================================
// behandelDoodlopendEinde()
// Voert een encoder-gestuurde 180°-draai uit (blocking, éénmalig),
// slaat een 'U' op in het labyrinthpad en gaat daarna in de
// TERUGRIJDEN-toestand om terug naar het vorige kruispunt te rijden.
// ============================================================
void behandelDoodlopendEinde() {
  Serial.println("Doodlopend einde – 180° draai uitvoeren...");

  // Sla U-bocht op in labyrinthpad (verkenstand)
  if (!herhaalmodus && aantalLabyrinthBeslissingen < MAX_LABYRINTH_BESLISSINGEN) {
    labyrinthPad[aantalLabyrinthBeslissingen++] = 'U';
  }

  // 180°-draai met encoder feedback – gemiddelde van beide wielen voor nauwkeurigheid
  long startLinks  = encoderLinks;
  long startRechts = encoderRechts;
  motorControl(-DRAAI_SNELHEID, DRAAI_SNELHEID);
  unsigned long startTijd = millis();

  while (true) {
    long deltaLinks  = encoderLinks  - startLinks;
    long deltaRechts = encoderRechts - startRechts;
    long gemGedraaid = (deltaLinks + deltaRechts) / 2;
    if (gemGedraaid >= TICKS_VOOR_180_GRADEN) break;
    if (millis() - startTijd > 3000) break;  // Veiligheidstime-out 3 s
    delay(2);
  }

  motorStop();
  delay(150);

  // Reset PID en zoekteller voor terugrijden
  integraal             = 0.0;
  vorigeFout            = 0.0;
  zoekTeller            = 0;
  lijnKwijtTimerGestart = false;
  kruispuntVerwerkt     = false;

  Serial.println("180° draai voltooid – terugrijden naar vorig kruispunt...");
  toestand = TERUGRIJDEN;
}

// ============================================================
// behandelTerugrijden()
// Volgt de lijn terug naar het vorige kruispunt.
// Bij detectie van het kruispunt (S1+S5 beide LOW) wordt
// opnieuw de KRUISPUNT-toestand ingegaan (nu met linkse-handregel
// vanuit de nieuwe rijrichting).
// ============================================================
void behandelTerugrijden() {
  // Vorig kruispunt bereikt: buitenste sensoren beide op lijn
  if (isKruispuntGedetecteerd()) {
    Serial.println("Vorig kruispunt bereikt – alternatieve route bepalen.");
    encoderLinksOpKruispunt  = encoderLinks;
    encoderRechtsOpKruispunt = encoderRechts;
    kruispuntFase      = KF_VOORUIT_CENTEREN;
    kruispuntStartTijd = millis();
    kruispuntVerwerkt  = false;
    toestand           = KRUISPUNT;
    return;
  }

  // Lijn kwijt tijdens terugrijden: draai naar laatste bekende richting
  if (alleSensorsWit()) {
    zoekTeller++;
    if (zoekTeller > MAX_ZOEK_ITERATIES) {
      Serial.println("Terugrijden: lijn niet gevonden – robot stopt.");
      motorStop();
      toestand = FINISH_BEREIKT;
      return;
    }
    if (laatsteBekendeRichting <= 0) {
      motorControl(-DRAAI_SNELHEID, DRAAI_SNELHEID);
    } else {
      motorControl(DRAAI_SNELHEID, -DRAAI_SNELHEID);
    }
    return;
  }

  // Normaal lijnvolgen (terug naar vorig kruispunt)
  zoekTeller = 0;
  float positie   = berekenLijnPositie();
  float correctie = calculatePID(positie);

  if (positie < -0.5)      laatsteBekendeRichting = -1;
  else if (positie > 0.5) laatsteBekendeRichting =  1;
  else                     laatsteBekendeRichting =  0;

  snelheidLinks  = BASISSNELHEID + (int)correctie;
  snelheidRechts = BASISSNELHEID - (int)correctie;
  motorControl(snelheidLinks, snelheidRechts);
}

// ============================================================
// slaOpLabyrinthPad()
// Sla het gevolgde labyrinthpad op in de NVS (Preferences).
// ============================================================
void slaOpLabyrinthPad() {
  if (aantalLabyrinthBeslissingen == 0) return;
  voorkeuringen.begin(NVS_NAMESPACE, false);
  size_t geschrevenBytes = voorkeuringen.putBytes(NVS_SLEUTEL_PAD, labyrinthPad, aantalLabyrinthBeslissingen);
  voorkeuringen.putInt(NVS_SLEUTEL_AANTAL, aantalLabyrinthBeslissingen);
  voorkeuringen.end();
  if (geschrevenBytes != (size_t)aantalLabyrinthBeslissingen) {
    Serial.println("FOUT: labyrinthpad kon niet volledig worden opgeslagen in NVS!");
  } else {
    Serial.print("Labyrinthpad opgeslagen (");
    Serial.print(aantalLabyrinthBeslissingen);
    Serial.println(" beslissingen).");
  }
}

// ============================================================
// laadLabyrinthPad()
// Laad het opgeslagen labyrinthpad uit de NVS.
// Geeft true als een geldig pad gevonden is (herhaalstand).
// ============================================================
bool laadLabyrinthPad() {
  voorkeuringen.begin(NVS_NAMESPACE, true);
  int aantal = voorkeuringen.getInt(NVS_SLEUTEL_AANTAL, 0);
  if (aantal <= 0 || aantal > MAX_LABYRINTH_BESLISSINGEN) {
    voorkeuringen.end();
    return false;
  }
  aantalLabyrinthBeslissingen = aantal;
  huidigLabyrinthIndex        = 0;
  voorkeuringen.getBytes(NVS_SLEUTEL_PAD, labyrinthPad, aantal);
  voorkeuringen.end();
  return true;
}

// ============================================================
// wisLabyrinthPad()
// Wist het opgeslagen labyrinthpad uit de NVS.
// ============================================================
void wisLabyrinthPad() {
  voorkeuringen.begin(NVS_NAMESPACE, false);
  voorkeuringen.clear();
  voorkeuringen.end();
  aantalLabyrinthBeslissingen = 0;
  huidigLabyrinthIndex        = 0;
  herhaalmodus                = false;
  Serial.println("Labyrinthpad gewist – volgende run in verkenstand.");
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
  Serial.println("Obstakelontwijking: zoeken naar lijn...");
  unsigned long startTijd = millis();
  motorControl(BASISSNELHEID, BASISSNELHEID);
  while (alleSensorsWit()) {
    readSensors();
    if (millis() - startTijd > LIJN_ZOEK_TIMEOUT_MS) break;
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
