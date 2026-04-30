#include <Preferences.h>
#include <BluetoothSerial.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is niet ingeschakeld! Update je board instellingen.
#endif

BluetoothSerial SerialBT;

// ============================================================
// Pin-definities – IR-sensoren (LOW = lijn/zwart)
// ============================================================
#define S1_PIN 25  // Meest links (Far Left)
#define S2_PIN 33   // Links (Near Left)
#define S3_PIN 32   // Midden (Center) - iets naar voor
#define S4_PIN 35   // Rechts (Near Right)
#define S5_PIN 34   // Meest rechts (Far Right)

// ------------------------------------------------------------
// Pin-definities – Knop (start / later mode)
// ------------------------------------------------------------
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
#define TRIG_PIN 16
#define ECHO_PIN 17

// ------------------------------------------------------------
// Pin-definities – Encoders
// ------------------------------------------------------------
#define ENC_L_A 23
#define ENC_L_B 22
#define ENC_R_A 21
#define ENC_R_B 19

// ------------------------------------------------------------
// PWM-instellingen (ESP32 LEDC)
// ------------------------------------------------------------
#define PWM_FREQ       1000   // Hz
#define PWM_RESOLUTION    8   // bits (0-255)

// ------------------------------------------------------------
// PID-parameters – pas Kp, Ki en Kd aan voor je robot
// ------------------------------------------------------------
float Kp = 12.0;
float Ki =  0.0;
float Kd = 25.0;

// ------------------------------------------------------------
// Rijsnelheid-instellingen (0-255)
// ------------------------------------------------------------
const int BASISSNELHEID  = 100;  // Verlaagd (was 120). Geeft de wielen meer tijd om grip te houden.
const int MAX_SNELHEID   = 170; // Verlaagd (was 200). Voorkomt uitschieters in de bocht.
const int MIN_SNELHEID   = 80; 
const int DRAAI_SNELHEID = 140;
const int OBSTAKEL_AFSTAND   = 15;  // cm – trigger obstakelontwijking

// ------------------------------------------------------------
// Doodlopend einde / labyrinth-constanten
// ------------------------------------------------------------
const unsigned long LIJN_KWIJT_DOODLOPEND_MS = 2500;  // ms lijn kwijt → doodlopend einde
const long   TICKS_VOOR_180_GRADEN           = 377;   // Encoderticks voor 180° (kalibreren!)
const int    MAX_LABYRINTH_BESLISSINGEN       = 50;    // Max. opgeslagen kruispuntbeslissingen

// ------------------------------------------------------------
// Kruispunt-tijden (ms) – kalibreren per robot
// ------------------------------------------------------------
const unsigned long KRUISPUNT_CENTERING_MS = 80;   // Verhoogd van 30 naar 80ms voor beter centreren
const unsigned long KRUISPUNT_DRAAI_MS     = 200;   // Verlaagd van 400 naar 250!

// ------------------------------------------------------------
// Sensorposities in cm (offset t.o.v. midden)
// ------------------------------------------------------------
const float SENSOR_POS[5] = { -6.5, -2.0, 0.0, 2.0, 6.5 };

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

// ------------------------------------------------------------
// Globale variabelen – Ultrasoon Rate Limiting
// ------------------------------------------------------------
unsigned long laatstePingTijd = 0;
const unsigned long PING_INTERVAL = 60; // HC-SR04 heeft ~60ms nodig tussen pings
float laatsteAfstand = 999.0;

// ------------------------------------------------------------
// Globale variabelen – zigzag bypass
// ------------------------------------------------------------
int  zigzagTeller       = 0;   // Telt snelle buitensensor-uitvallers
bool zigzagModusActief  = false;
int  zigzagStabielTeller = 0;  // Telt stabiele cycli om zigzag te verlaten
const int ZIGZAG_DREMPEL  = 3;  // Aantal uitvallers om bypass te activeren
const int ZIGZAG_STABIEL  = 10; // Stabiele cycli vereist om bypass te deactiveren

// ------------------------------------------------------------
// Globale variabelen – geforceerde kruispuntrichting (Y-splitsing terugrijden)
// ------------------------------------------------------------
bool geforceerdeRichting      = false;  // true = negeer linkse-handregel
char geforceerdeRichtingWaarde = '\0';  // Te gebruiken richting

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
const int MAX_ZOEK_ITERATIES = 250;  // ~2.5 s bij 10 ms lus

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
  SerialBT.begin("LineFollower_Onno"); 
  Serial.println("Bluetooth gestart! Koppel met je apparaat.");

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
    SerialBT.println("Opgeslagen labyrinthpad gevonden – herhaalstand actief.");
    SerialBT.print("Opgeslagen beslissingen: ");
    SerialBT.println(aantalLabyrinthBeslissingen);
  } else {
    SerialBT.println("Geen opgeslagen pad – verkenstand actief (linkse-handregel).");
  }

  SerialBT.println("Robot gereed. Druk kort om te starten, houd 3 s in om pad te wissen.");
}

// ============================================================
// HOOFD-LUS
// ============================================================
void loop() {
  unsigned long nu = millis();
  if (nu - vorigeTijd < LUS_INTERVAL) return;
  vorigeTijd = nu;

  readSensors();

  // --- 2. UNIVERSELE KNOP LOGICA (Toggle Pauze/Run & Reset) ---
  static unsigned long knopIngedruktTijd = 0;
  static bool knopWasIngedrukt = false;
  static bool knopActieUitgevoerd = false;

  bool knopIsIngedrukt = (digitalRead(BTN_PIN) == LOW);

  if (knopIsIngedrukt && !knopWasIngedrukt) {
    knopIngedruktTijd = millis();       // Start timer
    knopActieUitgevoerd = false;        // Reset status
  }

  // Check voor lang indrukken (flash reset)
  if (knopIsIngedrukt && !knopActieUitgevoerd) {
    if (millis() - knopIngedruktTijd > 3000) {
      wisLabyrinthPad();
      SerialBT.println("GEHEUGEN GEWIST! Laat knop los.");
      knopActieUitgevoerd = true;
      toestand = WACHT_OP_START;
      motorStop();
    }
  }

  // Check voor korte druk (loslaten vóór 3 sec)
  if (!knopIsIngedrukt && knopWasIngedrukt) {
    if (!knopActieUitgevoerd && (millis() - knopIngedruktTijd > 50)) { // 50ms debounce
      if (toestand == WACHT_OP_START) {
        SerialBT.println("Robot start/hervat!");
        toestand = LIJN_VOLGEN;
      } else {
        SerialBT.println("Robot PAUZE!");
        motorStop();
        toestand = WACHT_OP_START;
      }
    }
  }
  knopWasIngedrukt = knopIsIngedrukt;

  // 3. Obstakel Priority Override (Buiten de toestand-machine!)
  if (nu - laatstePingTijd >= PING_INTERVAL) {
    laatsteAfstand = meetAfstand();
    laatstePingTijd = nu;

    // NIEUW: Print de afstand als deze onder de 40 cm duikt (max 2 keer per seconde tegen BT spam)
    static unsigned long laatsteBTPrint = 0;
    if (laatsteAfstand < 40.0 && (nu - laatsteBTPrint > 500)) {
      SerialBT.print("Object gespot op: ");
      SerialBT.print(laatsteAfstand);
      SerialBT.println(" cm");
      laatsteBTPrint = nu;
    }
  }

  // De eigenlijke noodstop (triggert pas onder OBSTAKEL_AFSTAND, wat nu op 20cm staat)
  if (laatsteAfstand > 0.1 && laatsteAfstand < OBSTAKEL_AFSTAND && 
      toestand == LIJN_VOLGEN) { // Alleen triggeren TIJDENS het rijden
    
    SerialBT.print("OBSTAKEL BEVESTIGD op ");
    SerialBT.print(laatsteAfstand);
    SerialBT.println(" cm. Start ontwijking...");
    
    toestand = OBSTAKEL_ONTWIJKEN;
  }

  // 4. Toestandsmachine uitvoeren
  switch (toestand) {
    case WACHT_OP_START:      motorStop(); break;
    case LIJN_VOLGEN:         behandelLijnVolgen(); break;
    case LIJN_ZOEKEN:         behandelLijnZoeken(); break;
    case KRUISPUNT:           behandelKruispunt(); break;
    case DOODLOPEND_EINDE:    behandelDoodlopendEinde(); break;
    case TERUGRIJDEN:         behandelTerugrijden(); break;
    case OBSTAKEL_ONTWIJKEN:  avoidObstacle(); break;
    case FINISH_BEREIKT:      motorStop(); break;
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


// ============================================================
// behandelLijnVolgen()
// Normaal rijgedrag: PID + zigzag-detectie + kruispunt- en
// obstakel- en finishcontrole.
// ============================================================
void behandelLijnVolgen() {
  // --- Finish: alle sensoren zwart met ingebouwde vertraging ---
  // static variabele behoudt zijn waarde tussen loop-cycli
  static int finishTeller = 0; 
  
  if (alleSensorsZwart()) {
    finishTeller++;
    if (finishTeller >= 15) { // 15 x 10ms lus = 150ms continu zwart
      slaOpLabyrinthPad();
      SerialBT.println("FINISH gedetecteerd – pad opgeslagen, robot stopt!");
      motorStop();
      toestand = FINISH_BEREIKT;
      return;
    }
  } else {
    // Reset de teller direct als ook maar één sensor weer wit ziet
    finishTeller = 0; 
  }

  // --- Obstakelcontrole ---
  float afstand = meetAfstand();
  if (afstand < OBSTAKEL_AFSTAND) {
    SerialBT.print("Obstakel gedetecteerd op ");
    SerialBT.print(afstand);
    SerialBT.println(" cm – ontwijken...");
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
    SerialBT.println("Kruispunt gedetecteerd!");
    encoderLinksOpKruispunt  = encoderLinks;
    encoderRechtsOpKruispunt = encoderRechts;

    kruispuntLinksOptie     = false;
    kruispuntRechtdoorOptie = false;
    kruispuntRechtsOptie    = false;

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

  // --- Zigzag-detectie en bypass (rechtdoor rijden) ---
  // Triggers: S3 op lijn EN een buitensensor valt zojuist af (snelle slingers).
  // Zodra ZIGZAG_DREMPEL aflossingen zijn geteld: rij rechtdoor tot lijn stabiliseert.
  bool s1Uitgevallen = vorigeSensorOpLijn[0] && !sensorOpLijn[0];
  bool s5Uitgevallen = vorigeSensorOpLijn[4] && !sensorOpLijn[4];

  if (sensorOpLijn[2] && (s1Uitgevallen || s5Uitgevallen)) {
    // Snelle uitvaller gedetecteerd
    zigzagTeller++;
    zigzagStabielTeller = 0;
    if (zigzagTeller >= ZIGZAG_DREMPEL) {
      zigzagModusActief = true; //DEZE TERUG OP TRUE ZETTEN!!!!
      zigzagTeller      = 0;
      SerialBT.println("Zigzag gedetecteerd – rechtdoor rijden.");
    }
  } else if (zigzagModusActief && sensorOpLijn[2] && !s1Uitgevallen && !s5Uitgevallen) {
    // Lijn stabiel onder S3, tel stabiele cycli
    zigzagStabielTeller++;
    if (zigzagStabielTeller >= ZIGZAG_STABIEL) {
      zigzagModusActief   = false;
      zigzagTeller        = 0;
      zigzagStabielTeller = 0;
      SerialBT.println("Zigzag voorbij – PID hervat.");
    }
  } else if (!zigzagModusActief) {
    // Buiten zigzag-modus: wis teller bij elke stabiele cyclus
    zigzagTeller = 0;
  }

  if (zigzagModusActief) {
    // Negeer PID: rij rechtdoor om door de zigzag te 'snijden'
    motorControl(BASISSNELHEID, BASISSNELHEID);
    return;
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
    SerialBT.println("Lijn > 1 s kwijt – doodlopend einde gedetecteerd!");
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
    SerialBT.println("Lijn niet gevonden na zoeken – robot stopt.");
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

      // SCAN-FIX: Verzamel vertakkingen *tijdens* het centreren! 
      // Zodra een sensor in deze 100ms iets ziet, wordt de optie 'true' gelockt.
      if (sensorOpLijn[0] || sensorOpLijn[1]) kruispuntLinksOptie = true;
      if (sensorOpLijn[2])                    kruispuntRechtdoorOptie = true;
      if (sensorOpLijn[3] || sensorOpLijn[4]) kruispuntRechtsOptie = true;

      if (millis() - kruispuntStartTijd >= KRUISPUNT_CENTERING_MS) {
        
        if (geforceerdeRichting) {
          gekozenKruispuntRichting = geforceerdeRichtingWaarde;
          geforceerdeRichting      = false;
          SerialBT.print("Kruispunt (geforceerd) – richting: ");
        } else if (herhaalmodus && huidigLabyrinthIndex < aantalLabyrinthBeslissingen) {
          gekozenKruispuntRichting = labyrinthPad[huidigLabyrinthIndex++];
          SerialBT.print("Kruispunt (herhaal) – richting: ");
        } else {
          gekozenKruispuntRichting = bepaalKruispuntRichtingLinkseHand();
          SerialBT.print("Kruispunt (verken) – richting: ");
        }
        SerialBT.println(gekozenKruispuntRichting);

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
      bool lijnGevonden = false;

      // Stop zodra een van de drie middelste sensoren (S2, S3 of S4) de lijn ziet,
      // zodat de robot niet over de lijn heen draait bij encoder-gebaseerde aansturing.
      if (gekozenKruispuntRichting == 'S') {
        lijnGevonden = !alleSensorsWit();
      } else {
        lijnGevonden = sensorOpLijn[1] || sensorOpLijn[2] || sensorOpLijn[3];
      }

      if (lijnGevonden) {
        motorStop();  // Stop onmiddellijk om voorbijrijden door momentum te voorkomen
        integraal  = 0.0;
        vorigeFout = 0.0;
        toestand   = LIJN_VOLGEN;
      } else {
        // Blijf draaien tot een middelste sensor de lijn vindt
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
  SerialBT.println("Doodlopend einde – 180° draai uitvoeren...");

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

  SerialBT.println("180° draai voltooid – terugrijden naar vorig kruispunt...");
  toestand = TERUGRIJDEN;
}

// ============================================================
// behandelTerugrijden()
// Volgt de lijn terug naar het vorige kruispunt.
// Bij detectie van het kruispunt wordt de geforceerde richting
// bepaald op basis van de laatste beslissing in labyrinthPad,
// zodat de robot de onverkende tak neemt en niet terugrijdt
// op de originele hoofdlijn.
// ============================================================
void behandelTerugrijden() {
  // --- SUPER-GEVOELIGE KRUISPUNT-DETECTIE VOOR Y-MERGE ---
  bool kruispuntGezien = false;
  if ((sensorOpLijn[0] || sensorOpLijn[4]) && (sensorOpLijn[1] || sensorOpLijn[2] || sensorOpLijn[3])) {
    kruispuntGezien = true;
  }

  if (kruispuntGezien) {
    Serial.println("Vorig kruispunt bereikt (Y-Merge) – alternatieve route bepalen.");
    SerialBT.println("Y-Merge / Kruispunt bereikt op terugweg!");

    if (!herhaalmodus && aantalLabyrinthBeslissingen >= 2 &&
        labyrinthPad[aantalLabyrinthBeslissingen - 1] == 'U') {
      char vorigeRichting = labyrinthPad[aantalLabyrinthBeslissingen - 2];
      aantalLabyrinthBeslissingen -= 2; // Wis de foute keuze
      
      // GEOMETRIE FIX: Draai L en R om op de terugweg
      if      (vorigeRichting == 'L') geforceerdeRichtingWaarde = 'L'; 
      else if (vorigeRichting == 'R') geforceerdeRichtingWaarde = 'R'; 
      else                            geforceerdeRichtingWaarde = 'U';
      
      geforceerdeRichting = true;
      SerialBT.print("Verplichte correctie-draai: ");
      SerialBT.println(geforceerdeRichtingWaarde);
    }

    encoderLinksOpKruispunt  = encoderLinks;
    encoderRechtsOpKruispunt = encoderRechts;

    kruispuntLinksOptie     = false;
    kruispuntRechtdoorOptie = false;
    kruispuntRechtsOptie    = false;
    
    kruispuntFase      = KF_VOORUIT_CENTEREN;
    kruispuntStartTijd = millis();
    kruispuntVerwerkt  = false;
    toestand           = KRUISPUNT;
    return;
  }

  // --- Normaal Lijnzoeken/Volgen tijdens het terugrijden ---

  // Reset kruispunt-debounce zodra beide buitenste sensoren (S1, S5) de lijn niet meer zien,
  // zodat het volgende kruispunt correct gedetecteerd kan worden.
  if (!sensorOpLijn[0] && !sensorOpLijn[4]) {
    kruispuntVerwerkt = false;
  }

  if (alleSensorsWit()) {
    zoekTeller++;
    if (zoekTeller > MAX_ZOEK_ITERATIES) {
      Serial.println("Gestopt: Lijn helemaal kwijt tijdens terugrijden.");
      SerialBT.println("Gestopt: Lijn helemaal kwijt tijdens terugrijden.");
      motorStop();
      toestand = FINISH_BEREIKT;
      return;
    }
    if (laatsteBekendeRichting <= 0) motorControl(-DRAAI_SNELHEID, DRAAI_SNELHEID);
    else                             motorControl(DRAAI_SNELHEID, -DRAAI_SNELHEID);
    return;
  }

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
    SerialBT.println("FOUT: labyrinthpad kon niet volledig worden opgeslagen in NVS!");
  } else {
    SerialBT.print("Labyrinthpad opgeslagen (");
    SerialBT.print(aantalLabyrinthBeslissingen);
    SerialBT.println(" beslissingen).");
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
  SerialBT.println("Labyrinthpad gewist – volgende run in verkenstand.");
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
const int DRAAI90 = 500; // VOORBEELD: pas dit aan naar jouw gevonden BT-waarde

// --- Kalibratiewaarden voor 32mm wielen ---
const float TICKS_PER_CM = 11.94; 

// --- Ontwijkings-instellingen voor 20cm Cilinder (in Ticks) ---
// We nemen een ruime bocht om de 20cm cilinder niet te raken
const int ONTWIJK_AFSTAND_SCHUIN = (int)(18 * TICKS_PER_CM); // 18 cm schuin weg
const int ONTWIJK_AFSTAND_RECHT  = (int)(28 * TICKS_PER_CM); // 28 cm langs het object
const int ONTWIJK_DRAAI_HOEK     = (int)(TICKS_VOOR_180_GRADEN * 0.33); // Ca. 60 graden draai

// Time-out voor het terugzoeken naar de lijn na obstakelontwijking
const int LIJN_ZOEK_TIMEOUT_MS     = 3000;

void avoidObstacle() {
  SerialBT.println("Obstakelontwijking: Start op basis van ticks");
  
  // We gebruiken deze variabelen om de afgelegde afstand per stap te meten
  long startL, startR;

  auto resetEncd = [&]() { 
    startL = encoderLinks; 
    startR = encoderRechts; 
  };

  auto wachtOpTicks = [&](int doelTicks) {
    while (((encoderLinks - startL) + (encoderRechts - startR)) / 2 < doelTicks) {
      // yield() geeft de CPU aan andere taken (zoals encoder-interrupts) zonder vaste wachttijd.
      yield();
    }
  };

  // 1. Stapje achteruit (~5 cm) op basis van encoder-ticks
  resetEncd();
  motorControl(-DRAAI_SNELHEID, -DRAAI_SNELHEID);
  wachtOpTicks((int)(5 * TICKS_PER_CM));

  // 2. Draai schuin weg
  resetEncd();
  motorControl(-DRAAI_SNELHEID, DRAAI_SNELHEID);
  wachtOpTicks(ONTWIJK_DRAAI_HOEK);

  // 3. Rij schuin langs de cilinder
  resetEncd();
  motorControl(BASISSNELHEID, BASISSNELHEID);
  wachtOpTicks(ONTWIJK_AFSTAND_SCHUIN);

  // 4. Draai parallel aan de lijn
  resetEncd();
  motorControl(DRAAI_SNELHEID, -DRAAI_SNELHEID);
  wachtOpTicks(ONTWIJK_DRAAI_HOEK);

  // 5. Rij langs het object
  resetEncd();
  motorControl(BASISSNELHEID, BASISSNELHEID);
  wachtOpTicks(ONTWIJK_AFSTAND_RECHT);

  // 6. Draai schuin naar de lijn toe
  resetEncd();
  motorControl(DRAAI_SNELHEID, -DRAAI_SNELHEID);
  wachtOpTicks(ONTWIJK_DRAAI_HOEK);

  // 7. Rij naar de lijn toe (tot sensoren zwart zien)
  motorControl(BASISSNELHEID, BASISSNELHEID);
  unsigned long startZoek = millis();
  while (alleSensorsWit()) {
    readSensors();
    if (millis() - startZoek > 5000) break; // Veiligheidstime-out
  }

  // 8. Korte correctie om weer recht op de lijn te komen
  resetEncd();
  motorControl(-DRAAI_SNELHEID, DRAAI_SNELHEID);
  wachtOpTicks(ONTWIJK_DRAAI_HOEK / 2);

  // Hervat lijnvolgen
  integraal = 0.0;
  vorigeFout = 0.0;
  toestand = LIJN_VOLGEN;
  SerialBT.println("Lijn teruggevonden.");
}
