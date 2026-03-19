/*
 * Competitive Line-Following Robot
 * Platform : ESP32 DevKit + L298N motor driver + 5-sensor IR array
 *
 * ─── HOW IT WORKS ────────────────────────────────────────────────────────────
 *
 * PHASE 1 – EXPLORATION (default on power-up):
 *   • PID controller keeps the robot on the line on straight sections and
 *     curves.
 *   • When the sensor array detects a junction (intersection, T-crossing,
 *     triangle, dead end) the robot switches to mapping mode and uses the
 *     LEFT-HAND FOLLOW rule to decide which branch to take.
 *   • Every junction decision (L / R / S / B) is appended to a raw path
 *     string.
 *   • When the finish is reached the raw path is simplified (dead-end
 *     detours are collapsed) and the optimal path is saved to ESP32 NVS
 *     (non-volatile storage) so it survives a power cycle.
 *
 * PHASE 2 – RACE (hold the boot button while powering on):
 *   • Same fast PID controller on straight sections and curves.
 *   • At each junction the robot reads the next character from the stored
 *     optimal path and executes that turn directly – no left-hand logic
 *     needed.
 *
 * ─── HARDWARE WIRING ─────────────────────────────────────────────────────────
 *
 *  IR line sensors (active LOW = line detected):
 *    S1 (leftmost)  → GPIO 34
 *    S2             → GPIO 35
 *    S3 (center)    → GPIO 32
 *    S4             → GPIO 33
 *    S5 (rightmost) → GPIO 25
 *
 *  L298N motor driver:
 *    Left  motor PWM → GPIO 26   (LEDC channel 0)
 *    Left  FWD       → GPIO 27
 *    Left  BWD       → GPIO 14
 *    Right motor PWM → GPIO 18   (LEDC channel 1)
 *    Right FWD       → GPIO 19
 *    Right BWD       → GPIO 21
 *
 *  Mode selection:
 *    Boot button     → GPIO 0  (hold LOW at power-up for RACE mode)
 *
 * ─── PATH SIMPLIFICATION ─────────────────────────────────────────────────────
 *
 *  A dead-end detour adds the pattern  X + B + Y  to the path.
 *  This is equivalent to a single turn  Z  where:
 *      Z = (turn_degrees(X) + 180 + turn_degrees(Y))  mod 360
 *  with L = -90°, S = 0°, R = +90°, B = +180°.
 *
 *  The optimiser repeatedly replaces every such triplet until no B remains
 *  in a non-terminal position.
 */

#include <Preferences.h>

// ─── Pin definitions ──────────────────────────────────────────────────────────
#define S1_PIN  34   // Line sensor 1 – leftmost
#define S2_PIN  35   // Line sensor 2
#define S3_PIN  32   // Line sensor 3 – center
#define S4_PIN  33   // Line sensor 4
#define S5_PIN  25   // Line sensor 5 – rightmost

// Sensor output is LOW when the dark line is detected (common IR module logic).
#define LINE_DETECTED  LOW

#define MOTOR_L_PWM  26   // Left  motor PWM (speed)
#define MOTOR_L_FWD  27   // Left  motor forward direction
#define MOTOR_L_BWD  14   // Left  motor backward direction
#define MOTOR_R_PWM  18   // Right motor PWM (speed)
#define MOTOR_R_FWD  19   // Right motor forward direction
#define MOTOR_R_BWD  21   // Right motor backward direction

#define MODE_BTN_PIN  0   // Boot button – active LOW – hold for RACE mode

// ─── LEDC (ESP32 PWM) ─────────────────────────────────────────────────────────
#define LEDC_CH_L   0     // LEDC channel for left  motor
#define LEDC_CH_R   1     // LEDC channel for right motor
#define LEDC_FREQ   5000  // 5 kHz carrier frequency
#define LEDC_RES    8     // 8-bit resolution → values 0-255

// ─── PID tuning ───────────────────────────────────────────────────────────────
#define BASE_SPEED   120  // Exploration-phase forward speed  (0-255)
#define RACE_SPEED   180  // Race-phase   forward speed   (0-255)
#define MAX_SPEED    220  // Hard upper limit for any motor
#define KP  30.0f         // Proportional gain
#define KI   0.05f        // Integral gain
#define KD  20.0f         // Derivative gain

// ─── Timing (milliseconds) ────────────────────────────────────────────────────
#define JUNC_CROSS_MS   250  // Creep through junction centre before reading arms
#define TURN_90_MS      380  // Duration of a 90° spin turn
#define TURN_180_MS     700  // Duration of a 180° U-turn
#define ALIGN_TIMEOUT   800  // Maximum time to search for line after a turn
#define JUNCTION_COOLDOWN_MS 400  // Minimum gap between two junction events

// ─── Path limits ──────────────────────────────────────────────────────────────
#define MAX_JUNCTIONS  80

// ─── Phase ────────────────────────────────────────────────────────────────────
enum Phase { PHASE_EXPLORE, PHASE_RACE };

// ═══════════════════════════════════════════════════════════════════════════════
// GLOBAL STATE
// ═══════════════════════════════════════════════════════════════════════════════

static Phase phase;

// PID state
static float pidIntegral  = 0.0f;
static float pidLastError = 0.0f;

// Path recording (exploration)
static char  rawPath[MAX_JUNCTIONS + 1];  // one character per junction: L R S B
static int   rawLen = 0;

// Optimised path (loaded in RACE mode from NVS, or built at end of EXPLORE)
static char  optPath[MAX_JUNCTIONS + 1];
static int   optLen  = 0;
static int   raceIdx = 0;  // index into optPath during race

// Junction debounce
static unsigned long lastJunctionMs = 0;

// NVS
static Preferences prefs;

// ═══════════════════════════════════════════════════════════════════════════════
// FORWARD DECLARATIONS
// ═══════════════════════════════════════════════════════════════════════════════

struct SensorData { bool on[5]; int count; float pos; };
struct JuncInfo   { bool hasLeft; bool hasStraight; bool hasRight; };

static SensorData readSensors();
static bool       isAtJunction(const SensorData& sd);
static void       crossJunctionCentre();
static JuncInfo   sampleJunctionArms();
static void       pidStep(const SensorData& sd, int baseSpd);
static void       resetPid();
static void       setMotors(int left, int right);
static void       stopMotors();
static void       turnLeft();
static void       turnRight();
static void       turnAround();
static void       alignToLine();
static void       executeDir(char dir);
static char       leftHandChoice(const JuncInfo& ji);
static char       combineDirs(char x, char y);
static void       buildOptimalPath();
static void       saveOptimalPath();
static void       loadOptimalPath();
static void       clearStoredPath();

// ═══════════════════════════════════════════════════════════════════════════════
// SETUP & LOOP
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);

  // Sensor inputs
  pinMode(S1_PIN, INPUT);
  pinMode(S2_PIN, INPUT);
  pinMode(S3_PIN, INPUT);
  pinMode(S4_PIN, INPUT);
  pinMode(S5_PIN, INPUT);

  // Motor direction pins
  pinMode(MOTOR_L_FWD, OUTPUT);
  pinMode(MOTOR_L_BWD, OUTPUT);
  pinMode(MOTOR_R_FWD, OUTPUT);
  pinMode(MOTOR_R_BWD, OUTPUT);

  // Motor PWM via ESP32 LEDC peripheral
  ledcSetup(LEDC_CH_L, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(MOTOR_L_PWM, LEDC_CH_L);
  ledcSetup(LEDC_CH_R, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(MOTOR_R_PWM, LEDC_CH_R);

  // Mode selection button
  pinMode(MODE_BTN_PIN, INPUT_PULLUP);

  stopMotors();
  delay(100);

  if (digitalRead(MODE_BTN_PIN) == LOW) {
    phase = PHASE_RACE;
    loadOptimalPath();
    Serial.println(F("=== FASE 2: RACE MODUS ==="));
    Serial.print(F("Geladen pad ("));
    Serial.print(optLen);
    Serial.print(F(" kruispunten): "));
    Serial.println(optPath);
  } else {
    phase = PHASE_EXPLORE;
    clearStoredPath();
    rawLen = 0;
    rawPath[0] = '\0';
    Serial.println(F("=== FASE 1: VERKENNINGS MODUS ==="));
  }

  // Short countdown before start
  for (int i = 3; i > 0; i--) {
    Serial.print(i);
    Serial.println(F("..."));
    delay(1000);
  }
  Serial.println(F("START!"));
}

void loop() {
  if (phase == PHASE_EXPLORE) {
    exploreStep();
  } else {
    raceStep();
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// PHASE 1 – EXPLORATION
// ═══════════════════════════════════════════════════════════════════════════════

void exploreStep() {
  SensorData sd = readSensors();

  // ── End-of-maze / finish detection ────────────────────────────────────────
  // All sensors dark (end marker) or all sensors active at once (finish line).
  bool allOn  = (sd.count == 5);
  bool allOff = (sd.count == 0);

  if (allOff) {
    // Creep forward a tiny bit and re-check to rule out a glitch.
    setMotors(60, 60);
    delay(150);
    sd = readSensors();
    if (sd.count == 0) {
      stopMotors();
      Serial.println(F("[VERKENNING] Finish bereikt!"));
      buildOptimalPath();
      saveOptimalPath();
      Serial.println(F("[VERKENNING] Herstart terwijl je de knop inhoudt voor race modus."));
      while (true) delay(500);
    }
  }

  if (allOn) {
    // Treat wide-line finish marker: stop and declare done.
    stopMotors();
    delay(100);
    sd = readSensors();
    if (sd.count == 5) {
      Serial.println(F("[VERKENNING] Finish lijn gedetecteerd!"));
      buildOptimalPath();
      saveOptimalPath();
      Serial.println(F("[VERKENNING] Herstart terwijl je de knop inhoudt voor race modus."));
      while (true) delay(500);
    }
  }

  // ── Junction handling ──────────────────────────────────────────────────────
  unsigned long now = millis();
  if ((now - lastJunctionMs > JUNCTION_COOLDOWN_MS) && isAtJunction(sd)) {
    lastJunctionMs = now;

    crossJunctionCentre();        // Drive to middle of the intersection
    JuncInfo ji  = sampleJunctionArms();
    char     dir = leftHandChoice(ji);

    if (rawLen < MAX_JUNCTIONS) {
      rawPath[rawLen++] = dir;
      rawPath[rawLen]   = '\0';
    }

    Serial.print(F("[KRUISPUNT #"));
    Serial.print(rawLen);
    Serial.print(F("] links="));
    Serial.print(ji.hasLeft);
    Serial.print(F(" recht="));
    Serial.print(ji.hasStraight);
    Serial.print(F(" rechts="));
    Serial.print(ji.hasRight);
    Serial.print(F(" → "));
    Serial.println(dir);

    executeDir(dir);
    delay(80);
    return;
  }

  // ── Normal PID line following ──────────────────────────────────────────────
  pidStep(sd, BASE_SPEED);
}

// ═══════════════════════════════════════════════════════════════════════════════
// PHASE 2 – RACE
// ═══════════════════════════════════════════════════════════════════════════════

void raceStep() {
  SensorData sd = readSensors();

  // ── Finish detection ───────────────────────────────────────────────────────
  bool allOff = (sd.count == 0);
  bool allOn  = (sd.count == 5);

  if (allOff) {
    setMotors(RACE_SPEED, RACE_SPEED);
    delay(150);
    sd = readSensors();
    if (sd.count == 0) {
      stopMotors();
      Serial.println(F("[RACE] Finish! Klaar!"));
      while (true) delay(500);
    }
  }

  if (allOn) {
    stopMotors();
    delay(100);
    if (readSensors().count == 5) {
      Serial.println(F("[RACE] Finish lijn! Klaar!"));
      while (true) delay(500);
    }
  }

  // ── Junction handling ──────────────────────────────────────────────────────
  unsigned long now = millis();
  if ((now - lastJunctionMs > JUNCTION_COOLDOWN_MS) && isAtJunction(sd)) {
    lastJunctionMs = now;

    if (raceIdx < optLen) {
      char dir = optPath[raceIdx++];
      Serial.print(F("[RACE kruispunt #"));
      Serial.print(raceIdx);
      Serial.print(F("] → "));
      Serial.println(dir);

      crossJunctionCentre();    // Cross to junction centre (no arm sampling needed)
      executeDir(dir);
      delay(80);
    } else {
      // Path exhausted but junction detected: treat as straight.
      crossJunctionCentre();
    }
    return;
  }

  // ── Fast PID line following ────────────────────────────────────────────────
  pidStep(sd, RACE_SPEED);
}

// ═══════════════════════════════════════════════════════════════════════════════
// SENSOR READING
// ═══════════════════════════════════════════════════════════════════════════════

static SensorData readSensors() {
  SensorData sd;
  const int  pins[5]    = {S1_PIN, S2_PIN, S3_PIN, S4_PIN, S5_PIN};
  const int  weights[5] = {-4, -2, 0, 2, 4};

  int wSum = 0, cSum = 0;
  for (int i = 0; i < 5; i++) {
    sd.on[i] = (digitalRead(pins[i]) == LINE_DETECTED);
    if (sd.on[i]) { wSum += weights[i]; cSum++; }
  }
  sd.count = cSum;
  // Weighted centroid; if no sensors active, keep last known error direction.
  sd.pos = (cSum > 0) ? (float)wSum / cSum : pidLastError * 4.0f;
  return sd;
}

// ═══════════════════════════════════════════════════════════════════════════════
// JUNCTION DETECTION
// ═══════════════════════════════════════════════════════════════════════════════

// A junction is signalled when at least one outer sensor fires and three or
// more sensors are active total. This distinguishes normal curves (where only
// 1-2 adjacent sensors are active) from crossings/T-junctions.
static bool isAtJunction(const SensorData& sd) {
  bool outerFire = sd.on[0] || sd.on[4];
  return outerFire && (sd.count >= 3);
}

// Creep forward through the junction centre so the robot is roughly centred
// on the crossing before we sample which arms are present.
static void crossJunctionCentre() {
  setMotors(80, 80);
  delay(JUNC_CROSS_MS);
  stopMotors();
  delay(30);
}

// After centering: read sensors to discover which arms are available.
// Left arm  → outer-left or inner-left sensor active.
// Right arm → outer-right or inner-right sensor active.
// Straight  → centre sensor active.
static JuncInfo sampleJunctionArms() {
  SensorData sd = readSensors();
  JuncInfo ji;
  ji.hasLeft     = sd.on[0] || sd.on[1];
  ji.hasStraight = sd.on[2];
  ji.hasRight    = sd.on[3] || sd.on[4];
  return ji;
}

// ═══════════════════════════════════════════════════════════════════════════════
// PID LINE FOLLOWING
// ═══════════════════════════════════════════════════════════════════════════════

static void pidStep(const SensorData& sd, int baseSpd) {
  float err = sd.pos;

  pidIntegral += err;
  pidIntegral  = constrain(pidIntegral, -100.0f, 100.0f);  // anti-windup

  float deriv  = err - pidLastError;
  pidLastError = err;

  float corr   = KP * err + KI * pidIntegral + KD * deriv;

  int lSpd = constrain((int)(baseSpd - corr), -MAX_SPEED, MAX_SPEED);
  int rSpd = constrain((int)(baseSpd + corr), -MAX_SPEED, MAX_SPEED);
  setMotors(lSpd, rSpd);
}

static void resetPid() {
  pidIntegral  = 0.0f;
  pidLastError = 0.0f;
}

// ═══════════════════════════════════════════════════════════════════════════════
// MOTOR CONTROL
// ═══════════════════════════════════════════════════════════════════════════════

static void setMotors(int left, int right) {
  // Left motor
  digitalWrite(MOTOR_L_FWD, left  > 0 ? HIGH : LOW);
  digitalWrite(MOTOR_L_BWD, left  < 0 ? HIGH : LOW);
  ledcWrite(LEDC_CH_L, (uint32_t)constrain(abs(left),  0, 255));

  // Right motor
  digitalWrite(MOTOR_R_FWD, right > 0 ? HIGH : LOW);
  digitalWrite(MOTOR_R_BWD, right < 0 ? HIGH : LOW);
  ledcWrite(LEDC_CH_R, (uint32_t)constrain(abs(right), 0, 255));
}

static void stopMotors() {
  digitalWrite(MOTOR_L_FWD, LOW);
  digitalWrite(MOTOR_L_BWD, LOW);
  digitalWrite(MOTOR_R_FWD, LOW);
  digitalWrite(MOTOR_R_BWD, LOW);
  ledcWrite(LEDC_CH_L, 0);
  ledcWrite(LEDC_CH_R, 0);
}

// ═══════════════════════════════════════════════════════════════════════════════
// TURNING
// ═══════════════════════════════════════════════════════════════════════════════

// Spin on the spot for the given duration, then re-align to the line.
static void doSpin(int leftSpd, int rightSpd, unsigned long ms) {
  setMotors(leftSpd, rightSpd);
  delay(ms);
  stopMotors();
  alignToLine();
  resetPid();
}

static void turnLeft()   { doSpin(-BASE_SPEED,  BASE_SPEED, TURN_90_MS);  }
static void turnRight()  { doSpin( BASE_SPEED, -BASE_SPEED, TURN_90_MS);  }
static void turnAround() { doSpin(-BASE_SPEED,  BASE_SPEED, TURN_180_MS); }

// Creep forward until the centre sensor locks onto the line again.
static void alignToLine() {
  unsigned long t = millis();
  while (millis() - t < ALIGN_TIMEOUT) {
    SensorData sd = readSensors();
    if (sd.on[2]) break;   // centre sensor found the line
    setMotors(60, 60);
  }
  stopMotors();
}

// Execute a pre-computed direction character.
static void executeDir(char dir) {
  switch (dir) {
    case 'L': turnLeft();   break;
    case 'R': turnRight();  break;
    case 'B': turnAround(); break;
    case 'S': default:      break;  // straight – no spin needed
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// LEFT-HAND FOLLOW ALGORITHM
// ═══════════════════════════════════════════════════════════════════════════════
// Priority order: Left > Straight > Right > Back (U-turn / dead end)

static char leftHandChoice(const JuncInfo& ji) {
  if (ji.hasLeft)     return 'L';
  if (ji.hasStraight) return 'S';
  if (ji.hasRight)    return 'R';
  return 'B';  // dead end – U-turn
}

// ═══════════════════════════════════════════════════════════════════════════════
// PATH OPTIMISATION
// ═══════════════════════════════════════════════════════════════════════════════
//
// Any time the path contains  X + B + Y  (the robot went down a dead-end arm
// and came back) the triplet can be replaced by a single equivalent direction Z:
//
//   Z  =  (toDeg(X) + 180 + toDeg(Y))  mod 360
//
// where  L = 270° (-90°),  S = 0°,  R = 90°,  B = 180°.
//
// The loop repeats until no interior B remains.

static char combineDirs(char x, char y) {
  auto toDeg = [](char c) -> int {
    switch (c) {
      case 'L': return 270;  // -90 ≡ 270 (mod 360)
      case 'S': return   0;
      case 'R': return  90;
      case 'B': return 180;
      default:  return   0;
    }
  };
  auto toChar = [](int d) -> char {
    d = ((d % 360) + 360) % 360;
    switch (d) {
      case   0: return 'S';
      case  90: return 'R';
      case 180: return 'B';
      case 270: return 'L';
      default:  return 'S';
    }
  };
  int net = (toDeg(x) + 180 + toDeg(y)) % 360;
  return toChar(net);
}

static void buildOptimalPath() {
  // Start with a copy of the raw recorded path.
  memcpy(optPath, rawPath, (size_t)(rawLen + 1));
  optLen = rawLen;

  // Repeatedly collapse  X + B + Y  → Z  until no more interior B remains.
  bool changed = true;
  while (changed) {
    changed = false;
    for (int i = 0; i + 2 < optLen; i++) {
      if (optPath[i + 1] == 'B') {
        char merged = combineDirs(optPath[i], optPath[i + 2]);
        optPath[i] = merged;
        // Remove the two consumed characters by shifting the rest left.
        memmove(&optPath[i + 1], &optPath[i + 3],
                (size_t)(optLen - i - 2));
        optLen -= 2;
        optPath[optLen] = '\0';
        changed = true;
        break;  // Restart the scan from the beginning.
      }
    }
  }

  Serial.print(F("[PAD] Ruw pad ("));
  Serial.print(rawLen);
  Serial.print(F(" kruispunten): "));
  Serial.println(rawPath);
  Serial.print(F("[PAD] Optimaal ("));
  Serial.print(optLen);
  Serial.print(F(" kruispunten): "));
  Serial.println(optPath);
}

// ═══════════════════════════════════════════════════════════════════════════════
// NVS PERSISTENCE  (survives power-off between Phase 1 and Phase 2)
// ═══════════════════════════════════════════════════════════════════════════════

static void saveOptimalPath() {
  prefs.begin("linerobot", /*readOnly=*/false);
  prefs.putString("optPath", optPath);
  prefs.putInt("optLen", optLen);
  prefs.end();
  Serial.println(F("[NVS] Optimaal pad opgeslagen."));
}

static void loadOptimalPath() {
  prefs.begin("linerobot", /*readOnly=*/true);
  String s = prefs.getString("optPath", "");
  optLen   = prefs.getInt("optLen", 0);
  prefs.end();

  s.toCharArray(optPath, MAX_JUNCTIONS + 1);
  optPath[optLen] = '\0';

  if (optLen == 0) {
    Serial.println(F("[NVS] WAARSCHUWING: geen opgeslagen pad gevonden!"));
    Serial.println(F("[NVS] Voer eerst fase 1 (verkenning) uit."));
  }
}

static void clearStoredPath() {
  prefs.begin("linerobot", /*readOnly=*/false);
  prefs.clear();
  prefs.end();
}
