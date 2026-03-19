# linefollower

Competitieve lijnrobot voor de UA cursus.

---

## Bestanden

| Bestand | Doel |
|---|---|
| `linefollower_main/linefollower_main.ino` | **Hoofd-sketch** – volledig competitieprogramma (zie hieronder) |
| `Lijn_detectie.ino` | Testsketch – leest 5 IR-sensoren uit en print hun waarden |
| `ultrasone.ino` | Testsketch – meet afstand met HC-SR04 ultrasone sensor |

---

## Architectuur: twee fasen

### Fase 1 – Verkenning (standaard bij opstarten)

1. **PID lijnvolger** op rechte stukken en bochten  
   De vijf IR-sensoren berekenen een foutwaarde (gewogen middelpunt) en een
   PID-regelaar stuurt de twee motoren aan zodat de robot op de lijn blijft.

2. **Left-hand follow op kruispunten**  
   Zodra de buitenste sensoren samen vuren (kruispunt, T-splitsing, driehoek)
   schakelt de robot naar *mapping mode*:
   - de robot rijdt naar het midden van het kruispunt,
   - leest welke armen beschikbaar zijn (links / rechtdoor / rechts),
   - kiest altijd de meest **linkse** beschikbare richting (left-hand rule),
   - slaat de beslissing op als één teken: `L`, `R`, `S` of `B`.

3. **Padoptimalisatie bij de finish**  
   Het ruwe pad (`rawPath`) bevat doodlopende zijwegen als triplets `X + B + Y`.
   Deze worden teruggebracht naar één equivalent teken `Z`:
   ```
   Z = (graden(X) + 180 + graden(Y))  mod 360
   ```
   met L = −90°, S = 0°, R = +90°, B = 180°.  
   Het geoptimaliseerde pad wordt opgeslagen in de ESP32 NVS (niet-vluchtig
   geheugen) en overleeft een stroomonderbreking.

### Fase 2 – Race (houdt de bootknop ingedrukt bij het opstarten)

1. **Snelle PID lijnvolger** op rechte stukken en bochten.
2. Bij elk **kruispunt** haalt de robot het volgende teken op uit het opgeslagen
   optimale pad en voert die richting direct uit – geen left-hand logic meer
   nodig.

---

## Hardware

```
ESP32 DevKit  ←→  L298N motordriver  ←→  2× DC-motor
              ←→  5× IR-lijnsensor (GPIO 34, 35, 32, 33, 25)
              ←→  Bootknop GPIO 0 (houdt LOW = race modus)
```

### Pinout

| Signaal | GPIO |
|---|---|
| Sensor S1 (uiterst links) | 34 |
| Sensor S2 | 35 |
| Sensor S3 (midden) | 32 |
| Sensor S4 | 33 |
| Sensor S5 (uiterst rechts) | 25 |
| Linker motor PWM | 26 |
| Linker motor FWD | 27 |
| Linker motor BWD | 14 |
| Rechter motor PWM | 18 |
| Rechter motor FWD | 19 |
| Rechter motor BWD | 21 |
| Moduusknop | 0 (boot) |

Sensorlogica: **LOW = lijn gedetecteerd** (donkere lijn op lichte ondergrond).

---

## Gebruik

1. **Fase 1** – robot normaal opstarten (knop niet indrukken).  
   De robot verkent het parcours, slaat het geoptimaliseerde pad op en stopt
   bij de finish. Controleer via de seriële monitor (115200 baud) of het pad
   correct is.

2. **Fase 2** – robot opstarten terwijl de bootknop ingedrukt is.  
   De robot laadt het opgeslagen pad en rijdt het parcours zo snel mogelijk af.

---

## PID-parameters aanpassen

In `linefollower_main.ino`:

```cpp
#define BASE_SPEED   120   // Verkenningssnelheid  (0-255)
#define RACE_SPEED   180   // Racesnelheid         (0-255)
#define KP  30.0f          // Proportionele gain
#define KI   0.05f         // Integrale gain
#define KD  20.0f          // Differentiële gain
```

Begin met `KI = 0`, verhoog `KP` totdat de robot oscileert, verlaag dan
iets en verhoog `KD` om te dempen.  
Pas `TURN_90_MS` en `TURN_180_MS` aan voor jouw robot om exacte 90° / 180°
draaibewegingen te bereiken.
