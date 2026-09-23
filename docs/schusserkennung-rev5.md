# Schusserkennung Rev 5 – Piezo-Anker + robuste Ausgleichsrechnung

> **Umsetzungsstand (Rev 4.11.0):** Dieser Entwurf ist in
> `src/schiessstand_firmware.ino` als **optionaler** zweiter Auswertepfad
> integriert (`SET ALGO=CLASSIC|RIM`, Default weiterhin `CLASSIC` = bisheriges
> Verhalten unverändert) - siehe Rev-4.11.0-Eintrag im Datei-Header sowie
> `docs/protokoll-referenz.md` Abschnitte 4.5b/5.2/7.3b/7.4. Details, die von
> den Code-Beispielen unten abweichen: `computePostWait()`/`maxDistMm` sind in
> der Firmware `updatePostWaitUs()`/`updateRimMaxDist()` und berechnen die
> maximale Mic-Distanz **dynamisch** aus der aktuellen Geometrie (`SET
> MICHALFX`/`STANDOFFSTEEL`/`STANDOFFPAPER`) statt fest 320mm anzunehmen;
> `SET WINDOW`/`TDOA`/`BSHIFTPCT`/`BSHIFTCAP`/`RADIUS`/`MINCLUSTER`/
> `MAXPRECISION` sind **nicht** entfallen, sondern bleiben für `ALGO=CLASSIC`
> weiter in Kraft (nur bei `ALGO=RIM` wirkungslos). Die Hardware-Zeitstempel-
> Präzisionshinweise aus Abschnitt 4 (Zykluszähler zuerst) sind umgesetzt und
> wirken auf beide Pfade; die MCPWM-Hardware-Capture-Idee (Abschnitt 4, Punkt
> 3) ist **nicht** umgesetzt und bleibt ein möglicher weiterer Schritt.
> `host_sim.cpp`/`gen_rim_log.py` liegen unter `test/sim/`, nicht `test/`.

## 1. Was am bisherigen Zeitfenster nicht robust ist

Die **erste beliebige Mic-Flanke** öffnet das Fenster und wird zum Zeitnullpunkt.
Daraus folgen drei Fehlerbilder:

1. **Mündungsknall**: Bei 10 m und ~150 m/s kommt der Knall ca. 38 ms *vor* dem
   Einschlag an. Er öffnet das Fenster, der Schuss wird ohne Piezo ausgewertet
   (`piezo_ok=0`), danach startet die Sperrzeit (`SET DEBOUNCE`, Default 100 ms)
   – **der echte Einschlag fällt in die Sperrzeit und geht komplett verloren**,
   Piezo inklusive.
2. **Einzelne Störflanke** (Klick, Übersprechen, Vibration) kurz vor dem
   Einschlag setzt den Nullpunkt falsch. Die echten Flanken der fernen Mics
   liegen dann ggf. hinter `SET TDOA` und werden schon in der ISR verworfen.
3. Die Reihenfolge der Ereignisse entscheidet über das Ergebnis – nicht die
   Physik.

**Neu:** Die ISRs schreiben jede Flanke **ständig** in einen Ringpuffer je Kanal.
Ausgewertet wird **nur** auf das Piezo hin, und zwar *rückwirkend*: Der Einschlag
t0 muss in `[t_piezo − PIEZOMAX, t_piezo − PIEZOMIN]` liegen, je Mic kommen nur
Flanken in `[t0min + Standoff/c, t0max + maxDist/c]` in Frage. Was davor oder
danach passiert (Knall, Echos, Klingeln), ist irrelevant. Die Luft-Mics lösen
keine Sperrzeit mehr aus.

## 2. Positionsbestimmung (`include/shot_locator.h`)

- **MSAC-Hypothesen**: jede Mic-Dreierkombination × erste 2 Kandidatenflanken,
  bewertet nach Konsens der übrigen Mics. Ein Echo kann nicht mehr „gewinnen“.
- **Gauss-Newton mit Huber-Gewichten** über alle Inlier gleichzeitig,
  Unbekannte (x, y, t0). Ersetzt BSHIFT und die Cluster-Mittelung.
- **Lochrand-Modell**: Der erste Schall kommt vom mic-nächsten Punkt des
  Lochrands: `D_i = sqrt((rho_i − r)² + z²)`, r = 2,25 mm. Gefittet wird das
  Zentrum.
- **Güte**: `sigma_x/sigma_y` aus der Kovarianzmatrix (1σ, mm), `rms_ns`.

### Ehrliche Einordnung des Lochrands
Ein konstanter Rand-Versatz ist für alle Mics fast gleich und fällt bei TDOA
weitgehend heraus (steckt dann in t0). Relevant wird er nur, weil
`(rho−r)/D` je Mic leicht verschieden ist – vor allem bei Treffern weit außen.
Simulation: ohne Zeitrauschen 0,022 → 0,007 mm, bei 300 ns Rauschen kaum
messbar. **Der Lochdurchmesser ist nicht der begrenzende Faktor – die
Zeitmessung ist es** (Abschnitt 4).

### Simulationsergebnis (`test/sim/host_sim.cpp`, 4000 Schüsse je Szenario)

| Szenario | alt RMS / P95 / max | neu RMS / P95 / max |
|---|---|---|
| 300 ns Rauschen | 0,12 / 0,21 / 0,43 mm | 0,11 / 0,19 / 0,38 mm |
| + 5 % Störflanken je Mic | 14,8 / 3,3 / 146 mm | 0,10 / 0,18 / 0,32 mm |
| + 30 % Mündungsknall | 28,5 % Schüsse verloren | 0 % verloren, 0,11 mm |

Die 2σ-Angabe enthält die wahre Position in ~99 % der Fälle – taugt also als
`clean`-Kriterium.

```
g++ -O2 -std=c++17 -Iinclude test/sim/host_sim.cpp -o host_sim && ./host_sim
```

## 3. Einbau in die Firmware

### 3.1 Ringpuffer statt Sammelfenster (ersetzt airCC/airCount/firstAirCC)

```cpp
#include "shot_locator.h"

#define EDGE_RING 32
static volatile uint32_t ringCC[NUM_AIR][EDGE_RING];
static volatile uint32_t ringUs[NUM_AIR][EDGE_RING];   // Alterspruefung (Zyklus-Wrap 17,9 s)
static volatile uint8_t  ringHead[NUM_AIR];
static volatile bool     freezeActive = false;          // nach Piezo+postWait nichts ueberschreiben
static volatile uint32_t freezeCC     = 0;

void IRAM_ATTR airISR(void *arg)
{
    const uint32_t cc  = esp_cpu_get_cycle_count();      // ZUERST - vor allem anderen
    const uint32_t idx = (uint32_t)arg;
    const uint64_t nowUs = (uint64_t)esp_timer_get_time();
    if (testMode) { testModeHit((uint8_t)idx, nowUs); return; }
    if (freezeActive && (int32_t)(cc - freezeCC) > 0) return;

    const uint8_t h    = ringHead[idx];
    const uint8_t prev = (h + EDGE_RING - 1) % EDGE_RING;
    if ((uint32_t)(cc - ringCC[idx][prev]) < 20U * cpuMHz) return;   // Totzeit 20 us
    ringCC[idx][h] = cc;
    ringUs[idx][h] = (uint32_t)nowUs;
    ringHead[idx]  = (h + 1) % EDGE_RING;
}

static volatile bool     piezoPending = false;
static volatile uint32_t piezoCC  = 0;
static volatile uint64_t piezoUs  = 0;
static uint32_t postWaitUs = 1000;   // siehe computePostWait()

void IRAM_ATTR piezoISR()
{
    const uint32_t cc = esp_cpu_get_cycle_count();
    const uint64_t nowUs = (uint64_t)esp_timer_get_time();
    if (testMode) { testModeHit(NUM_AIR, nowUs); return; }
    if (piezoPending || nowUs < lockoutUntil) return;    // Sperrzeit NUR noch fuers Piezo
    piezoCC  = cc;
    piezoUs  = nowUs;
    freezeCC = cc + postWaitUs * cpuMHz;
    freezeActive = true;
    piezoPending = true;
}

// Spaetester Mic-Eingang nach dem Piezo: maxDist/c - PIEZOMIN + Reserve
static void computePostWait()
{
    const float maxDistMm = 320.0f;
    int32_t us = (int32_t)(maxDistMm / (cfg.soundSpeedMps * 1e-3f))
               - (int32_t)cfg.piezoMinUs + 150;
    postWaitUs = us < 200 ? 200 : (uint32_t)us;
}
```

### 3.2 loop()

```cpp
if (piezoPending && (uint64_t)esp_timer_get_time() - piezoUs >= postWaitUs) {
    processShotAnchored();
}
```
(Der bisherige `shotInProgress`-Block entfällt.)

### 3.3 Auswertung

```cpp
static void processShotAnchored()
{
    uint32_t cc[NUM_AIR][EDGE_RING], us[NUM_AIR][EDGE_RING];
    uint32_t pCC; uint64_t pUs;
    noInterrupts();
    memcpy(cc, (const void *)ringCC, sizeof(cc));
    memcpy(us, (const void *)ringUs, sizeof(us));
    pCC = piezoCC; pUs = piezoUs;
    lockoutUntil = (uint64_t)esp_timer_get_time() + (uint64_t)cfg.debounceMs * 1000ULL;
    piezoPending = false;
    freezeActive = false;
    interrupts();

    const float preNs  = (float)cfg.piezoMaxUs * 1000.0f + 50000.0f;
    const float postNs = (float)postWaitUs * 1000.0f;

    shotloc::Edges ed{};
    for (int i = 0; i < NUM_AIR; i++) {
        ed.enabled[i] = cfg.micEnabled[i];
        float tmp[EDGE_RING]; int n = 0;
        for (int k = 0; k < EDGE_RING; k++) {
            if ((uint32_t)((uint32_t)pUs - us[i][k]) > 20000U
                && (uint32_t)(us[i][k] - (uint32_t)pUs) > 20000U) continue;   // zu alt/leer
            const float tNs = (float)(int32_t)(cc[i][k] - pCC) * 1000.0f / (float)cpuMHz
                            - (float)cfg.micOffsetNs[i];
            if (tNs < -preNs || tNs > postNs) continue;
            tmp[n++] = tNs;
        }
        // aufsteigend sortieren (n ist klein)
        for (int a = 1; a < n; a++) { float v = tmp[a]; int b = a - 1;
            while (b >= 0 && tmp[b] > v) { tmp[b + 1] = tmp[b]; b--; } tmp[b + 1] = v; }
        ed.n[i] = 0;
        for (int k = 0; k < n && ed.n[i] < shotloc::MAX_EDGES; k++) ed.t[i][ed.n[i]++] = tmp[k];
    }

    shotloc::Geometry g;
    for (int i = 0; i < NUM_AIR; i++) { g.micX[i] = MIC_X[i]; g.micY[i] = MIC_Y[i]; }
    g.standoffMm     = micStandoffMm;
    g.soundMmPerNs   = soundMmPerNs;
    g.pelletRadiusMm = 2.25f;          // ggf. SET PELLETR
    g.maxDistMm      = 320.0f;

    shotloc::Gate gate;
    gate.active  = true;
    gate.t0MinNs = -(float)cfg.piezoMaxUs * 1000.0f;
    gate.t0MaxNs = (cfg.targetMode == TARGET_STEEL) ? 0.0f
                                                    : -(float)cfg.piezoMinUs * 1000.0f;

    shotloc::Params p;
    p.minMics = cfg.minMics;
    shotloc::Result r;
    const bool ok = shotloc::locate(g, ed, gate, p, &r) && r.valid;
    // Telegramm: x_um, y_um, sigma_x_um, sigma_y_um, rms_ns, used (Bitmaske),
    // t0_ns (= -Piezo-Verzoegerung), air_ns je Mic relativ zum Piezo.
    // clean = ok && max(sigma_x,sigma_y) <= SET MAXSIGMA
}
```

**STEEL-Modus:** `PIEZOMIN=0`, `PIEZOMAX` klein (z. B. 150 µs) setzen – das Piezo
kommt dort praktisch zeitgleich mit dem Einschlag.

**Ohne Piezo (`SET PIEZO=0`):** Koinzidenz-Trigger – erst auswerten, wenn
≥ MINMICS Kanäle innerhalb von `maxDist/c` eine Flanke haben; Anker = früheste
dieser Flanken, `gate = [anker − maxDist/c, anker − standoff/c]`.

### 3.4 Weitere Anpassungen
- `CAL START`: je Schuss die `Edges` speichern, Kosten = Σ `r.rmsNs²`
  (Gate aktiv). Offsets gehen vor dem Fit ab.
- `TESTSHOOT`: schreibt synthetische Einträge in `ringCC/ringUs` und setzt
  `piezoPending`.
- `replay_shot.py`: `air_ns` ist jetzt relativ zum Piezo.
- Entfallen: `SET WINDOW`, `SET TDOA` (in der ISR), `BSHIFTPCT/CAP`, `RADIUS`,
  `MINCLUSTER`, `MAXPRECISION` → neu `SET MAXSIGMA`.

## 4. Der eigentliche Präzisionshebel: Zeitstempel

1 µs Zeitfehler ≈ 0,36 mm. Die Software-Zeitstempel haben systematische Fehler,
die Kalibrierung nicht entfernen kann:

1. **`esp_timer_get_time()` vor dem Zykluszähler** (bisherige airISR) – kostet
   jedes Mal Zeit mit Jitter. Zykluszähler immer als erste Anweisung lesen.
2. **Sequenzielle GPIO-Interrupts**: Alle GPIOs teilen sich einen CPU-Interrupt.
   Der ESP-IDF-Dispatcher arbeitet erst GPIO 0–31 ab, **danach GPIO 32–39**,
   jeweils aufsteigend. Kommen Flanken innerhalb weniger µs (Treffer nahe der
   Mitte: links/rechts fast gleichzeitig!), bekommt der später bediente Kanal
   einen um die Handler-Laufzeit zu späten Zeitstempel – positionsabhängig,
   also nicht wegkalibrierbar.
   *Hypothese:* Das erklärt die Auffälligkeit an Kanal 4 (GPIO32) – und warum
   sie beim Umzug auf GPIO35 blieb (gleiche Bank). **Test:** ein
   Komparatorausgang gleichzeitig auf GPIO25 und GPIO32 legen, Differenz der
   Zeitstempel ansehen.
3. **WLAN-Interrupts** auf demselben Core verzögern einzelne Flanken um µs.

**Lösung: Hardware-Capture per MCPWM.** Der ESP32 hat 2 × 3 = 6
Capture-Kanäle – genau 6 Mics, über die GPIO-Matrix auf die bestehenden Pins
routbar, 12,5 ns Auflösung, unabhängig von Interrupt-Latenz. Die beiden
MCPWM-Gruppen laufen auf eigenen Timern; beim Start einmal über eine
gemeinsame GPIO-Sync-Quelle (`mcpwm_new_gpio_sync_src`, gleiche GPIO in beiden
Gruppen, `io_loop_back`) nullen. Das Piezo bleibt normaler GPIO-Interrupt –
es dient nur als Gate.

4. **Komparator-Laufzeit abhängig von der Amplitude** (fernes Mic = leiser =
   spätere Schwellenüberschreitung). Wirkt wie eine falsche Schallgeschwindigkeit
   und wird von `SET SOUNDSPEED` teilweise aufgefangen (die empirischen 355 m/s
   könnten auch daher kommen).
