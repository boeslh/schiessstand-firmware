/*
 * ============================================================================
 *  Elektronischer Schießstand – ESP32 Firmware
 *  Rev 3.10 – HELP/? gibt alle SET-Parameter als Hilfetext aus
 * ============================================================================
 *
 *  Neu in 3.4: HYBRID-Modus (SET HYBRID=1, persistent im NVS).
 *  Neu in 3.6: HYBRID=2 – reine Luftschall-Messung (kein Stahlblech noetig).
 *
 *  Konzept: 4 zusaetzliche Mikrofone (Piezo/Elektret) sitzen SEITLICH VOR
 *  dem Stahlblech (akustisch entkoppelt!). Der Einschlag erzeugt:
 *    1. die Stahlwelle (v~3000 m/s) -> Grobposition wie bisher
 *    2. direkten Luftschall (343 m/s) -> Feinposition (Faktor ~9 genauer)
 *  Problem: Die Stahlwelle strahlt unterwegs Luftschall ab ("Vorlaeufer"),
 *  der die Mikrofone VOR dem direkten Schall erreicht. Deshalb erfassen
 *  die Luftkanaele MEHRERE Flanken (Multi-Edge-Capture, bis zu 6 je Mic);
 *  der Stand-PC waehlt per Zeitfenster (aus Grobposition berechnet) die
 *  richtige Flanke aus und loest dann per TOA.
 *
 *  HYBRID=0 (Default): verhaelt sich exakt wie Rev 3.3 (nur Stahl).
 *  HYBRID=1: Stahl + Luft kombiniert (s.o.), Telegramm mit air_ns.
 *  HYBRID=2: NUR Luftmikrofone, keine Stahlsensoren beteiligt. Das erste
 *    erfasste Mikrofon loest das Sammelfenster selbst aus (kein Stahl-
 *    Vorlaeufer, daher auch kein Zeitfenster-Gating noetig). Gueltig ab
 *    3 von 4 Mikrofonen. Telegramm enthaelt nur air_ns (kein t_ns/t_us).
 *
 *  Pinbelegung:
 *    Stahl-Sensoren: GPIO 34, 35, 32, 33  (wie bisher, bei HYBRID=2 inaktiv)
 *    Luft-Mikrofone: GPIO 25, 26, 27, 14  (2. LM339, identisches Frontend)
 *
 *  Telegramm im Hybrid-Modus (zusaetzliches Feld air_ns):
 *    {"type":"shot","seq":7,"t_ns":[0,...],"air_ns":[[417,558,...],[...],
 *     [...],[...]],"hits":4,"ts":...}
 *    air_ns[i] = Liste ALLER erfassten Flanken von Mikrofon i, in ns
 *    relativ zum ersten Stahl-Hit. Leere Liste = keine Flanke im Fenster.
 *
 *  Telegramm im reinen Luftschall-Modus (HYBRID=2, kein t_ns/t_us):
 *    {"type":"shot","seq":8,"air_ns":[[0,...],[...],[...],[...]],
 *     "x_um":-22300,"y_um":-300,"pos_res_um":43910,"pos_valid":1,
 *     "hits":3,"ts":...}
 *    air_ns[i] relativ zum ersten erfassten Mikrofon (nicht zu einem
 *    Stahl-Hit, den gibt es in diesem Modus nicht).
 *
 *  Neu in 3.7: Trefferposition (x_um/y_um, 1 Einheit = 0.001 mm) wird im
 *  HYBRID=2-Telegramm direkt vom ESP32 berechnet (Hyperbel-Trilateration
 *  aus den ersten Flanken der 4 Luftmikrofone). Ursprung = Zentrum der
 *  Abprallflaeche, x positiv nach RECHTS, y positiv nach OBEN (jeweils aus
 *  Schuetzensicht). Geometrie (Ecken-Anordnung, siehe MIC_X/MIC_Y unten):
 *    GPIO25 = links unten   GPIO26 = rechts unten
 *    GPIO27 = links oben    GPIO14 = rechts oben
 *  je 125 mm horizontal / 100 mm vertikal vom Zentrum, 30 mm vor der
 *  Platte. Die 30%-Neigung der Platte zur Rueckwand ist irrelevant, da
 *  alles im plattenfesten Koordinatensystem gerechnet wird.
 *
 *  pos_valid=0, falls < 3 Mics ausgewertet werden konnten oder die
 *  Geometrie entartet war (x_um/y_um dann 0).
 *
 *  pos_res_um = Rest-Fehler der gewaehlten Loesung in 0.001mm (bei 4
 *  Treffern: die von 4 moeglichen "1 Mic ausschliessen, mit den anderen 3
 *  loesen, gegen das ausgeschlossene pruefen"-Kombinationen mit dem
 *  kleinsten Fehler; bei genau 3 Treffern immer 0, da keine Redundanz
 *  fuer einen Check besteht). WICHTIG: Mit nur 4 Mikrofonen gibt es genau
 *  1 redundante Messung - das reicht fuer einen groben Plausibilitaets-
 *  Hinweis, aber NICHT fuer eine zuverlaessige automatische Ausreisser-
 *  Erkennung in knappen Faellen (in Tests lagen die Rest-Fehler zweier
 *  Kombinationen teils nur ~0.1mm auseinander). Ein grosser pos_res_um
 *  (>> wenige mm) bedeutet: Vorsicht, die 4 Flankenzeiten waren nicht gut
 *  konsistent (z.B. Echo statt Direktschall an einem Mikrofon) - die
 *  Position ist dann mit Unsicherheit zu behandeln, nicht blind zu werten.
 *
 *  Neu in 3.8: SET DEBUG=0|1|2 (persistent im NVS) filtert, welche Schuss-/
 *  Reject-Telegramme ausgegeben werden (Zaehler/shotCounter laufen davon
 *  unabhaengig immer mit):
 *    0 (Default): nur sauber ermittelte Schuesse (Position bestimmbar,
 *      pos_res_um < SET OUTLIER-Schwelle; bei HYBRID=0/1 unveraendert alle
 *      gueltigen Schuesse)
 *    1: zusaetzlich Schuesse mit Mikrofon-Ausreisser bzw. nicht
 *      bestimmbarer Position (pos_valid=0)
 *    2: zusaetzlich Reject-Telegramme wegen zu weniger Hits/Mics (<3)
 *  Ausserdem garantiert emitLine() jetzt immer einen Zeilenvorschub auf
 *  der seriellen Schnittstelle.
 *
 *  Neu in 3.9: SET OUTLIER=<0.001mm> (persistent im NVS, Default 5000 =
 *  5.0mm) legt die Schwelle fest, ab der ein HYBRID=2-Schuss als
 *  "Mikrofon-Ausreisser" gilt (siehe DEBUG=1 oben). War zuvor ein fester
 *  Compile-Zeit-Wert - jetzt per Testschuss vor Ort feinjustierbar.
 *
 *  Neu in 3.10: Befehl HELP oder ? gibt eine Liste aller SET-Parameter mit
 *  gueltigem Wertebereich aus. Reiner Klartext nur ueber Serial (nicht per
 *  emitLine/TCP), damit der JSON-Zeilenstrom zum Stand-PC sauber bleibt.
 * ============================================================================
 *
 *  Neu in 3.3: Die Sensor-Zeitstempel kommen nicht mehr vom µs-Timer
 *  (esp_timer), sondern vom CPU-Zykluszaehler (esp_cpu_get_cycle_count).
 *
 *  Warum: Bei Koerperschall im Stahlblech (~3000 m/s) entspricht 1 µs
 *  bereits 3 mm Weg – die µs-Quantisierung war damit die dominante
 *  Fehlerquelle. Der Zykluszaehler laeuft mit CPU-Takt (240 MHz):
 *    Aufloesung ~4,2 ns  ->  ~0,013 mm Wegquantisierung. 
 *  Die Messunsicherheit wird damit von der Analogkette bestimmt,
 *  nicht mehr vom Timer.
 *
 *  Umsetzung:
 *    - ISR speichert den 32-bit-Zykluszaehler je Sensor
 *    - Differenzen werden in NANOSEKUNDEN umgerechnet und als neues
 *      Feld "t_ns" gesendet (zusaetzlich bleibt "t_us" fuer
 *      Rueckwaertskompatibilitaet erhalten, gerundet)
 *    - 32-bit-Ueberlauf (~17,9 s @ 240 MHz) ist unkritisch: Die
 *      unsigned-Differenz (cc - ccFirst) ist innerhalb des 5-ms-
 *      Sammelfensters immer korrekt, auch ueber den Wrap hinweg
 *    - WICHTIG: Jeder ESP32-Core hat seinen EIGENEN Zykluszaehler.
 *      Alle Sensor-ISRs werden aus setup() registriert und laufen
 *      damit auf demselben Core -> Zaehler sind vergleichbar.
 *    - Grobzeitlogik (Sperrzeit, Sammelfenster) nutzt weiter esp_timer
 *
 *  Telegramm neu:
 *    {"type":"shot","seq":12,"t_ns":[0,231042,584167,402375],
 *     "t_us":[0,231,584,402],"hits":4,"ts":123456789}
 *
 *  Konfiguration / Befehle: unveraendert zu Rev 3.2 (NVS, SET ...).
 *
 *  Build: Arduino IDE / PlatformIO, Board "ESP32 Dev Module"
 *         (Arduino-Core >= 2.x wegen esp_cpu_get_cycle_count)
 * ============================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <math.h>                 // Rev 3.7: sqrtf/fabsf/lroundf (Positionsberechnung)
#include "esp_cpu.h"
#include "driver/mcpwm_cap.h"     // Rev 3.5: Hardware-Capture
#include "soc/soc_caps.h"

// ============================================================================
//  Rev 3.5 – MCPWM-Hardware-Capture fuer die 4 Stahl-Sensoren
// ----------------------------------------------------------------------------
//  Warum: Die ISR-Methode (Rev 3.3/3.4) liest den Zykluszaehler ERST beim
//  Eintritt in die Interrupt-Routine. Zwischen echter Flanke und diesem
//  Lesen liegt die Interrupt-Latenz samt ihrer SCHWANKUNG (Jitter, real
//  ~1-3 µs, u.a. durch WLAN-Interrupts). Im Stahl (~3000 m/s) sind 2 µs
//  bereits 6 mm Wegunsicherheit.
//
//  MCPWM-Capture nimmt den Zeitstempel in HARDWARE zum exakten Flanken-
//  zeitpunkt (APB-Takt 80 MHz -> 12,5 ns Aufloesung, ohne CPU-Jitter). Die
//  Callback-Routine liest danach nur noch den bereits fixierten Wert.
//
//  Der Classic-ESP32 hat 2 MCPWM-Einheiten mit je 3 Capture-Kanaelen. 4
//  Stahlkanaele passen in Einheit 0 (3) + Einheit 1 (1). Die Luftkanaele
//  (Hybrid) laufen weiter per ISR (Zykluszaehler), da die Capture-Kanaele
//  fuer die Stahlsensoren verplant sind.
//
//  Umschaltbar per NVS:  SET CAPTURE=mcpwm (Default) | SET CAPTURE=isr
//  Faellt MCPWM-Init fehl (aeltere Core-Version), Fallback auf ISR.
// ============================================================================

// ---------------------------------------------------------------------------
// Konstanten & Werks-Defaults (greifen nur bei leerem NVS)
// ---------------------------------------------------------------------------

#define FW_VERSION   "3.10.0"
#define NUM_SENSORS  4
#define SERIAL_BAUD  115200
#define NVS_NS       "schiessstd"     // NVS-Namespace

static const uint8_t SENSOR_PINS[NUM_SENSORS] = {34, 35, 32, 33};

// Luft-Mikrofone (Hybrid-Modus): 2. LM339, identische Frontend-Schaltung
#define NUM_AIR        4
#define AIR_MAX_EDGES  6
static const uint8_t AIR_PINS[NUM_AIR] = {25, 26, 27, 14};

// HYBRID-Modi (SET HYBRID=0|1|2)
#define MODE_STEEL  0   // nur Stahlsensoren (Rev 3.3-Verhalten)
#define MODE_HYBRID 1   // Stahl + Luft kombiniert
#define MODE_AIR    2   // nur Luftmikrofone, kein Stahl

struct DeviceConfig {
    String   ssid;          // "" = WLAN deaktiviert
    String   pass;
    String   host;          // Stand-PC
    uint16_t port;
    uint16_t lane;
    uint32_t debounceMs;
    uint32_t windowMs;
    uint8_t  hybrid;       // Messmodus: 0=Stahl, 1=Hybrid, 2=nur Luftschall
    uint8_t  debug;        // Ausgabe-Filter: 0=nur saubere Schuesse (Default),
                            // 1=+Schuesse mit Mikrofon-Ausreisser, 2=+Reject
                            // wegen zu weniger Hits
    uint32_t airOutlierUm; // Schwelle (0.001mm) ab der HYBRID=2-Schuss als
                            // Mikrofon-Ausreisser gilt (SET OUTLIER)
    bool     useMcpwm;    // true = Hardware-Capture, false = ISR-Fallback
    // Netzwerk: statische IP (staticIP=false → DHCP)
    bool     staticIP;
    String   ip;
    String   gateway;
    String   subnet;
    String   dns;           // "" → Gateway als DNS verwenden
};

static DeviceConfig cfg;
static Preferences  prefs;

static void loadConfig()
{
    prefs.begin(NVS_NS, /*readOnly=*/true);
    cfg.ssid       = prefs.getString("ssid", "");
    cfg.pass       = prefs.getString("pass", "");
    cfg.host       = prefs.getString("host", "192.168.1.10");
    cfg.port       = prefs.getUShort("port", 9000);
    cfg.lane       = prefs.getUShort("lane", 1);
    cfg.debounceMs = prefs.getUInt("debounce", 100);
    cfg.windowMs   = prefs.getUInt("window", 5);
    cfg.hybrid     = prefs.getUChar("hybrid", MODE_STEEL);
    cfg.debug      = prefs.getUChar("debug", 0);
    cfg.airOutlierUm = prefs.getUInt("outlier", 5000);   // Default 5.0 mm
    cfg.useMcpwm   = prefs.getBool("mcpwm", true);    // Default: Hardware
    cfg.staticIP   = prefs.getBool("static_ip", false);
    cfg.ip         = prefs.getString("ip", "");
    cfg.gateway    = prefs.getString("gateway", "");
    cfg.subnet     = prefs.getString("subnet", "255.255.255.0");
    cfg.dns        = prefs.getString("dns", "");
    prefs.end();
}

// Einzelnen Wert persistieren (oeffnet kurz schreibend)
template <typename T>
static void saveVal(const char *key, T value);

template <> void saveVal<String>(const char *key, String v)
{ prefs.begin(NVS_NS, false); prefs.putString(key, v); prefs.end(); }
template <> void saveVal<uint16_t>(const char *key, uint16_t v)
{ prefs.begin(NVS_NS, false); prefs.putUShort(key, v); prefs.end(); }
template <> void saveVal<uint32_t>(const char *key, uint32_t v)
{ prefs.begin(NVS_NS, false); prefs.putUInt(key, v); prefs.end(); }
template <> void saveVal<bool>(const char *key, bool v)
{ prefs.begin(NVS_NS, false); prefs.putBool(key, v); prefs.end(); }
template <> void saveVal<uint8_t>(const char *key, uint8_t v)
{ prefs.begin(NVS_NS, false); prefs.putUChar(key, v); prefs.end(); }

// ---------------------------------------------------------------------------
// Schusserfassung – Rev 3.3: Zykluszaehler fuer die Praezisionsmessung
// ---------------------------------------------------------------------------

// Zykluszaehler-Stempel je Sensor. 0 hat hier eine Doppelbedeutung
// (kein Hit ODER zufaellig exakt Zaehlerstand 0) -> separates hitSeen-Flag.
static volatile uint32_t hitCC[NUM_SENSORS]  = {0};
static volatile bool     hitSeen[NUM_SENSORS] = {false};
static volatile uint32_t firstCC = 0;

// Grobzeit (esp_timer) nur fuer Fensterlogik und Sperrzeit
static volatile uint64_t firstHitTimeUs = 0;
static volatile bool     shotInProgress = false;
static volatile uint64_t lockoutUntil = 0;

static uint32_t shotCounter = 0;
static uint32_t sequenceNo  = 0;
static uint32_t cpuMHz      = 240;   // wird in setup() ermittelt

// Multi-Edge-Capture der Luftkanaele (HYBRID=1/2, bei HYBRID=0 inaktiv)
static volatile uint32_t airCC[NUM_AIR][AIR_MAX_EDGES];
static volatile uint8_t  airCount[NUM_AIR] = {0};
static volatile uint32_t airLastCC[NUM_AIR] = {0};
static volatile uint32_t firstAirCC = 0;   // CPU-Zyklen, Fenster-Nullpunkt
                                            // (Stahl-Hit bei HYBRID=1, sonst
                                            // erste Mikrofon-Flanke)

void IRAM_ATTR airISR(void *arg)
{
    if (cfg.hybrid == MODE_STEEL) return;

    const uint32_t idx = (uint32_t)arg;
    const uint32_t cc  = esp_cpu_get_cycle_count();

    if (cfg.hybrid == MODE_AIR) {
        // Reiner Luftschall-Modus: es gibt keinen Stahl-Hit, der das
        // Sammelfenster oeffnet -> die erste Mikrofon-Flanke tut es selbst.
        const uint64_t nowUs = (uint64_t)esp_timer_get_time();
        if (nowUs < lockoutUntil) return;
        if (!shotInProgress) {
            shotInProgress = true;
            firstAirCC     = cc;
            firstHitTimeUs = nowUs;
        }
    } else if (!shotInProgress) {
        return;                           // Hybrid: Luftflanken nur waehrend Stahl-Fenster
    }

    uint8_t n = airCount[idx];
    if (n >= AIR_MAX_EDGES) return;

    // Totzeit 20 µs zwischen Flanken desselben Kanals: blendet das
    // Eigenschwingen der Piezo-Resonanz (~4,4 kHz) nach jeder Flanke aus
    if (n > 0) {
        uint32_t dCC = cc - airLastCC[idx];          // wrap-sicher
        if (dCC < 20U * cpuMHz) return;
    }
    airCC[idx][n]   = cc;
    airLastCC[idx]  = cc;
    airCount[idx]   = n + 1;
}

void IRAM_ATTR sensorISR(void *arg)
{
    if (cfg.hybrid == MODE_AIR) return;   // reiner Luftschall-Modus: Stahl inaktiv

    const uint32_t idx = (uint32_t)arg;
    const uint32_t cc  = esp_cpu_get_cycle_count();   // ~4ns Aufloesung

    // Sperrzeit/Fensterlogik in Grobzeit (Wrap-sicher ueber Schuesse hinweg)
    const uint64_t nowUs = (uint64_t)esp_timer_get_time();
    if (nowUs < lockoutUntil) return;
    if (hitSeen[idx]) return;

    if (!shotInProgress) {
        shotInProgress  = true;
        firstCC         = cc;
        firstAirCC      = cc;   // gemeinsamer CPU-Nullpunkt fuer Luftkanaele
        firstHitTimeUs  = nowUs;
    }
    hitCC[idx]   = cc;
    hitSeen[idx] = true;
}

static void resetShotState()
{
    noInterrupts();
    for (int i = 0; i < NUM_SENSORS; i++) {
        hitCC[i]   = 0;
        hitSeen[i] = false;
    }
    firstCC        = 0;
    firstAirCC     = 0;
    firstHitTimeUs = 0;
    shotInProgress = false;
    for (int i = 0; i < NUM_AIR; i++) airCount[i] = 0;
    interrupts();
}

// ---------------------------------------------------------------------------
// Rev 3.5: MCPWM-Hardware-Capture-Backend fuer die 4 Stahl-Sensoren
// ---------------------------------------------------------------------------
// Jeder Stahlsensor bekommt einen MCPWM-Capture-Kanal. Der Callback laeuft
// bei jeder steigenden Flanke und speichert den von der HARDWARE fixierten
// Zaehlerstand (APB-Ticks). capSeen[]/firstCapTicks werden analog zum
// ISR-Pfad gefuellt; processShot() liest je nach aktivem Backend.

static bool     mcpwmReady = false;
static uint32_t apbMHz = 80;   // APB-Takt des Capture-Timers

static volatile uint32_t capTicks[NUM_SENSORS] = {0};
static volatile bool     capSeen[NUM_SENSORS]  = {false};
static volatile uint32_t firstCapTicks = 0;

static mcpwm_cap_timer_handle_t   capTimer[2] = {nullptr, nullptr};
static mcpwm_cap_channel_handle_t capChan[NUM_SENSORS] = {nullptr};

// Der classic ESP32 hat 2 MCPWM-Gruppen mit je einem EIGENEN, unabhaengigen
// Hardware-Zaehler (kein gemeinsamer Nullpunkt!). Bei 4 Sensoren + 3 Kanaelen
// je Timer landet Sensor 3 zwangsweise auf Gruppe 1, die restlichen auf
// Gruppe 0 - deren Zaehlerstaende sind NICHT direkt vergleichbar. capGroupOffset
// wird einmalig in mcpwmInit() per Soft-Catch-Kalibrierung bestimmt und auf
// jeden erfassten Tick-Wert addiert, um beide Gruppen in eine gemeinsame
// Zeitbasis (die von Gruppe 0) zu ueberfuehren.
static int32_t  capGroupOffset[2] = {0, 0};
static volatile bool     calActive = false;
static volatile bool     calSeen[2]  = {false, false};
static volatile uint32_t calTicks[2] = {0, 0};

static bool IRAM_ATTR capCallback(mcpwm_cap_channel_handle_t chan,
                                  const mcpwm_capture_event_data_t *edata,
                                  void *user_data)
{
    const uint32_t idx = (uint32_t)user_data;
    const uint32_t ticks = edata->cap_value;

    if (calActive) {
        // Boot-Kalibrierung laeuft immer, unabhaengig vom aktuellen Modus,
        // damit capGroupOffset auch bereitsteht, wenn spaeter zur Laufzeit
        // (ohne Reboot) auf HYBRID=1 umgeschaltet wird.
        const uint32_t g = idx / SOC_MCPWM_CAPTURE_CHANNELS_PER_TIMER;
        calTicks[g] = ticks;
        calSeen[g]  = true;
        return false;
    }
    if (cfg.hybrid == MODE_AIR) return false;   // reiner Luftschall-Modus: Stahl inaktiv

    const uint64_t nowUs = (uint64_t)esp_timer_get_time();
    if (nowUs < lockoutUntil) return false;
    if (capSeen[idx]) return false;

    const uint32_t adjTicks = ticks
        + (uint32_t)capGroupOffset[idx / SOC_MCPWM_CAPTURE_CHANNELS_PER_TIMER];

    if (!shotInProgress) {
        shotInProgress = true;
        firstCapTicks  = adjTicks;
        // CPU-Nullpunkt fuer die Luftkanaele; kleiner Callback-Latenz-
        // Versatz unkritisch (air_ns wird per Gating relativ ausgewertet).
        firstAirCC     = esp_cpu_get_cycle_count();
        firstHitTimeUs = nowUs;
    }
    capTicks[idx] = adjTicks;
    capSeen[idx]  = true;
    return false;
}

static bool mcpwmInit()
{
#if !SOC_MCPWM_SUPPORTED
    return false;
#else
    const int chPerTimer = SOC_MCPWM_CAPTURE_CHANNELS_PER_TIMER; // meist 3
    int timersNeeded = (NUM_SENSORS + chPerTimer - 1) / chPerTimer;
    if (timersNeeded > 2) return false;

    for (int t = 0; t < timersNeeded; t++) {
        mcpwm_capture_timer_config_t tcfg = {};
        tcfg.group_id = t;
        tcfg.clk_src  = MCPWM_CAPTURE_CLK_SRC_DEFAULT;
        if (mcpwm_new_capture_timer(&tcfg, &capTimer[t]) != ESP_OK)
            return false;
    }

    for (int i = 0; i < NUM_SENSORS; i++) {
        int t = i / chPerTimer;
        mcpwm_capture_channel_config_t ccfg = {};
        ccfg.gpio_num = SENSOR_PINS[i];
        ccfg.prescale = 1;
        ccfg.flags.pos_edge = true;
        ccfg.flags.neg_edge = false;
        ccfg.flags.pull_up  = false;
        if (mcpwm_new_capture_channel(capTimer[t], &ccfg, &capChan[i]) != ESP_OK)
            return false;
        mcpwm_capture_event_callbacks_t cbs = {};
        cbs.on_cap = capCallback;
        if (mcpwm_capture_channel_register_event_callbacks(
                capChan[i], &cbs, (void *)i) != ESP_OK)
            return false;
        if (mcpwm_capture_channel_enable(capChan[i]) != ESP_OK)
            return false;
    }

    for (int t = 0; t < timersNeeded; t++) {
        if (mcpwm_capture_timer_enable(capTimer[t]) != ESP_OK) return false;
        if (mcpwm_capture_timer_start(capTimer[t])  != ESP_OK) return false;
    }

    // Cross-Timer-Kalibrierung (nur noetig, wenn wirklich 2 Gruppen genutzt
    // werden): je Gruppe per Software einen Capture-Event ausloesen und den
    // dabei erfassten Zaehlerstand vergleichen. Beide Aufrufe liegen nur
    // wenige hundert ns auseinander -> Restfehler ist diese Aufruf-Latenz,
    // nicht mehr der unbegrenzte, zufaellige Boot-Zeit-Versatz.
    if (timersNeeded > 1) {
        calActive  = true;
        calSeen[0] = false;
        calSeen[1] = false;

        mcpwm_capture_channel_trigger_soft_catch(capChan[0]);
        mcpwm_capture_channel_trigger_soft_catch(capChan[chPerTimer]);

        uint32_t waitStart = millis();
        while ((!calSeen[0] || !calSeen[1]) && millis() - waitStart < 10) {
            // auf beide Soft-Catch-Callbacks warten
        }
        calActive = false;

        if (calSeen[0] && calSeen[1]) {
            capGroupOffset[1] = (int32_t)(calTicks[0] - calTicks[1]);
        } else {
            Serial.println("# Warnung: MCPWM-Cross-Timer-Kalibrierung fehlgeschlagen");
        }
    }

    uint32_t res = 0;
    if (capTimer[0] &&
        mcpwm_capture_timer_get_resolution(capTimer[0], &res) == ESP_OK
        && res > 0) {
        apbMHz = res / 1000000U;
        if (apbMHz < 1) apbMHz = 80;
    }
    return true;
#endif
}

static void resetCaptureState()
{
    noInterrupts();
    for (int i = 0; i < NUM_SENSORS; i++) {
        capTicks[i] = 0;
        capSeen[i]  = false;
    }
    firstCapTicks  = 0;
    firstAirCC     = 0;
    firstHitTimeUs = 0;
    shotInProgress = false;
    for (int i = 0; i < NUM_AIR; i++) airCount[i] = 0;
    interrupts();
}

// ---------------------------------------------------------------------------
// Netzwerk + Sendepuffer (identisch zu Rev 3.1)
// ---------------------------------------------------------------------------

static WiFiClient tcp;
static bool       wifiEnabled = false;
static uint32_t   nextConnectAttemptMs = 0;
static uint32_t   connectBackoffMs = 1000;

#define TXBUF_SLOTS 64
#define TXBUF_LINE  320
static char    txBuf[TXBUF_SLOTS][TXBUF_LINE];
static uint8_t txHead = 0, txTail = 0;

static inline bool txBufEmpty() { return txHead == txTail; }
static inline bool txBufFull()  { return (uint8_t)(txHead + 1) % TXBUF_SLOTS == txTail; }

static void txBufPush(const char *line)
{
    if (txBufFull()) txTail = (txTail + 1) % TXBUF_SLOTS;
    strncpy(txBuf[txHead], line, TXBUF_LINE - 1);
    txBuf[txHead][TXBUF_LINE - 1] = '\0';
    txHead = (txHead + 1) % TXBUF_SLOTS;
}

static void emitLine(const char *line)
{
    // Garantiert einen Zeilenvorschub auf der seriellen Schnittstelle, auch
    // falls ein Aufrufer mal vergisst, sein Format mit "\n" abzuschliessen.
    size_t len = strlen(line);
    Serial.print(line);
    if (len == 0 || line[len - 1] != '\n') Serial.print('\n');

    if (!wifiEnabled) return;

    if (tcp.connected()) {
        while (!txBufEmpty() && tcp.connected()) {
            tcp.print(txBuf[txTail]);
            txTail = (txTail + 1) % TXBUF_SLOTS;
        }
        if (tcp.connected()) {
            tcp.print(line);
            return;
        }
    }
    if (strstr(line, "\"type\":\"shot\"") || strstr(line, "\"type\":\"reject\"")) {
        txBufPush(line);
    }
}

static void emitf(const char *fmt, ...)
{
    char line[TXBUF_LINE];
    va_list args;
    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    emitLine(line);
}

// ---------------------------------------------------------------------------
// Telegramme
// ---------------------------------------------------------------------------

static void sendStatus()
{
    char ip[20] = "none";
    if (wifiEnabled && WiFi.status() == WL_CONNECTED) {
        snprintf(ip, sizeof(ip), "%s", WiFi.localIP().toString().c_str());
    }
    emitf("{\"type\":\"status\",\"version\":\"%s\",\"lane\":%u,"
          "\"uptime_s\":%llu,\"shots\":%u,\"window_ms\":%u,"
          "\"debounce_ms\":%u,\"sensors\":%d,\"wifi\":\"%s\",\"tcp\":%s,"
          "\"buffered\":%u,\"capture\":\"%s\"}\n",
          FW_VERSION, cfg.lane,
          (unsigned long long)(esp_timer_get_time() / 1000000ULL),
          shotCounter, cfg.windowMs, cfg.debounceMs, NUM_SENSORS,
          ip, tcp.connected() ? "true" : "false",
          (unsigned)((txHead - txTail + TXBUF_SLOTS) % TXBUF_SLOTS),
          mcpwmReady ? "mcpwm" : "isr");
}

static void sendShowConfig()
{
    emitf("{\"type\":\"config\",\"ssid\":\"%s\",\"pass\":\"%s\","
          "\"host\":\"%s\",\"port\":%u,\"lane\":%u,"
          "\"debounce_ms\":%u,\"window_ms\":%u,\"hybrid\":%d,\"debug\":%d,"
          "\"outlier_um\":%u,"
          "\"capture\":\"%s\","
          "\"static_ip\":%d,\"ip\":\"%s\",\"gateway\":\"%s\","
          "\"subnet\":\"%s\",\"dns\":\"%s\"}\n",
          cfg.ssid.c_str(),
          cfg.pass.length() ? "****" : "",
          cfg.host.c_str(), cfg.port, cfg.lane,
          cfg.debounceMs, cfg.windowMs, cfg.hybrid, cfg.debug,
          cfg.airOutlierUm,
          cfg.useMcpwm ? "mcpwm" : "isr",
          cfg.staticIP ? 1 : 0, cfg.ip.c_str(), cfg.gateway.c_str(),
          cfg.subnet.c_str(), cfg.dns.c_str());
}

// ---------------------------------------------------------------------------
// Rev 3.7: Positionsberechnung fuer den reinen Luftschall-Modus (HYBRID=2)
// ---------------------------------------------------------------------------
// Lokales Koordinatensystem der Abprallflaeche: Ursprung = Plattenzentrum,
// x nach rechts, y nach oben (aus Schuetzensicht), z senkrecht von der
// Platte weg Richtung Schuetze. Mic-Reihenfolge identisch zu AIR_PINS[]:
//   0 = GPIO25 = links unten   1 = GPIO26 = rechts unten
//   2 = GPIO27 = links oben    3 = GPIO14 = rechts oben

#define MIC_HALF_X       125.0f    // mm, horizontaler Abstand Mic<->Zentrum
#define MIC_HALF_Y       100.0f    // mm, vertikaler Abstand Mic<->Zentrum
#define MIC_STANDOFF      30.0f    // mm, Abstand Mic<->Plattenoberflaeche
#define SOUND_MM_PER_NS  0.000343f // 343 m/s (~20 C Raumtemperatur)
// Schwelle fuer "Mikrofon-Ausreisser" ist zur Laufzeit konfigurierbar:
// SET OUTLIER=<0.001mm>, siehe cfg.airOutlierUm (Default 5000 = 5.0mm).

static const float MIC_X[NUM_AIR] = { -MIC_HALF_X, +MIC_HALF_X, -MIC_HALF_X, +MIC_HALF_X };
static const float MIC_Y[NUM_AIR] = { -MIC_HALF_Y, -MIC_HALF_Y, +MIC_HALF_Y, +MIC_HALF_Y };

// Loest (x,y) aus 2 "Loese"-Mics relativ zu einer Referenz (Zeitnullpunkt)
// per Hyperbel-Trilateration (TDOA). Die Distanz Referenz<->Treffer wird
// als 3. Unbekannte mitgeloest (Linearisierung + quadratische Gleichung).
static bool solveAirPair(int ref, int a, int b, const int64_t tNs[NUM_AIR],
                          float *outX, float *outY, float *outD)
{
    const float Xr = MIC_X[ref], Yr = MIC_Y[ref];
    const float ra = (float)(tNs[a] - tNs[ref]) * SOUND_MM_PER_NS;
    const float rb = (float)(tNs[b] - tNs[ref]) * SOUND_MM_PER_NS;

    // Gleichung je Mic i: 2(Xi-Xr)x + 2(Yi-Yr)y + 2*ri*d = Ki-Kr-ri^2
    // (Ki = Xi^2+Yi^2; der Standoff Zi^2 kuerzt sich weg, da fuer alle
    // Mics identisch)
    const float A1 = 2.0f * (MIC_X[a] - Xr), B1 = 2.0f * (MIC_Y[a] - Yr), C1 = 2.0f * ra;
    const float D1 = (MIC_X[a]*MIC_X[a] + MIC_Y[a]*MIC_Y[a])
                    - (Xr*Xr + Yr*Yr) - ra*ra;
    const float A2 = 2.0f * (MIC_X[b] - Xr), B2 = 2.0f * (MIC_Y[b] - Yr), C2 = 2.0f * rb;
    const float D2 = (MIC_X[b]*MIC_X[b] + MIC_Y[b]*MIC_Y[b])
                    - (Xr*Xr + Yr*Yr) - rb*rb;

    // x = x0 + x1*d, y = y0 + y1*d (Cramer'sche Regel)
    const float det = A1*B2 - A2*B1;
    if (fabsf(det) < 1e-6f) return false;   // a, b kollinear mit ref

    const float x0 = (D1*B2 - D2*B1) / det;
    const float x1 = (C2*B1 - C1*B2) / det;
    const float y0 = (A1*D2 - A2*D1) / det;
    const float y1 = (A2*C1 - A1*C2) / det;

    // Einsetzen in d^2 = (x-Xr)^2 + (y-Yr)^2 + STANDOFF^2 -> quadratisch in d
    const float px = x0 - Xr, py = y0 - Yr;
    const float qa = 1.0f - x1*x1 - y1*y1;
    const float qb = -2.0f * (px*x1 + py*y1);
    const float qc = -(px*px + py*py + MIC_STANDOFF*MIC_STANDOFF);

    float d;
    if (fabsf(qa) < 1e-6f) {
        if (fabsf(qb) < 1e-6f) return false;
        d = -qc / qb;
    } else {
        const float disc = qb*qb - 4.0f*qa*qc;
        if (disc < 0.0f) return false;
        const float sq = sqrtf(disc);
        const float d1 = (-qb + sq) / (2.0f*qa);
        const float d2 = (-qb - sq) / (2.0f*qa);
        if (d1 > 0.0f && (d2 <= 0.0f || d1 < d2))  d = d1;   // kleinste
        else if (d2 > 0.0f)                         d = d2;   // positive Loesung
        else                                         return false;
    }
    if (d <= 0.0f) return false;

    *outX = x0 + x1*d;
    *outY = y0 + y1*d;
    *outD = d;
    return true;
}

// Bestimmt die Trefferposition aus den ersten Flanken der 4 Luftmikrofone.
//
// Mit nur 4 Mikrofonen gibt es genau 1 redundante Messung (4 Ankunfts-
// zeiten, aber nur 3 Unbekannte: x, y, Einschlagzeitpunkt) - das reicht
// fuer EINEN Plausibilitaetscheck, aber NICHT fuer eine zuverlaessige
// automatische Ausreisser-Erkennung in knappen Faellen (siehe Testschuss:
// zwei Kombinationen lagen beim Rest-Fehler nur ~0.1mm auseinander). Statt
// eine Erkennung vorzutaeuschen, die sie nicht sicher leisten kann, gibt
// die Funktion die beste (kleinster Rest-Fehler) der bis zu 4 moeglichen
// "1 Mic ausschliessen, mit den anderen 3 loesen, gegen das ausgeschlos-
// sene pruefen"-Kombinationen zurueck UND den zugehoerigen Rest-Fehler
// (outResidualMm). Ein grosser Rest-Fehler (>> wenige mm) zeigt an, dass
// die Messung fuer diesen Schuss nicht in sich konsistent war (z.B. eine
// Flanke war ein Echo statt Direktschall) - das muss dann der Empfaenger
// (Stand-PC/Bediener) beurteilen.
static bool solveAirPosition(const int64_t tNs[NUM_AIR], const bool seen[NUM_AIR],
                              float *outXmm, float *outYmm, float *outResidualMm)
{
    int all[NUM_AIR], nAll = 0;
    for (int i = 0; i < NUM_AIR; i++) if (seen[i]) all[nAll++] = i;
    if (nAll < 3) return false;

    if (nAll == 3) {
        // Keine Redundanz fuer einen Check - direkt mit allen 3 loesen.
        int ref = all[0];
        for (int k = 1; k < 3; k++) if (tNs[all[k]] < tNs[ref]) ref = all[k];
        int a = -1, b = -1;
        for (int k = 0; k < 3; k++) {
            if (all[k] == ref) continue;
            if (a < 0) a = all[k]; else b = all[k];
        }
        float d;
        *outResidualMm = 0.0f;
        return solveAirPair(ref, a, b, tNs, outXmm, outYmm, &d);
    }

    // nAll == 4: pro moeglichem Ausreisser (excl) die uebrigen 3 Mics
    // nutzen (kleinste Zeit darunter = Referenz), das ausgeschlossene Mic
    // zur Kontrolle verwenden. Kombination mit kleinstem Rest-Fehler gewinnt.
    bool  found = false;
    float bestResidual = -1.0f;
    for (int excl = 0; excl < NUM_AIR; excl++) {
        int cand[3], nCand = 0;
        for (int i = 0; i < NUM_AIR; i++) if (i != excl) cand[nCand++] = i;

        int ref = cand[0];
        for (int k = 1; k < 3; k++) if (tNs[cand[k]] < tNs[ref]) ref = cand[k];
        int a = -1, b = -1;
        for (int k = 0; k < 3; k++) {
            if (cand[k] == ref) continue;
            if (a < 0) a = cand[k]; else b = cand[k];
        }

        float x, y, d;
        if (!solveAirPair(ref, a, b, tNs, &x, &y, &d)) continue;

        const float dcCalc = sqrtf((x - MIC_X[excl])*(x - MIC_X[excl])
                                  + (y - MIC_Y[excl])*(y - MIC_Y[excl])
                                  + MIC_STANDOFF*MIC_STANDOFF);
        const float rc = (float)(tNs[excl] - tNs[ref]) * SOUND_MM_PER_NS;
        const float residual = fabsf(dcCalc - (d + rc));

        if (!found || residual < bestResidual) {
            found = true;
            bestResidual = residual;
            *outXmm = x;
            *outYmm = y;
        }
    }
    if (found) *outResidualMm = bestResidual;
    return found;
}

static void processShot()
{
    uint32_t localCC[NUM_SENSORS];
    bool     localSeen[NUM_SENSORS];
    uint32_t localFirstCC;
    uint64_t localFirstUs;

    uint32_t localAirCC[NUM_AIR][AIR_MAX_EDGES];
    uint8_t  localAirN[NUM_AIR];

    // Taktbasis der Stahl-Zeitstempel: MCPWM = APB (80 MHz),
    // ISR-Fallback = CPU-Zykluszaehler (240 MHz)
    uint32_t steelMHz;

    noInterrupts();
    if (mcpwmReady) {
        for (int i = 0; i < NUM_SENSORS; i++) {
            localCC[i]   = capTicks[i];
            localSeen[i] = capSeen[i];
        }
        localFirstCC = firstCapTicks;
        steelMHz     = apbMHz;
    } else {
        for (int i = 0; i < NUM_SENSORS; i++) {
            localCC[i]   = hitCC[i];
            localSeen[i] = hitSeen[i];
        }
        localFirstCC = firstCC;
        steelMHz     = cpuMHz;
    }
    uint32_t localFirstAirCC = firstAirCC;
    for (int i = 0; i < NUM_AIR; i++) {
        localAirN[i] = airCount[i];
        for (int e = 0; e < localAirN[i]; e++) localAirCC[i][e] = airCC[i][e];
    }
    localFirstUs = firstHitTimeUs;
    lockoutUntil = (uint64_t)esp_timer_get_time()
                 + (uint64_t)cfg.debounceMs * 1000ULL;
    interrupts();

    if (mcpwmReady) resetCaptureState();
    else            resetShotState();

    // ---- HYBRID=2: reiner Luftschall-Modus, keine Stahlsensoren beteiligt ----
    if (cfg.hybrid == MODE_AIR) {
        int airHits = 0;
        for (int i = 0; i < NUM_AIR; i++) if (localAirN[i] > 0) airHits++;

        if (airHits < 3) {
            if (cfg.debug >= 2) {
                emitf("{\"type\":\"reject\",\"reason\":\"only %d mic(s)\","
                      "\"hits\":%d}\n", airHits, airHits);
            }
            return;
        }

        shotCounter++;
        sequenceNo++;

        // Trefferposition aus den ersten Flanken der Mikrofone bestimmen
        int64_t airT0Ns[NUM_AIR];
        bool    airSeen[NUM_AIR];
        for (int i = 0; i < NUM_AIR; i++) {
            airSeen[i] = localAirN[i] > 0;
            if (airSeen[i]) {
                uint32_t dCC = localAirCC[i][0] - localFirstAirCC;   // wrap-sicher
                airT0Ns[i] = (int64_t)((uint64_t)dCC * 1000ULL / (uint64_t)cpuMHz);
            } else {
                airT0Ns[i] = 0;
            }
        }
        float posX = 0.0f, posY = 0.0f, posRes = 0.0f;
        bool  posOk = solveAirPosition(airT0Ns, airSeen, &posX, &posY, &posRes);
        long  xUm   = posOk ? lroundf(posX   * 1000.0f) : 0;
        long  yUm   = posOk ? lroundf(posY   * 1000.0f) : 0;
        long  resUm = posOk ? lroundf(posRes * 1000.0f) : 0;

        char line[640];
        int  n = snprintf(line, sizeof(line),
                          "{\"type\":\"shot\",\"seq\":%u,\"air_ns\":[", sequenceNo);
        for (int i = 0; i < NUM_AIR && n < (int)sizeof(line) - 32; i++) {
            if (i > 0) line[n++] = ',';
            line[n++] = '[';
            for (int e = 0; e < localAirN[i]
                         && n < (int)sizeof(line) - 24; e++) {
                if (e > 0) line[n++] = ',';
                // Luftkanaele referenzieren den ersten erfassten Mikrofon-Hit
                uint32_t dCC = localAirCC[i][e] - localFirstAirCC;   // wrap-sicher
                int64_t ns = (int64_t)((uint64_t)dCC * 1000ULL
                                       / (uint64_t)cpuMHz);
                n += snprintf(line + n, sizeof(line) - n, "%lld",
                              (long long)ns);
            }
            line[n++] = ']';
        }
        n += snprintf(line + n, sizeof(line) - n, "]");
        n += snprintf(line + n, sizeof(line) - n,
                      ",\"x_um\":%ld,\"y_um\":%ld,\"pos_res_um\":%ld,\"pos_valid\":%d",
                      xUm, yUm, resUm, posOk ? 1 : 0);
        snprintf(line + n, sizeof(line) - n, ",\"hits\":%d,\"ts\":%llu}\n",
                 airHits, (unsigned long long)(localFirstUs / 1000ULL));

        // DEBUG=0 (Default): nur sauber ermittelte Schuesse (kein Ausreisser,
        // Position bestimmbar) werden ausgegeben. DEBUG>=1: auch Schuesse mit
        // Mikrofon-Ausreisser bzw. nicht bestimmbarer Position.
        bool isClean = posOk && (unsigned long)resUm < cfg.airOutlierUm;
        if (isClean || cfg.debug >= 1) {
            emitLine(line);
        }
        return;
    }

    int hits = 0;
    for (int i = 0; i < NUM_SENSORS; i++) if (localSeen[i]) hits++;

    if (hits < 3) {
        if (cfg.debug >= 2) {
            emitf("{\"type\":\"reject\",\"reason\":\"only %d sensor(s)\","
                  "\"hits\":%d}\n", hits, hits);
        }
        return;
    }

    shotCounter++;
    sequenceNo++;

    // Differenzen in Nanosekunden. Unsigned 32-bit-Subtraktion ist
    // wrap-sicher, solange die Spanne < 2^32 Zyklen (~17,9 s) bleibt –
    // das Sammelfenster (<=50 ms) liegt weit darunter.
    // ns = zyklen * 1000 / MHz   (64-bit-Zwischenrechnung gegen Ueberlauf)
    int64_t tNs[NUM_SENSORS];
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (!localSeen[i]) {
            tNs[i] = -1;
        } else {
            uint32_t dCC = localCC[i] - localFirstCC;   // wrap-sicher
            tNs[i] = (int64_t)((uint64_t)dCC * 1000ULL / (uint64_t)steelMHz);
        }
    }

    // Telegramm: t_ns (Praezision) + t_us (Kompatibilitaet) + ggf. air_ns
    // Hybrid braucht mehr Platz: 4 Mics x 6 Edges -> eigener Puffer
    char line[640];
    int  n = snprintf(line, sizeof(line),
                      "{\"type\":\"shot\",\"seq\":%u,\"t_ns\":[", sequenceNo);
    for (int i = 0; i < NUM_SENSORS && n < (int)sizeof(line) - 40; i++) {
        if (i > 0) line[n++] = ',';
        n += snprintf(line + n, sizeof(line) - n, "%lld", (long long)tNs[i]);
    }
    n += snprintf(line + n, sizeof(line) - n, "],\"t_us\":[");
    for (int i = 0; i < NUM_SENSORS && n < (int)sizeof(line) - 24; i++) {
        if (i > 0) line[n++] = ',';
        if (tNs[i] < 0) {
            n += snprintf(line + n, sizeof(line) - n, "-1");
        } else {
            n += snprintf(line + n, sizeof(line) - n, "%lld",
                          (long long)((tNs[i] + 500) / 1000));
        }
    }
    n += snprintf(line + n, sizeof(line) - n, "]");

    if (cfg.hybrid) {
        n += snprintf(line + n, sizeof(line) - n, ",\"air_ns\":[");
        for (int i = 0; i < NUM_AIR && n < (int)sizeof(line) - 32; i++) {
            if (i > 0) line[n++] = ',';
            line[n++] = '[';
            for (int e = 0; e < localAirN[i]
                         && n < (int)sizeof(line) - 24; e++) {
                if (e > 0) line[n++] = ',';
                // Luftkanaele referenzieren IMMER den CPU-Nullpunkt
                uint32_t dCC = localAirCC[i][e] - localFirstAirCC; // wrap-sicher
                int64_t ns = (int64_t)((uint64_t)dCC * 1000ULL
                                       / (uint64_t)cpuMHz);
                n += snprintf(line + n, sizeof(line) - n, "%lld",
                              (long long)ns);
            }
            line[n++] = ']';
        }
        n += snprintf(line + n, sizeof(line) - n, "]");
    }

    snprintf(line + n, sizeof(line) - n, ",\"hits\":%d,\"ts\":%llu}\n",
             hits, (unsigned long long)(localFirstUs / 1000ULL));
    emitLine(line);
}

// ---------------------------------------------------------------------------
// Befehle – inkl. SET-Kommandos mit NVS-Persistenz
// ---------------------------------------------------------------------------

// Reiner Mensch-lesbarer Hilfetext (HELP oder ?), nur ueber Serial - nicht
// per emitLine/TCP, damit der JSON-Zeilenstrom fuer den Stand-PC nicht mit
// Kommentarzeilen vermischt wird (analog zu den "# ..."-Logzeilen in setup()).
static void sendHelp()
{
    static const char *lines[] = {
        "# Verfuegbare SET-Parameter:",
        "#   SET SSID=<text>          WLAN-Name (leer = WLAN aus, Reboot noetig)",
        "#   SET PASS=<text>          WLAN-Passwort (Reboot noetig)",
        "#   SET HOST=<ip/host>       Ziel-Host des Stand-PC (Reboot noetig)",
        "#   SET PORT=<1-65535>       TCP-Port des Stand-PC (Reboot noetig)",
        "#   SET LANE=<1-999>         Bahnnummer",
        "#   SET DEBOUNCE=<10-5000>   Sperrzeit nach Schuss in ms",
        "#   SET WINDOW=<1-50>        Sammelfenster in ms",
        "#   SET HYBRID=<0|1|2>       0=nur Stahl 1=Stahl+Luft 2=nur Luftschall",
        "#   SET DEBUG=<0|1|2>        Ausgabe-Filter: 0=nur saubere Schuesse,",
        "#                            1=+Mikrofon-Ausreisser, 2=+Reject (wenig Hits)",
        "#   SET OUTLIER=<0-500000>   Ausreisser-Schwelle in 0.001mm (Default 5000)",
        "#   SET CAPTURE=<mcpwm|isr>  Stahl-Erfassung (Reboot noetig)",
        "#   SET STATIC=<0|1>         Statische IP an/aus (Reboot noetig)",
        "#   SET IP=<ip>              Statische IP-Adresse (Reboot noetig)",
        "#   SET GW=<ip>              Gateway, auch: GATEWAY (Reboot noetig)",
        "#   SET SUBNET=<mask>        Subnetzmaske (Reboot noetig)",
        "#   SET DNS=<ip>             DNS-Server, leer = Gateway (Reboot noetig)",
        "# Weitere Befehle: SHOW  STATUS  PING  RESET  REBOOT  FACTORY  HELP/?",
    };
    for (size_t i = 0; i < sizeof(lines) / sizeof(lines[0]); i++) {
        Serial.println(lines[i]);
    }
}

// SET-Werte sind case-SENSITIV (SSID/Passwort!), deshalb wird hier nur das
// Schluesselwort normalisiert, nicht der Wert.
static bool handleSet(const String &raw)
{
    int eq = raw.indexOf('=');
    if (eq < 5) return false;                  // "SET x=" Minimum
    String key = raw.substring(4, eq);
    String val = raw.substring(eq + 1);
    key.trim(); key.toUpperCase();
    val.trim();

    if (key == "SSID") {
        cfg.ssid = val; saveVal<String>("ssid", val);
        emitf("{\"type\":\"ok\",\"set\":\"ssid\",\"reboot_required\":true}\n");
    } else if (key == "PASS") {
        cfg.pass = val; saveVal<String>("pass", val);
        emitf("{\"type\":\"ok\",\"set\":\"pass\",\"reboot_required\":true}\n");
    } else if (key == "HOST") {
        cfg.host = val; saveVal<String>("host", val);
        emitf("{\"type\":\"ok\",\"set\":\"host\",\"reboot_required\":true}\n");
    } else if (key == "PORT") {
        long p = val.toInt();
        if (p < 1 || p > 65535) { emitLine("{\"type\":\"error\",\"msg\":\"port 1-65535\"}\n"); return true; }
        cfg.port = (uint16_t)p; saveVal<uint16_t>("port", cfg.port);
        emitf("{\"type\":\"ok\",\"set\":\"port\",\"reboot_required\":true}\n");
    } else if (key == "LANE") {
        long l = val.toInt();
        if (l < 1 || l > 999) { emitLine("{\"type\":\"error\",\"msg\":\"lane 1-999\"}\n"); return true; }
        cfg.lane = (uint16_t)l; saveVal<uint16_t>("lane", cfg.lane);
        emitf("{\"type\":\"ok\",\"set\":\"lane\",\"value\":%u}\n", cfg.lane);
    } else if (key == "DEBOUNCE") {
        long v = val.toInt();
        if (v < 10 || v > 5000) { emitLine("{\"type\":\"error\",\"msg\":\"debounce 10-5000\"}\n"); return true; }
        cfg.debounceMs = (uint32_t)v; saveVal<uint32_t>("debounce", cfg.debounceMs);
        emitf("{\"type\":\"ok\",\"set\":\"debounce\",\"value\":%u}\n", cfg.debounceMs);
    } else if (key == "HYBRID") {
        long v = val.toInt();
        if (v < 0 || v > 2) { emitLine("{\"type\":\"error\",\"msg\":\"hybrid 0|1|2\"}\n"); return true; }
        cfg.hybrid = (uint8_t)v;
        saveVal<uint8_t>("hybrid", cfg.hybrid);
        // Modus wirkt sofort: laufendes Sammelfenster verwerfen
        if (mcpwmReady) resetCaptureState(); else resetShotState();
        emitf("{\"type\":\"ok\",\"set\":\"hybrid\",\"value\":%d}\n", cfg.hybrid);
    } else if (key == "DEBUG") {
        long v = val.toInt();
        if (v < 0 || v > 2) { emitLine("{\"type\":\"error\",\"msg\":\"debug 0|1|2\"}\n"); return true; }
        cfg.debug = (uint8_t)v;
        saveVal<uint8_t>("debug", cfg.debug);
        emitf("{\"type\":\"ok\",\"set\":\"debug\",\"value\":%d}\n", cfg.debug);
    } else if (key == "OUTLIER") {
        long v = val.toInt();
        if (v < 0 || v > 500000) { emitLine("{\"type\":\"error\",\"msg\":\"outlier 0-500000\"}\n"); return true; }
        cfg.airOutlierUm = (uint32_t)v;
        saveVal<uint32_t>("outlier", cfg.airOutlierUm);
        emitf("{\"type\":\"ok\",\"set\":\"outlier\",\"value\":%u}\n", cfg.airOutlierUm);
    } else if (key == "WINDOW") {
        long v = val.toInt();
        if (v < 1 || v > 50) { emitLine("{\"type\":\"error\",\"msg\":\"window 1-50\"}\n"); return true; }
        cfg.windowMs = (uint32_t)v; saveVal<uint32_t>("window", cfg.windowMs);
        emitf("{\"type\":\"ok\",\"set\":\"window\",\"value\":%u}\n", cfg.windowMs);
    } else if (key == "CAPTURE") {
        // mcpwm = Hardware-Capture (Default), isr = Zykluszaehler-Fallback.
        // Wirkt erst nach REBOOT (Capture-Peripherie wird in setup() init.).
        String v = val; v.toLowerCase();
        if (v == "mcpwm") {
            cfg.useMcpwm = true;
        } else if (v == "isr") {
            cfg.useMcpwm = false;
        } else {
            emitLine("{\"type\":\"error\",\"msg\":\"capture mcpwm|isr\"}\n");
            return true;
        }
        saveVal<bool>("mcpwm", cfg.useMcpwm);
        emitf("{\"type\":\"ok\",\"set\":\"capture\",\"value\":\"%s\","
              "\"reboot_required\":true}\n", cfg.useMcpwm ? "mcpwm" : "isr");
    } else if (key == "STATIC") {
        long v = val.toInt();
        if (v != 0 && v != 1) { emitLine("{\"type\":\"error\",\"msg\":\"static 0|1\"}\n"); return true; }
        cfg.staticIP = (v == 1);
        saveVal<bool>("static_ip", cfg.staticIP);
        emitf("{\"type\":\"ok\",\"set\":\"static_ip\",\"value\":%d,\"reboot_required\":true}\n",
              cfg.staticIP ? 1 : 0);
    } else if (key == "IP") {
        IPAddress tmp;
        if (!tmp.fromString(val)) { emitLine("{\"type\":\"error\",\"msg\":\"invalid IP address\"}\n"); return true; }
        cfg.ip = val; saveVal<String>("ip", val);
        emitf("{\"type\":\"ok\",\"set\":\"ip\",\"value\":\"%s\",\"reboot_required\":true}\n", val.c_str());
    } else if (key == "GW" || key == "GATEWAY") {
        IPAddress tmp;
        if (!tmp.fromString(val)) { emitLine("{\"type\":\"error\",\"msg\":\"invalid gateway address\"}\n"); return true; }
        cfg.gateway = val; saveVal<String>("gateway", val);
        emitf("{\"type\":\"ok\",\"set\":\"gateway\",\"value\":\"%s\",\"reboot_required\":true}\n", val.c_str());
    } else if (key == "SUBNET") {
        IPAddress tmp;
        if (!tmp.fromString(val)) { emitLine("{\"type\":\"error\",\"msg\":\"invalid subnet mask\"}\n"); return true; }
        cfg.subnet = val; saveVal<String>("subnet", val);
        emitf("{\"type\":\"ok\",\"set\":\"subnet\",\"value\":\"%s\",\"reboot_required\":true}\n", val.c_str());
    } else if (key == "DNS") {
        if (val.length() > 0) {
            IPAddress tmp;
            if (!tmp.fromString(val)) { emitLine("{\"type\":\"error\",\"msg\":\"invalid DNS address\"}\n"); return true; }
        }
        cfg.dns = val; saveVal<String>("dns", val);
        emitf("{\"type\":\"ok\",\"set\":\"dns\",\"value\":\"%s\",\"reboot_required\":true}\n", val.c_str());
    } else {
        emitLine("{\"type\":\"error\",\"msg\":\"unknown key\"}\n");
    }
    return true;
}

static void handleCommand(const String &rawCmd)
{
    String raw = rawCmd;
    raw.trim();
    if (raw.length() == 0) return;

    // Nur fuer Schluesselwort-Vergleich eine Gross-Kopie anlegen
    String upper = raw;
    upper.toUpperCase();

    if (upper.startsWith("SET ")) {
        handleSet(raw);                       // Original wg. case-sensitiver Werte
        return;
    }
    if (upper == "SHOW")    { sendShowConfig(); return; }
    if (upper == "HELP" || upper == "?") { sendHelp(); return; }
    if (upper == "PING")    { emitLine("{\"type\":\"pong\"}\n"); return; }
    if (upper == "STATUS")  { sendStatus(); return; }
    if (upper == "REBOOT")  {
        emitLine("{\"type\":\"ok\",\"cmd\":\"reboot\"}\n");
        delay(100);
        ESP.restart();
    }
    if (upper == "FACTORY") {
        prefs.begin(NVS_NS, false);
        prefs.clear();
        prefs.end();
        emitLine("{\"type\":\"ok\",\"cmd\":\"factory\",\"reboot_required\":true}\n");
        return;
    }
    if (upper == "RESET") {
        shotCounter = 0;
        sequenceNo  = 0;
        resetShotState();
        emitLine("{\"type\":\"ok\",\"cmd\":\"reset\"}\n");
        return;
    }
    // Kurzformen aus Rev 3.0/3.1 – jetzt ebenfalls persistent:
    if (upper.startsWith("DEBOUNCE=")) { handleSet("SET " + raw); return; }
    if (upper.startsWith("WINDOW="))   { handleSet("SET " + raw); return; }

    emitLine("{\"type\":\"error\",\"msg\":\"unknown command\"}\n");
}

// echo: Zeichen sofort auf s zurückschreiben (fuer Terminals ohne lokales
// Echo, z.B. MobaXterm seriell). Beim TCP-Kanal AUS, da die Gegenstelle
// dort reines JSON erwartet.
static void pollCommands(Stream &s, String &buf, bool echo)
{
    while (s.available() > 0) {
        char c = (char)s.read();
        if (echo) s.write(c);
        if (c == '\n' || c == '\r') {
            if (buf.length() > 0) {
                handleCommand(buf);
                buf = "";
            }
        } else if (buf.length() < 96) {
            buf += c;
        }
    }
}

// ---------------------------------------------------------------------------
// WLAN / TCP Verwaltung
// ---------------------------------------------------------------------------

static void maintainNetwork()
{
    if (!wifiEnabled) return;

    if (WiFi.status() != WL_CONNECTED) {
        if (tcp.connected()) tcp.stop();
        return;
    }
    if (!tcp.connected()) {
        uint32_t now = millis();
        if (now < nextConnectAttemptMs) return;

        Serial.printf("# TCP connect %s:%u ...\n", cfg.host.c_str(), cfg.port);
        if (tcp.connect(cfg.host.c_str(), cfg.port, 2000)) {
            tcp.setNoDelay(true);
            connectBackoffMs = 1000;
            Serial.println("# TCP verbunden");
            sendStatus();
        } else {
            nextConnectAttemptMs = now + connectBackoffMs;
            if (connectBackoffMs < 15000) connectBackoffMs *= 2;
        }
    }
}

// ---------------------------------------------------------------------------
// Setup / Loop
// ---------------------------------------------------------------------------

void setup()
{
    Serial.begin(SERIAL_BAUD);
    loadConfig();

    // CPU-Takt fuer die Zyklen->ns-Umrechnung ermitteln
    cpuMHz = getCpuFrequencyMhz();
    if (cpuMHz < 80) cpuMHz = 240;   // Fallback

    // --- Stahl-Sensoren: MCPWM-Hardware-Capture ODER ISR-Fallback ---
    mcpwmReady = false;
    if (cfg.useMcpwm) {
        mcpwmReady = mcpwmInit();
        if (mcpwmReady)
            Serial.printf("# Stahl-Capture: MCPWM (Hardware, %u MHz APB)\n",
                          apbMHz);
        else
            Serial.println("# MCPWM nicht verfuegbar – Fallback auf ISR");
    }
    if (!mcpwmReady) {
        // Alle ISRs auf gleichem Core (setup laeuft auf einem Core), damit
        // alle Sensoren denselben Zykluszaehler benutzen.
        for (uint32_t i = 0; i < NUM_SENSORS; i++) {
            pinMode(SENSOR_PINS[i], INPUT);
            attachInterruptArg(digitalPinToInterrupt(SENSOR_PINS[i]),
                               sensorISR, (void *)i, RISING);
        }
        Serial.println("# Stahl-Capture: ISR (CPU-Zykluszaehler)");
    }
    // Luft-Mikrofone laufen IMMER per ISR (Capture-Kanaele sind fuer Stahl
    // verplant). ISR prueft cfg.hybrid -> SET HYBRID=0|1|2 wirkt sofort.
    for (uint32_t i = 0; i < NUM_AIR; i++) {
        pinMode(AIR_PINS[i], INPUT_PULLUP);
        attachInterruptArg(digitalPinToInterrupt(AIR_PINS[i]),
                           airISR, (void *)i, RISING);
    }
    resetShotState();

    wifiEnabled = cfg.ssid.length() > 0;
    if (wifiEnabled) {
        WiFi.mode(WIFI_STA);
        WiFi.setAutoReconnect(true);
        WiFi.setSleep(false);

        if (cfg.staticIP && cfg.ip.length() > 0 && cfg.gateway.length() > 0) {
            IPAddress ipAddr, gwAddr, snAddr, dnsAddr;
            bool ok = ipAddr.fromString(cfg.ip)
                   && gwAddr.fromString(cfg.gateway)
                   && snAddr.fromString(cfg.subnet.length() > 0
                                        ? cfg.subnet : "255.255.255.0");
            if (ok) {
                if (cfg.dns.length() > 0) dnsAddr.fromString(cfg.dns);
                else                      dnsAddr = gwAddr;
                WiFi.config(ipAddr, gwAddr, snAddr, dnsAddr);
                Serial.printf("# Statische IP: %s  GW: %s  SM: %s  DNS: %s\n",
                              cfg.ip.c_str(), cfg.gateway.c_str(),
                              cfg.subnet.c_str(),
                              cfg.dns.length() > 0 ? cfg.dns.c_str()
                                                   : cfg.gateway.c_str());
            } else {
                Serial.println("# Warnung: Ung. statische IP-Konfig – DHCP wird verwendet");
            }
        } else if (cfg.staticIP) {
            Serial.println("# Warnung: STATIC=1 aber IP/GW fehlen – DHCP wird verwendet");
        }

        WiFi.begin(cfg.ssid.c_str(), cfg.pass.c_str());
        Serial.printf("# WLAN verbinde mit '%s' ...\n", cfg.ssid.c_str());
    } else {
        Serial.println("# WLAN aus (SSID leer) – SET SSID=... zum Aktivieren");
    }

    Serial.println();
    sendStatus();
    emitLine("{\"type\":\"ready\"}\n");
}

void loop()
{
    if (shotInProgress) {
        const uint64_t now = (uint64_t)esp_timer_get_time();
        if (now - firstHitTimeUs >= (uint64_t)cfg.windowMs * 1000ULL) {
            processShot();
        }
    }

    maintainNetwork();

    static String serialBuf, tcpBuf;
    pollCommands(Serial, serialBuf, true);
    if (tcp.connected()) {
        pollCommands(tcp, tcpBuf, false);
    }
}
