# Konzept: Erweiterte ESP32 ↔ Stand-PC Interaktion

Stand: 2026-09-06, bezogen auf Firmware Rev 4.8.10 (`src/schiessstand_firmware.ino`).

> **Umsetzungsstand (Rev 4.9.0):** Die Bausteine A (Konfiguration, ohnehin
> schon vorhanden), C (`ACTION LIGHT`/`ACTION TARGETCHANGE`), D (`CAL IMPORT`
> + passives Mitschneiden von `cal`/`"done"`) sowie das Netzwerk-Sicherheitsnetz
> aus Baustein B (`NET CONFIRM`/`NET STATUS`, Abschnitt 5.2/9 Schritt 1-3 und 6)
> sind implementiert, ebenso die Geraete-ID (`mac`-Feld, Abschnitt 2) und die
> Korrelations-ID (Abschnitt 3.1). Fuer den **verbindlichen Ist-Zustand des
> Protokolls** gilt ab jetzt `docs/protokoll-referenz.md`, nicht mehr dieses
> Dokument - hier unten steht weiterhin die urspruengliche Planung/Begruendung,
> als Nachschlagewerk fuer die getroffenen Entscheidungen. Noch offen: die
> serverseitige Umsetzung (Abschnitt 8, liegt ausserhalb dieser Firmware) und
> die optionale `AUTH`-Token-Absicherung (Abschnitt 3.2/9 Schritt 7).

## 0. Ausgangslage (was heute schon da ist)

Wichtig fuer dieses Konzept: ein guter Teil der Infrastruktur existiert bereits, auch
wenn sie bisher primaer fuer manuelle Bedienung (Serial-Terminal) gedacht war.

- Der ESP32 ist **TCP-Client** (`tcp.connect(cfg.host, cfg.port, ...)`,
  `maintainNetwork()`), der Stand-PC ist Server. Der ESP32 initiiert die Verbindung
  und haelt sie mit Backoff aufrecht - das erfuellt bereits die Grundforderung
  "Kommunikation geht vom ESP32 aus".
- Auf derselben TCP-Verbindung liest der ESP32 aber schon jetzt **Kommandos vom
  Stand-PC** (`pollCommands(tcp, tcpBuf, false)` in `loop()`), zeilenbasiert, gleiches
  Format wie am Serial-Terminal: `SET key=value`, `SHOW`, `SHOWNET`, `CAL ...`,
  `PIN ...`, `REBOOT`, `FACTORY`, `RESET`, `PING`, `STATUS`.
- Antworten sind einzeilige JSON-Objekte (`{"type":"ok"|"error"|"config"|"confignet"|...}`).
- Alle Handler sind bewusst **nicht-blockierend** (`servicePaperStepper()`,
  Kommentar bei `PAPER_STEP_PIN` zu genau diesem Punkt) - Schusserfassung, TDOA-ISRs
  und Netzwerk laufen unabhaengig weiter. Das ist die wichtigste bestehende
  Design-Regel und muss fuer alles Neue erhalten bleiben.
- Konfigurationsaenderungen (`SET ...`) sind bereits persistent (NVS/`Preferences`)
  und werden per `SHOW`/`SHOWNET` vollstaendig abfragbar gemacht.
- Netzwerkkonfiguration (`SET SSID/PASS/HOST/PORT/STATIC/IP/GATEWAY/SUBNET/DNS`)
  ist bereits fernsteuerbar, verlangt aber `reboot_required` und hat **keinerlei
  Absicherung** gegen Fehlkonfiguration (siehe 5.2).
- Kalibrierung (`CAL START/ABORT/STATUS/RESET`) landet nur im NVS des jeweiligen
  Geraets. Bei Tausch der Platine oder `FACTORY` ist sie weg. Es gibt schon ein
  `"type":"cal","state":"done","offsets_ns":[...]`-Telegramm gefolgt von
  `sendShowConfig()` (Zeile ~1910-1922) - der Stand-PC koennte das heute schon
  mitschneiden, tut es aber (vermutlich) nicht.
- Es gibt **keine stabile Geraete-ID**. `cfg.lane` ist eine vom Nutzer vergebene
  Bahnnummer, keine Hardware-Identitaet. Fuer serverseitige Zuordnung (welche
  Kalibrierung gehoert zu welchem physischen ESP32?) fehlt das.
- Es gibt **keine Authentifizierung** auf dem TCP-Kanal. Jeder im selben LAN/WLAN,
  der `cfg.host:cfg.port` erreicht, kann bereits heute `SET`, `FACTORY`, `REBOOT`
  senden.

Die vier gewuenschten Themen sind also unterschiedlich weit von "fertig" entfernt:

| Thema | Aufwand |
|---|---|
| Konfiguration remote aendern | **fast fertig** - Protokoll formalisieren, Server-seitig nutzen |
| Netzwerkkonfiguration remote aendern | Grundfunktion da, **fehlt: Absicherung/Rollback** |
| Aktionen (LED, Scheibenwechsel) | **fehlt komplett**, aber kleine Erweiterung |
| Kalibrierung serverseitig speichern | Daten fliessen teilweise schon, **fehlt: Geraete-ID, atomarer Export/Import** |

## 1. Leitprinzipien fuer alles Neue

1. **ESP32 bleibt aktiver Teil, Stand-PC reagiert/fragt nur an, wenn der Kanal
   offen ist.** Keine zweite Server-Rolle auf dem ESP32 (kein eigener HTTP/REST-
   Server dort) - zusaetzlicher Netzwerk-Stack, RAM-Verbrauch und Angriffsflaeche
   ohne Nutzen, da die bestehende TCP-Verbindung schon bidirektional ist.
2. **Ein Kanal, ein Format.** Weiter zeilenbasiertes Text/JSON auf derselben
   TCP-Verbindung. Kein zweiter Port, kein MQTT/WebSocket-Zusatzstack - der
   Stand-PC ist ohnehin schon Server dieser einen Verbindung.
3. **Kommandoverarbeitung bleibt nicht-blockierend.** Jeder neue Befehl muss
   in `loop()` in Mikrosekunden zurueckkehren. Alles, was Zeit braucht
   (Papiervorschub, WLAN-Reconnect), laeuft weiter ueber die bestehenden
   State-Machines (`servicePaperStepper()`, `maintainNetwork()`) statt ueber
   `delay()`.
4. **Rueckwaertskompatibel.** Bestehende `SET`/`SHOW`/`CAL`-Befehle bleiben wie
   sie sind (Tools wie `tools/calibrate_mics.py` und die manuelle Serial-Bedienung
   duerfen nicht brechen). Neues kommt als zusaetzliche Befehle/Felder hinzu.
5. **Jede Fernaenderung, die das Geraet unerreichbar machen koennte
   (Netzwerk!), braucht eine Rueckfallebene.** Sonst genuegt ein Tippfehler in
   der IP, um zur Bahn laufen und den ESP32 per USB neu flashen/konfigurieren zu
   muessen.

## 2. Geraete-Identitaet einfuehren

Voraussetzung fuer "Kalibrierung auf dem Server speichern" und generell fuer eine
saubere Server-Datenbank: eine stabile ID pro physischem Geraet, unabhaengig von
Bahnnummer oder IP.

- `WiFi.macAddress()` als `device_id` in **jedes** Telegramm aufnehmen, das der
  Server ohnehin schon empfaengt: `status`, `config`, `confignet`, `cal`. Kostet
  ~20 Byte pro Zeile, ist bereits vorhanden (kein neuer Zustand noetig).
- `cfg.lane` bleibt die *logische* Zuordnung (welche Bahn), `device_id` die
  *physische*. Der Stand-PC fuehrt serverseitig eine Tabelle `device_id -> lane,
  config, netconfig, calibration, last_seen`.

Das ist die kleinste, folgenreichste Aenderung in diesem Konzept - ohne sie ist
jede serverseitige Speicherung (Kalibrierung, Config-Historie) nur an die
zufaellig aktuelle Bahnnummer gebunden statt an die Hardware.

## 3. Protokoll-Erweiterung: Korrelation + optionale Authentifizierung

### 3.1 Korrelations-ID fuer Befehle

Aktuell kann der Stand-PC bei mehreren schnell hintereinander gesendeten
Befehlen die `ok`/`error`-Antworten nicht sicher zuordnen (keine ID im
Request/Response). Vorschlag: optionales Suffix, das im bestehenden
Text-Protokoll nicht stoert:

```
SET LANE=2 #7
{"type":"ok","set":"lane","value":2,"corr":7}
```

`handleCommand()` trennt `#<id>` vom Rest ab, bevor der Befehl geparst wird, und
jede `emitf`/`emitLine`-Antwort haengt (wenn vorhanden) `"corr":<id>` an. Minimal-
invasiv, bricht nichts Bestehendes (Suffix ist optional).

### 3.2 Authentifizierung fuer schreibende Befehle (empfohlen, nicht Kernthema)

Da jetzt Netzwerkkonfiguration und Aktionen (nicht nur Diagnose) fernsteuerbar
werden, lohnt sich ein einfacher Schutz gegen "irgendwer im WLAN sendet
`FACTORY`":

- Ein Shared Secret (`SET AUTHTOKEN=...`, nur ueber Serial/USB initial setzbar,
  danach nur mit gueltigem Token aenderbar) plus ein `AUTH <token>`-Befehl, der
  die aktuelle TCP-Session freischaltet.
- Lesende Befehle (`SHOW`, `SHOWNET`, `STATUS`, `PING`) bleiben ohne Auth erlaubt
  (Diagnose per Serial/Terminal soll nicht komplizierter werden).
- Schreibende Befehle (`SET`, `CAL`, `ACTION`, `FACTORY`, `REBOOT`) verlangen
  vorher `AUTH` auf derselben Verbindung, sonst `{"type":"error","msg":"auth
  required"}`.
- Bewusst kein TLS (Zertifikatsverwaltung auf dem ESP32 ist fuer ein LAN-only-
  Szenario auf einem Schiessstand unverhaeltnismaessig) - ein Shared-Secret-Gate
  reicht fuer "kein versehentliches/fremdes Kommando", nicht fuer ein
  Hochsicherheits-Netz.

Das ist optional und kann auch nach den drei fachlichen Themen umgesetzt werden -
aber je mehr fernsteuerbare Aktionen (v.a. Netzwerkaenderung) dazukommen, desto
mehr lohnt es sich.

## 4. Baustein A: Konfiguration remote aendern

Bereits vollstaendig ueber `SET <key>=<value>` + `SHOW` vorhanden. Fuer den
Stand-PC braucht es nur noch die Nutzungskonvention:

1. Beim Verbindungsaufbau sendet der ESP32 ohnehin `sendStatus()` +
   `{"type":"ready"}` (siehe `setup()`/`maintainNetwork()`). Stand-PC antwortet
   darauf mit `SHOW` + `SHOWNET`, um seinen lokalen Stand zu synchronisieren,
   statt eigene Annahmen ueber den Geraetezustand zu treffen.
2. Jede `SET`-Aenderung vom Stand-PC aus loest bereits eine `{"type":"ok",
   "set":...}`-Bestaetigung aus - der Stand-PC sollte diese abwarten
   (per Korrelations-ID aus 3.1), bevor er die naechste Aenderung schickt,
   statt mehrere `SET` "blind" hintereinanderzufeuern.
3. Kein neuer Firmware-Code noetig ausser 3.1 (Korrelations-ID) und 2
   (`device_id` in `config`-Telegramm).

## 5. Baustein B: Netzwerkkonfiguration remote aendern

### 5.1 Was schon geht
`SET SSID/PASS/HOST/PORT/STATIC/IP/GATEWAY/SUBNET/DNS` speichert bereits ins NVS
und markiert `reboot_required`. Ein `REBOOT`-Befehl direkt danach uebernimmt es.

### 5.2 Das fehlende Sicherheitsnetz
Problem: Wenn die neue SSID/IP falsch ist, verbindet sich der ESP32 nach dem
Reboot nicht mehr - der Stand-PC hat keinen Kanal mehr, um es zu korrigieren.
Klassisches "Ich sitze auf dem Ast, den ich gerade absaege"-Problem bei
Fernkonfiguration von Netzwerkgeraeten. Loesung (wie bei Consumer-Routern
"Bestaetigen Sie die neue Einstellung innerhalb von 30s"):

1. Vor dem Schreiben der neuen Netz-Werte: aktuelle Werte in einen zweiten
   NVS-Key-Satz kopieren (`ssid_prev`, `pass_prev`, `host_prev`, ... bzw. ein
   `net_pending=true`-Flag mit Timestamp).
2. Nach `REBOOT` mit den neuen Werten: ein Watchdog-Zeitfenster
   (z.B. 3 Minuten, `NET_CONFIRM_TIMEOUT_MS`) startet. Bleibt in dieser Zeit
   *keine* TCP-Verbindung zum (neuen) `cfg.host:cfg.port` stehen (WLAN verbindet
   gar nicht, oder verbindet, aber der Host ist nicht erreichbar), schreibt der
   ESP32 die `_prev`-Werte zurueck und rebootet erneut - Ruecksprung auf den
   letzten bekannt-funktionierenden Zustand.
3. Steht die Verbindung, sendet der Stand-PC (der die neuen Werte ja selbst
   vorgegeben hat) einen expliziten Bestaetigungsbefehl, z.B. `NET CONFIRM`.
   Erst der loescht die `_prev`-Werte/das `net_pending`-Flag endgueltig. Ohne
   explizite Bestaetigung bleibt die Zuruecksetzung "scharf", auch wenn die
   Verbindung nur kurz zufaellig zustande kam.
4. Sonderfall SSID/Passwort falsch (WLAN verbindet nie): Watchdog laeuft nach
   `NET_CONFIRM_TIMEOUT_MS` auch ohne jede Verbindung ab - reine
   Boot-Zeit-Messung (`millis()` seit Boot mit `net_pending=true`), kein
   TCP-Ereignis noetig, um zurueckzurollen.

Das ist der wichtigste neue Firmware-Baustein in diesem Konzept, weil er die
einzige Fernaenderung absichert, die das Geraet unerreichbar machen kann.
Aufwand: ueberschaubar (ein paar zusaetzliche NVS-Keys + eine Zeitpruefung in
`setup()`/`loop()`), aber sicherheitskritisch fuer den Praxiseinsatz.

## 6. Baustein C: Aktionen (LED, Scheibenwechsel)

Neuer Befehls-Namensraum `ACTION <name>[=<wert>]`, bewusst getrennt von `SET`
(Aktionen sind Ereignisse, keine persistente Konfiguration).

### 6.1 LED ein/aus
Aktuell steuert `serviceLight()` `LIGHT_PIN` vollautomatisch anhand von
`tcp.connected()` (State-Machine `LP_WAIT_CONN/LP_OFF_WAIT/LP_BLINK/LP_OFF_DONE`).
Fuer eine manuelle Aktion braucht es einen Override, den `serviceLight()` vor
seiner eigenen Logik prueft:

```
static enum { LIGHT_AUTO, LIGHT_FORCE_ON, LIGHT_FORCE_OFF } lightOverride = LIGHT_AUTO;
```

- `ACTION LIGHT=ON` / `ACTION LIGHT=OFF` setzt `lightOverride`, schaltet
  `LIGHT_PIN` sofort, `serviceLight()` gibt bei `lightOverride != LIGHT_AUTO`
  sofort zurueck (automatische Verbindungsanzeige pausiert).
- `ACTION LIGHT=AUTO` gibt die Kontrolle an `serviceLight()` zurueck.
- Nicht NVS-persistent (wie `SET TESTMODE`) - nach Reboot wieder `LIGHT_AUTO`,
  damit die Verbindungsanzeige nicht dauerhaft "kaputt" bleibt, falls jemand
  vergisst, zurueckzuschalten.
- Antwort: `{"type":"ok","action":"light","state":"on"}`.

### 6.2 Scheibenwechsel
"Scheibenwechsel" = Papier ein Stueck weiterziehen fuer eine frische Flaeche,
unabhaengig von einem echten Schuss. Es gibt strukturell schon fast genau das:
`TESTSHOOTPAPER` (loest `startPaperFeed()` aus, ohne `shotCounter` zu erhoehen
oder ein `shot`-Telegramm zu senden). Der einzige Unterschied ist die Absicht
("Diagnose" vs. "produktiver Bedienbefehl") - technisch reicht ein zweiter,
sprechenderer Einstieg auf denselben Code:

```
ACTION TARGETCHANGE  →  identischer Codepfad wie TESTSHOOTPAPER
{"type":"ok","action":"targetchange"}          // sofort
{"type":"paperfeed","state":"done"}            // nach Abschluss, aus servicePaperStepper()
```

`TESTSHOOTPAPER` bleibt zusaetzlich fuer Diagnose bestehen (kein Grund, es zu
entfernen) - `ACTION TARGETCHANGE` ist der bewusst benannte Bedienbefehl fuer
den Stand-PC. Ein `{"type":"error","msg":"paper feed busy"}` (wie schon bei
`TESTSHOOTPAPER`) verhindert ueberlappende Anforderungen.

Erweiterbar fuer spaetere Aktionen (z.B. `ACTION PAPERJOG=FWD|REV|STOP` als
Fernsteuerung des Einfaedel-Modus, falls gewuenscht) - der `ACTION`-Namensraum
ist dafuer gedacht, offen zu bleiben.

## 7. Baustein D: Kalibrierung serverseitig speichern

Zwei Faelle, die beide abgedeckt werden sollten:

### 7.1 Passiv mitschneiden (kein neuer Firmware-Code noetig)
Der ESP32 sendet bei `CAL START` bereits laufend Fortschritt
(`"state":"waiting"`), am Ende `"state":"done"` mit `offsets_ns` **und**
anschliessend automatisch die volle `sendShowConfig()` (enthaelt auch
`sound_mps`, die Standoff-Werte etc.). Der Stand-PC muss dafuer nur:

- auf `"type":"cal","state":"done"` **und** das darauf folgende
  `"type":"config"` lauschen (beide auf derselben Verbindung, direkt
  nacheinander),
- zusammen mit der `device_id` (siehe Abschnitt 2) als
  Kalibrierungs-Snapshot fuer dieses Geraet speichern (z.B. Tabelle
  `calibration_history(device_id, timestamp, mic_offsets_ns[], sound_mps, ...)`).

Das ist die eigentliche Antwort auf "Kalibrierung soll auf dem Server
gespeichert werden koennen" - **passives Mitschneiden vorhandener Telegramme**,
kein Uploadbefehl noetig. Minimal-invasiv, keine Aenderung an der Firmware.

### 7.2 Aktiv zuruckspielen (Restore nach Tausch/Factory-Reset)
Fuer den Fall "Mikrofon-Platine getauscht" oder "`FACTORY` versehentlich
ausgefuehrt" braucht es einen Weg, eine gespeicherte Kalibrierung wieder in den
ESP32 zu schreiben. Heute ginge das nur mit N einzelnen `SET OFS<i>=...`-Befehlen
hintereinander (fehleranfaellig: Verbindung bricht mitten in der Sequenz ab ->
inkonsistenter Zwischenzustand). Vorschlag: ein atomarer Bulk-Befehl:

```
CAL IMPORT ofs0=120,ofs1=-45,ofs2=0,ofs3=30,ofs4=-10,ofs5=5,sound_mps=354
{"type":"ok","cmd":"cal_import"}
```

- Parser validiert **erst alle** Werte (Bereichspruefung wie bei den
  bestehenden einzelnen `SET OFS<i>`-Handlern), schreibt **erst dann** alle
  NVS-Keys - kein Teilzustand bei Abbruch mitten in der Uebertragung.
- Reine Erweiterung des bestehenden `handleSet()`/`handleCommand()`-Musters,
  keine neue Systematik.
- `tools/calibrate_mics.py` (druckt heute `SET MPOSX<i>=...`-Zeilen zum
  manuellen Einfuegen) koennte spaeter direkt `CAL IMPORT ...` erzeugen -
  optionaler Folgeschritt, kein Teil der Firmware-Aenderung selbst.

## 8. Stand-PC-seitige Architektur (Empfehlung)

Im Repo existiert aktuell keine Stand-PC-Serverkomponente (nur
Offline-Analyse-Tools unter `tools/`). Fuer die Umsetzung dieses Konzepts
braucht die Serverseite - unabhaengig von der gewaehlten Sprache/Stack -
im Kern:

1. **Eine Tabelle/Store pro physischem Geraet** (Key = `device_id` aus
   Abschnitt 2): letzter bekannter `lane`, `config`, `netconfig` (Passwort
   verschluesselt oder gar nicht spiegeln - siehe unten), `calibration`-Historie,
   `last_seen`.
2. **Ein Empfangs-Loop pro TCP-Verbindung**, der jede eingehende Zeile parst und
   je nach `"type"` in den Store schreibt (passives Mitschneiden, Abschnitt 7.1)
   - unabhaengig davon, ob der Stand-PC gerade aktiv einen Befehl gesendet hat
   oder nicht (der ESP32 sendet `status`/`config`/`confignet` teils unaufgefordert,
   z.B. nach Verbindungsaufbau).
3. **Eine Sende-Funktion, die nur auf einer bestehenden, offenen Verbindung
   schreibt** - nichts puffern/erzwingen, wenn der ESP32 gerade nicht verbunden
   ist (passt zur Leitlinie "ESP32 initiiert"). Ein UI/API-Klick auf dem
   Stand-PC loest `tcp.write(...)` auf dem offenen Socket aus, fertig.
4. **Sicherheitshinweis:** `pass` erscheint im `confignet`-Telegramm aktuell
   maskiert (`****`) - das WLAN-Passwort selbst sollte serverseitig trotzdem nie
   im Klartext-Log landen, wenn es via `SET PASS=...` gesetzt wird (der Befehl
   selbst enthaelt es im Klartext, da es sonst nicht gesetzt werden koennte).

## 9. Umsetzungsreihenfolge (Vorschlag)

Kleine, unabhaengig auslieferbare Schritte, jeweils rueckwaertskompatibel:

1. **`device_id` (MAC) in `status`/`config`/`confignet`/`cal`-Telegramme.**
   Voraussetzung fuer alles Weitere, minimal-invasiv (Baustein 2).
2. **Korrelations-ID `#<id>`** fuer Befehle/Antworten (Baustein 3.1).
3. **`ACTION LIGHT=ON|OFF|AUTO`** und **`ACTION TARGETCHANGE`** (Baustein C) -
   kleinster fachlicher Nutzen, kein Risiko fuer bestehende Funktionen.
4. **Stand-PC: passives Mitschneiden** von `config`/`confignet`/`cal`-Telegrammen
   in eine Geraete-Tabelle (Baustein 7.1) - reine Serverseite, keine
   Firmware-Aenderung noetig, sobald Schritt 1 steht.
5. **`CAL IMPORT`** atomarer Bulk-Restore-Befehl (Baustein 7.2).
6. **Netzwerk-Absicherung mit Rollback-Watchdog** (Baustein 5.2) - groesster
   Einzelbaustein, aber unabhaengig vom Rest umsetzbar; bis dahin gilt fuer
   Netzwerkaenderungen "vor Ort testen, bevor man sich per Fernzugriff darauf
   verlaesst".
7. **Optional: `AUTH`-Token** fuer schreibende Befehle (Baustein 3.2), sobald
   mehr als Diagnose ueber den Kanal laeuft.

## 10. Offene Entscheidungen

- **Reichweite von `ACTION`:** Reicht LED + Scheibenwechsel fuer den ersten
  Schritt, oder sollen jetzt schon weitere Aktionen (z.B. Papier-Einfaedel-Jog
  fernsteuerbar) mitgedacht werden? Der `ACTION`-Namensraum ist dafuer offen,
  aendert aber nichts an der Reihenfolge oben.
- **Rollback-Timeout fuer Netzwerkaenderungen:** 3 Minuten als Vorschlag - haengt
  davon ab, wie lange ein WLAN-Verbindungsaufbau in der realen Umgebung
  (Stahlbau/Daempfung im Schiessstand) typischerweise braucht.
- **Auth-Token:** Aufwand/Nutzen haengt davon ab, wie offen das WLAN/LAN am
  Standort tatsaechlich ist (eigenes isoliertes Netz vs. mitgenutztes Vereins-WLAN).
