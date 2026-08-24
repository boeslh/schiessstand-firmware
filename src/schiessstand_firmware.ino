/*
 * ============================================================================
 *  Elektronischer Schießstand – ESP32 Firmware
 *  Rev 4.9 – Komplettumbau der Schusserfassung auf einen kontinuierlich
 *            laufenden Ringpuffer je Mikrofon; Piezo ist jetzt der alleinige,
 *            EREIGNISGESTEUERTE Trigger (kein Polling/Timeout mehr)
 * ============================================================================
 *
 *  Hintergrund: Das bisherige Modell ("die erste erfasste Luft-Flanke
 *  oeffnet ein Sammelfenster, SET TDOA verwirft in der ISR alles, was zu
 *  weit vom schnellsten Mic dieses Fensters abweicht") hatte einen
 *  strukturellen Webfehler: WELCHES Ereignis das Fenster oeffnet, war reiner
 *  Zufall (oft der Muendungsknall statt des echten Einschlags) - alles, was
 *  VOR diesem Zufallsereignis lag, war unwiederbringlich verloren, noch
 *  bevor das Piezo ueberhaupt etwas dazu sagen konnte. Rev 4.5/4.6 haben das
 *  mit wachsendem Aufwand (PIEZOLEAD/PIEZOCONFIRM/immer laenger bemessene
 *  SET TDOA/PIEZOCONFIRM-Sammelfenster) kompensiert, ohne den Kernfehler zu
 *  beheben - und ein zu spaet oeffnendes bzw. zu frueh laufendes Zeitfenster
 *  fuehrte wiederholt zu "reject: only 0 mic(s)"-Ausfaellen.
 *
 *  Neu: Jedes Luft-Mikrofon zeichnet KONTINUIERLICH (unabhaengig von jedem
 *  Trigger) seine letzten Flanken in einen eigenen Ringpuffer auf
 *  (AIR_RING_SIZE=128 Slots, siehe airRing[]/airISR() weiter unten) - es
 *  gibt kein Fenster mehr, das durch ein beliebiges erstes Ereignis geoeffnet
 *  wird, und damit auch keine Geometrie-Plausibilitaetspruefung mehr in der
 *  ISR (SET TDOA/cfg.airMaxTdoaUs sind komplett entfallen).
 *
 *  Das Piezo (Koerperschall auf der Stahlplatte) ist jetzt der EINZIGE
 *  Ausloeser: Feuert es (SET PIEZO=1, siehe piezoISR()), wird nach einer
 *  kurzen Nachlaufzeit (TARGET=PAPER: SET PIEZOTRAIL, TARGET=STEEL: SET
 *  PIEZOMAX) sofort der Ringpuffer jedes Mikrofons ausgewertet - kein
 *  Polling/Timeout mehr wie bei SET PIEZOCONFIRM (Rev 4.6, komplett
 *  entfallen): loest das Piezo nicht aus (z.B. reiner Muendungsknall ohne
 *  Einschlag), passiert schlicht nichts - keine Sperrzeit, kein Reject-
 *  Telegramm, das System ist sofort wieder bereit.
 *
 *  TARGET=PAPER durchsucht den Ringpuffer RUECKWAERTS ab dem Piezo-Zeitpunkt
 *  (SET PIEZOLEAD/PIEZOLEADMIN, unveraendertes Konzept aus Rev 4.5, plus ein
 *  kleines Nachlauf-Fenster SET PIEZOTRAIL) - je Mikrofon wird weiterhin die
 *  zeitlich AM NAECHSTEN an Piezo liegende Flanke im erlaubten Bereich
 *  gewaehlt (Rev-4.5-Algorithmus, jetzt auf den Ringpuffer statt ein starres
 *  Capture-Array angewendet). TARGET=STEEL durchsucht dagegen VORWAERTS ab
 *  dem Piezo-Zeitpunkt (0..SET PIEZOMAX) - dort sitzt das Piezo direkt auf
 *  der Trefferflaeche und loest schneller aus als jeder Luftschall.
 *
 *  SET RINGBUFFER (Default 10ms, siehe cfg.ringBufferMs) begrenzt zusaetzlich
 *  grob, wie weit rueckwaerts ueberhaupt gesucht wird (deckelt SET PIEZOLEAD
 *  nach oben) - ein einfacher, unabhaengiger Parameter fuer "wie viel
 *  Vorgeschichte soll ueberhaupt betrachtet werden", getrennt von der
 *  feinen Auswahllogik.
 *
 *  Ist SET PIEZO=0 (kein Piezo verbaut): Fallback auf das urspruengliche
 *  Verhalten - die erste Luft-Flanke nach Ablauf der Sperrzeit ist der
 *  Trigger, danach wird SET WINDOW lang vorwaerts gesammelt (je Mikrofon die
 *  fruehste Flanke in diesem Fenster). SET WINDOW ist damit ausschliesslich
 *  fuer diesen Fallback-Fall relevant.
 *
 *  Entfallen (nicht mehr benoetigt): SET TDOA, SET PIEZOMIN (die "Verzoegerung
 *  relativ zum ersten Luft-Ereignis"-Pruefung ergibt keinen Sinn mehr, wenn
 *  das Piezo selbst der Zeit-Nullpunkt ist), SET PIEZOCONFIRM,
 *  shotInProgress/firstAirCC/das starre airCC[][]-Capture-Array. Die
 *  Telegrammfelder "piezo_ns"/"piezo_ok" entfallen ebenfalls (piezo_ns waere
 *  jetzt trivial immer 0, da Piezo selbst der Zeitbezug ist) - die
 *  Verzoegerung jedes Mikrofons relativ zu Piezo steht direkt und praeziser
 *  in air_ns. air_ns ist jetzt ein FLACHES Array (ein Wert je Mikrofon,
 *  null = nicht erfasst) statt einer Liste aller Rohkandidaten je Mikrofon -
 *  die frueher dafuer noetige manuelle Sichtung ist mit der jetzt robusten
 *  Piezo-Anker-Auswahl nicht mehr noetig (SET DEBUG=3 liefert bei Bedarf
 *  weiterhin die pro Mic-Kombination verwendeten Zeiten).
 * ============================================================================
 *
 *  Rev 4.7 – Automatische Schallgeschwindigkeits-Kalibrierung (Rev 4.4)
 *            wieder entfernt: CAL START kalibriert nur noch die Mic-Timing-
 *            Offsets, SET SOUNDSPEED ist wieder ein rein manueller Wert
 * ============================================================================
 *
 *  Hintergrund: Die in Rev 4.4 eingefuehrte automatische Schallgeschwindig-
 *  keits-Schaetzung (per Koordinatenabstieg auf precision_um ODER die in
 *  Rev "analytische Median-Schaetzung"-Variante) lieferte in der Praxis
 *  wiederholt schlechte/unplausible Ergebnisse - vermutlich weil die
 *  Kalibrier-Schuesse durch Mehrwege-/Nachhall-Effekte und die AIR4/AIR5-
 *  Hardwarethematik oft selbst nicht sauber genug sind, um zuverlaessig auf
 *  v zu schliessen (siehe Testmodus-Analysen). SET SOUNDSPEED/SET CALMODE
 *  als von CAL START mitkalibrierter Wert wurden deshalb wieder entfernt -
 *  SET SOUNDSPEED bleibt als manueller Parameter bestehen, siehe unten.
 * ============================================================================
 *
 *  Rev 4.6 – (HISTORISCH, per Rev 4.9 komplett durch den Ringpuffer-Umbau
 *            ersetzt - SET PIEZOCONFIRM/SET TDOA existieren nicht mehr,
 *            siehe Rev-4.9-Hinweis oben) Schnelles Verwerfen ohne Piezo-
 *            Bestaetigung (SET PIEZOCONFIRM, NUR TARGET=PAPER): kommt nach
 *            der ersten Luft-Flanke innerhalb PIEZOCONFIRM kein Piezo-
 *            Signal, wird der Ausloeser SOFORT verworfen (keine Sperrzeit)
 *            statt lange auf ein Piezo zu warten
 * ============================================================================
 *
 *  Hintergrund: Rev 4.5 (PIEZOLEAD) loeste zwar aus, WELCHE Luft-Flanke fuer
 *  die Trilateration verwendet wird - das Sammelfenster musste dafuer aber
 *  bis SET PIEZOMAX offen bleiben. In der Praxis kann das Piezo bei einem
 *  echten Muendungsknall-Fehltrigger aber viele ms (teils >30ms, siehe
 *  Testmodus-Messungen) oder gar nicht kommen - ein so lang bemessenes
 *  PIEZOMAX wuerde jeden Fehltrigger minutenlang "totlaufen" lassen, bevor
 *  der naechste (echte) Schuss ueberhaupt erkannt werden kann.
 *
 *  Neu (NUR TARGET=PAPER, NUR bei aktivem Piezo): Das Sammelfenster bleibt
 *  kurz (SET PIEZOCONFIRM, Default 5000us=5ms). Ist bis dahin KEIN Piezo
 *  erfasst, wird der Ausloeser in loop() SOFORT verworfen (resetShotState(),
 *  kein processShot(), keine Sperrzeit - lockoutUntil wird nur in
 *  processShot() gesetzt) - das System ist dadurch ohne jede Pause wieder
 *  fuer den naechsten Ausloeser bereit. Kommt Piezo dagegen rechtzeitig, wird
 *  wie gewohnt verarbeitet (inkl. PIEZOLEAD-Flankenauswahl aus Rev 4.5).
 *
 *  PIEZOMAX (Rev 4.3) ist damit fuer TARGET=PAPER nicht mehr relevant -
 *  PIEZOCONFIRM uebernimmt dort dessen bisherige Rolle beim Bemessen des
 *  Sammelfensters; PIEZOMAX bleibt ausschliesslich fuer TARGET=STEEL aktiv
 *  (dort kommt das Piezo planmaessig VOR/gleichzeitig mit den Mics, ein
 *  Verwerfen mangels Piezo-Bestaetigung ergibt dort keinen Sinn).
 *
 *  WICHTIG: SET TDOA muss weiterhin mindestens SET PIEZOCONFIRM abdecken,
 *  sonst verwirft airISR() die fuer PIEZOLEAD benoetigten Flanken schon,
 *  bevor processShot() sie auswerten kann.
 * ============================================================================
 *
 *  Hintergrund: Empfindlicher eingestellte Mikrofone (fuer weniger Jitter/
 *  praezisere Flankenerkennung) fangen leichter auch Fremdgeraeusche ein -
 *  eigener Muendungsknall, aber auch Schuesse von benachbarten Staenden.
 *  Bisher (Rev 4.3) wurde das nur NACHTRAEGLICH per PIEZOMIN/PIEZOMAX
 *  erkannt (Schuss wird als nicht "sauber" markiert), die Positions-
 *  berechnung selbst nutzte aber weiterhin blind airCC[i][0] (erste
 *  erfasste Flanke) - bei einer fremden, frueher eintreffenden Flanke also
 *  eine von vornherein falsche Grundlage.
 *
 *  Neu (NUR TARGET=PAPER, NUR bei aktivem Piezo, SET PIEZOLEAD, Default
 *  2000us=2ms): Fuer jedes Mikrofon wird jetzt aus dessen Multi-Edge-Capture-
 *  Liste (bis zu 6 Flanken) gezielt diejenige gewaehlt, die hoechstens
 *  PIEZOLEAD vor der Piezo-Ausloesung liegt - der erwartete Bereich fuer den
 *  echten Einschlag (Papier-Durchschlag kurz vor dem Stahl-Kontakt, siehe
 *  Rev-4.3-Hinweis unten). Frueher eingetroffene Flanken (Muendungsknall,
 *  Nachbarstaende) werden dabei ignoriert, auch wenn sie als airCC[i][0]
 *  erfasst wurden. Findet sich fuer ein Mikrofon keine Flanke in diesem
 *  Fenster, gilt es fuer diesen Schuss als nicht erfasst.
 *
 *  WICHTIG: SET TDOA (Geometrie-Plausibilitaetsfenster in airISR(), Default
 *  750us) muss dafuer grosszuegig genug sein (mindestens PIEZOMAX +
 *  PIEZOLEAD) - sonst verwirft airISR() die benoetigten spaeten Flanken
 *  schon, bevor sie ueberhaupt in processShot() ankommen.
 *
 *  STEEL ist bewusst ausgenommen (dort loest das Piezo VOR oder gleichzeitig
 *  mit den Luftmikrofonen aus, siehe Rev-4.3-Hinweis unten - ein "davor"-
 *  Fenster ergibt dort keinen Sinn); dort bleibt es bei airCC[i][0].
 * ============================================================================
 *
 *  Neu in 4.4: SOUND_MM_PER_NS ist keine feste Konstante mehr, sondern
 *  cfg.soundSpeedMps (SET SOUNDSPEED=<300-400>, Default 355 m/s - empirisch
 *  per Testschuessen ermittelt, hoeher als die klassischen 343 m/s/20C).
 *  Ein radial mit dem Abstand vom Zentrum WACHSENDER Fehler (Treffer am Rand
 *  werden zu nah am Zentrum berechnet) ist ein typisches Anzeichen fuer eine
 *  zu NIEDRIG angenommene Schallgeschwindigkeit.
 *
 *  CAL START kalibrierte ab dieser Revision zusaetzlich zu den Mic-Offsets
 *  versuchsweise auch automatisch die Schallgeschwindigkeit mit - das wurde
 *  in Rev 4.7 wegen unzuverlaessiger Ergebnisse wieder entfernt (siehe
 *  Rev-4.7-Hinweis ganz oben). SET SOUNDSPEED bleibt als manueller Parameter
 *  bestehen, nur eben nicht mehr automatisch mitkalibriert.
 * ============================================================================
 *
 *  Rev 4.3 – Piezo-Trigger-Bestaetigung (SET PIEZO) zur Erkennung von
 *            verfrueht durch den Muendungsknall ausgeloesten Sammelfenstern;
 *            reiner Sensor-Testmodus (SET TESTMODE) zur Kommissionierung
 * ============================================================================
 *
 *  SET TESTMODE=1 (nicht persistent, nach Reboot immer aus): schaltet die
 *  komplette Schuss-/TDOA-Logik ab und gibt bei jeder Sensor-Ausloesung
 *  (Luft-Mic ODER Piezo) sofort eine Klartext-Zeile mit Sensor-Bezeichnung
 *  UND Zeitstempel aus, z.B. "AIR2 links-oben        +0.585 ms" - je
 *  Sensor max. 1 Meldung alle 3s (Prellen/Nachschwinger beim Testen per
 *  Hand/Klopfen). Der erste Sensor nach einer Trennlinie (bzw. nach dem
 *  Aktivieren) beginnt eine neue Serie bei "+0.000 ms", alle folgenden
 *  zeigen die Verzoegerung dazu - damit laesst sich bei einem ECHTEN Schuss
 *  direkt ablesen, in welchem zeitlichen Abstand die Sensoren (inkl. Piezo)
 *  wirklich ausloesen, um z.B. SET TDOA/PIEZOMIN/PIEZOMAX passend zu
 *  justieren. Kommt 5s lang von keinem Sensor ein Signal, wird EINMALIG
 *  eine Trennlinie aus 10 "-" ausgegeben (startet die naechste Serie).
 *  Siehe testMode/testModeHit() sowie loop() weiter unten.
 * ============================================================================
 *
 *  Neu in 4.3: Optionales Piezo-Kontaktmikrofon (Koerperschall) auf der
 *  Stahlplatte hinter der Papierscheibe, an PIEZO_PIN (GPIO34).
 *
 *  Hintergrund: Das Projektil ist mit ~150 m/s LANGSAMER als der Schall
 *  (343 m/s) - der Muendungsknall der Waffe kann die Luft-Mikrofone am Ziel
 *  daher VOR dem eigentlichen Einschlag erreichen und das Sammelfenster
 *  verfrueht (mit falscher Richtung/Geometrie) oeffnen. Das fuehrt zu
 *  scheinbar "sauberen", aber physikalisch falschen Positionen.
 *
 *  Das Piezo sitzt in der Stahlplatte und spricht nur auf echten
 *  Koerperschall (Projektil-Einschlag) an, nicht auf Luftschall - damit ist
 *  es unempfindlich gegen den Muendungsknall und dient seit Rev 4.9 als
 *  alleiniger, ereignisgesteuerter Ausloeser der Schusserfassung (siehe
 *  Rev-4.9-Hinweis ganz oben und piezoISR()/loop()/processShot() weiter
 *  unten). SET PIEZO=0 deaktiviert das Piezo komplett - dann uebernimmt die
 *  erste Luft-Flanke nach Ablauf der Sperrzeit die Trigger-Rolle (Default 1/an).
 *
 *  WICHTIG - TARGET=STEEL unterscheidet sich hier grundlegend von PAPER:
 *  Im STEEL-Modus IST die Stahlplatte die Trefferflaeche, das Piezo sitzt
 *  also direkt darauf und erkennt den Einschlag quasi latenzfrei per
 *  Kontaktschall - schneller als die Luftschall-Laufzeit zu JEDEM Mikrofon.
 *  Nach dem Ausloesen wird daher VORWAERTS (0..SET PIEZOMAX) im Ringpuffer
 *  gesucht. TARGET=PAPER: das Geschoss durchquert erst noch die Papier-
 *  Stahl-Luecke, das Piezo loest also SPAETER aus als der Luftschall-Treffer
 *  am Papier - hier wird RUECKWAERTS gesucht (SET PIEZOLEAD/PIEZOLEADMIN,
 *  plus ein kleines Nachlauf-Fenster SET PIEZOTRAIL).
 * ============================================================================
 *
 *  Die Messung über das Stahlblech (Körperschall-Sensoren + MCPWM-Hardware-
 *  Capture) hat sich in der Praxis nicht als zuverlässig genug erwiesen und
 *  wurde komplett entfernt. Die Positionsbestimmung erfolgt ausschließlich
 *  über Mikrofone (Piezo/Elektret) in der Seitenwand, die den Luftschall des
 *  Einschlags per TDOA (Time-Difference-of-Arrival) auswerten.
 *
 *  Ablauf (seit Rev 4.9): Jedes Mikrofon zeichnet fortlaufend, unabhaengig
 *  von jedem Trigger, seine Flanken in einen eigenen Ringpuffer auf
 *  (AIR_RING_SIZE Slots je Kanal, blendet per 20us-Totzeit weiterhin
 *  Nachschwinger/Echos direkt am selben Kanal aus). Ausgewertet wird erst,
 *  wenn das Piezo (oder im SET PIEZO=0-Fallback die erste Luft-Flanke)
 *  ausloest - siehe Rev-4.9-Hinweis oben fuer Details der Fenster-/
 *  Auswahllogik. Gültig ab SET MINMICS Mikrofonen von 6 (Default 5, siehe
 *  cfg.minMics).
 *
 *  Pinbelegung Luft-Mikrofone (LM339-Frontend), je 3 pro Seitenwand:
 *    GPIO25 = links unten     GPIO26 = rechts unten
 *    GPIO27 = links oben      GPIO14 = rechts oben
 *    GPIO32 = links mitte     GPIO33 = rechts mitte
 *
 *  Telegramm:
 *    {"type":"shot","seq":8,"air_ns":[-4200,-4150,null,-4300,-4100,-4250],
 *     "x_um":-22300,"y_um":-300,"pos_res_um":43910,"precision_um":120,
 *     "cluster_hits":3,"pos_valid":1,"hits":4,"ts":123456789}
 *    air_ns[i] = die fuer Mikrofon i AUSGEWAEHLTE Flanke (siehe Rev-4.9-
 *    Hinweis oben), in ns relativ zum Ausloeser (Piezo bzw. im SET PIEZO=0-
 *    Fallback die erste Luft-Flanke). null = kein passender Treffer fuer
 *    dieses Mikrofon gefunden.
 *
 *  Trefferposition (x_um/y_um, 1 Einheit = 0.001 mm) wird per Hyperbel-
 *  Trilateration aus den ersten Flanken der Mikrofone berechnet. Ursprung
 *  = Zentrum der Zielflaeche, x positiv nach RECHTS, y positiv nach OBEN
 *  (aus Schützensicht). Geometrie (siehe MIC_X/MIC_Y/applyTargetGeometry()
 *  unten) je Seitenwand (x = ±115 mm vom Zentrum, bei beiden Zielarten
 *  gleich), 3 Mikrofone auf Höhe Mitte/+/-Y, per SET TARGET umschaltbar
 *  (Default STEEL, kein Reboot noetig):
 *    STEEL (Stahlblech-Abprallflaeche): Y = ±100 mm, 30 mm Standoff
 *    PAPER (Papierscheibe, misst den Durchschlagpunkt statt des Abpralls,
 *      z.B. bei zu starker Streuung auf Metall): Y = ±85 mm, 28 mm Standoff
 *
 *  pos_valid=0, falls < 3 Mics ausgewertet werden konnten oder die
 *  Geometrie entartet war (x_um/y_um/pos_res_um/precision_um/cluster_hits
 *  dann 0).
 *
 *  pos_res_um = Rest-Fehler der gewählten Lösung in 0.001mm. Bei mehr als
 *  3 Treffern probiert solveAirPosition() alle Dreier-Kombinationen der
 *  erfassten Mics für die direkte Lösung durch und bewertet jede anhand
 *  des mittleren Rest-Fehlers der übrigen (nicht an der Lösung beteiligten)
 *  Mics; die Kombination mit dem kleinsten Rest-Fehler gewinnt. Bei genau
 *  3 Treffern gibt es keine übrigen Mics zum Prüfen -> Rest-Fehler immer 0.
 *  Mit 6 statt 4 Mikrofonen steigt die Redundanz (bis zu 3 statt 1
 *  Kontroll-Mics) und damit die Robustheit gegenüber einzelnen Ausreißer-
 *  Flanken deutlich. Ein großer pos_res_um (>> wenige mm) bedeutet trotzdem:
 *  Vorsicht, die Flankenzeiten waren nicht gut konsistent (z.B. Echo statt
 *  Direktschall an einem Mikrofon) - die Position ist dann mit Unsicherheit
 *  zu behandeln, nicht blind zu werten.
 *
 *  precision_um / cluster_hits: Jede der oben genannten Dreier-Kombinationen
 *  liefert unabhängig eine eigene Kandidatenposition (x,y). precision_um ist
 *  die quadratisch gemittelte Abweichung (RMS, 0.001mm) der bis zu 2
 *  NÄCHSTGELEGENEN zusätzlichen Kandidaten von der gewählten Lösung -
 *  bewusst nur die besten 2, damit einzelne weit abweichende Ausreisser-
 *  Kombinationen (z.B. durch Echos) den Wert nicht dominieren; je kleiner,
 *  desto genauer bestätigen die naechsten unabhängigen Mic-Kombinationen die
 *  gewählte Lösung. cluster_hits zählt dagegen ALLE Kandidatenpositionen
 *  (nicht nur die besten 2) innerhalb von SET RADIUS (Default 200 = 0.2mm)
 *  um die gewählte Lösung. Bei genau 3 Treffern gibt es nur eine
 *  Kombination -> precision_um immer 0, cluster_hits immer 1.
 *
 *  SET DEBUG=0-3 (persistent im NVS) filtert, welche Schuss-/Reject-/
 *  Kandidaten-Telegramme ausgegeben werden (Zähler/shotCounter laufen davon
 *  unabhängig immer mit):
 *    0 (Default): nur sauber ermittelte Schüsse (Position bestimmbar,
 *      pos_res_um < SET OUTLIER-Schwelle, cluster_hits >= SET MINCLUSTER,
 *      precision_um <= SET MAXPRECISION)
 *    1: zusätzlich Schüsse mit Mikrofon-Ausreißer bzw. nicht
 *      bestimmbarer Position (pos_valid=0)
 *    2: zusätzlich Reject-Telegramme wegen zu weniger Mics (<3)
 *    3: zusätzlich je Schuss eine Zeile PRO ausgewerteter Mic-Kombination
 *      ({"type":"cand",...}), mit deren Ergebnis (x_mm/y_mm, 2 Nachkomma-
 *      stellen) und den dafuer verwendeten Laufzeiten (t_ref_ns/t_a_ns/
 *      t_b_ns der beteiligten Mics ref/a/b) - siehe solveAirPosition().
 *
 *  SET OUTLIER=<0.001mm> (persistent im NVS, Default 5000 = 5.0mm) legt
 *  die Schwelle fest, ab der ein Schuss als "Mikrofon-Ausreißer" gilt.
 *
 *  SET MINCLUSTER=<0-20> (persistent im NVS, Default 2) legt fest, wie
 *  viele cluster_hits (siehe precision_um/cluster_hits oben) mindestens
 *  vorliegen muessen, damit ein Schuss als sauber gilt - bei genau 3
 *  Treffern ist cluster_hits immer 1 (keine Redundanz), solche Schuesse
 *  fallen mit dem Default also automatisch unter DEBUG=1.
 *
 *  SET MAXPRECISION=<0.001mm> (persistent im NVS, Default 2000 = 2.0mm)
 *  legt die Schwelle fest, ab der precision_um einen Schuss als nicht mehr
 *  sauber gilt.
 *
 *  Befehl HELP oder ? gibt eine Liste aller SET-Parameter mit gültigem
 *  Wertebereich aus. Reiner Klartext nur über Serial (nicht per emitLine/
 *  TCP), damit der JSON-Zeilenstrom zum Stand-PC sauber bleibt.
 *
 *  Zeitstempel: Die Flankenzeiten stammen vom CPU-Zykluszähler
 *  (esp_cpu_get_cycle_count, ~4,2 ns Auflösung bei 240 MHz), nicht vom
 *  µs-Timer. Grobzeitlogik (Sperrzeit, Sammelfenster) nutzt weiter
 *  esp_timer. WICHTIG: Alle Sensor-ISRs werden aus setup() registriert und
 *  laufen damit auf demselben Core -> Zykluszähler-Werte sind vergleichbar.
 *
 *  Konfiguration / Befehle: SET ... (NVS-persistent), SHOW, STATUS, PING,
 *  RESET, REBOOT, FACTORY, HELP/?
 *
 *  Kalibrierung (Timing-Offset je Mikrofon, SET OFS0..OFS5 in ns): CAL START
 *  sammelt die naechsten SET CALSHOTS (Default 5) gueltigen Schuesse an
 *  BELIEBIGEN, vorher nicht festgelegten Stellen der Scheibe und berechnet
 *  daraus per Koordinatenabstieg automatisch einen Timing-Offset je
 *  Mikrofon (keine Benutzerinteraktion noetig), der direkt persistiert und
 *  ab dem naechsten Schuss angewendet wird. Kompensiert werden damit
 *  systematische Laufzeitunterschiede der Kanaele (Komparator-Schwelle,
 *  Kabellaenge) - keine 3D-Neuvermessung der Mic-Positionen (siehe
 *  runCalibration() fuer die Begruendung). SET SOUNDSPEED wird bewusst NICHT
 *  mitkalibriert (siehe Kommentar bei calCost() weiter unten - automatische
 *  Schallgeschwindigkeits-Schaetzung lieferte in der Praxis unzuverlaessige
 *  Ergebnisse). Nach den Offsets gibt CAL START zusaetzlich (Rev 4.11) je
 *  gesammeltem Schuss eine "calshot"-Zeile mit den ROHEN Mikrofon-Messwerten
 *  aus (dasselbe air_ns-Format wie ein normales "shot"-Telegramm, siehe
 *  sendCalRawDump()) - damit steht am Ende immer eine vollstaendige,
 *  wiederverwendbare Kopie der Kalibrier-Rohdaten bereit (z.B. fuer
 *  tools/calibrate_mics.py oder um spaeter mit anderen Annahmen offline
 *  nachzurechnen, ohne erneut schiessen zu muessen). Die Mikrofonwerte sind
 *  bereits relativ zu Piezo (bzw. im SET PIEZO=0-Fallback zur ausloesenden
 *  Luft-Flanke) - ein separater Piezo-Messwert waere hier immer 0 und daher
 *  nicht informativ. CAL ABORT bricht ab, CAL STATUS
 *  zeigt den Fortschritt, CAL RESET setzt alle Offsets auf 0 und die
 *  Schallgeschwindigkeit auf den Default (355 m/s) zurueck.
 *
 *  Build: Arduino IDE / PlatformIO, Board "ESP32 Dev Module"
 *         (Arduino-Core >= 2.x wegen esp_cpu_get_cycle_count)
 * ============================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <math.h>                 // sqrtf/fabsf/lroundf (Positionsberechnung)
#include "esp_cpu.h"

// ---------------------------------------------------------------------------
// Konstanten & Werks-Defaults (greifen nur bei leerem NVS)
// ---------------------------------------------------------------------------

#define FW_VERSION   "4.11.0"
#define SERIAL_BAUD  115200
#define NVS_NS       "schiessstd"     // NVS-Namespace

// Luft-Mikrofone: LM339-Frontend, 3 pro Seitenwand (links/rechts)
#define NUM_AIR        6
#define AIR_RING_SIZE  128 // Rev 4.9: Groesse des kontinuierlich laufenden
                            // Ringpuffers je Mikrofon (siehe airRing[] weiter
                            // unten) - unabhaengig von jedem Trigger immer
                            // aktiv, aeltere Flanken werden beim Vollwerden
                            // einfach ueberschrieben. 128 Slots decken selbst
                            // bei dichtem Nachhall (siehe Testmodus-Messungen:
                            // teils <1ms zwischen Flanken je Kanal) deutlich
                            // mehr als die per SET RINGBUFFER/PIEZOLEAD
                            // gesuchte Vorgeschichte ab.
// Reihenfolge: 0=links unten 1=rechts unten 2=links oben 3=rechts oben
//              4=links mitte 5=rechts mitte  (siehe MIC_X/MIC_Y weiter unten)
static const uint8_t AIR_PINS[NUM_AIR] = {25, 26, 27, 14, 32, 33};

// Rev 4.9: Explizite Vorab-Deklarationen fuer processShot() und seine
// Ringpuffer-Helfer (siehe weiter unten fuer die eigentlichen Definitionen).
// Der ctags-basierte Arduino-Prototyp-Generator erzeugt fuer processShot()
// sonst einen fehlerhaften, an falscher Stelle eingefuegten Prototyp
// (beobachteter Build-Fehler "processShot declared void" mitten in
// loadConfig()) - eine eigene, frueh im File stehende Deklaration umgeht das.
enum TriggerSource { TRIG_PIEZO, TRIG_FALLBACK_AIR };
static bool ringFindEdge(const uint32_t ring[AIR_RING_SIZE], uint16_t head,
                          uint32_t count, uint32_t refCC,
                          int64_t loNs, int64_t hiNs,
                          int64_t exLoNs, int64_t exHiNs, int64_t *outNs);
static void processShot(TriggerSource src, uint32_t refCC, uint64_t refUs);

// Optionales Piezo-Kontaktmikrofon (Koerperschall) auf der Stahlplatte,
// dient als Trigger-Bestaetigung gegen verfrueh durch den Muendungsknall
// geoeffnete Sammelfenster (siehe SET PIEZO, Rev-4.3-Hinweis oben).
// GPIO34 hat KEINEN internen Pull-Up/Down (ESP32 GPIO 34-39) - das
// Piezo-Modul muss push-pull treiben (uebliche LM393-Komparator-Module tun das).
#define PIEZO_PIN  34

// Messmodus (SET TARGET, siehe applyTargetGeometry() weiter unten): legt
// fest, welches Geometrie-Preset (Mic-Y-Abstand/Standoff) verwendet wird -
// Stahlblech (Abprallflaeche) oder Papierscheibe (Durchschlag).
#define TARGET_STEEL  0
#define TARGET_PAPER  1

struct DeviceConfig {
    String   ssid;          // "" = WLAN deaktiviert
    String   pass;
    String   host;          // Stand-PC
    uint16_t port;
    uint16_t lane;
    uint32_t debounceMs;
    uint32_t windowMs;
    uint8_t  debug;        // Ausgabe-Filter: 0=nur saubere Schuesse (Default),
                            // 1=+Schuesse mit Mikrofon-Ausreisser, 2=+Reject
                            // wegen zu weniger Hits
    uint32_t airOutlierUm; // Schwelle (0.001mm) ab der ein Schuss als
                            // Mikrofon-Ausreisser gilt (SET OUTLIER)
    uint32_t clusterRadiusUm; // Umkreis (0.001mm) fuer cluster_hits (SET RADIUS)
    uint8_t  minClusterHits; // Mindestzahl cluster_hits, ab der ein Schuss
                            // als sauber gilt (SET MINCLUSTER, Default 2)
    uint32_t maxPrecisionUm; // Schwelle (0.001mm) fuer precision_um, ab der
                            // ein Schuss NICHT mehr als sauber gilt
                            // (SET MAXPRECISION, Default 2000 = 2.0mm)
    uint8_t  minMics;       // Mindestzahl Mics fuer eine gueltige Auswertung
                            // (SET MINMICS, 3-NUM_AIR, Default 5)
    uint32_t ringBufferMs;  // Rev 4.9: wie weit rueckwaerts der kontinuier-
                            // lich laufende Ringpuffer je Mikrofon bei der
                            // Piezo-Anker-Auswahl hoechstens durchsucht wird
                            // (SET RINGBUFFER, Default 10ms) - deckelt SET
                            // PIEZOLEAD grob nach oben, siehe processShot()
    int32_t  micOffsetNs[NUM_AIR]; // Timing-Offset je Mikrofon in ns, per
                            // Kalibrierung (CAL START) ermittelt oder
                            // manuell per SET OFS0..OFS<NUM_AIR-1>
    int32_t  micPosOffsetXUm[NUM_AIR]; // Rev 4.9: Montage-Justage je Mikrofon
    int32_t  micPosOffsetYUm[NUM_AIR]; // in 0.001mm, relativ zur Nominal-
    int32_t  micPosOffsetZUm[NUM_AIR]; // Geometrie (SET MPOSX/MPOSY/MPOSZ<i>,
                            // Default 0) - siehe applyTargetGeometry(). Z ist
                            // die Standoff-Richtung (rechtwinklig zur Platte/
                            // Scheibe). Offline per tools/calibrate_mics.py
                            // ermittelt (Grid-Search ueber Position+Speed),
                            // nicht durch CAL START (siehe dortige Kommentare)
    uint8_t  calShotCount;  // Anzahl Kalibrier-Schuesse (SET CALSHOTS,
                            // 3-MAX_CAL_SHOTS, Default 5)
    uint8_t  targetMode;    // TARGET_STEEL (Default) oder TARGET_PAPER,
                            // siehe applyTargetGeometry() (SET TARGET)
    float    standoffSteelMm; // Mic-Standoff (rechtwinklig zur Platte) in mm
                            // im STEEL-Modus (SET STANDOFFSTEEL, Default 30.0)
    float    standoffPaperMm; // Wie standoffSteelMm, fuer PAPER-Modus
                            // (SET STANDOFFPAPER, Default 28.0)
    bool     usePiezo;      // Piezo (Stahlplatte, PIEZO_PIN) als alleiniger
                            // Ausloeser der Schusserfassung nutzen? (SET
                            // PIEZO, Default 1/an) - 0: Fallback auf die
                            // erste Luft-Flanke als Ausloeser, siehe loop()
    uint32_t piezoMaxUs;    // NUR TARGET=STEEL: wie lange nach dem Piezo-
                            // Ausloesen gewartet wird, bevor der Ringpuffer
                            // vorwaerts ausgewertet wird (SET PIEZOMAX,
                            // Default 1400us) - dort sitzt das Piezo direkt
                            // auf der Trefferflaeche und loest VOR den
                            // Luftmikrofonen aus, siehe processShot()
    uint32_t piezoLeadMaxUs; // NUR TARGET=PAPER, NUR bei aktivem Piezo: wie
                            // weit VOR der Piezo-Ausloesung eine Luft-Flanke
                            // je Mikrofon hoechstens liegen darf, um fuer die
                            // Trilateration verwendet zu werden (SET
                            // PIEZOLEAD, Default 2000us=2ms) - macht das
                            // Piezo zum zentralen Anker-Element gegen
                            // Fremdgeraeusche (Muendungsknall, Nachbarstaende),
                            // siehe processShot()
    uint32_t piezoTrailMaxUs; // NUR TARGET=PAPER: wie piezoLeadMaxUs, aber
                            // wie weit eine Luft-Flanke NACH der Piezo-
                            // Ausloesung noch liegen darf (SET PIEZOTRAIL,
                            // Default 1000us=1ms) - das echte Treffer-Cluster
                            // streut leicht UM das Piezo herum, nicht nur
                            // davor. Bestimmt zugleich, wie lange in loop()
                            // nach dem Ausloesen gewartet wird, bevor
                            // ausgewertet wird (Nachlauf-Flanken muessen erst
                            // im Ringpuffer ankommen)
    uint32_t piezoLeadMinUs; // Untergrenze zu piezoLeadMaxUs (SET PIEZOLEADMIN,
                            // Default 0=kein Ausschluss): Luft-Flanken, die
                            // WENIGER als piezoLeadMinUs vor der Piezo-
                            // Ausloesung liegen, werden ignoriert - damit
                            // laesst sich gezielt ein Bereich DIREKT vor
                            // Piezo ausschliessen (z.B. Muendungsknall-Reste),
                            // waehrend ein Bereich weiter davor (bis
                            // piezoLeadMaxUs) weiter zulaessig bleibt
    uint32_t testCooldownMs; // Min. Abstand zwischen 2 Meldungen desselben
                            // Sensors im Testmodus, in ms (SET TESTCOOLDOWN,
                            // Default 3000) - siehe testModeHit()
    int32_t  offsetXUm;    // Konstanter Korrektur-Offset auf x_um/y_um in
    int32_t  offsetYUm;    // 0.001mm, z.B. zum Ausgleich einer Mess-
                            // gitter-Verschiebung (SET OFFSETX/OFFSETY,
                            // Default 0) - wird erst NACH der Trilateration
                            // addiert, siehe processShot()
    uint16_t soundSpeedMps; // Angenommene Schallgeschwindigkeit in m/s
                            // (SET SOUNDSPEED, Default 355) - siehe
                            // applySoundSpeed(). Rein manueller Parameter,
                            // wird NICHT von CAL START mitkalibriert (siehe
                            // Kommentar bei calCost()/runCalibration())
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
    cfg.windowMs   = prefs.getUInt("window", 1);
    cfg.debug      = prefs.getUChar("debug", 0);
    cfg.airOutlierUm = prefs.getUInt("outlier", 5000);   // Default 5.0 mm
    cfg.clusterRadiusUm = prefs.getUInt("cluster_r", 200); // Default 0.2 mm
    cfg.minClusterHits = prefs.getUChar("min_clust", 2);
    cfg.maxPrecisionUm = prefs.getUInt("max_prec", 2000);  // Default 2.0 mm
    cfg.minMics    = prefs.getUChar("min_mics", 5);
    cfg.ringBufferMs = prefs.getUInt("ring_ms", 10);
    for (int i = 0; i < NUM_AIR; i++) {
        char key[8];
        snprintf(key, sizeof(key), "ofs%d", i);
        cfg.micOffsetNs[i] = prefs.getInt(key, 0);
        snprintf(key, sizeof(key), "mpx%d", i);
        cfg.micPosOffsetXUm[i] = prefs.getInt(key, 0);
        snprintf(key, sizeof(key), "mpy%d", i);
        cfg.micPosOffsetYUm[i] = prefs.getInt(key, 0);
        snprintf(key, sizeof(key), "mpz%d", i);
        cfg.micPosOffsetZUm[i] = prefs.getInt(key, 0);
    }
    cfg.calShotCount = prefs.getUChar("cal_n", 5);
    cfg.targetMode = prefs.getUChar("target", TARGET_STEEL);
    cfg.standoffSteelMm = prefs.getFloat("standoff_st", 30.0f);
    cfg.standoffPaperMm = prefs.getFloat("standoff_pa", 28.0f);
    cfg.usePiezo   = prefs.getBool("use_piezo", true);
    cfg.piezoMaxUs = prefs.getUInt("piezo_max", 1400);
    cfg.piezoLeadMaxUs = prefs.getUInt("piezo_lead", 2000);
    cfg.piezoTrailMaxUs = prefs.getUInt("piezo_trail", 1000);
    cfg.piezoLeadMinUs = prefs.getUInt("piezo_leadmin", 0);
    cfg.testCooldownMs = prefs.getUInt("test_cd_ms", 3000);
    cfg.offsetXUm = prefs.getInt("ofs_x_um", 0);
    cfg.offsetYUm = prefs.getInt("ofs_y_um", 0);
    cfg.soundSpeedMps = prefs.getUShort("sound_mps", 355);
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
template <> void saveVal<int32_t>(const char *key, int32_t v)
{ prefs.begin(NVS_NS, false); prefs.putInt(key, v); prefs.end(); }
template <> void saveVal<float>(const char *key, float v)
{ prefs.begin(NVS_NS, false); prefs.putFloat(key, v); prefs.end(); }

// ---------------------------------------------------------------------------
// Schusserfassung – Luftschall-Mikrofone (Multi-Edge-Capture)
// ---------------------------------------------------------------------------

// Grobzeit (esp_timer) fuer Sperrzeit nach einem verarbeiteten Ausloeser
static volatile uint64_t lockoutUntil = 0;

static uint32_t shotCounter = 0;
static uint32_t sequenceNo  = 0;
static uint32_t cpuMHz      = 240;   // wird in setup() ermittelt

// Timing-Offset-Kalibrierung (siehe CAL START / runCalibration() weiter
// unten): sammelt die naechsten cfg.calShotCount gueltigen Schuesse mit
// ihren ROHEN (unkorrigierten) Erst-Flankenzeiten, danach wird daraus per
// Koordinatenabstieg ein Timing-Offset je Mikrofon bestimmt.
#define MAX_CAL_SHOTS 20
static bool    calActive    = false;
static uint8_t calCollected = 0;
static int64_t calRawNs[MAX_CAL_SHOTS][NUM_AIR];
static bool    calSeenBuf[MAX_CAL_SHOTS][NUM_AIR];

// Ringpuffer je Luftkanal (Rev 4.9): laeuft KONTINUIERLICH, unabhaengig von
// jedem Trigger - airRingHead[i] ist der naechste Schreib-Index (wrapt bei
// AIR_RING_SIZE), airRingCount[i] die Gesamtzahl je Kanal seit dem Boot
// (fuer die Auswertung auf min(airRingCount[i], AIR_RING_SIZE) begrenzt,
// siehe ringFindEdge()). airLastCC[i] dient weiterhin nur der 20us-Totzeit.
static volatile uint32_t airRing[NUM_AIR][AIR_RING_SIZE];
static volatile uint16_t airRingHead[NUM_AIR]  = {0};
static volatile uint32_t airRingCount[NUM_AIR] = {0};
static volatile uint32_t airLastCC[NUM_AIR]    = {0};

// Rev 4.9: Fallback-Trigger fuer SET PIEZO=0 (kein Piezo verbaut) - die
// erste Luft-Flanke nach Ablauf der Sperrzeit loest die Auswertung aus,
// analog zum urspruenglichen "erste Flanke oeffnet Fenster"-Verhalten.
static volatile bool     airTrigPending   = false;
static volatile uint32_t airTrigPendingCC = 0;
static volatile uint64_t airTrigPendingUs = 0;

// ---------------------------------------------------------------------------
// Reiner Sensor-Testmodus (SET TESTMODE) - Kommissionierung/Hardware-Test:
// jede Sensor-Flanke (Luft-Mic ODER Piezo) erzeugt sofort eine Klartext-
// Zeile mit der Sensor-Bezeichnung, unabhaengig von Schuss-/TDOA-Logik.
// Bewusst NICHT in NVS persistiert (kein saveVal in SET TESTMODE) - nach
// jedem Reboot ist der Testmodus immer wieder aus.
// ---------------------------------------------------------------------------
// Cooldown ist per SET TESTCOOLDOWN einstellbar (cfg.testCooldownMs, Default
// 3000ms) - fuer reines Antippen von Hand reichen 3s, um Prellen/
// Nachschwinger auszublenden. Bei einem ECHTEN Schuss koennen an einem
// Mikrofon aber mehrere echte, zeitlich getrennte Ereignisse ankommen
// (Muendungsknall, Einschlag Papier, Einschlag Stahl) - mit 3s Cooldown
// wuerde nur das jeweils erste je Sensor sichtbar, alle spaeteren werden
// unterdrueckt. Fuer die Analyse echter Schuesse SET TESTCOOLDOWN deutlich
// kleiner stellen (z.B. 50ms), um alle Ereignisse je Sensor zu sehen.
#define TEST_IDLE_US             5000000ULL   // Trennlinie nach 5s Stille
static bool testMode = false;
static volatile bool     testFired[NUM_AIR + 1]   = {false};  // Index NUM_AIR = Piezo
static volatile uint64_t testLastFireUs[NUM_AIR + 1] = {0};
static uint64_t testLastActivityUs = 0;   // fuer 5s-Trennlinie, nur in loop()
static bool     testSeparatorShown = false; // nur 1x Trennlinie je Stille-Periode
static uint64_t testSeriesStartUs  = 0;   // Zeitpunkt des 1. Sensors nach der
                                           // letzten Trennlinie (fuer +ms-Anzeige)
static bool     testSeriesActive   = false;
static const char *TEST_SENSOR_NAMES[NUM_AIR + 1] = {
    "AIR0 links-unten", "AIR1 rechts-unten", "AIR2 links-oben",
    "AIR3 rechts-oben",  "AIR4 links-mitte",  "AIR5 rechts-mitte",
    "PIEZO stahlplatte",
};

// Gemeinsame Testmodus-Behandlung fuer Luft-Mics und Piezo: Cooldown pruefen/
// merken und Meldeflag setzen, komplett unabhaengig von Schuss-/TDOA-Zustand.
static inline void IRAM_ATTR testModeHit(uint8_t sensorIdx, uint64_t nowUs)
{
    const uint64_t cooldownUs = (uint64_t)cfg.testCooldownMs * 1000ULL;
    if (testLastFireUs[sensorIdx] != 0
        && (nowUs - testLastFireUs[sensorIdx]) < cooldownUs) {
        return;
    }
    testLastFireUs[sensorIdx] = nowUs;
    testFired[sensorIdx]      = true;
}

void IRAM_ATTR airISR(void *arg)
{
    const uint32_t idx = (uint32_t)arg;
    const uint64_t nowUs = (uint64_t)esp_timer_get_time();
    if (testMode) { testModeHit((uint8_t)idx, nowUs); return; }

    const uint32_t cc = esp_cpu_get_cycle_count();

    // Totzeit 20 µs zwischen Flanken desselben Kanals: blendet das
    // Eigenschwingen der Piezo-Resonanz (~4,4 kHz) nach jeder Flanke aus.
    // Gilt IMMER, unabhaengig vom Trigger-Zustand - der Ringpuffer laeuft
    // kontinuierlich (siehe Rev-4.9-Hinweis ganz oben).
    if (airRingCount[idx] > 0) {
        uint32_t dCC = cc - airLastCC[idx];          // wrap-sicher
        if (dCC < 20U * cpuMHz) return;
    }
    const uint16_t head = airRingHead[idx];
    airRing[idx][head] = cc;
    airLastCC[idx]      = cc;
    airRingHead[idx]    = (uint16_t)((head + 1) % AIR_RING_SIZE);
    airRingCount[idx]   = airRingCount[idx] + 1;

    // SET PIEZO=0-Fallback: die erste Luft-Flanke nach Ablauf der Sperrzeit
    // ist der einzige verfuegbare Ausloeser (siehe Rev-4.9-Hinweis oben).
    if (!cfg.usePiezo && !airTrigPending && nowUs >= lockoutUntil) {
        airTrigPendingCC = cc;
        airTrigPendingUs = nowUs;
        airTrigPending   = true;
    }
}

// Piezo (Koerperschall auf der Stahlplatte, siehe PIEZO_PIN) - seit Rev 4.9
// der alleinige, ereignisgesteuerte Ausloeser der Schusserfassung (siehe
// Rev-4.9-Hinweis ganz oben): loop() wartet nach piezoPending noch kurz auf
// nachlaufende Luft-Flanken (SET PIEZOTRAIL/PIEZOMAX) und ruft dann
// processShot() auf, das den Ringpuffer jedes Mikrofons relativ zu
// piezoPendingCC durchsucht.
static volatile bool     piezoPending   = false;
static volatile uint32_t piezoPendingCC = 0;
static volatile uint64_t piezoPendingUs = 0;

void IRAM_ATTR piezoISR()
{
    const uint64_t nowUs = (uint64_t)esp_timer_get_time();
    if (testMode) { testModeHit(NUM_AIR, nowUs); return; }
    if (!cfg.usePiezo) return;      // SET PIEZO=0: Fallback uebernimmt airISR()
    if (nowUs < lockoutUntil) return;
    if (piezoPending) return;        // voriges Ereignis wartet noch auf Auswertung

    piezoPendingCC = esp_cpu_get_cycle_count();
    piezoPendingUs = nowUs;
    piezoPending   = true;
}

// Setzt nur noch die Trigger-Zustaende zurueck - die Ringpuffer selbst
// laufen unabhaengig davon kontinuierlich weiter (siehe Rev-4.9-Hinweis oben).
static void resetShotState()
{
    noInterrupts();
    piezoPending   = false;
    airTrigPending = false;
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
    // Trailer des Aufrufers (falls vorhanden) abschneiden und einheitlich
    // neu anhaengen: auf Serial IMMER "\r\n" - reines "\n" laesst manches
    // Terminal (z.B. PuTTY im Raw-Modus) zwar eine Zeile nach unten
    // springen, aber nicht an den Zeilenanfang zurueckkehren (fehlendes
    // CR). Der TCP-Stream zum Stand-PC bleibt bewusst reines "\n" (JSON-
    // Zeilenstrom, kein Terminal).
    size_t len = strlen(line);
    while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) len--;

    Serial.write((const uint8_t *)line, len);
    Serial.print("\r\n");

    if (!wifiEnabled) return;

    if (tcp.connected()) {
        while (!txBufEmpty() && tcp.connected()) {
            tcp.print(txBuf[txTail]);
            txTail = (txTail + 1) % TXBUF_SLOTS;
        }
        if (tcp.connected()) {
            tcp.write((const uint8_t *)line, len);
            tcp.print('\n');
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
          "\"debounce_ms\":%u,\"mics\":%d,\"wifi\":\"%s\",\"tcp\":%s,"
          "\"buffered\":%u,\"test_mode\":%d}\n",
          FW_VERSION, cfg.lane,
          (unsigned long long)(esp_timer_get_time() / 1000000ULL),
          shotCounter, cfg.windowMs, cfg.debounceMs, NUM_AIR,
          ip, tcp.connected() ? "true" : "false",
          (unsigned)((txHead - txTail + TXBUF_SLOTS) % TXBUF_SLOTS),
          testMode ? 1 : 0);
}

static void sendShowConfig()
{
    char ofsBuf[80], mpxBuf[80], mpyBuf[80], mpzBuf[80];
    int  on = 0, onx = 0, ony = 0, onz = 0;
    for (int i = 0; i < NUM_AIR; i++) {
        on  += snprintf(ofsBuf + on,  sizeof(ofsBuf) - on,  "%s%ld", i > 0 ? "," : "", (long)cfg.micOffsetNs[i]);
        onx += snprintf(mpxBuf + onx, sizeof(mpxBuf) - onx, "%s%ld", i > 0 ? "," : "", (long)cfg.micPosOffsetXUm[i]);
        ony += snprintf(mpyBuf + ony, sizeof(mpyBuf) - ony, "%s%ld", i > 0 ? "," : "", (long)cfg.micPosOffsetYUm[i]);
        onz += snprintf(mpzBuf + onz, sizeof(mpzBuf) - onz, "%s%ld", i > 0 ? "," : "", (long)cfg.micPosOffsetZUm[i]);
    }
    // Eigener, grosszuegig bemessener Puffer statt emitf() (dessen interner
    // Puffer nur TXBUF_LINE=320 Byte fasst) - die SHOW-Zeile ist mit allen
    // Feldern (SSID/Host/IP/Offsets/Piezo/...) laenger und wuerde sonst
    // stillschweigend abgeschnitten.
    char line[1100];
    int  n = snprintf(line, sizeof(line),
          "{\"type\":\"config\",\"ssid\":\"%s\",\"pass\":\"%s\","
          "\"host\":\"%s\",\"port\":%u,\"lane\":%u,"
          "\"debounce_ms\":%u,\"window_ms\":%u,\"debug\":%d,"
          "\"outlier_um\":%u,\"cluster_radius_um\":%u,\"min_cluster_hits\":%d,"
          "\"max_precision_um\":%u,\"min_mics\":%d,\"ring_buffer_ms\":%u,"
          "\"mic_offset_ns\":[%s],"
          "\"mic_pos_offset_x_um\":[%s],\"mic_pos_offset_y_um\":[%s],"
          "\"mic_pos_offset_z_um\":[%s],\"cal_shots\":%d,"
          "\"target\":\"%s\",\"standoff_steel_mm\":%.2f,\"standoff_paper_mm\":%.2f,"
          "\"use_piezo\":%d,\"piezo_max_us\":%u,"
          "\"piezo_lead_max_us\":%u,\"piezo_lead_min_us\":%u,\"piezo_trail_max_us\":%u,"
          "\"test_cooldown_ms\":%u,\"offset_x_um\":%ld,\"offset_y_um\":%ld,"
          "\"sound_mps\":%u,"
          "\"static_ip\":%d,\"ip\":\"%s\",\"gateway\":\"%s\","
          "\"subnet\":\"%s\",\"dns\":\"%s\"}\n",
          cfg.ssid.c_str(),
          cfg.pass.length() ? "****" : "",
          cfg.host.c_str(), cfg.port, cfg.lane,
          cfg.debounceMs, cfg.windowMs, cfg.debug,
          cfg.airOutlierUm, cfg.clusterRadiusUm, cfg.minClusterHits,
          cfg.maxPrecisionUm, cfg.minMics, cfg.ringBufferMs,
          ofsBuf, mpxBuf, mpyBuf, mpzBuf, cfg.calShotCount,
          cfg.targetMode == TARGET_PAPER ? "paper" : "steel",
          cfg.standoffSteelMm, cfg.standoffPaperMm,
          cfg.usePiezo ? 1 : 0, cfg.piezoMaxUs,
          cfg.piezoLeadMaxUs, cfg.piezoLeadMinUs, cfg.piezoTrailMaxUs,
          cfg.testCooldownMs, (long)cfg.offsetXUm, (long)cfg.offsetYUm,
          cfg.soundSpeedMps,
          cfg.staticIP ? 1 : 0, cfg.ip.c_str(), cfg.gateway.c_str(),
          cfg.subnet.c_str(), cfg.dns.c_str());
    (void)n;
    emitLine(line);
}

// ---------------------------------------------------------------------------
// Positionsberechnung fuer die Luftschall-Messung
// ---------------------------------------------------------------------------
// Lokales Koordinatensystem der Abprallflaeche: Ursprung = Plattenzentrum,
// x nach rechts, y nach oben (aus Schuetzensicht), z senkrecht von der
// Platte weg Richtung Schuetze. Mic-Reihenfolge identisch zu AIR_PINS[]:
//   0 = GPIO25 = links unten   1 = GPIO26 = rechts unten
//   2 = GPIO27 = links oben    3 = GPIO14 = rechts oben
//   4 = GPIO32 = links mitte   5 = GPIO33 = rechts mitte

// x-Abstand Mic-Spalte<->vertikale Mittellinie ist bei beiden Zielarten
// (Stahl/Papier) baugleich, daher ein fester Wert fuer beide Modi.
#define MIC_HALF_X          115.0f  // mm, horizontaler Abstand Mic-Spalte<->Zentrum
// Stahlblech (Abprallflaeche, Rev <= 4.x Default): Mics 100mm ueber/unter
// Mitte, rechtwinklig vor der Platte (Standoff, Default 30mm - je nach
// Frontplattenstaerke abweichend, seit Rev 4.6 per SET STANDOFFSTEEL
// konfigurierbar, siehe cfg.standoffSteelMm/applyTargetGeometry()).
#define MIC_HALF_Y_STEEL    100.0f  // mm
#define MIC_STANDOFF_STEEL   30.0f  // mm, nur Initialwert - siehe oben
// Papierscheibe (Durchschlag-Messung, SET TARGET=PAPER): Mics 85mm
// ueber/unter Mitte, rechtwinklig vor der Scheibe (Standoff, Default 28mm,
// per SET STANDOFFPAPER konfigurierbar, siehe cfg.standoffPaperMm).
#define MIC_HALF_Y_PAPER     85.0f  // mm
#define MIC_STANDOFF_PAPER   28.0f  // mm, nur Initialwert - siehe oben

// Schallgeschwindigkeit ist zur Laufzeit konfigurierbar (SET SOUNDSPEED,
// cfg.soundSpeedMps, Default 355 m/s - empirisch ermittelt, hoeher als die
// klassischen 343 m/s/20C) - siehe applySoundSpeed(). Ein radial mit dem
// Abstand vom Zentrum WACHSENDER Fehler (Treffer am Rand werden zu nah am
// Zentrum berechnet) ist ein typisches Anzeichen fuer eine zu NIEDRIG
// angenommene Schallgeschwindigkeit und kann darueber ausgeglichen werden.
// Wird zusaetzlich automatisch durch CAL START mitkalibriert, siehe
// runCalibration() weiter unten.
#define SOUND_SPEED_MIN_MPS  300
#define SOUND_SPEED_MAX_MPS  400
static float soundMmPerNs = 0.000355f;

// Setzt soundMmPerNs passend zum per SET SOUNDSPEED gewaehlten Wert. Wird
// beim Booten (nach loadConfig()) und bei jeder Aenderung von SET SOUNDSPEED
// aufgerufen - wirkt sofort, kein Reboot noetig.
static void applySoundSpeed()
{
    soundMmPerNs = (float)cfg.soundSpeedMps * 1.0e-6f;   // (m/s) -> mm/ns
}

// Schwelle fuer "Mikrofon-Ausreisser" ist zur Laufzeit konfigurierbar:
// SET OUTLIER=<0.001mm>, siehe cfg.airOutlierUm (Default 5000 = 5.0mm).
// Zulaessiger Bereich fuer den per Kalibrierung (CAL START) ermittelten
// bzw. per SET OFS<i> manuell gesetzten Timing-Offset je Mikrofon - bewusst
// eng an einem physikalisch plausiblen Kabel-/Komparator-Zeitversatz
// gehalten. War versuchsweise auf 150000 angehoben (Testfrage: findet CAL
// START bei viel Spielraum einen stabilen Wert?) - Antwort: ja, aber im
// Bereich von zehntausenden ns, also weit ausserhalb dessen, was ein
// echter Hardware-Effekt erklaeren koennte (das entspraeche Kabellaengen
// im Kilometerbereich). Das war Ueberanpassung an die jeweiligen
// Kalibrier-Schuesse, kein uebertragbarer Kalibrierwert - daher wieder
// auf den urspruenglichen, plausiblen Bereich zurueckgesetzt.
#define MIC_OFS_MAX_NS  5000

// Zulaessiger Bereich fuer die per-Mikrofon-Positionsjustage (SET MPOSX/
// MPOSY/MPOSZ<i>, 0.001mm) - etwas grosszuegiger als der von tools/
// calibrate_mics.py durchsuchte Bereich (+-3mm, siehe dortiger Kommentar),
// damit ein manuell nachjustierter Wert nicht knapp an der SET-Grenze haengt.
#define MIC_POS_OFS_MAX_UM  5000

// Nominal-Positionen (Baugeometrie ohne Justage) - MIC_X_NOM ist fuer beide
// Zielmodi gleich, MIC_Y_NOM haengt vom Modus ab (siehe applyTargetGeometry()).
static const float MIC_X_NOM[NUM_AIR] = { -MIC_HALF_X, +MIC_HALF_X, -MIC_HALF_X, +MIC_HALF_X, -MIC_HALF_X, +MIC_HALF_X };

// MIC_X/MIC_Y/MIC_Z sind die EFFEKTIVEN (laufzeitveraenderlichen) Mic-
// Positionen = Nominal-Geometrie + Montage-Justage (SET MPOSX/MPOSY/MPOSZ<i>,
// Default 0 - siehe applyTargetGeometry() unten). MIC_Z ersetzt den bisher
// EINEN gemeinsamen Standoff-Skalar durch einen Wert je Mikrofon, damit
// jedes Mikrofon einen eigenen Z-Justagewert bekommen kann.
static float MIC_X[NUM_AIR] = { -MIC_HALF_X, +MIC_HALF_X, -MIC_HALF_X, +MIC_HALF_X, -MIC_HALF_X, +MIC_HALF_X };
static float MIC_Y[NUM_AIR] = { -MIC_HALF_Y_STEEL, -MIC_HALF_Y_STEEL, +MIC_HALF_Y_STEEL, +MIC_HALF_Y_STEEL, 0.0f, 0.0f };
static float MIC_Z[NUM_AIR] = { MIC_STANDOFF_STEEL, MIC_STANDOFF_STEEL, MIC_STANDOFF_STEEL,
                                 MIC_STANDOFF_STEEL, MIC_STANDOFF_STEEL, MIC_STANDOFF_STEEL };

// Setzt MIC_X[]/MIC_Y[]/MIC_Z[] passend zum per SET TARGET gewaehlten
// Messmodus UND den per-Mikrofon-Justagewerten (SET MPOSX/MPOSY/MPOSZ<i>).
// Wird beim Booten (nach loadConfig()) und bei jeder Aenderung von SET
// TARGET/MPOSX/MPOSY/MPOSZ/STANDOFFSTEEL/STANDOFFPAPER aufgerufen - wirkt
// sofort, kein Reboot noetig.
static void applyTargetGeometry()
{
    const float halfY   = (cfg.targetMode == TARGET_PAPER) ? MIC_HALF_Y_PAPER : MIC_HALF_Y_STEEL;
    const float baseZ   = (cfg.targetMode == TARGET_PAPER) ? cfg.standoffPaperMm : cfg.standoffSteelMm;
    static const float yNomSign[NUM_AIR] = { -1.0f, -1.0f, +1.0f, +1.0f, 0.0f, 0.0f };
    for (int i = 0; i < NUM_AIR; i++) {
        MIC_X[i] = MIC_X_NOM[i]      + (float)cfg.micPosOffsetXUm[i] / 1000.0f;
        MIC_Y[i] = yNomSign[i]*halfY + (float)cfg.micPosOffsetYUm[i] / 1000.0f;
        MIC_Z[i] = baseZ             + (float)cfg.micPosOffsetZUm[i] / 1000.0f;
    }
}

// Loest (x,y) aus 2 "Loese"-Mics relativ zu einer Referenz (Zeitnullpunkt)
// per Hyperbel-Trilateration (TDOA). Die Distanz Referenz<->Treffer wird
// als 3. Unbekannte mitgeloest (Linearisierung + quadratische Gleichung).
static bool solveAirPair(int ref, int a, int b, const int64_t tNs[NUM_AIR],
                          float *outX, float *outY, float *outD)
{
    const float Xr = MIC_X[ref], Yr = MIC_Y[ref], Zr = MIC_Z[ref];
    const float ra = (float)(tNs[a] - tNs[ref]) * soundMmPerNs;
    const float rb = (float)(tNs[b] - tNs[ref]) * soundMmPerNs;

    // Gleichung je Mic i: 2(Xi-Xr)x + 2(Yi-Yr)y + 2*ri*d = Ki-Kr-ri^2
    // (Ki = Xi^2+Yi^2+Zi^2 - seit Rev 4.9 per-Mikrofon-Z (MIC_Z[], Montage-
    // Justage SET MPOSZ<i>), daher Zi^2 explizit statt implizit gekuerzt)
    const float A1 = 2.0f * (MIC_X[a] - Xr), B1 = 2.0f * (MIC_Y[a] - Yr), C1 = 2.0f * ra;
    const float D1 = (MIC_X[a]*MIC_X[a] + MIC_Y[a]*MIC_Y[a] + MIC_Z[a]*MIC_Z[a])
                    - (Xr*Xr + Yr*Yr + Zr*Zr) - ra*ra;
    const float A2 = 2.0f * (MIC_X[b] - Xr), B2 = 2.0f * (MIC_Y[b] - Yr), C2 = 2.0f * rb;
    const float D2 = (MIC_X[b]*MIC_X[b] + MIC_Y[b]*MIC_Y[b] + MIC_Z[b]*MIC_Z[b])
                    - (Xr*Xr + Yr*Yr + Zr*Zr) - rb*rb;

    // x = x0 + x1*d, y = y0 + y1*d (Cramer'sche Regel)
    const float det = A1*B2 - A2*B1;
    if (fabsf(det) < 1e-6f) return false;   // a, b kollinear mit ref

    const float x0 = (D1*B2 - D2*B1) / det;
    const float x1 = (C2*B1 - C1*B2) / det;
    const float y0 = (A1*D2 - A2*D1) / det;
    const float y1 = (A2*C1 - A1*C2) / det;

    // Einsetzen in d^2 = (x-Xr)^2 + (y-Yr)^2 + Zr^2 -> quadratisch in d
    const float px = x0 - Xr, py = y0 - Yr;
    const float qa = 1.0f - x1*x1 - y1*y1;
    const float qb = -2.0f * (px*x1 + py*y1);
    const float qc = -(px*px + py*py + Zr*Zr);

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

// Bestimmt die Trefferposition aus den ersten Flanken der erfassten
// Luftmikrofone (3 bis NUM_AIR).
//
// Jede moegliche Dreier-Kombination der erfassten Mics wird direkt geloest
// (kleinste Zeit darunter = Referenz); die dabei NICHT beteiligten, aber
// ebenfalls erfassten Mics dienen als Kontrolle (mittlerer Rest-Fehler
// gegen die geloeste Position). Die Kombination mit dem kleinsten
// mittleren Rest-Fehler gewinnt. Bei genau 3 erfassten Mics gibt es keine
// Kontroll-Mics -> Rest-Fehler immer 0 (keine Redundanz). Je mehr Mics
// erfasst wurden, desto mehr Kontroll-Mics stehen zur Verfuegung und desto
// robuster ist die Ausreisser-Erkennung. Ein grosser Rest-Fehler (>> wenige
// mm) zeigt an, dass die Messung fuer diesen Schuss nicht in sich
// konsistent war (z.B. eine Flanke war ein Echo statt Direktschall) - das
// muss dann der Empfaenger (Stand-PC/Bediener) beurteilen.
// Maximale Zahl an Dreier-Kombinationen bei NUM_AIR=6 Mics: C(6,3) = 20
#define AIR_MAX_COMBOS  20

static bool solveAirPosition(const int64_t tNs[NUM_AIR], const bool seen[NUM_AIR],
                              float clusterRadiusMm, bool emitCandidateDebug,
                              float *outXmm, float *outYmm, float *outResidualMm,
                              float *outPrecisionMm, int *outClusterHits)
{
    int all[NUM_AIR], nAll = 0;
    for (int i = 0; i < NUM_AIR; i++) if (seen[i]) all[nAll++] = i;
    if (nAll < 3) return false;

    // Kandidatenpositionen aller loesbaren Dreier-Kombinationen, fuer die
    // spaetere Praezisions-/Cluster-Auswertung (siehe unten).
    float candX[AIR_MAX_COMBOS], candY[AIR_MAX_COMBOS];
    int   nCand = 0;

    bool  found = false;
    int   bestCandIdx = -1;
    float bestResidual = -1.0f, bestX = 0.0f, bestY = 0.0f;
    for (int i0 = 0; i0 < nAll; i0++) {
    for (int i1 = i0 + 1; i1 < nAll; i1++) {
    for (int i2 = i1 + 1; i2 < nAll; i2++) {
        const int trio[3] = { all[i0], all[i1], all[i2] };
        int ref = trio[0];
        for (int k = 1; k < 3; k++) if (tNs[trio[k]] < tNs[ref]) ref = trio[k];
        int a = -1, b = -1;
        for (int k = 0; k < 3; k++) {
            if (trio[k] == ref) continue;
            if (a < 0) a = trio[k]; else b = trio[k];
        }

        float x, y, d;
        if (!solveAirPair(ref, a, b, tNs, &x, &y, &d)) continue;

        int candIdx = -1;
        if (nCand < AIR_MAX_COMBOS) { candIdx = nCand; candX[nCand] = x; candY[nCand] = y; nCand++; }

        // SET DEBUG=3: jedes einzelne Kombinations-Ergebnis mit den dafuer
        // verwendeten Laufzeiten ausgeben (nur fuer echte Schuesse, nicht
        // waehrend der internen Kalibrier-Kostenauswertung - siehe calCost()).
        if (emitCandidateDebug) {
            emitf("{\"type\":\"cand\",\"seq\":%u,\"x_mm\":%.2f,\"y_mm\":%.2f,"
                  "\"ref\":%d,\"a\":%d,\"b\":%d,"
                  "\"t_ref_ns\":%lld,\"t_a_ns\":%lld,\"t_b_ns\":%lld}\n",
                  sequenceNo, x, y, ref, a, b,
                  (long long)tNs[ref], (long long)tNs[a], (long long)tNs[b]);
        }

        // Rest-Fehler gegen alle erfassten, an dieser Loesung nicht
        // beteiligten Mics (Mittelwert statt nur eines einzelnen Checks).
        float residualSum = 0.0f;
        int   nCheck = 0;
        for (int k = 0; k < nAll; k++) {
            const int m = all[k];
            if (m == ref || m == a || m == b) continue;
            const float dc = sqrtf((x - MIC_X[m])*(x - MIC_X[m])
                                  + (y - MIC_Y[m])*(y - MIC_Y[m])
                                  + MIC_Z[m]*MIC_Z[m]);
            const float rc = (float)(tNs[m] - tNs[ref]) * soundMmPerNs;
            residualSum += fabsf(dc - (d + rc));
            nCheck++;
        }
        const float residual = (nCheck > 0) ? (residualSum / nCheck) : 0.0f;

        if (!found || residual < bestResidual) {
            found = true;
            bestResidual = residual;
            bestX = x;
            bestY = y;
            bestCandIdx = candIdx;
        }
    }}}
    if (!found) return false;

    // Praezision: quadratisch gemittelte Abweichung (RMS) der bis zu 2
    // NAECHSTGELEGENEN zusaetzlichen Kandidatenpositionen (aus den uebrigen
    // Mic-Kombinationen) von der gewaehlten Loesung - bewusst nur die besten
    // 2, damit einzelne weit abweichende Ausreisser-Kombinationen (z.B.
    // durch Echos) den Wert nicht dominieren. Bei genau 3 erfassten Mics
    // gibt es keine zusaetzliche Kombination -> precision immer 0.
    // cluster_hits zaehlt weiterhin ALLE Kandidaten innerhalb von
    // clusterRadiusMm um die Loesung (SET RADIUS).
    float d1 = -1.0f, d2 = -1.0f;   // zwei kleinste Abstaende (mm)
    int   inRadius = 0;
    for (int i = 0; i < nCand; i++) {
        const float dx = candX[i] - bestX, dy = candY[i] - bestY;
        const float dist = sqrtf(dx*dx + dy*dy);
        if (dist <= clusterRadiusMm) inRadius++;
        if (i == bestCandIdx) continue;
        if (d1 < 0.0f || dist < d1)      { d2 = d1; d1 = dist; }
        else if (d2 < 0.0f || dist < d2) { d2 = dist; }
    }
    int   nNear = (d1 >= 0.0f ? 1 : 0) + (d2 >= 0.0f ? 1 : 0);
    float sumSq = (d1 >= 0.0f ? d1*d1 : 0.0f) + (d2 >= 0.0f ? d2*d2 : 0.0f);

    *outXmm         = bestX;
    *outYmm         = bestY;
    *outResidualMm  = bestResidual;
    *outPrecisionMm = (nNear > 0) ? sqrtf(sumSq / (float)nNear) : 0.0f;
    *outClusterHits = inRadius;
    return true;
}

// ---------------------------------------------------------------------------
// Timing-Offset-Kalibrierung (CAL START)
// ---------------------------------------------------------------------------
// Idee: Elektronik-/Kabellaufzeit-Unterschiede zwischen den Mikrofon-Kanaelen
// (LM339-Komparator-Schwelle, Kabellaenge) aeussern sich als naeherungsweise
// KONSTANTER Timing-Fehler je Mikrofon (0.343 mm/ns - schon wenige ns
// entsprechen mm-Abweichungen) und dominieren typischerweise die mechanische
// Einbautoleranz der Mikrofone. calCost()/runCalibration() bestimmen daher
// GEZIELT nur einen skalaren ns-Offset je Mikrofon, keine 3D-Neuvermessung
// der Mic-Positionen - letzteres waere ein deutlich schwierigeres
// Selbstlokalisierungsproblem (unbekannte Mic- UND Schusspositionen
// gleichzeitig) und mit wenigen Schuessen an unbekannter Position numerisch
// riskant.
//
// Eichfreiheitsgrad: Eine gemeinsame Konstante zu ALLEN Offsets addiert
// aendert an keiner TDOA-Differenz etwas (sie kuerzt sich in jeder
// Zeitdifferenz zwischen zwei Mics weg) - Offset[0] wird deshalb fix auf 0
// gehalten, alle anderen Offsets sind relativ dazu.
//
// Optimierung: Pattern-Search / Koordinatenabstieg je Mikrofon (kein
// Matrix-Solver noetig): Fuer jedes Mic wird abwechselnd ein kleiner
// Schritt in beide Richtungen probiert und die Aenderung uebernommen, die
// die Kosten (Summe der solveAirPosition()-Rest-Fehler ueber alle
// Kalibrier-Schuesse) am staerksten senkt; die Schrittweite wird pro Runde
// halbiert (grob -> fein).
//
// Rev 4.4-4.6 hatten hier zusaetzlich eine automatische Schallgeschwindig-
// keits-Kalibrierung (SET SOUNDSPEED wurde mitoptimiert, wahlweise per
// Koordinatenabstieg auf precision_um oder per analytischer Median-
// Schaetzung). In der Praxis lieferte das aber wiederholt schlechte/
// unplausible Ergebnisse (vermutlich weil die Kalibrier-Schuesse durch
// Mehrwege-/Nachhall-Effekte und die AIR4/AIR5-Hardwarethematik oft selbst
// nicht sauber genug sind, um daraus zuverlaessig auf v zu schliessen) und
// wurde deshalb wieder entfernt - SET SOUNDSPEED ist wieder ein rein
// manueller Parameter, den CAL START nicht mehr anfasst.

// Kosten der aktuell angenommenen Offsets: Summe des solveAirPosition()-
// Rest-Fehlers ueber alle gesammelten Kalibrier-Schuesse. Schuesse, die mit
// den aktuellen Offsets keine Loesung mehr ergeben (geometrisch entartet),
// werden mit einem Strafwert belegt statt ignoriert zu werden.
static float calCost(const float offsets[NUM_AIR])
{
    float total = 0.0f;
    for (int k = 0; k < calCollected; k++) {
        int64_t corrected[NUM_AIR];
        for (int i = 0; i < NUM_AIR; i++) {
            corrected[i] = calSeenBuf[k][i]
                         ? calRawNs[k][i] - (int64_t)lroundf(offsets[i])
                         : 0;
        }
        float x, y, res, prec;
        int   hitsN;
        if (solveAirPosition(corrected, calSeenBuf[k], 0.0f, false,
                              &x, &y, &res, &prec, &hitsN)) {
            total += res;
        } else {
            total += 1000.0f;   // Strafe: Offsets machen Schuss unloesbar
        }
    }
    return total;
}

static void runCalibration()
{
    float offsets[NUM_AIR];
    for (int i = 0; i < NUM_AIR; i++) offsets[i] = 0.0f;   // Neukalibrierung

    const int   refMic = 0;      // Eichfreiheitsgrad: fix auf Offset 0
    // Startschrittweite/Rundenzahl bewusst so gewaehlt, dass die Summe aller
    // Schritte (geometrische Reihe, Faktor 0.5) den vollen erlaubten Bereich
    // (+-MIC_OFS_MAX_NS=5000, siehe Kommentar dort) tatsaechlich erreichen
    // kann: 2500*(2-2^-8) ~ 4990ns, letzter Schritt ~9.8ns.
    float       stepNs = 2500.0f;
    for (int pass = 0; pass < 9; pass++) {
        for (int i = 0; i < NUM_AIR; i++) {
            if (i == refMic) continue;
            const float base     = offsets[i];
            const float baseCost = calCost(offsets);

            offsets[i] = constrain(base + stepNs, (float)-MIC_OFS_MAX_NS, (float)MIC_OFS_MAX_NS);
            const float costPlus = calCost(offsets);

            offsets[i] = constrain(base - stepNs, (float)-MIC_OFS_MAX_NS, (float)MIC_OFS_MAX_NS);
            const float costMinus = calCost(offsets);

            if (baseCost <= costPlus && baseCost <= costMinus) {
                offsets[i] = base;
            } else if (costPlus < costMinus) {
                offsets[i] = constrain(base + stepNs, (float)-MIC_OFS_MAX_NS, (float)MIC_OFS_MAX_NS);
            } else {
                offsets[i] = constrain(base - stepNs, (float)-MIC_OFS_MAX_NS, (float)MIC_OFS_MAX_NS);
            }
        }
        stepNs *= 0.5f;
    }

    char line[220];
    int  n = snprintf(line, sizeof(line),
                      "{\"type\":\"cal\",\"state\":\"done\",\"offsets_ns\":[");
    for (int i = 0; i < NUM_AIR; i++) {
        cfg.micOffsetNs[i] = (int32_t)lroundf(offsets[i]);
        char key[8];
        snprintf(key, sizeof(key), "ofs%d", i);
        saveVal<int32_t>(key, cfg.micOffsetNs[i]);
        n += snprintf(line + n, sizeof(line) - n, "%s%ld",
                      i > 0 ? "," : "", (long)cfg.micOffsetNs[i]);
    }
    snprintf(line + n, sizeof(line) - n, "],\"shots\":%d}\n", calCollected);
    emitLine(line);
    sendShowConfig();   // komplette Konfiguration direkt im Anschluss zeigen
    sendCalRawDump();   // Rohdaten aller Kalibrier-Schuesse wiederverwendbar ausgeben
}

// Gibt die ROHEN (unkorrigierten) Mikrofon-Messwerte aller gerade
// gesammelten Kalibrier-Schuesse aus (calRawNs/calSeenBuf - dieselben
// Werte, aus denen runCalibration() oben die Offsets berechnet hat) - eine
// Zeile je Schuss, im selben air_ns-Format wie ein normales "shot"-
// Telegramm (siehe processShot()). Zweck: diese Rohdaten liessen sich
// bisher nur zufaellig ueber SET DEBUG>=1 sichtbare "shot"-Telegramme
// rekonstruieren (und "sauber" gefilterte Schuesse mit SET DEBUG=0 gar
// nicht) - mit dieser Ausgabe steht am Ende von CAL START immer eine
// vollstaendige, direkt wiederverwendbare Kopie bereit (z.B. fuer
// tools/calibrate_mics.py oder um spaeter mit anderen Parametern
// offline nachzurechnen, ohne erneut schiessen zu muessen).
// Da Piezo (bei aktivem SET PIEZO) selbst den Zeit-Nullpunkt bildet (siehe
// Rev-4.9-Hinweis ganz oben), IST die Piezo-Information bereits vollstaendig
// in air_ns enthalten (jeder Wert = Abstand dieses Mikrofons zu Piezo) - ein
// separater "piezo_ns"-Wert waere hier immer 0 und daher nicht informativ.
static void sendCalRawDump()
{
    for (int k = 0; k < calCollected; k++) {
        char cline[300];
        int  cn = snprintf(cline, sizeof(cline),
                           "{\"type\":\"calshot\",\"idx\":%d,\"trigger\":\"%s\",\"air_ns\":[",
                           k, cfg.usePiezo ? "piezo" : "air_fallback");
        for (int i = 0; i < NUM_AIR; i++) {
            if (i > 0) cline[cn++] = ',';
            if (calSeenBuf[k][i]) {
                cn += snprintf(cline + cn, sizeof(cline) - cn, "%lld", (long long)calRawNs[k][i]);
            } else {
                cn += snprintf(cline + cn, sizeof(cline) - cn, "null");
            }
        }
        snprintf(cline + cn, sizeof(cline) - cn, "]}\n");
        emitLine(cline);
    }
}

// ---------------------------------------------------------------------------
// Ringpuffer-Auswertung (Rev 4.9) - siehe Rev-4.9-Hinweis ganz oben
// ---------------------------------------------------------------------------

// Wrap-sichere SIGNIERTE Zeitdifferenz (a - b) in ns fuer den 32-Bit CPU-
// Zykluszaehler: die Differenz zweier uint32_t-Werte "wrapt" bei
// Unterlauf/Ueberlauf automatisch korrekt (Zweierkomplement), die
// Reinterpretation als int32_t liefert daraus das richtige VORZEICHEN -
// vorausgesetzt |a-b| liegt klar unter der int32_t-Wrap-Periode (~8,9s bei
// 240MHz), was fuer alle hier verwendeten ms-Fenster immer gilt.
static inline int64_t ccDiffNs(uint32_t a, uint32_t b)
{
    return (int64_t)((int32_t)(a - b)) * 1000LL / (int64_t)cpuMHz;
}

// Sucht in Mikrofon i's (lokal gespiegeltem) Ringpuffer die zu refCC am
// naechsten liegende Flanke innerhalb [loNs,hiNs] (relativ zu refCC),
// ausserhalb eines optionalen Ausschlussbereichs (exLoNs,exHiNs]
// (SET PIEZOLEADMIN; exLoNs>=exHiNs deaktiviert den Ausschluss). Der
// Ringpuffer ist chronologisch geordnet -> die Rueckwaertssuche vom
// neuesten Eintrag kann abbrechen, sobald ns<loNs unterschritten wird (alle
// noch aelteren Eintraege liegen dann garantiert ebenfalls ausserhalb).
static bool ringFindEdge(const uint32_t ring[AIR_RING_SIZE], uint16_t head,
                          uint32_t count, uint32_t refCC,
                          int64_t loNs, int64_t hiNs,
                          int64_t exLoNs, int64_t exHiNs, int64_t *outNs)
{
    uint32_t cnt = (count > AIR_RING_SIZE) ? AIR_RING_SIZE : count;
    bool     found = false;
    int64_t  bestNs = 0, bestDist = 0;
    for (uint32_t k = 0; k < cnt; k++) {
        const uint16_t idx = (uint16_t)((head + AIR_RING_SIZE - 1 - k) % AIR_RING_SIZE);
        const int64_t  ns  = ccDiffNs(ring[idx], refCC);
        if (ns < loNs) break;              // alle weiteren (aelteren) Eintraege auch zu alt
        if (ns > hiNs) continue;           // noch zu neu (Vorwaertsfenster: kommt vor)
        if (exLoNs < exHiNs && ns > exLoNs && ns <= exHiNs) continue;  // Ausschlussbereich
        const int64_t dist = (ns >= 0) ? ns : -ns;
        if (!found || dist < bestDist) { found = true; bestNs = ns; bestDist = dist; }
    }
    if (found) *outNs = bestNs;
    return found;
}

static void processShot(TriggerSource src, uint32_t refCC, uint64_t refUs)
{
    // static statt lokal auf dem Stack: der gespiegelte Ringpuffer allein
    // waere schon NUM_AIR*AIR_RING_SIZE*4 = 3072 Byte, dazu weiter unten
    // "line" (bis zu 3800 Byte) - beides zusammen auf dem Stack der Arduino-
    // Loop-Task (Default 8KB) plus die Aufrufkette (solveAirPosition() etc.)
    // laesst zu wenig Reserve und kann zu einem stillen Stack-Overflow/
    // Absturz fuehren (siehe historischer Stack-Overflow-Fund). processShot()
    // wird ausschliesslich aus loop() heraus aufgerufen (nie parallel/
    // rekursiv), daher ist "static" hier sicher.
    static uint32_t localRing[NUM_AIR][AIR_RING_SIZE];
    uint16_t localRingHead[NUM_AIR];
    uint32_t localRingCount[NUM_AIR];

    noInterrupts();
    for (int i = 0; i < NUM_AIR; i++) {
        localRingHead[i]  = airRingHead[i];
        localRingCount[i] = airRingCount[i];
        for (int s = 0; s < AIR_RING_SIZE; s++) localRing[i][s] = airRing[i][s];
    }
    lockoutUntil = (uint64_t)esp_timer_get_time()
                 + (uint64_t)cfg.debounceMs * 1000ULL;
    interrupts();

    // ROHE (unkorrigierte) Auswahl-Flankenzeiten je Mikrofon, in ns relativ
    // zu refCC (Piezo bzw. im SET PIEZO=0-Fallback die ausloesende Luft-
    // Flanke selbst) - siehe Rev-4.9-Hinweis ganz oben fuer die Herleitung
    // der Fenstergrenzen je Modus. Wird unabhaengig vom Reject-Filter
    // berechnet, da auch fuer die Kalibrierung gebraucht.
    int64_t airT0NsRaw[NUM_AIR];
    bool    airSeen[NUM_AIR];

    if (src == TRIG_PIEZO) {
        int64_t loNs, hiNs, exLoNs, exHiNs;
        if (cfg.targetMode == TARGET_PAPER) {
            // Rueckwaerts-Suche: [-PIEZOLEAD..-PIEZOLEADMIN] vor Piezo, plus
            // ein Nachlauf-Fenster [0..+PIEZOTRAIL] danach (das echte
            // Treffer-Cluster streut leicht UM die Piezo-Zeit). SET
            // RINGBUFFER deckelt PIEZOLEAD zusaetzlich grob nach oben.
            const uint32_t ringLeadUs = cfg.ringBufferMs * 1000UL;
            const uint32_t leadUs = (cfg.piezoLeadMaxUs < ringLeadUs) ? cfg.piezoLeadMaxUs : ringLeadUs;
            loNs   = -(int64_t)leadUs * 1000LL;
            hiNs   =  (int64_t)cfg.piezoTrailMaxUs * 1000LL;
            exLoNs = -(int64_t)cfg.piezoLeadMinUs * 1000LL;
            exHiNs =  0;
        } else {
            // TARGET_STEEL: Piezo sitzt auf der Trefferflaeche und loest VOR
            // den Luftmikrofonen aus -> reine Vorwaerts-Suche [0..PIEZOMAX].
            loNs = 0;
            hiNs = (int64_t)cfg.piezoMaxUs * 1000LL;
            exLoNs = exHiNs = 0;   // kein Ausschlussbereich
        }
        for (int i = 0; i < NUM_AIR; i++) {
            int64_t ns;
            airSeen[i] = ringFindEdge(localRing[i], localRingHead[i], localRingCount[i],
                                       refCC, loNs, hiNs, exLoNs, exHiNs, &ns);
            airT0NsRaw[i] = airSeen[i] ? ns : 0;
        }
    } else {
        // TRIG_FALLBACK_AIR (SET PIEZO=0): einfaches Vorwaerts-Fenster ab
        // der ausloesenden Luft-Flanke, analog zum urspruenglichen "erste
        // Flanke oeffnet Sammelfenster"-Verhalten - je Mikrofon die
        // fruehste Flanke im Fenster (entspricht bei einem rein
        // vorwaertsgerichteten Fenster automatisch der zu refCC
        // naechstgelegenen, siehe ringFindEdge()).
        const int64_t hiNs = (int64_t)cfg.windowMs * 1000000LL;
        for (int i = 0; i < NUM_AIR; i++) {
            int64_t ns;
            airSeen[i] = ringFindEdge(localRing[i], localRingHead[i], localRingCount[i],
                                       refCC, 0, hiNs, 0, 0, &ns);
            airT0NsRaw[i] = airSeen[i] ? ns : 0;
        }
    }

    int airHits = 0;
    for (int i = 0; i < NUM_AIR; i++) if (airSeen[i]) airHits++;

    // Kalibrierung (CAL START) laeuft unabhaengig vom normalen Reject-Filter
    // und vom SET DEBUG-Level mit - der Bediener braucht sofort Feedback.
    // Trilateration braucht grundsaetzlich mindestens 3 Mics, unabhaengig
    // von der (ggf. hoeher gesetzten) SET MINMICS-Schwelle.
    if (calActive) {
        if (airHits < 3) {
            emitf("{\"type\":\"cal\",\"state\":\"skipped\",\"reason\":\"only %d mic(s)\","
                  "\"progress\":%d,\"need\":%d}\n",
                  airHits, calCollected, cfg.calShotCount);
        } else if (calCollected < MAX_CAL_SHOTS) {
            for (int i = 0; i < NUM_AIR; i++) {
                calSeenBuf[calCollected][i] = airSeen[i];
                calRawNs[calCollected][i]   = airT0NsRaw[i];
            }
            calCollected++;
            if (calCollected >= cfg.calShotCount) {
                runCalibration();
                calActive    = false;
                calCollected = 0;
            } else {
                emitf("{\"type\":\"cal\",\"state\":\"waiting\",\"progress\":%d,"
                      "\"need\":%d}\n", calCollected, cfg.calShotCount);
            }
        }
    }

    if (airHits < cfg.minMics) {
        if (cfg.debug >= 2) {
            emitf("{\"type\":\"reject\",\"reason\":\"only %d mic(s)\","
                  "\"hits\":%d,\"triggered_by\":\"%s\"}\n",
                  airHits, airHits, src == TRIG_PIEZO ? "piezo" : "air");
        }
        return;
    }

    shotCounter++;
    sequenceNo++;

    // Kalibrierten Timing-Offset je Mikrofon abziehen (SET OFS<i> bzw.
    // CAL START, Default 0) - siehe runCalibration().
    int64_t airT0Ns[NUM_AIR];
    for (int i = 0; i < NUM_AIR; i++) {
        airT0Ns[i] = airSeen[i] ? airT0NsRaw[i] - cfg.micOffsetNs[i] : 0;
    }
    float posX = 0.0f, posY = 0.0f, posRes = 0.0f, posPrecision = 0.0f;
    int   clusterHits = 0;
    bool  posOk = solveAirPosition(airT0Ns, airSeen,
                                    (float)cfg.clusterRadiusUm / 1000.0f,
                                    cfg.debug >= 3,
                                    &posX, &posY, &posRes,
                                    &posPrecision, &clusterHits);
    long  xUm    = posOk ? lroundf(posX         * 1000.0f) : 0;
    long  yUm    = posOk ? lroundf(posY         * 1000.0f) : 0;
    long  resUm  = posOk ? lroundf(posRes       * 1000.0f) : 0;
    long  precUm = posOk ? lroundf(posPrecision * 1000.0f) : 0;
    int   clusterN = posOk ? clusterHits : 0;
    // Konstante Nachkorrektur (SET OFFSETX/OFFSETY, Default 0) - erst NACH
    // der Trilateration angewandt, z.B. zum Ausgleich einer Messgitter-
    // Verschiebung. Wirkt sich NICHT auf pos_res_um/precision_um aus, da
    // die reine Geometrieguete der Loesung unveraendert bleibt.
    if (posOk) {
        xUm += cfg.offsetXUm;
        yUm += cfg.offsetYUm;
    }

    // static statt lokal - unkritisch da processShot() nie parallel/rekursiv
    // aufgerufen wird (siehe Kommentar bei localRing oben).
    static char line[600];
    int  n = snprintf(line, sizeof(line),
                      "{\"type\":\"shot\",\"seq\":%u,\"air_ns\":[", sequenceNo);
    for (int i = 0; i < NUM_AIR; i++) {
        if (i > 0) line[n++] = ',';
        if (airSeen[i]) {
            n += snprintf(line + n, sizeof(line) - n, "%lld", (long long)airT0NsRaw[i]);
        } else {
            n += snprintf(line + n, sizeof(line) - n, "null");
        }
    }
    n += snprintf(line + n, sizeof(line) - n,
                  "],\"x_um\":%ld,\"y_um\":%ld,\"pos_res_um\":%ld,"
                  "\"precision_um\":%ld,\"cluster_hits\":%d,\"pos_valid\":%d,"
                  "\"hits\":%d,\"ts\":%llu}\n",
                  xUm, yUm, resUm, precUm, clusterN, posOk ? 1 : 0,
                  airHits, (unsigned long long)(refUs / 1000ULL));

    // DEBUG=0 (Default): nur sauber ermittelte Schuesse (kein Ausreisser,
    // Position bestimmbar, genug uebereinstimmende Mic-Kombinationen,
    // precision_um innerhalb der Schwelle) werden ausgegeben. DEBUG>=1: auch
    // Schuesse mit Mikrofon-Ausreisser oder nicht bestimmbarer Position.
    // Ein durch den Muendungsknall verfrueht ausgeloester Schuss kann seit
    // Rev 4.9 gar nicht mehr erst hier ankommen (Piezo IST der Ausloeser,
    // siehe Rev-4.9-Hinweis ganz oben) - die fruehere piezo_ok-Pruefung
    // entfaellt daher ersatzlos.
    bool isClean = posOk
                 && (unsigned long)resUm < cfg.airOutlierUm
                 && clusterN >= (int)cfg.minClusterHits
                 && (unsigned long)precUm <= cfg.maxPrecisionUm;
    if (isClean || cfg.debug >= 1) {
        emitLine(line);
    }
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
        "#   SET WINDOW=<1-50>        Sammelfenster in ms NUR fuer den SET PIEZO=0-",
        "#                            Fallback (kein Piezo verbaut) relevant",
        "#   SET DEBUG=<0-3>          Ausgabe-Filter: 0=nur saubere Schuesse,",
        "#                            1=+Mikrofon-Ausreisser, 2=+Reject (wenig Hits),",
        "#                            3=+Kandidaten-Zeilen je Mic-Kombination (type=cand)",
        "#   SET OUTLIER=<0-500000>   Ausreisser-Schwelle in 0.001mm (Default 5000)",
        "#   SET RADIUS=<0-500000>    Umkreis fuer cluster_hits in 0.001mm (Default 200)",
        "#   SET MINCLUSTER=<0-20>    Mindest-cluster_hits fuer sauberen Schuss (Default 2)",
        "#   SET MAXPRECISION=<0-500000>   Max. precision_um fuer sauberen Schuss",
        "#                            in 0.001mm (Default 2000)",
        "#   SET MINMICS=<3-6>        Mindestzahl Mics fuer gueltigen Schuss (Default 5)",
        "#   SET RINGBUFFER=<1-500>   Wie weit rueckwaerts der kontinuierlich",
        "#                            laufende Ringpuffer je Mikrofon bei der Piezo-",
        "#                            Anker-Auswahl hoechstens durchsucht wird, in ms",
        "#                            (Default 10) - deckelt SET PIEZOLEAD nach oben",
        "#   SET TARGET=<STEEL|PAPER> Messmodus/Mic-Geometrie (Default STEEL)",
        "#   SET STANDOFFSTEEL=<5.0-100.0>  Mic-Standoff (rechtwinklig zur Platte)",
        "#                            in mm, STEEL-Modus (Default 30.0)",
        "#   SET STANDOFFPAPER=<5.0-100.0>  Wie STANDOFFSTEEL, PAPER-Modus (Default 28.0)",
        "#   SET PIEZO=<0|1>          Piezo (Stahlplatte, GPIO34) als alleinigen,",
        "#                            ereignisgesteuerten Ausloeser der Schuss-",
        "#                            erfassung nutzen (Default 1/an). 0: Fallback",
        "#                            auf die erste Luft-Flanke als Ausloeser",
        "#                            (siehe SET WINDOW)",
        "#   SET PIEZOMAX=<0-5000>    NUR TARGET=STEEL: wie lange nach dem Piezo-",
        "#                            Ausloesen gewartet wird, bevor der Ringpuffer",
        "#                            vorwaerts (0..PIEZOMAX) ausgewertet wird",
        "#                            (Default 1400us) - dort sitzt das Piezo direkt",
        "#                            auf der Trefferflaeche und loest VOR den",
        "#                            Luftmikrofonen aus",
        "#   SET PIEZOLEAD=<0-20000>  NUR TARGET=PAPER: wie weit VOR der Piezo-",
        "#                            Ausloesung eine Luft-Flanke je Mikrofon",
        "#                            hoechstens liegen darf, um fuer die Position",
        "#                            verwendet zu werden (Default 2000us=2ms,",
        "#                            zusaetzlich durch SET RINGBUFFER gedeckelt)",
        "#   SET PIEZOTRAIL=<0-20000> Wie PIEZOLEAD, aber NACH der Piezo-",
        "#                            Ausloesung (Default 1000us=1ms) - das echte",
        "#                            Treffer-Cluster streut leicht UM Piezo herum.",
        "#                            Bestimmt zugleich, wie lange nach dem",
        "#                            Ausloesen gewartet wird, bevor ausgewertet wird",
        "#   SET PIEZOLEADMIN=<0-20000>    Untergrenze zu PIEZOLEAD (Default 0=",
        "#                            aus): Flanken WENIGER als PIEZOLEADMIN vor",
        "#                            Piezo werden ignoriert - schliesst gezielt",
        "#                            einen Bereich DIREKT vor Piezo aus (z.B.",
        "#                            Muendungsknall-Reste), gueltig bleibt nur",
        "#                            [PIEZOLEAD..PIEZOLEADMIN] vor Piezo",
        "#   SET TESTMODE=<0|1>       Reiner Sensor-Testmodus (Klartext-Zeile je",
        "#                            Sensor-Ausloesung mit +ms seit dem 1. Sensor",
        "#                            der Serie, 10 '-' nach 5s Stille) - NICHT",
        "#                            persistent, nach Reboot immer aus (Default 0)",
        "#   SET TESTCOOLDOWN=<0-10000>    Min. Abstand zw. 2 Meldungen desselben",
        "#                            Sensors im Testmodus in ms (Default 3000).",
        "#                            Fuer die Analyse echter Schuesse (mehrere",
        "#                            echte Ereignisse je Sensor moeglich: Knall/",
        "#                            Papier/Stahl) deutlich kleiner stellen (z.B. 50)",
        "#   SET OFFSETX=<-50000..50000>   Konstanter Korrektur-Offset auf x_um",
        "#                            in 0.001mm, NACH der Trilateration addiert",
        "#                            (Default 0, z.B. gg. Messgitter-Versatz)",
        "#   SET OFFSETY=<-50000..50000>   Wie OFFSETX, fuer y_um (Default 0)",
        "#   SET SOUNDSPEED=<300-400> Angenommene Schallgeschwindigkeit in m/s",
        "#                            (Default 355). Radial mit dem Abstand",
        "#                            wachsender Fehler (Rand zu nah am Zentrum)",
        "#                            -> Wert erhoehen. Rein manueller Wert, wird",
        "#                            NICHT von CAL START mitkalibriert.",
        "#   SET CALSHOTS=<3-20>      Anzahl Kalibrier-Schuesse fuer CAL START (Default 5)",
        "#   SET OFS0..OFS5=<-5000..5000>  Timing-Offset je Mikrofon in ns (Default 0,",
        "#                            wird durch CAL START automatisch gesetzt)",
        "#   SET MPOSX0..MPOSX5=<-5000..5000>  Montage-Justage je Mikrofon in",
        "#                            X-Richtung, 0.001mm (Default 0)",
        "#   SET MPOSY0..MPOSY5=<-5000..5000>  Wie MPOSX, Y-Richtung (Default 0)",
        "#   SET MPOSZ0..MPOSZ5=<-5000..5000>  Wie MPOSX, Z-Richtung (Standoff-",
        "#                            Achse, rechtwinklig zur Platte/Scheibe,",
        "#                            Default 0). MPOSX/Y/Z werden NICHT durch",
        "#                            CAL START gesetzt, sondern offline per",
        "#                            tools/calibrate_mics.py ermittelt",
        "#   SET STATIC=<0|1>         Statische IP an/aus (Reboot noetig)",
        "#   SET IP=<ip>              Statische IP-Adresse (Reboot noetig)",
        "#   SET GW=<ip>              Gateway, auch: GATEWAY (Reboot noetig)",
        "#   SET SUBNET=<mask>        Subnetzmaske (Reboot noetig)",
        "#   SET DNS=<ip>             DNS-Server, leer = Gateway (Reboot noetig)",
        "# Kalibrierung (Timing-Offset je Mikrofon):",
        "#   CAL START                startet Sammlung von SET CALSHOTS Schuessen,",
        "#                            berechnet und speichert die Offsets danach automatisch",
        "#                            und gibt zusaetzlich je Schuss eine wiederverwend-",
        "#                            bare 'calshot'-Zeile mit den rohen Mic-Messwerten aus",
        "#   CAL ABORT                bricht laufende Kalibrierung ab (Offsets unveraendert)",
        "#   CAL STATUS               zeigt Kalibrier-Fortschritt",
        "#   CAL RESET                setzt alle Mikrofon-Offsets auf 0 und die",
        "#                            Schallgeschwindigkeit auf 355 m/s zurueck",
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
    } else if (key == "DEBUG") {
        long v = val.toInt();
        if (v < 0 || v > 3) { emitLine("{\"type\":\"error\",\"msg\":\"debug 0-3\"}\n"); return true; }
        cfg.debug = (uint8_t)v;
        saveVal<uint8_t>("debug", cfg.debug);
        emitf("{\"type\":\"ok\",\"set\":\"debug\",\"value\":%d}\n", cfg.debug);
    } else if (key == "OUTLIER") {
        long v = val.toInt();
        if (v < 0 || v > 500000) { emitLine("{\"type\":\"error\",\"msg\":\"outlier 0-500000\"}\n"); return true; }
        cfg.airOutlierUm = (uint32_t)v;
        saveVal<uint32_t>("outlier", cfg.airOutlierUm);
        emitf("{\"type\":\"ok\",\"set\":\"outlier\",\"value\":%u}\n", cfg.airOutlierUm);
    } else if (key == "RADIUS") {
        long v = val.toInt();
        if (v < 0 || v > 500000) { emitLine("{\"type\":\"error\",\"msg\":\"radius 0-500000\"}\n"); return true; }
        cfg.clusterRadiusUm = (uint32_t)v;
        saveVal<uint32_t>("cluster_r", cfg.clusterRadiusUm);
        emitf("{\"type\":\"ok\",\"set\":\"radius\",\"value\":%u}\n", cfg.clusterRadiusUm);
    } else if (key == "MINCLUSTER") {
        long v = val.toInt();
        if (v < 0 || v > AIR_MAX_COMBOS) { emitf("{\"type\":\"error\",\"msg\":\"mincluster 0-%d\"}\n", AIR_MAX_COMBOS); return true; }
        cfg.minClusterHits = (uint8_t)v;
        saveVal<uint8_t>("min_clust", cfg.minClusterHits);
        emitf("{\"type\":\"ok\",\"set\":\"mincluster\",\"value\":%d}\n", cfg.minClusterHits);
    } else if (key == "MAXPRECISION") {
        long v = val.toInt();
        if (v < 0 || v > 500000) { emitLine("{\"type\":\"error\",\"msg\":\"maxprecision 0-500000\"}\n"); return true; }
        cfg.maxPrecisionUm = (uint32_t)v;
        saveVal<uint32_t>("max_prec", cfg.maxPrecisionUm);
        emitf("{\"type\":\"ok\",\"set\":\"maxprecision\",\"value\":%u}\n", cfg.maxPrecisionUm);
    } else if (key == "MINMICS") {
        long v = val.toInt();
        if (v < 3 || v > NUM_AIR) { emitf("{\"type\":\"error\",\"msg\":\"minmics 3-%d\"}\n", NUM_AIR); return true; }
        cfg.minMics = (uint8_t)v;
        saveVal<uint8_t>("min_mics", cfg.minMics);
        emitf("{\"type\":\"ok\",\"set\":\"minmics\",\"value\":%d}\n", cfg.minMics);
    } else if (key == "RINGBUFFER") {
        long v = val.toInt();
        if (v < 1 || v > 500) { emitLine("{\"type\":\"error\",\"msg\":\"ringbuffer 1-500\"}\n"); return true; }
        cfg.ringBufferMs = (uint32_t)v;
        saveVal<uint32_t>("ring_ms", cfg.ringBufferMs);
        emitf("{\"type\":\"ok\",\"set\":\"ringbuffer\",\"value\":%u}\n", cfg.ringBufferMs);
    } else if (key == "TARGET") {
        String v = val; v.toUpperCase();
        uint8_t newMode;
        if (v == "STEEL")      newMode = TARGET_STEEL;
        else if (v == "PAPER") newMode = TARGET_PAPER;
        else { emitLine("{\"type\":\"error\",\"msg\":\"target steel|paper\"}\n"); return true; }
        cfg.targetMode = newMode;
        saveVal<uint8_t>("target", cfg.targetMode);
        applyTargetGeometry();
        emitf("{\"type\":\"ok\",\"set\":\"target\",\"value\":\"%s\"}\n",
              cfg.targetMode == TARGET_PAPER ? "paper" : "steel");
    } else if (key == "STANDOFFSTEEL") {
        float v = val.toFloat();
        if (v < 5.0f || v > 100.0f) { emitLine("{\"type\":\"error\",\"msg\":\"standoffsteel 5.0-100.0\"}\n"); return true; }
        cfg.standoffSteelMm = v;
        saveVal<float>("standoff_st", cfg.standoffSteelMm);
        applyTargetGeometry();
        emitf("{\"type\":\"ok\",\"set\":\"standoffsteel\",\"value\":%.2f}\n", cfg.standoffSteelMm);
    } else if (key == "STANDOFFPAPER") {
        float v = val.toFloat();
        if (v < 5.0f || v > 100.0f) { emitLine("{\"type\":\"error\",\"msg\":\"standoffpaper 5.0-100.0\"}\n"); return true; }
        cfg.standoffPaperMm = v;
        saveVal<float>("standoff_pa", cfg.standoffPaperMm);
        applyTargetGeometry();
        emitf("{\"type\":\"ok\",\"set\":\"standoffpaper\",\"value\":%.2f}\n", cfg.standoffPaperMm);
    } else if (key == "PIEZO") {
        long v = val.toInt();
        if (v != 0 && v != 1) { emitLine("{\"type\":\"error\",\"msg\":\"piezo 0|1\"}\n"); return true; }
        cfg.usePiezo = (v == 1);
        saveVal<bool>("use_piezo", cfg.usePiezo);
        emitf("{\"type\":\"ok\",\"set\":\"piezo\",\"value\":%d}\n", cfg.usePiezo ? 1 : 0);
    } else if (key == "PIEZOMAX") {
        long v = val.toInt();
        if (v < 0 || v > 5000) { emitLine("{\"type\":\"error\",\"msg\":\"piezomax 0-5000\"}\n"); return true; }
        cfg.piezoMaxUs = (uint32_t)v;
        saveVal<uint32_t>("piezo_max", cfg.piezoMaxUs);
        emitf("{\"type\":\"ok\",\"set\":\"piezomax\",\"value\":%u}\n", cfg.piezoMaxUs);
    } else if (key == "PIEZOLEAD") {
        long v = val.toInt();
        if (v < 0 || v > 20000) { emitLine("{\"type\":\"error\",\"msg\":\"piezolead 0-20000\"}\n"); return true; }
        cfg.piezoLeadMaxUs = (uint32_t)v;
        saveVal<uint32_t>("piezo_lead", cfg.piezoLeadMaxUs);
        emitf("{\"type\":\"ok\",\"set\":\"piezolead\",\"value\":%u}\n", cfg.piezoLeadMaxUs);
    } else if (key == "PIEZOTRAIL") {
        long v = val.toInt();
        if (v < 0 || v > 20000) { emitLine("{\"type\":\"error\",\"msg\":\"piezotrail 0-20000\"}\n"); return true; }
        cfg.piezoTrailMaxUs = (uint32_t)v;
        saveVal<uint32_t>("piezo_trail", cfg.piezoTrailMaxUs);
        emitf("{\"type\":\"ok\",\"set\":\"piezotrail\",\"value\":%u}\n", cfg.piezoTrailMaxUs);
    } else if (key == "PIEZOLEADMIN") {
        long v = val.toInt();
        if (v < 0 || v > 20000) { emitLine("{\"type\":\"error\",\"msg\":\"piezoleadmin 0-20000\"}\n"); return true; }
        cfg.piezoLeadMinUs = (uint32_t)v;
        saveVal<uint32_t>("piezo_leadmin", cfg.piezoLeadMinUs);
        emitf("{\"type\":\"ok\",\"set\":\"piezoleadmin\",\"value\":%u}\n", cfg.piezoLeadMinUs);
    } else if (key == "TESTMODE") {
        long v = val.toInt();
        if (v != 0 && v != 1) { emitLine("{\"type\":\"error\",\"msg\":\"testmode 0|1\"}\n"); return true; }
        // Bewusst NICHT persistiert (kein saveVal) - siehe Testmodus-
        // Hinweis bei testMode oben: nach Reboot immer wieder aus.
        testMode = (v == 1);
        resetShotState();
        noInterrupts();
        for (int i = 0; i <= NUM_AIR; i++) { testFired[i] = false; testLastFireUs[i] = 0; }
        interrupts();
        testLastActivityUs = (uint64_t)esp_timer_get_time();
        testSeparatorShown = false;
        testSeriesActive   = false;
        emitf("{\"type\":\"ok\",\"set\":\"testmode\",\"value\":%d}\n", testMode ? 1 : 0);
    } else if (key == "TESTCOOLDOWN") {
        long v = val.toInt();
        if (v < 0 || v > 10000) { emitLine("{\"type\":\"error\",\"msg\":\"testcooldown 0-10000\"}\n"); return true; }
        cfg.testCooldownMs = (uint32_t)v;
        saveVal<uint32_t>("test_cd_ms", cfg.testCooldownMs);
        emitf("{\"type\":\"ok\",\"set\":\"testcooldown\",\"value\":%u}\n", cfg.testCooldownMs);
    } else if (key == "OFFSETX") {
        long v = val.toInt();
        if (v < -50000 || v > 50000) { emitLine("{\"type\":\"error\",\"msg\":\"offsetx -50000..50000\"}\n"); return true; }
        cfg.offsetXUm = (int32_t)v;
        saveVal<int32_t>("ofs_x_um", cfg.offsetXUm);
        emitf("{\"type\":\"ok\",\"set\":\"offsetx\",\"value\":%ld}\n", (long)cfg.offsetXUm);
    } else if (key == "OFFSETY") {
        long v = val.toInt();
        if (v < -50000 || v > 50000) { emitLine("{\"type\":\"error\",\"msg\":\"offsety -50000..50000\"}\n"); return true; }
        cfg.offsetYUm = (int32_t)v;
        saveVal<int32_t>("ofs_y_um", cfg.offsetYUm);
        emitf("{\"type\":\"ok\",\"set\":\"offsety\",\"value\":%ld}\n", (long)cfg.offsetYUm);
    } else if (key == "SOUNDSPEED") {
        long v = val.toInt();
        if (v < SOUND_SPEED_MIN_MPS || v > SOUND_SPEED_MAX_MPS) {
            emitf("{\"type\":\"error\",\"msg\":\"soundspeed %d-%d\"}\n",
                  SOUND_SPEED_MIN_MPS, SOUND_SPEED_MAX_MPS);
            return true;
        }
        cfg.soundSpeedMps = (uint16_t)v;
        saveVal<uint16_t>("sound_mps", cfg.soundSpeedMps);
        applySoundSpeed();
        emitf("{\"type\":\"ok\",\"set\":\"soundspeed\",\"value\":%u}\n", cfg.soundSpeedMps);
    } else if (key == "CALSHOTS") {
        long v = val.toInt();
        if (v < 3 || v > MAX_CAL_SHOTS) { emitf("{\"type\":\"error\",\"msg\":\"calshots 3-%d\"}\n", MAX_CAL_SHOTS); return true; }
        cfg.calShotCount = (uint8_t)v;
        saveVal<uint8_t>("cal_n", cfg.calShotCount);
        emitf("{\"type\":\"ok\",\"set\":\"calshots\",\"value\":%d}\n", cfg.calShotCount);
    } else if (key.startsWith("OFS") && key.length() == 4 && isDigit(key[3])) {
        int idx = key[3] - '0';
        if (idx >= NUM_AIR) { emitLine("{\"type\":\"error\",\"msg\":\"unknown key\"}\n"); return true; }
        long v = val.toInt();
        if (v < -MIC_OFS_MAX_NS || v > MIC_OFS_MAX_NS) {
            emitf("{\"type\":\"error\",\"msg\":\"ofs%d %d..%d\"}\n", idx, -MIC_OFS_MAX_NS, MIC_OFS_MAX_NS);
            return true;
        }
        cfg.micOffsetNs[idx] = (int32_t)v;
        char key2[8];
        snprintf(key2, sizeof(key2), "ofs%d", idx);
        saveVal<int32_t>(key2, cfg.micOffsetNs[idx]);
        emitf("{\"type\":\"ok\",\"set\":\"ofs%d\",\"value\":%ld}\n", idx, (long)cfg.micOffsetNs[idx]);
    } else if (key.startsWith("MPOSX") && key.length() == 6 && isDigit(key[5])) {
        int idx = key[5] - '0';
        if (idx >= NUM_AIR) { emitLine("{\"type\":\"error\",\"msg\":\"unknown key\"}\n"); return true; }
        long v = val.toInt();
        if (v < -MIC_POS_OFS_MAX_UM || v > MIC_POS_OFS_MAX_UM) {
            emitf("{\"type\":\"error\",\"msg\":\"mposx%d %d..%d\"}\n", idx, -MIC_POS_OFS_MAX_UM, MIC_POS_OFS_MAX_UM);
            return true;
        }
        cfg.micPosOffsetXUm[idx] = (int32_t)v;
        char key2[8];
        snprintf(key2, sizeof(key2), "mpx%d", idx);
        saveVal<int32_t>(key2, cfg.micPosOffsetXUm[idx]);
        applyTargetGeometry();
        emitf("{\"type\":\"ok\",\"set\":\"mposx%d\",\"value\":%ld}\n", idx, (long)cfg.micPosOffsetXUm[idx]);
    } else if (key.startsWith("MPOSY") && key.length() == 6 && isDigit(key[5])) {
        int idx = key[5] - '0';
        if (idx >= NUM_AIR) { emitLine("{\"type\":\"error\",\"msg\":\"unknown key\"}\n"); return true; }
        long v = val.toInt();
        if (v < -MIC_POS_OFS_MAX_UM || v > MIC_POS_OFS_MAX_UM) {
            emitf("{\"type\":\"error\",\"msg\":\"mposy%d %d..%d\"}\n", idx, -MIC_POS_OFS_MAX_UM, MIC_POS_OFS_MAX_UM);
            return true;
        }
        cfg.micPosOffsetYUm[idx] = (int32_t)v;
        char key2[8];
        snprintf(key2, sizeof(key2), "mpy%d", idx);
        saveVal<int32_t>(key2, cfg.micPosOffsetYUm[idx]);
        applyTargetGeometry();
        emitf("{\"type\":\"ok\",\"set\":\"mposy%d\",\"value\":%ld}\n", idx, (long)cfg.micPosOffsetYUm[idx]);
    } else if (key.startsWith("MPOSZ") && key.length() == 6 && isDigit(key[5])) {
        int idx = key[5] - '0';
        if (idx >= NUM_AIR) { emitLine("{\"type\":\"error\",\"msg\":\"unknown key\"}\n"); return true; }
        long v = val.toInt();
        if (v < -MIC_POS_OFS_MAX_UM || v > MIC_POS_OFS_MAX_UM) {
            emitf("{\"type\":\"error\",\"msg\":\"mposz%d %d..%d\"}\n", idx, -MIC_POS_OFS_MAX_UM, MIC_POS_OFS_MAX_UM);
            return true;
        }
        cfg.micPosOffsetZUm[idx] = (int32_t)v;
        char key2[8];
        snprintf(key2, sizeof(key2), "mpz%d", idx);
        saveVal<int32_t>(key2, cfg.micPosOffsetZUm[idx]);
        applyTargetGeometry();
        emitf("{\"type\":\"ok\",\"set\":\"mposz%d\",\"value\":%ld}\n", idx, (long)cfg.micPosOffsetZUm[idx]);
    } else if (key == "WINDOW") {
        long v = val.toInt();
        if (v < 1 || v > 50) { emitLine("{\"type\":\"error\",\"msg\":\"window 1-50\"}\n"); return true; }
        cfg.windowMs = (uint32_t)v; saveVal<uint32_t>("window", cfg.windowMs);
        emitf("{\"type\":\"ok\",\"set\":\"window\",\"value\":%u}\n", cfg.windowMs);
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
    if (upper.startsWith("CAL")) {
        String sub = upper.substring(3);
        sub.trim();
        if (sub == "START" || sub.length() == 0) {
            calActive    = true;
            calCollected = 0;
            emitf("{\"type\":\"cal\",\"state\":\"start\",\"need\":%d}\n", cfg.calShotCount);
        } else if (sub == "ABORT") {
            calActive    = false;
            calCollected = 0;
            emitLine("{\"type\":\"cal\",\"state\":\"aborted\"}\n");
        } else if (sub == "STATUS") {
            emitf("{\"type\":\"cal\",\"state\":\"%s\",\"progress\":%d,\"need\":%d}\n",
                  calActive ? "waiting" : "idle", calCollected, cfg.calShotCount);
        } else if (sub == "RESET") {
            for (int i = 0; i < NUM_AIR; i++) {
                cfg.micOffsetNs[i] = 0;
                char key2[8];
                snprintf(key2, sizeof(key2), "ofs%d", i);
                saveVal<int32_t>(key2, 0);
            }
            cfg.soundSpeedMps = 355;
            saveVal<uint16_t>("sound_mps", cfg.soundSpeedMps);
            applySoundSpeed();
            emitLine("{\"type\":\"ok\",\"cmd\":\"cal_reset\"}\n");
        } else {
            emitLine("{\"type\":\"error\",\"msg\":\"unknown cal command\"}\n");
        }
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
    applyTargetGeometry();
    applySoundSpeed();

    // CPU-Takt fuer die Zyklen->ns-Umrechnung ermitteln
    cpuMHz = getCpuFrequencyMhz();
    if (cpuMHz < 80) cpuMHz = 240;   // Fallback

    // Alle ISRs auf gleichem Core (setup laeuft auf einem Core), damit
    // alle Mikrofone denselben Zykluszaehler benutzen.
    for (uint32_t i = 0; i < NUM_AIR; i++) {
        pinMode(AIR_PINS[i], INPUT_PULLUP);
        attachInterruptArg(digitalPinToInterrupt(AIR_PINS[i]),
                           airISR, (void *)i, RISING);
    }
    // GPIO34 hat keinen internen Pull-Up (siehe PIEZO_PIN oben) - normaler
    // INPUT-Modus, das Piezo-Modul muss aktiv treiben. Wird unabhaengig von
    // SET PIEZO immer registriert (kostet nichts), nur processShot()
    // wertet die Flanken je nach cfg.usePiezo aus.
    pinMode(PIEZO_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIEZO_PIN), piezoISR, RISING);
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
    // Reiner Sensor-Testmodus (SET TESTMODE=1): laeuft komplett unabhaengig
    // von der Schusserfassung (die ISRs schreiben in diesem Modus weder in
    // den Ringpuffer noch setzen sie piezoPending/airTrigPending, siehe
    // airISR()/piezoISR() oben).
    if (testMode) {
        // Erst ALLE seit dem letzten loop()-Durchlauf ausgeloesten Sensoren
        // einsammeln (mit ISR-Zeitstempel, nicht Poll-Zeit) und das fruehste
        // Ereignis dieses Schwungs bestimmen, BEVOR irgendetwas ausgegeben
        // wird - sonst wuerde bei mehreren Sensoren im selben loop()-
        // Durchlauf (typisch bei einem echten Schuss) der Array-Index statt
        // der tatsaechlichen zeitlichen Reihenfolge ueber die 0ms-Referenz
        // entscheiden.
        bool     fired[NUM_AIR + 1];
        uint64_t fireUs[NUM_AIR + 1];
        bool     anyFired     = false;
        bool     haveBatchMin = false;
        uint64_t batchMinUs   = 0;
        for (int i = 0; i <= NUM_AIR; i++) {
            fired[i] = testFired[i];
            if (!fired[i]) continue;
            noInterrupts();
            testFired[i] = false;
            fireUs[i] = testLastFireUs[i];
            interrupts();
            anyFired = true;
            if (!haveBatchMin || fireUs[i] < batchMinUs) {
                batchMinUs   = fireUs[i];
                haveBatchMin = true;
            }
        }

        if (anyFired) {
            // Der erste Sensor nach einer Trennlinie startet die Serie bei
            // 0ms, alle weiteren zeigen die Verzoegerung dazu - damit laesst
            // sich bei einem echten Schuss direkt ablesen, welches
            // Zeitfenster (z.B. SET PIEZOLEAD/PIEZOMAX) zuschlagen koennte.
            if (!testSeriesActive) {
                testSeriesStartUs = batchMinUs;
                testSeriesActive  = true;
            }
            for (int i = 0; i <= NUM_AIR; i++) {
                if (!fired[i]) continue;
                char buf[48];
                int64_t deltaUs = (int64_t)fireUs[i] - (int64_t)testSeriesStartUs;
                snprintf(buf, sizeof(buf), "%-20s +%.3f ms", TEST_SENSOR_NAMES[i],
                          (double)deltaUs / 1000.0);
                emitLine(buf);
            }
        }

        const uint64_t nowUs = (uint64_t)esp_timer_get_time();
        if (anyFired) {
            testLastActivityUs = nowUs;
            testSeparatorShown = false;
        } else if (!testSeparatorShown && nowUs - testLastActivityUs >= TEST_IDLE_US) {
            emitLine("----------");
            testSeparatorShown = true;
            testSeriesActive   = false;
        }
    }

    // Rev 4.9: Piezo ist der alleinige, ereignisgesteuerte Ausloeser (siehe
    // Rev-4.9-Hinweis ganz oben) - kein Polling/Timeout mehr auf eine
    // Piezo-"Bestaetigung". Nach dem Ausloesen wird noch kurz gewartet, bis
    // nachlaufende Luft-Flanken im Ringpuffer angekommen sind (TARGET=PAPER:
    // SET PIEZOTRAIL, das echte Treffer-Cluster streut leicht NACH Piezo;
    // TARGET=STEEL: SET PIEZOMAX, dort kommen die Luft-Flanken NACH dem
    // quasi latenzfreien Piezo-Kontaktschall an), dann wird ausgewertet.
    if (piezoPending) {
        const uint64_t now = (uint64_t)esp_timer_get_time();
        const uint64_t waitUs = (cfg.targetMode == TARGET_STEEL)
                               ? (uint64_t)cfg.piezoMaxUs
                               : (uint64_t)cfg.piezoTrailMaxUs;
        if (now - piezoPendingUs >= waitUs) {
            noInterrupts();
            const uint32_t refCC = piezoPendingCC;
            const uint64_t refUs = piezoPendingUs;
            piezoPending = false;
            interrupts();
            processShot(TRIG_PIEZO, refCC, refUs);
        }
    } else if (airTrigPending) {
        // SET PIEZO=0-Fallback: SET WINDOW lang ab der ausloesenden
        // Luft-Flanke vorwaerts sammeln, dann auswerten.
        const uint64_t now = (uint64_t)esp_timer_get_time();
        if (now - airTrigPendingUs >= (uint64_t)cfg.windowMs * 1000ULL) {
            noInterrupts();
            const uint32_t refCC = airTrigPendingCC;
            const uint64_t refUs = airTrigPendingUs;
            airTrigPending = false;
            interrupts();
            processShot(TRIG_FALLBACK_AIR, refCC, refUs);
        }
    }

    maintainNetwork();

    static String serialBuf, tcpBuf;
    pollCommands(Serial, serialBuf, true);
    if (tcp.connected()) {
        pollCommands(tcp, tcpBuf, false);
    }
}
