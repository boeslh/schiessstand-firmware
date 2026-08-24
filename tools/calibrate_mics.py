#!/usr/bin/env python3
"""
Offline-Kalibrierung: sucht per Grid-Search ueber Mikrofon-Positions-
Justage (dx,dy,dz je Mikrofon) UND Schallgeschwindigkeit diejenige
Kombination, die den Rest-Fehler ueber eine Reihe echter Schuesse minimiert.

Nutzung:
    python calibrate_mics.py shots.txt --mics 6
    python calibrate_mics.py shots.txt --mics 4

shots.txt: Konsolen-Ausgabe des ESP32 (SHOW-Zeile + "shot"-Zeilen, z.B. per
Copy-Paste aus PuTTY), Firmware Rev >= 4.9 (flaches air_ns-Array, siehe
schiessstand_firmware.ino Rev-4.9-Hinweis). Braucht mindestens 3 (--mics 4)
bzw. 3 (--mics 6) erfasste Mikrofone pro Schuss, mehr Schuesse UND mehr
Streuung der Trefferpositionen verbessern die Identifizierbarkeit deutlich -
mit nur ~10 Schuessen ist das Ergebnis eine grobe Naeherung, kein exaktes
Messergebnis (siehe Kommentar zu Freiheitsgraden/Walk-Error weiter unten).

--mics 4: nutzt nur AIR0-AIR3 (die 4 Eck-Mikrofone) - fuer den Fall, dass
AIR4/AIR5 (gemeinsamer IC, siehe Firmware-Historie) als unzuverlaessig
ausgeschlossen werden sollen.
--mics 6: nutzt alle 6 Mikrofone.

WICHTIG (Walk-Error): Die Mikrofon-Frontends sind Komparatoren mit fester
Schwellspannung - ein schwaecheres (weiter entferntes) Signal ueberschreitet
dieselbe Schwelle spaeter als ein starkes. Dieser Fehler ist AMPLITUDEN-,
nicht positionsabhaengig und kann durch KEINE Kombination aus Mikrofon-
Position/Timing-Offset/Schallgeschwindigkeit exakt wegkalibriert werden -
das Ergebnis dieses Skripts ist bestenfalls die beste ERKLAERBARE Naeherung
innerhalb des durchsuchten Bereichs, kein physikalisch exaktes Messergebnis.

Ausgabe: pro Mikrofon der gefundene (dx,dy,dz)-Versatz in mm + fertige
SET-Befehle (SET MPOSX/MPOSY/MPOSZ<i>, SET SOUNDSPEED) zum Einspielen in die
Firmware (Rev >= 4.10, siehe cfg.micPosOffsetXUm/YUm/ZUm).

--truth FILE (OPTIONAL): reale, vermessene Trefferpositionen je Schuss, eine
Zeile pro Schuss in mm, in der EXAKT GLEICHEN Reihenfolge wie die Schuesse
chronologisch (nach "ts") im Log auftreten. Format je Zeile: "x,y" oder
"idx: x,y" (das "idx:"-Praefix wird ignoriert, erlaubt direktes Copy-Paste
einer nummerierten Liste). Ohne --truth optimiert das Skript wie bisher
gegen die INTERNE Konsistenz der Mikrofon-Kombinationen (Rest-Fehler
zwischen unabhaengigen Loesungen) - das findet zwar einen Kompromiss, weiss
aber nicht, ob dieser Kompromiss tatsaechlich nah an der Wahrheit liegt. Mit
--truth wird stattdessen DIREKT der Abstand zwischen berechneter und echter
Position minimiert - deutlich aussagekraeftiger, aber eben nur nutzbar, wenn
zu jedem Kalibrier-Schuss eine echte Messung vorliegt. Fuer eine spaetere
Verifizierung (ohne vermessene Positionen) bleibt der interne Modus nutzbar.
Anzahl der Zeilen in FILE muss exakt zur Anzahl verwertbarer Schuesse im Log
passen (>=3 erfasste Mikrofone) - bei Abweichung bricht das Skript ab, das
ist meist ein Hinweis auf uebrig gebliebene/doppelte Telegramme im Log.
"""

import argparse
import itertools
import json
import sys
import time

import numpy as np

MIC_HALF_X = 115.0
GEOMETRY = {
    "steel": {"half_y": 100.0, "standoff": 30.0},
    "paper": {"half_y": 85.0, "standoff": 28.0},
}
# Reihenfolge wie in der Firmware: 0=links unten 1=rechts unten 2=links oben
# 3=rechts oben 4=links mitte 5=rechts mitte
Y_SIGN = np.array([-1.0, -1.0, +1.0, +1.0, 0.0, 0.0])
X_SIGN = np.array([-1.0, +1.0, -1.0, +1.0, -1.0, +1.0])


# ---------------------------------------------------------------------------
# Log-Parsing (SHOW config + "shot"-Zeilen, Firmware Rev >= 4.9 Format)
# ---------------------------------------------------------------------------

def parse_log(text):
    # Akzeptiert sowohl normale "shot"-Telegramme als auch die "calshot"-
    # Zeilen, die CAL START seit Firmware Rev 4.11 am Ende einer Kalibrierung
    # ausgibt (sendCalRawDump() in schiessstand_firmware.ino) - beide tragen
    # air_ns im selben flachen Format, "calshot" nutzt "idx" statt "seq".
    # Damit laesst sich der komplette CAL-START-Output 1:1 hier einspeisen.
    cfg = None
    shots = []
    for line in text.splitlines():
        line = line.strip()
        if not line.startswith("{"):
            continue
        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            continue
        if obj.get("type") == "config":
            cfg = obj
        elif obj.get("type") in ("shot", "calshot"):
            air_ns = obj.get("air_ns")
            if air_ns is None or len(air_ns) != 6:
                continue
            seen = [v is not None for v in air_ns]
            t = [float(v) if v is not None else 0.0 for v in air_ns]
            seq = obj.get("seq", obj.get("idx"))
            shots.append({"seq": seq, "ts": obj.get("ts", 0), "t": t, "seen": seen})
    return cfg, shots


def parse_truth(text):
    """Eine (x_mm,y_mm)-Zeile je Schuss, optional mit "idx:"-Praefix (siehe
    Modul-Docstring). Leere Zeilen werden uebersprungen."""
    truth = []
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        if ":" in line:
            line = line.split(":", 1)[1].strip()
        parts = [p for p in line.replace(";", ",").split(",") if p.strip()]
        if len(parts) != 2:
            parts = line.split()
        if len(parts) != 2:
            raise ValueError(f"Kann Truth-Zeile nicht lesen: {line!r}")
        truth.append((float(parts[0]), float(parts[1])))
    return truth


# ---------------------------------------------------------------------------
# Vektorisierte Trilateration: alle Groessen (Xr,Yr,Zr,...) duerfen Skalare
# ODER numpy-Arrays gleicher Broadcast-Form sein - wird zum Absuchen eines
# Positions-Grids fuer EIN Mikrofon genutzt (das gesuchte Mikrofon bekommt
# ein Array der Grid-Kandidaten, alle anderen bleiben Skalare; numpy
# broadcastet automatisch in die passende Form).
# ---------------------------------------------------------------------------

def solve_pair_vec(Xr, Yr, Zr, Xa, Ya, Za, Xb, Yb, Zb, tr, ta, tb, sound_mm_per_ns):
    ra = (ta - tr) * sound_mm_per_ns
    rb = (tb - tr) * sound_mm_per_ns
    A1 = 2 * (Xa - Xr); B1 = 2 * (Ya - Yr); C1 = 2 * ra
    D1 = (Xa**2 + Ya**2 + Za**2) - (Xr**2 + Yr**2 + Zr**2) - ra**2
    A2 = 2 * (Xb - Xr); B2 = 2 * (Yb - Yr); C2 = 2 * rb
    D2 = (Xb**2 + Yb**2 + Zb**2) - (Xr**2 + Yr**2 + Zr**2) - rb**2

    det = A1 * B2 - A2 * B1
    ok = np.abs(det) > 1e-6
    det_s = np.where(ok, det, 1.0)

    x0 = (D1 * B2 - D2 * B1) / det_s
    x1 = (C2 * B1 - C1 * B2) / det_s
    y0 = (A1 * D2 - A2 * D1) / det_s
    y1 = (A2 * C1 - A1 * C2) / det_s

    px = x0 - Xr
    py = y0 - Yr
    qa = 1.0 - x1**2 - y1**2
    qb = -2.0 * (px * x1 + py * y1)
    qc = -(px**2 + py**2 + Zr**2)

    qa_small = np.abs(qa) < 1e-6
    qa_s = np.where(qa_small, 1.0, qa)
    disc = qb**2 - 4.0 * qa * qc
    disc_ok = disc >= 0.0
    sq = np.sqrt(np.clip(disc, 0.0, None))
    d_plus = (-qb + sq) / (2.0 * qa_s)
    d_minus = (-qb - sq) / (2.0 * qa_s)
    pick_plus = (d_plus > 0) & ((d_minus <= 0) | (d_plus < d_minus))
    d_quad = np.where(pick_plus, d_plus, d_minus)
    d_quad_ok = disc_ok & ((d_plus > 0) | (d_minus > 0)) & ~qa_small

    qb_small = np.abs(qb) < 1e-6
    qb_s = np.where(qb_small, 1.0, qb)
    d_lin = -qc / qb_s
    d_lin_ok = qa_small & ~qb_small & (d_lin > 0)

    d = np.where(qa_small, d_lin, d_quad)
    ok = ok & np.where(qa_small, d_lin_ok, d_quad_ok)

    x = x0 + x1 * d
    y = y0 + y1 * d
    return x, y, d, ok


def total_cost(shots, trios_per_shot, X, Y, Z, sound_mm_per_ns, truth=None):
    """X,Y,Z: (n_mics,) Arrays, JEDES Element darf Skalar ODER ein (G,)-Array
    sein (numpy broadcastet automatisch) - liefert (G,) Kosten-Array (oder
    Skalar, falls kein Element ein Array ist).

    Je Schuss wird - wie im echten Firmware-Betrieb - die Trio-Kombination
    mit dem kleinsten INTERNEN Rest-Fehler gewaehlt (das Auswahlkriterium
    aendert sich bewusst NICHT durch --truth, sonst waere die Kalibrierung
    unrealistisch optimistisch - im echten Einsatz steht ja auch keine
    Ground-Truth zur Trio-Auswahl zur Verfuegung). Nur die KOSTEN, die
    daraus fuer die Optimierung entstehen, unterscheiden sich: ohne truth
    der interne Rest-Fehler selbst, mit truth der Abstand der so gewaehlten
    Loesung zur echten Position (siehe Modul-Docstring)."""
    total = None
    for i, (shot, trios) in enumerate(zip(shots, trios_per_shot)):
        t = shot["t"]
        best_res = best_x = best_y = None
        for (ref, a, b, others) in trios:
            x, y, d, ok = solve_pair_vec(
                X[ref], Y[ref], Z[ref], X[a], Y[a], Z[a], X[b], Y[b], Z[b],
                t[ref], t[a], t[b], sound_mm_per_ns)
            res_sum = np.zeros_like(x) if hasattr(x, "shape") else 0.0
            n = 0
            for m in others:
                dc = np.sqrt((x - X[m])**2 + (y - Y[m])**2 + Z[m]**2)
                rc = (t[m] - t[ref]) * sound_mm_per_ns
                res_sum = res_sum + np.abs(dc - (d + rc))
                n += 1
            residual = res_sum / n if n else np.zeros_like(x)
            residual = np.where(ok, residual, 1e6)
            if best_res is None:
                best_res, best_x, best_y = residual, x, y
            else:
                take = residual < best_res
                best_x = np.where(take, x, best_x)
                best_y = np.where(take, y, best_y)
                best_res = np.where(take, residual, best_res)

        if best_res is None:
            shot_cost = 1e6
        elif truth is not None and truth[i] is not None:
            tx, ty = truth[i]
            shot_cost = np.sqrt((best_x - tx)**2 + (best_y - ty)**2)
            shot_cost = np.where(best_res >= 1e6, 1e6, shot_cost)
        else:
            shot_cost = best_res
        total = shot_cost if total is None else total + shot_cost
    return total


def build_trios(shots, mic_idx):
    """Fuer jeden Schuss: alle loesbaren 3er-Kombinationen der erfassten
    Mikrofone (ref = kleinste Zeit), je mit der Liste der uebrigen (Kontroll-)
    Mikrofone - unabhaengig von der Geometrie, also nur einmal berechnet."""
    trios_per_shot = []
    for shot in shots:
        seen_idx = [i for i in mic_idx if shot["seen"][i]]
        trios = []
        if len(seen_idx) >= 3:
            for i0, i1, i2 in itertools.combinations(seen_idx, 3):
                trio = [i0, i1, i2]
                ref = min(trio, key=lambda k: shot["t"][k])
                a, b = [k for k in trio if k != ref]
                others = [k for k in seen_idx if k not in (ref, a, b)]
                trios.append((ref, a, b, others))
        trios_per_shot.append(trios)
    return trios_per_shot


def make_grid(rng_mm, step_mm):
    vals = np.arange(-rng_mm, rng_mm + step_mm / 2, step_mm)
    gx, gy, gz = np.meshgrid(vals, vals, vals, indexing="ij")
    return gx.ravel(), gy.ravel(), gz.ravel()


def run_calibration(shots, mic_idx, mic_x_nom, mic_y_nom, base_z,
                     pos_range_mm, pos_step_mm, speed_lo, speed_hi, speed_step,
                     passes, truth=None):
    n_mics = 6
    label = "Positions-Fehler (gegen echte Messwerte)" if truth is not None else "Rest-Fehler (interne Konsistenz)"
    # aktuelle beste Position/Geschwindigkeit (float, mm bzw. m/s)
    X = np.array(mic_x_nom, dtype=float)
    Y = np.array(mic_y_nom, dtype=float)
    Z = np.full(n_mics, base_z, dtype=float)
    speed = 345.0
    sound_mm_per_ns = speed * 1e-6

    trios_per_shot = build_trios(shots, mic_idx)

    gx, gy, gz = make_grid(pos_range_mm, pos_step_mm)
    speed_grid = np.arange(speed_lo, speed_hi + speed_step / 2, speed_step)
    print(f"Grid je Mikrofon: {len(gx)} Punkte "
          f"({int(round(2*pos_range_mm/pos_step_mm))+1} pro Achse), "
          f"Geschwindigkeits-Sweep: {len(speed_grid)} Punkte")

    def cost_scalar(Xc, Yc, Zc, speed_mps):
        return float(total_cost(shots, trios_per_shot, Xc, Yc, Zc, speed_mps * 1e-6, truth))

    baseline_cost = cost_scalar(X, Y, Z, speed)
    print(f"Baseline (Nominal-Geometrie, {speed:.1f} m/s): "
          f"Summe {label} = {baseline_cost:.3f} mm ueber {len(shots)} Schuesse")

    t0 = time.time()
    prev_cost = baseline_cost
    for p in range(passes):
        for m in mic_idx:
            Xg = X.copy().astype(object)
            Yg = Y.copy().astype(object)
            Zg = Z.copy().astype(object)
            Xg[m] = mic_x_nom[m] + gx
            Yg[m] = mic_y_nom[m] + gy
            Zg[m] = base_z + gz
            costs = total_cost(shots, trios_per_shot, Xg, Yg, Zg, sound_mm_per_ns, truth)
            best = int(np.argmin(costs))
            X[m] = mic_x_nom[m] + gx[best]
            Y[m] = mic_y_nom[m] + gy[best]
            Z[m] = base_z + gz[best]

        speed_costs = np.array([cost_scalar(X, Y, Z, s) for s in speed_grid])
        best_s = int(np.argmin(speed_costs))
        speed = float(speed_grid[best_s])
        sound_mm_per_ns = speed * 1e-6

        cur_cost = cost_scalar(X, Y, Z, speed)
        print(f"Runde {p+1}/{passes}: Summe {label} = {cur_cost:.3f} mm, "
              f"v = {speed:.1f} m/s  ({time.time()-t0:.1f}s seit Start)")
        if abs(prev_cost - cur_cost) < 1e-4:
            print("Konvergiert (keine Verbesserung mehr) - breche fruehzeitig ab.")
            break
        prev_cost = cur_cost

    elapsed = time.time() - t0
    final_cost = cost_scalar(X, Y, Z, speed)
    return X, Y, Z, speed, baseline_cost, final_cost, elapsed


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("logfile", nargs="?", help="Datei mit SHOW-/shot-JSON-Zeilen (Default: stdin)")
    p.add_argument("--mics", type=int, choices=[4, 6], default=6,
                   help="4 = nur AIR0-AIR3, 6 = alle Mikrofone (Default 6)")
    p.add_argument("--target", choices=["steel", "paper"], default=None, help="Ueberschreibt target aus SHOW")
    p.add_argument("--standoff", type=float, default=None, help="Basis-Standoff in mm (Default aus SHOW/Zielmodus)")
    p.add_argument("--pos-range", type=float, default=3.0, help="Positions-Suchbereich +-mm je Achse (Default 3.0)")
    p.add_argument("--pos-step", type=float, default=0.1, help="Positions-Schrittweite in mm (Default 0.1)")
    p.add_argument("--speed-min", type=float, default=335.0, help="Min. Schallgeschwindigkeit m/s (Default 335)")
    p.add_argument("--speed-max", type=float, default=355.0, help="Max. Schallgeschwindigkeit m/s (Default 355)")
    p.add_argument("--speed-step", type=float, default=0.5, help="Schallgeschwindigkeits-Schrittweite (Default 0.5)")
    p.add_argument("--passes", type=int, default=5, help="Anzahl Kalibrier-Runden (Default 5)")
    p.add_argument("--truth", default=None,
                   help="OPTIONAL: Datei mit echten, vermessenen Trefferpositionen (mm), "
                        "eine Zeile je Schuss in chronologischer Reihenfolge (siehe Modul-Docstring)")
    args = p.parse_args()

    text = open(args.logfile, encoding="utf-8").read() if args.logfile else sys.stdin.read()
    cfg, shots = parse_log(text)

    mic_idx = list(range(4)) if args.mics == 4 else list(range(6))
    usable = [s for s in shots if sum(1 for i in mic_idx if s["seen"][i]) >= 3]
    # Chronologisch sortieren (ts, stabil) - insbesondere fuer --truth wichtig,
    # da die Truth-Datei in Schuss-/Feuerreihenfolge vorliegt, waehrend die
    # Reihenfolge im Log (Copy-Paste, mehrere Sessions) das nicht garantiert.
    usable.sort(key=lambda s: s["ts"])
    print(f"{len(shots)} Schuss-Telegramme gefunden, {len(usable)} davon mit "
          f">=3 erfassten Mikrofonen aus AIR0-{mic_idx[-1]}.")
    if len(usable) < 3:
        print("Zu wenige verwertbare Schuesse (<3) - Abbruch.")
        return
    if len(usable) < 10:
        print(f"WARNUNG: nur {len(usable)} statt der empfohlenen >=10 Schuesse - "
              f"Ergebnis entsprechend unsicherer.")

    truth = None
    if args.truth:
        truth_text = open(args.truth, encoding="utf-8").read()
        truth = parse_truth(truth_text)
        if len(truth) != len(usable):
            print(f"FEHLER: {len(truth)} Truth-Zeilen, aber {len(usable)} verwertbare Schuesse "
                  f"im Log - das muss exakt uebereinstimmen (chronologische 1:1-Zuordnung). "
                  f"Haeufigste Ursache: uebrig gebliebene/doppelte Telegramme im Log (alte "
                  f"Werte im Copy-Buffer) - Log pruefen und ggf. bereinigen. Abbruch.")
            return
        print(f"Ground-Truth-Modus: {len(truth)} echte Positionen geladen - "
              f"optimiere gegen tatsaechlichen Abstand statt interne Konsistenz.")

    target = args.target or (cfg.get("target") if cfg else None) or "paper"
    geo = GEOMETRY[target]
    standoff = args.standoff if args.standoff is not None else (
        (cfg.get("standoff_paper_mm") if target == "paper" else cfg.get("standoff_steel_mm"))
        if cfg else geo["standoff"])
    mic_x_nom = (X_SIGN * MIC_HALF_X).tolist()
    mic_y_nom = (Y_SIGN * geo["half_y"]).tolist()
    print(f"Zielmodus={target}  Standoff-Basis={standoff:.2f}mm  "
          f"Mikrofone={'AIR0-3' if args.mics == 4 else 'AIR0-5'}")

    X, Y, Z, speed, base_cost, final_cost, elapsed = run_calibration(
        usable, mic_idx, mic_x_nom, mic_y_nom, standoff,
        args.pos_range, args.pos_step, args.speed_min, args.speed_max, args.speed_step,
        args.passes, truth)

    label = "Positions-Fehler (ggu. Messwerten)" if truth is not None else "Rest-Fehler (interne Konsistenz)"
    print(f"\n=== Ergebnis ({elapsed:.1f}s Rechenzeit) ===")
    print(f"Summe {label}: {base_cost:.3f} mm -> {final_cost:.3f} mm "
          f"({(1-final_cost/base_cost)*100 if base_cost else 0:.1f}% Verbesserung)")
    if truth is not None:
        print(f"Durchschnittlicher Positions-Fehler je Schuss: "
              f"{base_cost/len(usable):.2f} mm -> {final_cost/len(usable):.2f} mm")
    print(f"Schallgeschwindigkeit: {speed:.1f} m/s\n")
    print("Justage je Mikrofon (mm) und fertige SET-Befehle:")
    print(f"{'mic':<5}{'dx':>8}{'dy':>8}{'dz':>8}")
    for i in mic_idx:
        dx = X[i] - mic_x_nom[i]
        dy = Y[i] - mic_y_nom[i]
        dz = Z[i] - standoff
        print(f"AIR{i:<3}{dx:8.2f}{dy:8.2f}{dz:8.2f}")
    print()
    for i in mic_idx:
        dx_um = round((X[i] - mic_x_nom[i]) * 1000)
        dy_um = round((Y[i] - mic_y_nom[i]) * 1000)
        dz_um = round((Z[i] - standoff) * 1000)
        print(f"SET MPOSX{i}={dx_um}")
        print(f"SET MPOSY{i}={dy_um}")
        print(f"SET MPOSZ{i}={dz_um}")
    print(f"SET SOUNDSPEED={round(speed)}")

    if elapsed > 0:
        # ESP32 (240MHz, keine Vektorisierung/HW-Gleitkomma-Batch wie numpy)
        # grob abschaetzen: numpy-vektorisierte Grid-Auswertung nutzt SIMD/
        # gecachte Speicherzugriffe, die ein Single-Core-Mikrocontroller ohne
        # Vektoreinheit nicht hat - konservativ 50-100x langsamer je
        # Gleitkomma-Operation, siehe Bewertung in der Textantwort.
        est_min = elapsed * 50 / 60
        est_max = elapsed * 100 / 60
        print(f"\n(Diese PC-Rechenzeit betrug {elapsed:.1f}s; auf einem ESP32 "
              f"ohne SIMD/Vektorisierung ist bei dieser Aufloesung mit grob "
              f"{est_min:.0f}-{est_max:.0f} Minuten zu rechnen - siehe Bewertung "
              f"zur 30s-Grenze in der Textantwort.)")


if __name__ == "__main__":
    main()
