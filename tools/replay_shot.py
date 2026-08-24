#!/usr/bin/env python3
"""
Rechnet "shot"-Telegramme des Schiessstand-ESP32 offline nach.

Bildet solveAirPair()/solveAirPosition() aus schiessstand_firmware.ino 1:1
nach (Hyperbel-Trilateration, siehe dortige Kommentare) - u.a. gedacht, um
Was-waere-wenn-Fragen zu beantworten (z.B. "wie wuerde sich cluster_hits mit
einer anderen Schallgeschwindigkeit aendern?"), ohne dafuer neue Firmware
flashen und Schuesse abfeuern zu muessen.

Nutzung: Konsolen-Ausgabe des ESP32 (SHOW-Zeile + beliebig viele shot-Zeilen,
z.B. per Copy-Paste aus PuTTY) in eine Datei speichern und

    python replay_shot.py log.txt --soundspeed 357

Die SHOW-Zeile (type=config) liefert automatisch Ziel-Modus/Mic-Offsets/
Cluster-Radius als Basis; alles davon kann per Kommandozeile ueberschrieben
werden (z.B. um mehrere Schallgeschwindigkeiten durchzuprobieren, ohne das
Geraet neu konfigurieren zu muessen). Ohne Datei-Argument wird stdin gelesen.
Jede erkannte "shot"-Zeile wird nachgerechnet und der eigenen Neuberechnung
gegenuebergestellt (falls x_um/y_um/etc. im Telegramm vorhanden sind).
"""

import argparse
import itertools
import json
import math
import sys

MIC_HALF_X = 115.0
GEOMETRY = {
    "steel": {"half_y": 100.0, "standoff": 30.0},
    "paper": {"half_y": 85.0, "standoff": 28.0},
}


def mic_positions(half_y):
    x = [-MIC_HALF_X, +MIC_HALF_X, -MIC_HALF_X, +MIC_HALF_X, -MIC_HALF_X, +MIC_HALF_X]
    y = [-half_y, -half_y, +half_y, +half_y, 0.0, 0.0]
    return x, y


def solve_pair(ref, a, b, t, mic_x, mic_y, standoff, sound_mm_per_ns):
    xr, yr = mic_x[ref], mic_y[ref]
    ra = (t[a] - t[ref]) * sound_mm_per_ns
    rb = (t[b] - t[ref]) * sound_mm_per_ns
    a1, b1, c1 = 2 * (mic_x[a] - xr), 2 * (mic_y[a] - yr), 2 * ra
    d1 = (mic_x[a] ** 2 + mic_y[a] ** 2) - (xr ** 2 + yr ** 2) - ra ** 2
    a2, b2, c2 = 2 * (mic_x[b] - xr), 2 * (mic_y[b] - yr), 2 * rb
    d2 = (mic_x[b] ** 2 + mic_y[b] ** 2) - (xr ** 2 + yr ** 2) - rb ** 2

    det = a1 * b2 - a2 * b1
    if abs(det) < 1e-6:
        return None
    x0 = (d1 * b2 - d2 * b1) / det
    x1 = (c2 * b1 - c1 * b2) / det
    y0 = (a1 * d2 - a2 * d1) / det
    y1 = (a2 * c1 - a1 * c2) / det

    px, py = x0 - xr, y0 - yr
    qa = 1 - x1 * x1 - y1 * y1
    qb = -2 * (px * x1 + py * y1)
    qc = -(px * px + py * py + standoff * standoff)

    if abs(qa) < 1e-6:
        if abs(qb) < 1e-6:
            return None
        d = -qc / qb
    else:
        disc = qb * qb - 4 * qa * qc
        if disc < 0:
            return None
        sq = math.sqrt(disc)
        d_plus = (-qb + sq) / (2 * qa)
        d_minus = (-qb - sq) / (2 * qa)
        if d_plus > 0 and (d_minus <= 0 or d_plus < d_minus):
            d = d_plus
        elif d_minus > 0:
            d = d_minus
        else:
            return None
    if d <= 0:
        return None
    return x0 + x1 * d, y0 + y1 * d, d


def solve_position(t, seen, mic_x, mic_y, standoff, sound_mm_per_ns, cluster_radius_mm):
    all_idx = [i for i in range(6) if seen[i]]
    if len(all_idx) < 3:
        return None

    cand_x, cand_y = [], []
    found = False
    best_residual = best_x = best_y = 0.0
    best_cand_idx = -1

    for i0, i1, i2 in itertools.combinations(all_idx, 3):
        trio = [i0, i1, i2]
        ref = min(trio, key=lambda k: t[k])
        a, b = [k for k in trio if k != ref]
        result = solve_pair(ref, a, b, t, mic_x, mic_y, standoff, sound_mm_per_ns)
        if result is None:
            continue
        x, y, d = result
        cand_idx = len(cand_x)
        cand_x.append(x)
        cand_y.append(y)

        residual_sum, n_check = 0.0, 0
        for m in all_idx:
            if m in (ref, a, b):
                continue
            dc = math.sqrt((x - mic_x[m]) ** 2 + (y - mic_y[m]) ** 2 + standoff ** 2)
            rc = (t[m] - t[ref]) * sound_mm_per_ns
            residual_sum += abs(dc - (d + rc))
            n_check += 1
        residual = residual_sum / n_check if n_check else 0.0

        if not found or residual < best_residual:
            found = True
            best_residual, best_x, best_y, best_cand_idx = residual, x, y, cand_idx

    if not found:
        return None

    d1 = d2 = -1.0
    in_radius = 0
    for i, (cx, cy) in enumerate(zip(cand_x, cand_y)):
        dist = math.hypot(cx - best_x, cy - best_y)
        if dist <= cluster_radius_mm:
            in_radius += 1
        if i == best_cand_idx:
            continue
        if d1 < 0 or dist < d1:
            d1, d2 = dist, d1
        elif d2 < 0 or dist < d2:
            d2 = dist

    n_near = (1 if d1 >= 0 else 0) + (1 if d2 >= 0 else 0)
    sum_sq = (d1 * d1 if d1 >= 0 else 0.0) + (d2 * d2 if d2 >= 0 else 0.0)
    precision = math.sqrt(sum_sq / n_near) if n_near else 0.0

    return best_x, best_y, best_residual, precision, in_radius


def first_edges(air_ns, mic_offset_ns):
    # Seit Firmware Rev 4.9 ist air_ns ein FLACHES Array (ein Wert je Mikrofon,
    # null = nicht erfasst), relativ zum Ausloeser (Piezo bzw. im PIEZO=0-
    # Fallback die erste Luft-Flanke) statt einer Liste aller Rohkandidaten
    # relativ zum ersten Mikrofon-Hit (Vor-4.9-Format).
    seen = [v is not None for v in air_ns]
    t = [(air_ns[i] - mic_offset_ns[i]) if seen[i] else 0 for i in range(len(air_ns))]
    return t, seen


def replay(shot, cfg):
    standoff = cfg.get("standoff") or GEOMETRY[cfg["target"]]["standoff"]
    half_y = cfg.get("half_y") or GEOMETRY[cfg["target"]]["half_y"]
    mic_x, mic_y = mic_positions(half_y)
    sound_mm_per_ns = cfg["soundspeed"] * 1e-6

    t, seen = first_edges(shot["air_ns"], cfg["mic_offset_ns"])
    result = solve_position(t, seen, mic_x, mic_y, standoff, sound_mm_per_ns, cfg["cluster_radius_mm"])

    print(f"seq {shot.get('seq', '?')}  (soundspeed={cfg['soundspeed']:.0f}m/s, "
          f"target={cfg['target']}, standoff={standoff}mm, half_y={half_y}mm)")
    if result is None:
        print("  -> keine Loesung (weniger als 3 Mics oder Geometrie entartet)")
        return
    x, y, res, prec, cluster = result
    print(f"  neu berechnet : x={x:7.3f}mm  y={y:7.3f}mm  "
          f"res={res:6.3f}mm  prec={prec:6.3f}mm  cluster_hits={cluster}")
    if "x_um" in shot:
        fx, fy = shot["x_um"] / 1000.0, shot["y_um"] / 1000.0
        fres = shot.get("pos_res_um", 0) / 1000.0
        fprec = shot.get("precision_um", 0) / 1000.0
        fcluster = shot.get("cluster_hits", 0)
        print(f"  Telegramm-Wert: x={fx:7.3f}mm  y={fy:7.3f}mm  "
              f"res={fres:6.3f}mm  prec={fprec:6.3f}mm  cluster_hits={fcluster}")


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("logfile", nargs="?", help="Datei mit SHOW-/shot-JSON-Zeilen (Default: stdin)")
    p.add_argument("--soundspeed", type=float, default=343.0, help="Schallgeschwindigkeit in m/s (Default 343)")
    p.add_argument("--target", choices=["steel", "paper"], default=None, help="Ueberschreibt target aus SHOW")
    p.add_argument("--standoff", type=float, default=None, help="Ueberschreibt Standoff in mm")
    p.add_argument("--half-y", type=float, default=None, dest="half_y", help="Ueberschreibt Mic-Y-Halbabstand in mm")
    p.add_argument("--offsets", default=None, help="Mic-Offsets in ns, kommagetrennt, z.B. 0,1588,1588,1588,-1588,-1588")
    p.add_argument("--cluster-radius", type=float, default=None, dest="cluster_radius_mm", help="Cluster-Radius in mm")
    return p.parse_args()


def main():
    args = parse_args()
    text = open(args.logfile, encoding="utf-8").read() if args.logfile else sys.stdin.read()

    cfg = {
        "soundspeed": args.soundspeed,
        "target": args.target or "steel",
        "standoff": args.standoff,
        "half_y": args.half_y,
        "mic_offset_ns": [int(v) for v in args.offsets.split(",")] if args.offsets else [0] * 6,
        "cluster_radius_mm": args.cluster_radius_mm if args.cluster_radius_mm is not None else 0.2,
    }

    for line in text.splitlines():
        line = line.strip()
        if not line.startswith("{"):
            continue
        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            continue

        if obj.get("type") == "config":
            if args.target is None:
                cfg["target"] = obj.get("target", cfg["target"])
            if args.offsets is None and "mic_offset_ns" in obj:
                cfg["mic_offset_ns"] = obj["mic_offset_ns"]
            if args.cluster_radius_mm is None and "cluster_radius_um" in obj:
                cfg["cluster_radius_mm"] = obj["cluster_radius_um"] / 1000.0
            print(f"# config aus SHOW uebernommen: target={cfg['target']} "
                  f"mic_offset_ns={cfg['mic_offset_ns']} cluster_radius_mm={cfg['cluster_radius_mm']}")
            continue

        if obj.get("type") == "shot":
            replay(obj, cfg)
            print()


if __name__ == "__main__":
    main()
