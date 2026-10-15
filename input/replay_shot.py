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

Lochrand-Modell (siehe rim_model.py)
------------------------------------
Zusaetzlich wird jeder Schuss mit dem Lochrand-Modell gefittet: der Schall
entsteht am Lochrand (wirksamer Radius --rim, Default 2.25 mm), und wo ein
frueheres Loch derselben Scheibe den neuen Rand schon "weggestanzt" hat, nur
am verbleibenden Teilkreis. Welche Schuesse auf derselben Scheibe liegen,
bestimmt --sheet:
  auto  (Default) neue Scheibe nach jedem Schuss, der laut SHOW-Konfiguration
        (paper_auto/paper_trigger) einen Vorschub ausgeloest hat, sowie nach
        "testshootpaper"/"targetchange"-Quittungen
  all   alle Schuesse auf einer Scheibe (z.B. PAPERAUTO=0)
  each  jeder Schuss auf eigener Scheibe (keine Ueberdeckung)
Eine Zeile "# SHEET" im Log erzwingt zusaetzlich eine neue Scheibe.

    python replay_shot.py log.txt --radius-scan 0:2.5:0.1

sucht den wirksamen Radius, der die Laufzeiten ALLER Schuesse am besten
erklaert (inkl. 1-Sigma-Bereich). Aussagekraeftig wird das erst mit
Schuessen, die ueber die ganze Scheibe verteilt sind - der Randversatz ist
fuer alle Mics fast gleich und nur in der Mic-abhaengigen Restkomponente
beobachtbar (am staerksten bei Treffern weit aussen).
"""

import argparse
import itertools
import json
import math
import sys

import rim_model

MIC_HALF_X = 115.0
GEOMETRY = {
    "steel": {"half_y": 100.0, "standoff": 30.0},
    "paper": {"half_y": 85.0, "standoff": 28.0},
}


def mic_positions(half_y, half_x=MIC_HALF_X):
    x = [-half_x, +half_x, -half_x, +half_x, -half_x, +half_x]
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


def all_edges(air_ns, mic_offset_ns):
    """
    Liefert je Mic die Liste ALLER Kandidatenflanken (ns, offset-korrigiert).
    Unterstuetzt beide Telegramm-Formate:
      verschachtelt  [[0,15200],[820],[],...]  (aktuelle Firmware, 4.10.x)
      flach          [0, 820, null, ...]       (Zwischenstand 4.9)
    """
    out = []
    for i, v in enumerate(air_ns):
        if v is None:
            vals = []
        elif isinstance(v, list):
            vals = [x for x in v if x is not None]
        else:
            vals = [v]
        out.append([x - mic_offset_ns[i] for x in sorted(vals)])
    return out


def first_edges(air_ns, mic_offset_ns):
    edges = all_edges(air_ns, mic_offset_ns)
    seen = [bool(e) for e in edges]
    t = [e[0] if e else 0 for e in edges]
    return t, seen


def geometry(cfg):
    standoff = cfg.get("standoff") or cfg.get("standoff_" + cfg["target"]) \
        or GEOMETRY[cfg["target"]]["standoff"]
    half_y = cfg.get("half_y") or GEOMETRY[cfg["target"]]["half_y"]
    mic_x, mic_y = mic_positions(half_y, cfg.get("half_x") or MIC_HALF_X)
    return mic_x, mic_y, standoff, half_y


def replay(shot, cfg, rim_fit=None):
    mic_x, mic_y, standoff, half_y = geometry(cfg)
    sound_mm_per_ns = cfg["soundspeed"] * 1e-6

    t, seen = first_edges(shot["air_ns"], cfg["mic_offset_ns"])
    seen = [s and e for s, e in zip(seen, cfg["mic_enabled"])]
    result = solve_position(t, seen, mic_x, mic_y, standoff, sound_mm_per_ns, cfg["cluster_radius_mm"])

    print(f"seq {shot.get('seq', '?')}  (soundspeed={cfg['soundspeed']:.0f}m/s, "
          f"target={cfg['target']}, standoff={standoff}mm, half_y={half_y}mm)")
    if result is None:
        print("  -> keine Loesung (weniger als 3 Mics oder Geometrie entartet)")
    else:
        x, y, res, prec, cluster = result
        print(f"  alt (Punkt)   : x={x:7.3f}mm  y={y:7.3f}mm  "
              f"res={res:6.3f}mm  prec={prec:6.3f}mm  cluster_hits={cluster}")
    if "x_um" in shot:
        fx, fy = shot["x_um"] / 1000.0, shot["y_um"] / 1000.0
        fres = shot.get("pos_res_um", 0) / 1000.0
        fprec = shot.get("precision_um", 0) / 1000.0
        fcluster = shot.get("cluster_hits", 0)
        print(f"  Telegramm-Wert: x={fx:7.3f}mm  y={fy:7.3f}mm  "
              f"res={fres:6.3f}mm  prec={fprec:6.3f}mm  cluster_hits={fcluster}")
    if rim_fit is not None:
        r = rim_fit["res"]
        if r is None:
            print("  Lochrand      : keine Loesung")
        else:
            sig = (f"  sigma=({r['sigma_mm'][0]:.3f},{r['sigma_mm'][1]:.3f})mm"
                   if r["sigma_mm"] else "  sigma=- (nur 3 Mics)")
            rms = math.sqrt(r["rss_ns2"] / r["n"]) if r["n"] else 0.0
            print(f"  Lochrand r={cfg['rim']:.2f}: x={r['x']:7.3f}mm  y={r['y']:7.3f}mm  "
                  f"rms={rms:6.0f}ns  mics={r['n']}{sig}"
                  + ("" if r["valid"] else "  (zu wenige konsistente Mics)"))
            print("                  Residuen ns: " + "  ".join(
                f"M{i}:{v:+.0f}" for i, v in sorted(r["resid_ns"].items())))
        if rim_fit["full_cover"]:
            print("  !! Loch liegt komplett in einem frueheren Loch - kein Papier "
                  "durchtrennt, akustisch nicht messbar")
        elif rim_fit["overlap"] > 0:
            print(f"  Ueberdeckung  : {100 * rim_fit['overlap']:.0f}% des Rands lag in "
                  f"einem frueheren Loch -> Teilkreis-Modell")


def parse_range(text):
    a, b, st = (float(v) for v in text.split(":"))
    out, v = [], a
    while v <= b + 1e-9:
        out.append(round(v, 6))
        v += st
    return out


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("logfile", nargs="?", help="Datei mit SHOW-/shot-JSON-Zeilen (Default: stdin)")
    p.add_argument("--soundspeed", type=float, default=None,
                   help="Schallgeschwindigkeit in m/s (Default: aus SHOW, sonst 355)")
    p.add_argument("--target", choices=["steel", "paper"], default=None, help="Ueberschreibt target aus SHOW")
    p.add_argument("--standoff", type=float, default=None, help="Ueberschreibt Standoff in mm")
    p.add_argument("--half-y", type=float, default=None, dest="half_y", help="Ueberschreibt Mic-Y-Halbabstand in mm")
    p.add_argument("--offsets", default=None, help="Mic-Offsets in ns, kommagetrennt, z.B. 0,1588,1588,1588,-1588,-1588")
    p.add_argument("--cluster-radius", type=float, default=None, dest="cluster_radius_mm", help="Cluster-Radius in mm")
    p.add_argument("--rim", type=float, default=rim_model.R_PHYS_DEFAULT,
                   help="wirksamer akustischer Lochradius in mm fuer den Einzel-Replay (Default 2.25)")
    p.add_argument("--r-phys", type=float, default=rim_model.R_PHYS_DEFAULT, dest="r_phys",
                   help="physikalischer Lochradius fuer die Ueberdeckung in mm (Default 2.25)")
    p.add_argument("--sheet", choices=["auto", "all", "each"], default="auto",
                   help="welche Schuesse auf derselben Scheibe liegen (siehe oben)")
    p.add_argument("--no-overlap", action="store_true", dest="no_overlap",
                   help="Teilkreis-Modell abschalten (immer Vollkreis)")
    p.add_argument("--radius-scan", default=None, dest="radius_scan", metavar="MIN:MAX:STEP",
                   help="wirksamen Radius suchen, z.B. 0:2.5:0.25")
    p.add_argument("--quiet", action="store_true", help="beim Scan keine Einzel-Schuesse ausgeben")
    return p.parse_args()


def paper_fed(obj, cfg):
    """Hat diese Ausloesung laut Konfiguration einen Papiervorschub ausgeloest?"""
    if not cfg.get("paper_auto", True):
        return False
    trig = cfg.get("paper_trigger", "piezo")
    if trig == "any":
        return True
    if trig == "clean":
        return obj.get("clean", 0) == 1
    # piezo: Feld fehlt, wenn SET PIEZO=0 -> unbekannt, Vorschub annehmen
    return obj.get("piezo_ns", 0) is not None


def main():
    args = parse_args()
    text = open(args.logfile, encoding="utf-8").read() if args.logfile else sys.stdin.read()

    cfg = {
        "soundspeed": args.soundspeed or 355.0,
        "target": args.target or "steel",
        "standoff": args.standoff,
        "half_y": args.half_y,
        "half_x": None,
        "mic_offset_ns": [int(v) for v in args.offsets.split(",")] if args.offsets else [0] * 6,
        "mic_enabled": [True] * 6,
        "cluster_radius_mm": args.cluster_radius_mm if args.cluster_radius_mm is not None else 0.2,
        "rim": args.rim,
    }

    # Erst alles einlesen: Scheiben-Zuordnung braucht die Reihenfolge
    shots, sheets, current, unknown_holes = [], [], [], 0
    sheet_has_unknown = []

    def new_sheet():
        nonlocal current, unknown_holes
        if current or unknown_holes:
            sheets.append(current)
            sheet_has_unknown.append(unknown_holes)
        current, unknown_holes = [], 0

    for line in text.splitlines():
        line = line.strip()
        if line.upper().startswith("# SHEET"):
            new_sheet()
            continue
        if not line.startswith("{"):
            continue
        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            continue
        typ = obj.get("type")

        if typ == "config":
            if args.target is None:
                cfg["target"] = obj.get("target", cfg["target"])
            if args.offsets is None and "mic_offset_ns" in obj:
                cfg["mic_offset_ns"] = obj["mic_offset_ns"]
            if "mic_enabled" in obj:
                cfg["mic_enabled"] = [bool(v) for v in obj["mic_enabled"]]
            if args.cluster_radius_mm is None and "cluster_radius_um" in obj:
                cfg["cluster_radius_mm"] = obj["cluster_radius_um"] / 1000.0
            if args.soundspeed is None and "sound_mps" in obj:
                cfg["soundspeed"] = float(obj["sound_mps"])
            if "mic_half_x_mm" in obj:
                cfg["half_x"] = obj["mic_half_x_mm"]
            for k in ("steel", "paper"):
                if f"standoff_{k}_mm" in obj:
                    cfg[f"standoff_{k}"] = obj[f"standoff_{k}_mm"]
            cfg["paper_auto"] = bool(obj.get("paper_auto", 1))
            cfg["paper_trigger"] = obj.get("paper_trigger", "piezo")
            print(f"# config aus SHOW uebernommen: target={cfg['target']} "
                  f"soundspeed={cfg['soundspeed']:.0f} mic_offset_ns={cfg['mic_offset_ns']} "
                  f"paper_auto={int(cfg['paper_auto'])} paper_trigger={cfg['paper_trigger']}")
            continue

        if typ == "ok" and (obj.get("cmd") == "testshootpaper" or obj.get("action") == "targetchange"):
            new_sheet()
            continue

        if typ in ("shot", "reject"):
            if obj.get("synthetic"):
                continue          # TESTSHOOT: kein echtes Loch
            if typ == "shot" and "air_ns" in obj:
                edges = all_edges(obj["air_ns"], cfg["mic_offset_ns"])
                edges = [e if en else [] for e, en in zip(edges, cfg["mic_enabled"])]
                shots.append((obj, dict(cfg)))
                current.append((obj.get("seq"), edges))
            else:
                unknown_holes += 1    # Loch vorhanden, Lage unbekannt
            if args.sheet == "each" or (args.sheet == "auto" and paper_fed(obj, cfg)):
                new_sheet()
    new_sheet()

    if not shots:
        print("Keine auswertbaren shot-Telegramme gefunden.")
        return

    mic_x, mic_y, standoff, _ = geometry(cfg)
    geo = rim_model.Geo(mic_x, mic_y, standoff, cfg["soundspeed"] * 1e-6)
    use_overlap = not args.no_overlap

    multi = [len(sh) for sh in sheets if len(sh) > 1]
    print(f"# {len(shots)} Schuesse auf {len(sheets)} Scheibe(n), "
          f"{sum(multi)} davon auf Scheiben mit mehreren Treffern")
    if any(sheet_has_unknown):
        print(f"# Hinweis: {sum(sheet_has_unknown)} reject-Ausloesung(en) ohne Position - "
              f"deren Loecher koennen im Teilkreis-Modell nicht beruecksichtigt werden")
    print()

    fits = rim_model.fit_sheets(geo, sheets, args.rim, args.r_phys, use_overlap)
    fit_by_seq = {f["seq"]: f for f in fits}
    if not (args.radius_scan and args.quiet):
        for obj, scfg in shots:
            replay(obj, scfg, fit_by_seq.get(obj.get("seq")))
            print()

    if args.radius_scan:
        radii = parse_range(args.radius_scan)
        rows = rim_model.radius_scan(geo, sheets, radii, args.r_phys, use_overlap)
        print("# Radius-Scan (wirksamer akustischer Lochradius)")
        print("#   r_eff[mm]   RMS[ns]   Schuesse  Freiheitsgrade")
        for r, rss, dof, ns in rows:
            rms = math.sqrt(rss / dof) if dof else float("nan")
            print(f"#   {r:8.2f}   {rms:7.1f}   {ns:8d}  {dof:14d}")
        ci = rim_model.confidence_interval(rows)
        if ci is None:
            print("# Zu wenig Redundanz (Schuesse mit >= 4 Mics noetig).")
        else:
            print(f"# Bester Radius: {ci['r_best']:.2f} mm, 1-Sigma-Bereich "
                  f"{ci['r_lo']:.2f} .. {ci['r_hi']:.2f} mm "
                  f"(Zeitrauschen ~{ci['sigma_ns']:.0f} ns)")
            if ci["at_edge"]:
                print("# Minimum liegt am Rand des Scanbereichs - Bereich erweitern.")
            if ci["r_hi"] - ci["r_lo"] >= 0.75 * (radii[-1] - radii[0]):
                print("# Kurve ist flach: die Daten koennen den Radius (noch) nicht "
                      "unterscheiden. Mehr Schuesse, v.a. weit aussen, oder genauere "
                      "Zeitstempel noetig.")


if __name__ == "__main__":
    main()
