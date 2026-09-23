#!/usr/bin/env python3
"""
Erzeugt ein synthetisches Log (SHOW + shot-Telegramme im Firmware-Format)
mit BEKANNTEM wirksamem Radius und absichtlich ueberlappenden Treffern, um
replay_shot.py --radius-scan und das Teilkreis-Modell zu pruefen.

    python gen_rim_log.py --r-eff 1.5 --noise 100 --shots 60 > sim.log
    python ../../tools/replay_shot.py sim.log --sheet all --radius-scan 0:2.5:0.1 --quiet

Die wahren Positionen stehen als "true_x_mm"/"true_y_mm" im Telegramm.
"""
import argparse
import json
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "tools"))
import rim_model  # noqa: E402

p = argparse.ArgumentParser()
p.add_argument("--r-eff", type=float, default=1.5)
p.add_argument("--noise", type=float, default=100.0, help="Zeitrauschen 1 sigma in ns")
p.add_argument("--shots", type=int, default=60)
p.add_argument("--overlap-every", type=int, default=4,
               help="jeder n-te Schuss trifft 1-3 mm neben einen frueheren")
p.add_argument("--area", type=float, default=60.0, help="Trefferradius um die Mitte in mm")
p.add_argument("--seed", type=int, default=7)
a = p.parse_args()

rng = random.Random(a.seed)
HX, HY, Z, C = 115.0, 85.0, 28.0, 355e-6
mic_x = [-HX, HX, -HX, HX, -HX, HX]
mic_y = [-HY, -HY, HY, HY, 0.0, 0.0]

print(json.dumps({"type": "config", "target": "paper", "mic_offset_ns": [0] * 6,
                  "mic_enabled": [1] * 6, "sound_mps": 355, "mic_half_x_mm": HX,
                  "standoff_paper_mm": Z, "paper_auto": 0, "paper_trigger": "piezo"}))
holes = []
for seq in range(1, a.shots + 1):
    if holes and seq % a.overlap_every == 0:
        hx, hy = rng.choice(holes)
        d, ang = rng.uniform(1.0, 3.0), rng.uniform(0, 2 * math.pi)
        x, y = hx + d * math.cos(ang), hy + d * math.sin(ang)
    else:
        rr, ang = a.area * math.sqrt(rng.random()), rng.uniform(0, 2 * math.pi)
        x, y = rr * math.cos(ang), rr * math.sin(ang)
    near = [h for h in holes if math.hypot(h[0] - x, h[1] - y) < 2 * 2.25 + 1]
    mask = rim_model.emitting_mask(x, y, near, 2.25) if near else None
    if mask is not None and not any(mask):
        holes.append((x, y))
        continue          # komplett ueberdeckt: kein Schall
    t = []
    for i in range(6):
        d = rim_model.path_and_grad(x, y, mic_x[i], mic_y[i], Z, a.r_eff, mask)[0]
        t.append(d / C + rng.gauss(0, a.noise))
    t0 = min(t)
    air = [[round(v - t0), round(v - t0 + rng.uniform(150000, 350000))] for v in t]
    frac = 1.0 if mask is None else sum(mask) / len(mask)
    print(json.dumps({"type": "shot", "seq": seq, "air_ns": air, "piezo_ns": 800000,
                      "true_x_mm": round(x, 4), "true_y_mm": round(y, 4),
                      "true_overlap": round(1 - frac, 3)}))
    holes.append((x, y))
