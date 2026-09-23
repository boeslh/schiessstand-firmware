#!/usr/bin/env python3
"""
Lochrand-Modell fuer die Offline-Auswertung (wird von replay_shot.py genutzt).

Physikalisches Modell
---------------------
Der Schall entsteht dort, wo das Papier durchtrennt wird - am Lochrand. Jedes
Mikrofon loest beim ERSTEN eintreffenden Schall aus, also beim Randpunkt, der
ihm am naechsten liegt:

    D_i = min_{u in U}  sqrt( |c + r_eff * u - m_i|_xy^2 + z^2 )

  c      Lochzentrum (gesucht)
  r_eff  wirksamer akustischer Radius (0 .. ~2.25 mm, per Scan bestimmt)
  U      Richtungen (Einheitsvektoren) des Randes, an denen WIRKLICH Papier
         durchtrennt wurde.

Teilkreis bei Ueberdeckung: Liegt ein frueheres Loch derselben Scheibe
teilweise im neuen, fehlt dort bereits Papier. Ein Randpunkt c + R*u (R =
physikalischer Kugelradius 2.25 mm) erzeugt nur dann Schall, wenn er NICHT
innerhalb eines frueheren Lochs liegt. U ist damit ein oder mehrere
Kreisboegen statt des Vollkreises. Ist U leer (Loch komplett ueberdeckt),
wird kein Papier durchtrennt - der Schuss ist akustisch nicht messbar.

Weil der naechste Punkt eines Bogens auch ein Bogen-ENDE sein kann, verschiebt
eine Teilueberdeckung die wirksame Quelle zur Seite - bei Voll-Kreis-Annahme
entsteht dann ein systematischer Positionsfehler Richtung Restbogen.

Nur Standardbibliothek (kein numpy), damit das Tool ueberall laeuft.
"""

import itertools
import math

N_RIM = 180            # Randdiskretisierung (2 Grad)
R_PHYS_DEFAULT = 2.25  # mm, 4,5-mm-Diabolo

RIM_DIRS = [(math.cos(2 * math.pi * k / N_RIM), math.sin(2 * math.pi * k / N_RIM))
            for k in range(N_RIM)]


# ---------------------------------------------------------------------------
# Ueberdeckung
# ---------------------------------------------------------------------------
def emitting_mask(cx, cy, prev_holes, r_phys):
    """Liste bool je Randrichtung: True = dort wurde Papier durchtrennt."""
    mask = []
    for ux, uy in RIM_DIRS:
        px, py = cx + r_phys * ux, cy + r_phys * uy
        free = True
        for hx, hy in prev_holes:
            if (px - hx) ** 2 + (py - hy) ** 2 < r_phys * r_phys:
                free = False
                break
        mask.append(free)
    return mask


def emitting_fraction(mask):
    return sum(mask) / len(mask) if mask else 1.0


# ---------------------------------------------------------------------------
# Modell + Gradient (Einhuellenden-Satz: Gradient am naechsten Randpunkt)
# ---------------------------------------------------------------------------
def path_and_grad(cx, cy, mx, my, z, r_eff, mask):
    best = None
    if r_eff <= 0.0:
        dx, dy = cx - mx, cy - my
        d = math.sqrt(dx * dx + dy * dy + z * z)
        return d, dx / d, dy / d
    if mask is None or all(mask):
        # Vollkreis: geschlossene Form
        dx, dy = cx - mx, cy - my
        rho = math.hypot(dx, dy) or 1e-6
        rr = max(rho - r_eff, 0.0)
        d = math.sqrt(rr * rr + z * z)
        k = rr / (d * rho)
        return d, k * dx, k * dy
    for (ux, uy), ok in zip(RIM_DIRS, mask):
        if not ok:
            continue
        qx, qy = cx + r_eff * ux - mx, cy + r_eff * uy - my
        d2 = qx * qx + qy * qy
        if best is None or d2 < best[0]:
            best = (d2, qx, qy)
    if best is None:
        return None
    d = math.sqrt(best[0] + z * z)
    return d, best[1] / d, best[2] / d


def _solve3(a, b):
    """3x3 lineares Gleichungssystem (Gauss mit Pivot). None bei Singularitaet."""
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for col in range(3):
        piv = max(range(col, 3), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-12:
            return None
        m[col], m[piv] = m[piv], m[col]
        for r in range(3):
            if r != col:
                f = m[r][col] / m[col][col]
                for c in range(col, 4):
                    m[r][c] -= f * m[col][c]
    return [m[i][3] / m[i][i] for i in range(3)]


def _inv3(a):
    cols = []
    for k in range(3):
        e = [1.0 if i == k else 0.0 for i in range(3)]
        col = _solve3(a, e)
        if col is None:
            return None
        cols.append(col)
    return [[cols[c][r] for c in range(3)] for r in range(3)]


# ---------------------------------------------------------------------------
# Startwert: Punktquelle, Dreier-Kombinationen (wie Firmware)
# ---------------------------------------------------------------------------
def _trio(ref, a, b, s, mic_x, mic_y, z):
    xr, yr = mic_x[ref], mic_y[ref]
    ra, rb = s[a] - s[ref], s[b] - s[ref]
    a1, b1, c1 = 2 * (mic_x[a] - xr), 2 * (mic_y[a] - yr), 2 * ra
    d1 = mic_x[a] ** 2 + mic_y[a] ** 2 - xr ** 2 - yr ** 2 - ra ** 2
    a2, b2, c2 = 2 * (mic_x[b] - xr), 2 * (mic_y[b] - yr), 2 * rb
    d2 = mic_x[b] ** 2 + mic_y[b] ** 2 - xr ** 2 - yr ** 2 - rb ** 2
    det = a1 * b2 - a2 * b1
    if abs(det) < 1e-9:
        return None
    x0, x1 = (d1 * b2 - d2 * b1) / det, (c2 * b1 - c1 * b2) / det
    y0, y1 = (a1 * d2 - a2 * d1) / det, (a2 * c1 - a1 * c2) / det
    px, py = x0 - xr, y0 - yr
    qa = 1 - x1 * x1 - y1 * y1
    qb = -2 * (px * x1 + py * y1)
    qc = -(px * px + py * py + z * z)
    if abs(qa) < 1e-9:
        if abs(qb) < 1e-9:
            return None
        d = -qc / qb
    else:
        disc = qb * qb - 4 * qa * qc
        if disc < 0:
            return None
        sq = math.sqrt(disc)
        cands = [v for v in ((-qb + sq) / (2 * qa), (-qb - sq) / (2 * qa)) if v > 0]
        if not cands:
            return None
        d = min(cands)
    return x0 + x1 * d, y0 + y1 * d, s[ref] - d


# ---------------------------------------------------------------------------
# Fit eines Schusses
# ---------------------------------------------------------------------------
class Geo:
    def __init__(self, mic_x, mic_y, standoff, sound_mm_per_ns):
        self.mic_x, self.mic_y = mic_x, mic_y
        self.z = standoff
        self.c = sound_mm_per_ns


def fit_shot(geo, edges, r_eff, mask, tol_ns=3000.0, huber_ns=800.0, min_mics=4):
    """
    edges: Liste je Mic mit Kandidatenzeiten in ns (bereits offset-korrigiert),
           leere Liste = Mic nicht erfasst.
    mask:  emitting_mask(...) oder None (= Vollkreis). r_eff=0 -> Punktquelle.
    Rueckgabe: dict oder None.
    """
    c = geo.c
    mics = [i for i, e in enumerate(edges) if e]
    if len(mics) < 3:
        return None
    tol = tol_ns * c
    kh = huber_ns * c

    def assign(x, y, s0):
        pick, cost = {}, 0.0
        for i in mics:
            pr = path_and_grad(x, y, geo.mic_x[i], geo.mic_y[i], geo.z, r_eff, mask)
            if pr is None:
                return None, 1e30
            best = min(edges[i], key=lambda t: abs(t * c - s0 - pr[0]))
            r = abs(best * c - s0 - pr[0])
            if r < tol:
                pick[i] = best
                cost += r * r
            else:
                cost += tol * tol
        return pick, cost

    # Hypothesen (MSAC) aus je den ersten 2 Flanken
    best_h = None
    for trio in itertools.combinations(mics, 3):
        for sel in itertools.product(*[edges[i][:2] for i in trio]):
            s = {i: t * c for i, t in zip(trio, sel)}
            ref = min(trio, key=lambda k: s[k])
            a, b = [k for k in trio if k != ref]
            sol = _trio(ref, a, b, s, geo.mic_x, geo.mic_y, geo.z)
            if sol is None:
                continue
            x, y, _ = sol
            if abs(x) > 400 or abs(y) > 400:
                continue
            # s0 an Randmodell anpassen (Randversatz ist fast gleichtaktig)
            ps = [path_and_grad(x, y, geo.mic_x[k], geo.mic_y[k], geo.z, r_eff, mask) for k in trio]
            if any(p is None for p in ps):
                continue
            s0 = sum(s[k] - p[0] for k, p in zip(trio, ps)) / 3
            pick, cost = assign(x, y, s0)
            if pick is None:
                continue
            key = (-len(pick), cost)
            if best_h is None or key < best_h[0]:
                best_h = (key, x, y, s0, pick)
    if best_h is None or len(best_h[4]) < 3:
        return None
    _, x, y, s0, pick = best_h

    ata = None
    for rnd in range(2):
        for _ in range(20):
            ata = [[0.0] * 3 for _ in range(3)]
            atb = [0.0] * 3
            for i, t in pick.items():
                d, gx, gy = path_and_grad(x, y, geo.mic_x[i], geo.mic_y[i], geo.z, r_eff, mask)
                r = t * c - s0 - d
                w = 1.0 if abs(r) <= kh else kh / abs(r)
                j = (gx, gy, 1.0)
                for u in range(3):
                    atb[u] += w * j[u] * r
                    for v in range(3):
                        ata[u][v] += w * j[u] * j[v]
            step = _solve3(ata, atb)
            if step is None:
                return None
            x, y, s0 = x + step[0], y + step[1], s0 + step[2]
            if abs(step[0]) < 1e-5 and abs(step[1]) < 1e-5:
                break
        if rnd == 0:
            new_pick, _ = assign(x, y, s0)
            if new_pick is None or len(new_pick) < 3:
                break
            pick = new_pick

    resid = {}
    for i, t in pick.items():
        d = path_and_grad(x, y, geo.mic_x[i], geo.mic_y[i], geo.z, r_eff, mask)[0]
        resid[i] = (t * c - s0 - d) / c   # ns
    n = len(resid)
    rss_ns2 = sum(v * v for v in resid.values())
    inv = _inv3(ata) if ata else None
    sig = None
    if inv and n > 3:
        s2 = rss_ns2 * c * c / (n - 3)
        sig = (math.sqrt(max(s2 * inv[0][0], 0)), math.sqrt(max(s2 * inv[1][1], 0)))
    return {
        "x": x, "y": y, "t0_ns": s0 / c, "n": n, "resid_ns": resid,
        "rss_ns2": rss_ns2, "dof": n - 3, "sigma_mm": sig,
        "valid": n >= min_mics,
    }


# ---------------------------------------------------------------------------
# Scheibe fuer Scheibe auswerten (Ueberdeckung aus frueheren Treffern)
# ---------------------------------------------------------------------------
def fit_sheets(geo, sheets, r_eff, r_phys=R_PHYS_DEFAULT, use_overlap=True, **kw):
    """
    sheets: Liste von Scheiben, jede eine Liste von (seq, edges).
    Liefert je Schuss dict inkl. 'overlap' (Anteil verdeckter Rand) und
    'full_cover' (komplett ueberdeckt).
    """
    out = []
    for sheet in sheets:
        holes = []
        for seq, edges in sheet:
            # 1. Fit mit Vollkreis, um die ungefaehre Lage zu kennen
            res = fit_shot(geo, edges, r_eff, None, **kw)
            mask, frac = None, 1.0
            if res is not None and use_overlap and holes:
                # 2. Maske an dieser Lage bestimmen, neu fitten, Maske nachziehen
                for _ in range(3):
                    near = [h for h in holes if math.hypot(h[0] - res["x"], h[1] - res["y"]) < 2 * r_phys + 1.0]
                    if not near:
                        mask = None
                        break
                    m = emitting_mask(res["x"], res["y"], near, r_phys)
                    frac = emitting_fraction(m)
                    if frac == 0.0:
                        mask = m
                        break
                    if frac == 1.0:
                        mask = None
                        break
                    r2 = fit_shot(geo, edges, r_eff, m, **kw)
                    if r2 is None:
                        break
                    moved = math.hypot(r2["x"] - res["x"], r2["y"] - res["y"])
                    res, mask = r2, m
                    if moved < 0.01:
                        break
            entry = {"seq": seq, "res": res, "overlap": 1.0 - frac,
                     "full_cover": mask is not None and frac == 0.0}
            out.append(entry)
            if res is not None:
                holes.append((res["x"], res["y"]))
    return out


def radius_scan(geo, sheets, radii, r_phys=R_PHYS_DEFAULT, use_overlap=True, **kw):
    """
    Fuer jeden Radius alle Schuesse fitten, gesamte Residuen-Quadratsumme
    und Freiheitsgrade sammeln. Nur Schuesse mit >= 4 Mics tragen bei.
    Rueckgabe: Liste (r, rss_ns2, dof, n_shots).
    """
    rows = []
    for r in radii:
        fits = fit_sheets(geo, sheets, r, r_phys, use_overlap, **kw)
        rss, dof, ns = 0.0, 0, 0
        for f in fits:
            res = f["res"]
            if res is None or res["dof"] <= 0 or f["full_cover"]:
                continue
            rss += res["rss_ns2"]
            dof += res["dof"]
            ns += 1
        rows.append((r, rss, dof, ns))
    return rows


def confidence_interval(rows):
    """
    chi^2-Kurve: chi2(r) = RSS(r) / sigma^2 mit sigma^2 = RSS_min / dof.
    1-Sigma-Bereich = alle r mit chi2 <= chi2_min + 1.
    """
    valid = [row for row in rows if row[2] > 0]
    if not valid:
        return None
    best = min(valid, key=lambda row: row[1])
    s2 = best[1] / best[2]
    if s2 <= 0:
        return None
    chi_min = best[1] / s2
    inside = [row[0] for row in valid if row[1] / s2 <= chi_min + 1.0]
    # Parabel durch die 3 Punkte um das Minimum fuer ein feineres Optimum
    idx = valid.index(best)
    r_opt = best[0]
    if 0 < idx < len(valid) - 1:
        (r0, y0, *_), (r1, y1, *_), (r2, y2, *_) = valid[idx - 1], valid[idx], valid[idx + 1]
        den = (r0 - r1) * (r0 - r2) * (r1 - r2)
        if abs(den) > 1e-12:
            a = (r2 * (y1 - y0) + r1 * (y0 - y2) + r0 * (y2 - y1)) / den
            b = (r2 * r2 * (y0 - y1) + r1 * r1 * (y2 - y0) + r0 * r0 * (y1 - y2)) / den
            if a > 0:
                r_opt = -b / (2 * a)
    return {"r_best": r_opt, "r_lo": min(inside), "r_hi": max(inside),
            "sigma_ns": math.sqrt(s2), "at_edge": idx in (0, len(valid) - 1)}
