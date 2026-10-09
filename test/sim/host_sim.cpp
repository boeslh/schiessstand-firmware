// Host-Simulation: vergleicht den bisherigen Algorithmus (Rev 4.10.1,
// solveAirPosition inkl. BSHIFT + Stufe 2) mit shot_locator.h - und seit
// Rev 4.12.0 zusaetzlich die neue Stufe 2 (SET CLUSTERCORR/CLUSTERITER/LSQ).
//   g++ -O2 -std=c++17 -I../../include host_sim.cpp -o host_sim && ./host_sim
// Regressionspruefung (Exit-Code 1 bei Fehler): die 4.12.0-Portierung mit
// CLUSTERCORR=0, CLUSTERITER=0, LSQ=0 muss dasselbe Ergebnis liefern wie die
// Portierung des bisherigen Algorithmus (Toleranz 1 um).
#include "shot_locator.h"
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>

using namespace shotloc;

// ------------------------- Portierung Rev 4.10.1 --------------------------
struct OldCfg { int bshiftPct = 50; float bshiftCap = 3.0f; float clusterR = 0.2f; };

static bool oldSolvePair(const Geometry &g, int ref, int a, int b, const double t[NMIC],
                         float *X, float *Y, float *Dd)
{
    const int m[3] = {ref, a, b};
    const float s[3] = {0.0f, (float)((t[a]-t[ref])*g.soundMmPerNs), (float)((t[b]-t[ref])*g.soundMmPerNs)};
    float s0;
    if (!solveTrio(g, m, s, X, Y, &s0)) return false;
    *Dd = -s0;
    return true;
}

static bool oldSolve(const Geometry &g, const OldCfg &oc, const double t[NMIC], const bool seen[NMIC],
                     float *ox, float *oy)
{
    int all[NMIC], n = 0;
    for (int i = 0; i < NMIC; i++) if (seen[i]) all[n++] = i;
    if (n < 3) return false;
    float cx[20], cy[20]; int nc = 0, bestIdx = -1;
    bool found = false; float bestRes = 0, bx = 0, by = 0, bd = 0; int bref = -1, ba = -1, bb = -1;
    for (int i0 = 0; i0 < n; i0++) for (int i1 = i0+1; i1 < n; i1++) for (int i2 = i1+1; i2 < n; i2++) {
        int tr[3] = {all[i0], all[i1], all[i2]};
        int ref = tr[0]; for (int k = 1; k < 3; k++) if (t[tr[k]] < t[ref]) ref = tr[k];
        int a = -1, b = -1; for (int k = 0; k < 3; k++) { if (tr[k]==ref) continue; if (a<0) a=tr[k]; else b=tr[k]; }
        float x, y, d; if (!oldSolvePair(g, ref, a, b, t, &x, &y, &d)) continue;
        int ci = nc; cx[nc] = x; cy[nc] = y; nc++;
        float rs = 0; int nk = 0;
        for (int k = 0; k < n; k++) { int m = all[k]; if (m==ref||m==a||m==b) continue;
            float dc = sqrtf((x-g.micX[m])*(x-g.micX[m])+(y-g.micY[m])*(y-g.micY[m])+g.standoffMm*g.standoffMm);
            float rc = (float)((t[m]-t[ref])*g.soundMmPerNs); rs += fabsf(dc-(d+rc)); nk++; }
        float r = nk ? rs/nk : 0;
        if (!found || r < bestRes) { found = true; bestRes = r; bx = x; by = y; bd = d; bref = ref; ba = a; bb = b; bestIdx = ci; }
    }
    if (!found) return false;
    if (oc.bshiftPct > 0) {
        float sx = 0, sy = 0;
        for (int k = 0; k < n; k++) { int m = all[k]; if (m==bref||m==ba||m==bb) continue;
            float mdx = g.micX[m]-bx, mdy = g.micY[m]-by, dm = sqrtf(mdx*mdx+mdy*mdy);
            float dc = sqrtf(dm*dm + g.standoffMm*g.standoffMm);
            float rc = (float)((t[m]-t[bref])*g.soundMmPerNs);
            float sh = (dc-(bd+rc))*oc.bshiftPct/100.0f;
            sh = std::max(-oc.bshiftCap, std::min(oc.bshiftCap, sh));
            sx += sh*mdx/dm; sy += sh*mdy/dm; }
        bx += sx; by += sy;
    }
    float sx = bx, sy = by; int ns = 1;
    for (int i = 0; i < nc; i++) { if (i==bestIdx) continue;
        float dx = cx[i]-bx, dy = cy[i]-by; if (sqrtf(dx*dx+dy*dy) <= oc.clusterR) { sx += cx[i]; sy += cy[i]; ns++; } }
    *ox = sx/ns; *oy = sy/ns;
    return true;
}

// ------------------------- Portierung Rev 4.12.0 --------------------------
// Entspricht solveAirPosition()/bulletShiftFor()/lsqRefine() der Firmware.
struct FwCfg {
    int   bshiftPct   = 40;     // SET BSHIFTPCT (Firmware-Default)
    float bshiftCap   = 3.0f;   // SET BSHIFTCAP
    float clusterR    = 0.2f;   // SET RADIUS
    bool  clusterCorr = true;   // SET CLUSTERCORR
    int   clusterIter = 5;      // SET CLUSTERITER
    bool  lsq         = false;  // SET LSQ
    float lsqPelletR  = 2.25f;  // SET LSQPELLETR
};

static void fwShift(const Geometry &g, const FwCfg &fc, float x, float y, float d,
                    int ref, int a, int b, const int all[NMIC], int n, const double t[NMIC],
                    float *osx, float *osy)
{
    float sx = 0, sy = 0;
    for (int k = 0; k < n; k++) {
        const int m = all[k];
        if (m == ref || m == a || m == b) continue;
        const float mdx = g.micX[m] - x, mdy = g.micY[m] - y;
        const float distM = sqrtf(mdx*mdx + mdy*mdy);
        if (distM < 1.0e-3f) continue;
        const float dc = sqrtf(mdx*mdx + mdy*mdy + g.standoffMm*g.standoffMm);
        const float rc = (float)((t[m] - t[ref]) * g.soundMmPerNs);
        float sh = (dc - (d + rc)) * ((float)fc.bshiftPct / 100.0f);
        sh = std::max(-fc.bshiftCap, std::min(fc.bshiftCap, sh));
        sx += sh * (mdx / distM);
        sy += sh * (mdy / distM);
    }
    *osx = sx; *osy = sy;
}

static float fwLsqPath(const Geometry &g, const FwCfg &fc, int i, float x, float y, float *gx, float *gy)
{
    const float dx = x - g.micX[i], dy = y - g.micY[i];
    float rho = sqrtf(dx*dx + dy*dy);
    if (rho < 1.0e-3f) rho = 1.0e-3f;
    float rr = rho - fc.lsqPelletR;
    if (rr < 0.0f) rr = 0.0f;
    const float D = sqrtf(rr*rr + g.standoffMm*g.standoffMm);
    const float k = rr / (D * rho);
    *gx = k * dx; *gy = k * dy;
    return D;
}

static float fwDet3(const float m[3][3])
{
    return m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])
         - m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])
         + m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
}

static bool fwLsqFit(const Geometry &g, const FwCfg &fc, const float tr[NMIC], const bool use[NMIC],
                     float x0, float y0, float *ox, float *oy, float res[NMIC])
{
    float x = x0, y = y0, s0 = 0, gx, gy;
    int n = 0;
    for (int i = 0; i < NMIC; i++) {
        if (!use[i]) continue;
        s0 += fwLsqPath(g, fc, i, x, y, &gx, &gy) - g.soundMmPerNs * tr[i];
        n++;
    }
    if (n < 4) return false;
    s0 /= (float)n;
    for (int it = 0; it < 6; it++) {
        float A[3][3] = {{0}}, b[3] = {0}, dl[3];
        for (int i = 0; i < NMIC; i++) {
            if (!use[i]) continue;
            const float D = fwLsqPath(g, fc, i, x, y, &gx, &gy);
            const float r = D - (s0 + g.soundMmPerNs * tr[i]);
            const float J[3] = { gx, gy, -1.0f };
            for (int p = 0; p < 3; p++) {
                for (int q = 0; q < 3; q++) A[p][q] += J[p] * J[q];
                b[p] -= J[p] * r;
            }
        }
        const float det = fwDet3(A);
        if (fabsf(det) < 1.0e-12f) return false;
        for (int k = 0; k < 3; k++) {
            float m[3][3];
            for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) m[r][c] = (c == k) ? b[r] : A[r][c];
            dl[k] = fwDet3(m) / det;
        }
        x += dl[0]; y += dl[1]; s0 += dl[2];
        if (sqrtf(dl[0]*dl[0] + dl[1]*dl[1]) < 1.0e-5f) break;
    }
    if (sqrtf((x-x0)*(x-x0) + (y-y0)*(y-y0)) > 10.0f) return false;
    for (int i = 0; i < NMIC; i++)
        res[i] = use[i] ? fwLsqPath(g, fc, i, x, y, &gx, &gy) - (s0 + g.soundMmPerNs * tr[i]) : 0.0f;
    *ox = x; *oy = y;
    return true;
}

static float fwLsqRms(const float res[NMIC], const bool use[NMIC])
{
    float s = 0; int n = 0;
    for (int i = 0; i < NMIC; i++) if (use[i]) { s += res[i]*res[i]; n++; }
    return n ? sqrtf(s / n) : 0.0f;
}

static void fwLsqRefine(const Geometry &g, const FwCfg &fc, const double t[NMIC], const bool seen[NMIC],
                        float *x, float *y)
{
    // Firmware rechnet mit ganzzahligen ns relativ zur ersten Flanke
    double tmin = 1e300;
    for (int i = 0; i < NMIC; i++) if (seen[i]) tmin = std::min(tmin, t[i]);
    float tr[NMIC];
    for (int i = 0; i < NMIC; i++) tr[i] = seen[i] ? (float)(int64_t)llround(t[i] - tmin) : 0.0f;
    bool use[NMIC]; int n = 0;
    for (int i = 0; i < NMIC; i++) { use[i] = seen[i]; if (use[i]) n++; }
    float fx, fy, res[NMIC];
    if (!fwLsqFit(g, fc, tr, use, *x, *y, &fx, &fy, res)) return;
    const float bestRms = fwLsqRms(res, use);
    if (n >= 5 && bestRms > 0.1f) {
        int ord[NMIC], no = 0;
        for (int i = 0; i < NMIC; i++) if (use[i]) ord[no++] = i;
        std::stable_sort(ord, ord + no, [&](int p, int q) { return fabsf(res[p]) > fabsf(res[q]); });
        for (int k = 0; k < 2 && k < no; k++) {
            use[ord[k]] = false;
            float x2, y2, r2[NMIC];
            if (fwLsqFit(g, fc, tr, use, *x, *y, &x2, &y2, r2) && fwLsqRms(r2, use) < 0.5f * bestRms) {
                fx = x2; fy = y2;
                break;
            }
            use[ord[k]] = true;
        }
    }
    *x = fx; *y = fy;
}

static bool fwSolve(const Geometry &g, const FwCfg &fc, const double t[NMIC], const bool seen[NMIC],
                    float *ox, float *oy)
{
    int all[NMIC], n = 0;
    for (int i = 0; i < NMIC; i++) if (seen[i]) all[n++] = i;
    if (n < 3) return false;
    float cx[20], cy[20], cd[20]; int cr[20], ca[20], cb[20];
    int nc = 0, bestIdx = -1;
    bool found = false; float bestRes = 0, bx = 0, by = 0, bd = 0; int bref = -1, ba = -1, bb = -1;
    for (int i0 = 0; i0 < n; i0++) for (int i1 = i0+1; i1 < n; i1++) for (int i2 = i1+1; i2 < n; i2++) {
        int tr[3] = {all[i0], all[i1], all[i2]};
        int ref = tr[0]; for (int k = 1; k < 3; k++) if (t[tr[k]] < t[ref]) ref = tr[k];
        int a = -1, b = -1; for (int k = 0; k < 3; k++) { if (tr[k]==ref) continue; if (a<0) a=tr[k]; else b=tr[k]; }
        float x, y, d; if (!oldSolvePair(g, ref, a, b, t, &x, &y, &d)) continue;
        int ci = nc; cx[nc] = x; cy[nc] = y; cd[nc] = d; cr[nc] = ref; ca[nc] = a; cb[nc] = b; nc++;
        float rs = 0; int nk = 0;
        for (int k = 0; k < n; k++) { int m = all[k]; if (m==ref||m==a||m==b) continue;
            float dc = sqrtf((x-g.micX[m])*(x-g.micX[m])+(y-g.micY[m])*(y-g.micY[m])+g.standoffMm*g.standoffMm);
            float rc = (float)((t[m]-t[ref])*g.soundMmPerNs); rs += fabsf(dc-(d+rc)); nk++; }
        float r = nk ? rs/nk : 0;
        if (!found || r < bestRes) { found = true; bestRes = r; bx = x; by = y; bd = d; bref = ref; ba = a; bb = b; bestIdx = ci; }
    }
    if (!found) return false;
    if (fc.bshiftPct > 0) {
        float sx, sy;
        fwShift(g, fc, bx, by, bd, bref, ba, bb, all, n, t, &sx, &sy);
        bx += sx; by += sy;
    }
    // Stufe 2: Cluster (ggf. aus korrigierten Schnittpunkten) + Mean-Shift
    float px[20], py[20];
    const bool corrAll = fc.clusterCorr && fc.bshiftPct > 0;
    for (int i = 0; i < nc; i++) {
        if (i == bestIdx) { px[i] = bx; py[i] = by; continue; }
        px[i] = cx[i]; py[i] = cy[i];
        if (corrAll) {
            float sx, sy;
            fwShift(g, fc, cx[i], cy[i], cd[i], cr[i], ca[i], cb[i], all, n, t, &sx, &sy);
            px[i] += sx; py[i] += sy;
        }
    }
    float vx = bx, vy = by;
    for (int pass = 0; pass <= fc.clusterIter; pass++) {
        float sx = 0, sy = 0; int ns = 0;
        for (int k = -1; k < nc; k++) {
            const int i = (k < 0) ? bestIdx : k;
            if (i < 0 || (k >= 0 && i == bestIdx)) continue;
            const float dx = px[i] - vx, dy = py[i] - vy;
            if (sqrtf(dx*dx + dy*dy) <= fc.clusterR) { sx += px[i]; sy += py[i]; ns++; }
        }
        if (ns == 0) break;
        const float nx = sx / ns, ny = sy / ns;
        const float moved = sqrtf((nx-vx)*(nx-vx) + (ny-vy)*(ny-vy));
        vx = nx; vy = ny;
        if (moved < 1.0e-4f) break;
    }
    if (fc.lsq) fwLsqRefine(g, fc, t, seen, &vx, &vy);
    *ox = vx; *oy = vy;
    return true;
}

// ------------------------------ Simulation --------------------------------
struct Scenario {
    const char *name;
    float jitterNs;       // Gauss-Rauschen je Flanke
    float pSpurious;      // Wahrsch. je Mic fuer Stoerflanke VOR dem Direktschall (im Fenster)
    float pMuzzle;        // Wahrsch. fuer Muendungsknall ~38ms vorher (alle Mics)
    float targetR;        // Treffer gleichverteilt im Kreis mit diesem Radius
};

static void stats(std::vector<float> &e, const char *label, int fails, int total)
{
    if (e.empty()) { printf("  %-26s keine Loesungen\n", label); return; }
    std::sort(e.begin(), e.end());
    double s2 = 0; for (float v : e) s2 += v*v;
    printf("  %-26s RMS %6.3f mm  P95 %6.3f mm  max %7.2f mm  ohne Ergebnis %4.1f%%\n",
           label, sqrt(s2/e.size()), e[(size_t)(0.95*(e.size()-1))], e.back(), 100.0*fails/total);
}

int main()
{
    Geometry g{};
    const float hx = 115, hy = 85;
    const float mx[NMIC] = {-hx, hx, -hx, hx, -hx, hx};
    const float my[NMIC] = {-hy, -hy, hy, hy, 0, 0};
    memcpy(g.micX, mx, sizeof(mx)); memcpy(g.micY, my, sizeof(my));
    g.standoffMm = 28; g.soundMmPerNs = 0.000355f; g.pelletRadiusMm = 2.25f; g.maxDistMm = 320;

    Params p; p.minMics = 4;
    Scenario sc[] = {
        {"Ideal (20 ns Rauschen)",                20, 0.00f, 0.0f, 40},
        {"Realistisch (300 ns Rauschen)",        300, 0.00f, 0.0f, 40},
        {"300 ns + 5% Stoerflanken je Mic",      300, 0.05f, 0.0f, 40},
        {"300 ns + 5% Stoer + 30% Muendungsknall",300, 0.05f, 0.3f, 40},
        {"300 ns, Treffer bis 70 mm vom Zentrum",300, 0.00f, 0.0f, 70},
    };
    const int N = 4000;
    // Regressionspruefung: 4.12.0 mit allen neuen Optionen aus gegen "alt"
    long regChecked = 0, regFailed = 0;
    float regMaxDiff = 0.0f;
    FwCfg fcLegacy; fcLegacy.bshiftPct = 50; fcLegacy.clusterCorr = false; fcLegacy.clusterIter = 0;
    FwCfg fc4115 = fcLegacy; fc4115.bshiftPct = 40;   // Verhalten 4.11.5 mit Firmware-Defaults
    FwCfg fcDef;                                      // Firmware-Defaults 4.12.0
    FwCfg fcLsq; fcLsq.lsq = true;
    for (auto &s : sc) {
        std::mt19937 rng(1234);
        std::normal_distribution<float> nd(0, 1);
        std::uniform_real_distribution<float> ud(0, 1);
        std::vector<float> eOld, eOld0, eNew, eNewPt, e4115, eDef, eLsq;
        int fOld = 0, fOld0 = 0, fNew = 0, fNewPt = 0, f4115 = 0, fDef = 0, fLsq = 0;
        double sigSum = 0; int sigN = 0, cover = 0;
        for (int k = 0; k < N; k++) {
            float rr = s.targetR * sqrtf(ud(rng)), ph = 6.2831853f*ud(rng);
            float X = rr*cosf(ph), Y = rr*sinf(ph);
            // Echter Einschlag t0 relativ zum Piezo: PAPER, Luecke 8-18cm @150m/s
            float delta = 530000 + 670000*ud(rng);
            float t0 = -delta;
            Edges ed{}; double oldT[NMIC]; bool oldSeen[NMIC];
            bool muzzle = ud(rng) < s.pMuzzle;
            for (int i = 0; i < NMIC; i++) {
                ed.enabled[i] = true;
                float tDirect = t0 + pathMm(g, i, X, Y, nullptr, nullptr)/g.soundMmPerNs + s.jitterNs*nd(rng);
                std::vector<float> v;
                if (muzzle) v.push_back(t0 - 38e6f + 2000*ud(rng));
                if (ud(rng) < s.pSpurious) v.push_back(tDirect - 50000 - 300000*ud(rng));
                v.push_back(tDirect);
                v.push_back(tDirect + 150000 + 200000*ud(rng));   // Echo
                std::sort(v.begin(), v.end());
                ed.n[i] = 0; for (float t : v) ed.t[i][ed.n[i]++] = t;
            }
            // Alter Algorithmus: erste Flanke oeffnet Fenster, TDOA-Filter 750us.
            // (Bei Muendungsknall landet der echte Einschlag in der Sperrzeit.)
            float first = 1e30f; for (int i = 0; i < NMIC; i++) first = std::min(first, ed.t[i][0]);
            int hits = 0;
            for (int i = 0; i < NMIC; i++) {
                oldSeen[i] = false;
                for (int q = 0; q < ed.n[i]; q++)
                    if (ed.t[i][q] - first <= 750000) { oldT[i] = ed.t[i][q]; oldSeen[i] = true; break; }
                hits += oldSeen[i];
            }
            float ox, oy;
            OldCfg oc; OldCfg oc0; oc0.bshiftPct = 0;
            bool okOld = hits >= 5 && !muzzle && oldSolve(g, oc, oldT, oldSeen, &ox, &oy);
            if (okOld) eOld.push_back(hypotf(ox-X, oy-Y)); else fOld++;
            okOld = hits >= 5 && !muzzle && oldSolve(g, oc0, oldT, oldSeen, &ox, &oy);
            if (okOld) eOld0.push_back(hypotf(ox-X, oy-Y)); else fOld0++;

            // Rev 4.12.0 (gleiche Eingangsdaten wie "alt")
            const bool fwIn = hits >= 5 && !muzzle;
            if (fwIn) {
                float ax, ay, bx2, by2;
                const bool oa = oldSolve(g, oc, oldT, oldSeen, &ax, &ay);
                const bool ob = fwSolve(g, fcLegacy, oldT, oldSeen, &bx2, &by2);
                const float diff = (oa && ob) ? hypotf(ax-bx2, ay-by2) : 0.0f;
                regChecked++;
                if (oa != ob || diff > 1.0e-3f) regFailed++;
                regMaxDiff = std::max(regMaxDiff, diff);
            }
            if (fwIn && fwSolve(g, fc4115, oldT, oldSeen, &ox, &oy)) e4115.push_back(hypotf(ox-X, oy-Y)); else f4115++;
            if (fwIn && fwSolve(g, fcDef,  oldT, oldSeen, &ox, &oy)) eDef.push_back(hypotf(ox-X, oy-Y));  else fDef++;
            if (fwIn && fwSolve(g, fcLsq,  oldT, oldSeen, &ox, &oy)) eLsq.push_back(hypotf(ox-X, oy-Y));  else fLsq++;

            Gate gate{true, -1400000.0f, -100000.0f};
            Result r;
            if (locate(g, ed, gate, p, &r) && r.valid) {
                float err = hypotf(r.xMm-X, r.yMm-Y);
                eNew.push_back(err);
                sigSum += hypotf(r.sigmaXMm, r.sigmaYMm); sigN++;
                if (fabsf(r.xMm-X) < 2*r.sigmaXMm && fabsf(r.yMm-Y) < 2*r.sigmaYMm) cover++;
            } else fNew++;
            Geometry gp = g; gp.pelletRadiusMm = 0;
            if (locate(gp, ed, gate, p, &r) && r.valid) eNewPt.push_back(hypotf(r.xMm-X, r.yMm-Y)); else fNewPt++;
        }
        printf("\n%s\n", s.name);
        stats(eOld,   "alt (BSHIFT 50%/3mm)", fOld, N);
        stats(eOld0,  "alt (BSHIFT aus)", fOld0, N);
        stats(e4115,  "4.11.5 (BSHIFT 40%)", f4115, N);
        stats(eDef,   "4.12.0 CORR+ITER=5", fDef, N);
        stats(eLsq,   "4.12.0 CORR+ITER+LSQ", fLsq, N);
        stats(eNewPt, "neu, Punktquelle", fNewPt, N);
        stats(eNew,   "neu, Lochrand-Modell", fNew, N);
        if (sigN) printf("  %-26s mittl. sigma %.3f mm, 2-sigma trifft in %.0f%%\n", "neu: Guete-Angabe",
                         sigSum/sigN, 100.0*cover/sigN);
    }
    printf("\nRegression 4.12.0 (CLUSTERCORR=0, CLUSTERITER=0, LSQ=0) vs. alt: "
           "%ld Schuesse, %ld Abweichungen, max. Differenz %.6f mm -> %s\n",
           regChecked, regFailed, regMaxDiff, regFailed ? "FEHLER" : "OK");
    return regFailed ? 1 : 0;
}
