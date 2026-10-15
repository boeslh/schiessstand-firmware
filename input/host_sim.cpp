// Host-Simulation: vergleicht den bisherigen Algorithmus (Rev 4.10.1,
// solveAirPosition inkl. BSHIFT + Stufe 2) mit shot_locator.h.
//   g++ -O2 -std=c++17 -I../include host_sim.cpp -o host_sim && ./host_sim
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
    for (auto &s : sc) {
        std::mt19937 rng(1234);
        std::normal_distribution<float> nd(0, 1);
        std::uniform_real_distribution<float> ud(0, 1);
        std::vector<float> eOld, eOld0, eNew, eNewPt;
        int fOld = 0, fOld0 = 0, fNew = 0, fNewPt = 0;
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
        stats(eNewPt, "neu, Punktquelle", fNewPt, N);
        stats(eNew,   "neu, Lochrand-Modell", fNew, N);
        if (sigN) printf("  %-26s mittl. sigma %.3f mm, 2-sigma trifft in %.0f%%\n", "neu: Guete-Angabe",
                         sigSum/sigN, 100.0*cover/sigN);
    }
    return 0;
}
