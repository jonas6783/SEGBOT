# -*- coding: utf-8 -*-
"""
pfadplanung.py — Schleifpfade: planvoll statt reaktiv
=====================================================
Aus den uebertoleranten Punkten (nur Material DRUEBER) werden je Zone
DBSCAN-Cluster gebildet und daraus Wegpunkte mit Flaechennormalen:
Grate als Mittellinie entlang der Hauptachse, Anguesse als
Serpentinen-Raster in der Tangentialebene. Danach sorgt die Strategie
dafuer, dass das Bauteil moeglichst wenig umorientiert werden muss:
split_regions_by_side zerlegt Striche an starken Normalen-Kippungen und
an der Bauteil-Trennebene (seitenweises Abarbeiten), optimize_grind_order
sortiert die Regionen nach kleinster Normalen-Verdrehung — der Gewinn
wird in Grad gemessen und mitgeloggt.

Bekannte Grenzen (bewusst einfach gehalten): Die Mittellinie nutzt die
PCA-Hauptachse — ein stark GEBOGENER Grat, der als eine Region
geclustert wird, wuerde falsch verdichtet (unsere Grate sind gerade
Leisten, und der Seiten-Split entschaerft Kanten zusaetzlich). Das
Raster ueberspringt leere Zellen ohne Abheben — bei ringfoermigen
Befunden liefe der Pfad ueber das Loch (unsere Angussreste sind
kompakte Flecken).
"""

import numpy as np
import open3d as o3d

from konfig import (GRIND_AREA_STEPOVER, GRIND_DBSCAN_EPS,
                    GRIND_DBSCAN_MIN, GRIND_ENABLE, GRIND_INCLUDE_STANDARD,
                    GRIND_LINE_SPACING, GRIND_MIN_REGION_PTS,
                    SPLIT_EBENE_Y)

def centerline_waypoints(fpts, normals, spacing):
    """Grat: Wegpunkte entlang der Hauptachse (Mittellinie)."""
    c = fpts.mean(0); P = fpts - c
    w, V = np.linalg.eigh(np.cov(P.T)); axis = V[:, np.argmax(w)]
    t = P @ axis
    lo, hi = t.min(), t.max()
    nb = max(2, int(np.ceil((hi - lo) / spacing)))
    edges = np.linspace(lo, hi, nb + 1); wps = []
    for b in range(nb):
        m = (t >= edges[b]) & (t <= edges[b + 1])
        if m.sum() == 0:
            continue
        pos = fpts[m].mean(0); nor = normals[m].mean(0); nor /= np.linalg.norm(nor) + 1e-9
        wps.append((pos, nor))
    return wps


def raster_waypoints(fpts, normals, stepover):
    """Anguss: Serpentinen-Raster ueber die Grundflaeche."""
    c = fpts.mean(0); n = normals.mean(0); n /= np.linalg.norm(n) + 1e-9
    P = fpts - c
    Ptan = P - np.outer(P @ n, n)
    w, V = np.linalg.eigh(np.cov(Ptan.T)); o = np.argsort(w)[::-1]
    u, v = V[:, o[0]], V[:, o[1]]
    uu, vv = P @ u, P @ v
    nu = max(1, int(np.ceil((uu.max() - uu.min()) / stepover)))
    nv = max(1, int(np.ceil((vv.max() - vv.min()) / stepover)))
    ue = np.linspace(uu.min(), uu.max(), nu + 1); ve = np.linspace(vv.min(), vv.max(), nv + 1)
    wps = []
    for iu in range(nu):
        rng = range(nv) if iu % 2 == 0 else range(nv - 1, -1, -1)
        for iv in rng:
            cell = (uu >= ue[iu]) & (uu <= ue[iu + 1]) & (vv >= ve[iv]) & (vv <= ve[iv + 1])
            if cell.sum() == 0:
                continue
            pos = fpts[cell].mean(0); nor = normals[cell].mean(0); nor /= np.linalg.norm(nor) + 1e-9
            wps.append((pos, nor))
    return wps


def detect_grind_regions(scan_pts, dist, foot, foot_nrm, assignment, zones):
    if not GRIND_ENABLE:
        return []
    regions = []; rid = 0
    for zi, z in enumerate(zones):
        if z["is_standard"] and not GRIND_INCLUDE_STANDARD:
            continue
        mask = (assignment == zi) & (dist > z["tolerance_m"])   # nur Material DRUEBER
        if mask.sum() < GRIND_MIN_REGION_PTS:
            continue
        pts, fpts, nrm, dv = scan_pts[mask], foot[mask], foot_nrm[mask], dist[mask]
        sub = o3d.geometry.PointCloud(); sub.points = o3d.utility.Vector3dVector(pts)
        labels = np.array(sub.cluster_dbscan(eps=GRIND_DBSCAN_EPS, min_points=GRIND_DBSCAN_MIN))
        for lab in sorted(set(labels)):
            if lab < 0:
                continue
            cm = labels == lab
            if cm.sum() < GRIND_MIN_REGION_PTS:
                continue
            is_grat = "grat" in z["name"].lower()
            if is_grat:
                wp = centerline_waypoints(fpts[cm], nrm[cm], GRIND_LINE_SPACING); typ = "line"
            else:
                wp = raster_waypoints(fpts[cm], nrm[cm], GRIND_AREA_STEPOVER); typ = "area"
            if not wp:
                continue
            regions.append({
                "id": rid, "zone": z["name"], "type": typ, "n_points": int(cm.sum()),
                "max_removal_mm": round(float(dv[cm].max() * 1000), 3),
                "waypoints": [{"xyz": [round(float(p[0]) * 1000, 3), round(float(p[1]) * 1000, 3),
                                       round(float(p[2]) * 1000, 3)],
                               "normal": [round(float(nn[0]), 4), round(float(nn[1]), 4),
                                          round(float(nn[2]), 4)]}
                              for p, nn in wp],
            })
            rid += 1
    return regions


def _region_normal(r):
    n = np.array([w["normal"] for w in r["waypoints"]], float).mean(0)
    return n / (np.linalg.norm(n) + 1e-9)


def _total_reorient_deg(regions):
    """Summe der Winkel zwischen aufeinanderfolgenden Regionen-Normalen (Grad) =
    Mass fuer das gesamte Umorientieren des Bauteils zwischen den Regionen."""
    ns = [_region_normal(r) for r in regions]
    return float(sum(np.degrees(np.arccos(np.clip(np.dot(a, b), -1, 1)))
                     for a, b in zip(ns[:-1], ns[1:])))


def split_regions_by_side(regions, side_angle_deg):
    """Zerlegt eine Region in zusammenhaengende Teilstuecke, sobald (a) die
    Flaechennormale zu stark kippt oder (b) der Strich die Trennebene
    (Y = SPLIT_EBENE_Y, s. konfig.py) ueberquert, die das Bauteil halbiert. So wird z.B. ein Grat
    der um eine Kante laeuft seitenweise abgearbeitet statt am Stueck quer
    ueber das ganze Teil."""
    cosT = np.cos(np.radians(side_angle_deg))
    out = []
    for r in regions:
        W = r["waypoints"]
        if len(W) < 2:
            out.append(r); continue
        segs, seg = [], [W[0]]
        ref = np.array(W[0]["normal"], float)
        side = 1 if W[0]["xyz"][1] >= SPLIT_EBENE_Y else -1
        for w in W[1:]:
            nw = np.array(w["normal"], float)
            sw = 1 if w["xyz"][1] >= SPLIT_EBENE_Y else -1
            if float(np.dot(ref, nw)) < cosT or sw != side:
                segs.append(seg); seg = [w]; ref = nw; side = sw
            else:
                seg.append(w); ref = ref + nw; ref = ref / (np.linalg.norm(ref) + 1e-9)
        segs.append(seg)
        merged = []                                  # winzige Stuecke an Nachbarn haengen
        for s in segs:
            if merged and len(s) < 2:
                merged[-1].extend(s)
            else:
                merged.append(s)
        if len(merged) >= 2 and len(merged[0]) < 2:  # fuehrendes Mini-Stueck nach vorne mergen
            merged[1][:0] = merged[0]; merged.pop(0)
        if len(merged) == 1:
            out.append(r); continue
        for s in merged:
            rr = dict(r); rr["waypoints"] = s
            out.append(rr)
    return out


def optimize_grind_order(regions):
    """Ordnet die Schleifregionen so, dass das Bauteil moeglichst wenig umorientiert
    werden muss: Kosten = Winkel zwischen Flaechennormalen (dominiert) + Weg als
    Tie-Breaker. Greedy naechster-Nachbar; Striche duerfen umgedreht werden, damit
    Ein-/Ausstieg zusammenpassen. Rueckgabe (geordnete_regionen, (vorher_deg, nachher_deg))."""
    if len(regions) <= 1:
        return regions, None
    feat = []
    for r in regions:
        W = r["waypoints"]
        feat.append({"n": _region_normal(r),
                     "p0": np.array(W[0]["xyz"], float),
                     "p1": np.array(W[-1]["xyz"], float)})
    ALPHA, BETA = 1.0, 2.0                       # rad bzw. pro Meter (Weg nur Tie-Breaker)
    def ang(a, b): return float(np.arccos(np.clip(np.dot(a, b), -1, 1)))

    n = len(regions)
    start = max(range(n), key=lambda i: len(regions[i]["waypoints"]))  # groesste zuerst
    order, rev = [start], {start: False}
    unused = set(range(n)) - {start}
    cur_n, cur_p = feat[start]["n"], feat[start]["p1"]
    while unused:
        best = None
        for j in unused:
            d0 = np.linalg.norm(feat[j]["p0"] - cur_p)
            d1 = np.linalg.norm(feat[j]["p1"] - cur_p)
            entry_rev = d1 < d0                  # naeheres Endstueck wird Einstieg
            cost = ALPHA * ang(cur_n, feat[j]["n"]) + BETA * (min(d0, d1) / 1000.0)
            if best is None or cost < best[0]:
                best = (cost, j, entry_rev)
        _, j, entry_rev = best
        order.append(j); rev[j] = entry_rev; unused.discard(j)
        cur_n = feat[j]["n"]
        cur_p = feat[j]["p0"] if entry_rev else feat[j]["p1"]

    before = _total_reorient_deg(regions)
    out = []
    for new_id, i in enumerate(order):
        r = dict(regions[i])
        if rev[i]:
            r["waypoints"] = list(reversed(r["waypoints"]))
        r["id"] = new_id
        out.append(r)
    return out, (before, _total_reorient_deg(out))


