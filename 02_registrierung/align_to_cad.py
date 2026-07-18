#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
align_to_cad.py — Scans ans CAD ausrichten (Stufe 2 der Pipeline)
=================================================================

Was dieses Programm macht, in einem Satz: Es nimmt alle Einzelscans aus
Stufe 1 (capture.py), legt jeden davon passgenau auf das CAD-Modell und
fügt sie zu EINER sauberen Gesamtpunktwolke im CAD-Koordinatensystem
zusammen — genau die Datei, die Stufe 3 (die Markier-Pipeline) als
"registrierten Scan" erwartet.

Wie es grob funktioniert (kein Hexenwerk, nur Fleiß):
  1. Startpositionen raten: Aus den Hauptrichtungen von Scan und CAD
     werden 24 mögliche Ausgangslagen gebaut (alle Achsvertauschungen
     und Umklappungen).
  2. Grob einrasten: Jede Startlage wird kurz aufs CAD gezogen; die drei
     besten kommen weiter.
  3. Fein nachziehen: Die Favoriten werden in mehreren Stufen (grob nach
     fein) exakt eingepasst. Bewertet wird der mittlere Abstand zum CAD.
  4. Spiegel-Kontrolle: Symmetrische Bauteile rasten gern "verkehrt
     herum" ein. Deshalb wird jede Lösung testweise um alle drei Achsen
     um 180° gedreht — passt die gedrehte Version besser, gewinnt sie.
  5. Nachsitzen im Konsens: Scans, die trotzdem wackelig sind, werden
     nochmal mit zusätzlichen Startlagen probiert. Bei mehreren fast
     gleich guten, aber stark verdrehten Lösungen entscheidet der
     Vergleich mit den bereits sicher ausgerichteten Scans ("Anker").
  6. Zusammenlegen & putzen: Alle guten Scans werden vereint, einmal
     gemeinsam final aufs CAD gezogen und von Ausreißern befreit. In den
     Schutzzonen (z. B. Anguss) gilt dabei eine größere Toleranz, damit
     genau die Abweichungen, die uns interessieren, NICHT weggefiltert
     werden.

Aufruf (alles optional, Standardwerte stehen in der KONFIGURATION):
    python align_to_cad.py --cad Bauteil.ply --scans pointclouds \
                           --zonen ../config/Zonen.json \
                           --ausgabe scan_registriert.ply

Ausgaben:
    scan_registriert.ply      Gesamtwolke im CAD-System, Einheit METER
                              (Stufe 3 mit scan_einheit = "m" fuettern)
    registrierung_info.json   was mit jedem Scan passiert ist
                              (Ausrichtung, Guete, verworfen ja/nein)

Voraussetzungen:
    pip install numpy scipy open3d
    pip install rich          # optional, nur fuer huebschere Ausgabe
"""

import argparse
import copy
import glob
import itertools
import json
import os
import re as _re
import sys
import time

import numpy as np

try:
    import open3d as o3d
except ImportError:
    print("Das Paket 'open3d' fehlt — es macht hier die ganze "
          "3D-Arbeit.\nInstallieren mit:  pip install open3d")
    sys.exit(1)
from scipy.spatial import cKDTree

# --- Konsolenausgabe: huebsch mit "rich", sonst einfaches print --------------
try:
    from rich import box as _box
    from rich.console import Console
    from rich.panel import Panel
    from rich.table import Table
    _con = Console()
    _RICH = True
except Exception:
    _con = None
    _RICH = False


def cprint(msg=""):
    if _RICH:
        _con.print(msg)
    else:
        print(_re.sub(r"\[/?[^\]]*\]", "", str(msg)))


def crule(title):
    if _RICH:
        _con.rule(f"[bold cyan]{title}", align="left")
    else:
        print(f"\n=== {title} ===")


def _scan_line(idx, name, fit, rmse, sc, status, lowfit, flipped, dt):
    sym = {"ok": "[green]✓ OK[/]", "warn": "[yellow]⚠ WARNUNG[/]",
           "reject": "[red]✗ VERWORFEN[/]"}[status]
    sc_col = {"ok": "green", "warn": "yellow", "reject": "red"}[status]
    note = ""
    if flipped:
        note = "  [magenta]GESPIEGELT (auf der anderen Seite passte es besser)[/]"
    elif lowfit:
        note = "  [yellow]WACKELIG (kommt in die Konsens-Runde)[/]"
    return (f"  [bold]{idx:02d}[/] [dim]{name:<26}[/] "
            f"fit [cyan]{fit:5.3f}[/]   rmse [cyan]{rmse*1000:5.2f}[/] mm   "
            f"score [{sc_col}]{sc*1000:6.3f}[/] mm   {sym}{note}   "
            f"[dim]{dt:.1f}s[/]")


# ===========================================================================
# EINSTELLUNGEN (per Kommandozeile ueberschreibbar)
# ===========================================================================
# Wichtig zu wissen: Die Scans aus capture.py sind in METERN, das CAD ist
# in MILLIMETERN. Deshalb wird das CAD unten mit CAD_SCALE auf Meter
# gebracht — intern rechnet alles in Metern.

CAD_PATH = "Bauteil.ply"                 # unser CAD-Modell (PLY/STL)
SCANS_DIR = "pointclouds"                # Ordner mit den Scans aus Stufe 1
ZONES_PATH = "../config/Zonen.json"      # Schutzzonen (Anguss usw.)
OUTPUT_PATH = "scan_registriert.ply"     # Ergebnis fuer Stufe 3

CAD_SCALE = 0.001        # CAD in mm -> mal 0.001 = Meter (1.0 = schon Meter)
CAD_REG_POINTS = 2_000_000   # so viele Punkte werden vom CAD-Netz abgetastet

# "Voxel" = Kantenlaenge der Wuerfelchen, auf die die Wolke fuer einen
# Rechenschritt zusammengefasst wird. Grosse Wuerfel = grob & schnell,
# kleine = fein & langsam. Die Einpassung laeuft diese Stufen von grob
# nach fein durch (Werte in Metern):
ICP_VOXELS = [0.010, 0.005, 0.002, 0.001]
ICP_MAX_ITER = 100

# Stufe 1 (Vorauswahl): alle 24 Startlagen kurz anpassen, bewerten,
# die TOP_K besten kommen in die Feinrunde.
STAGE1_MAX_ITER = 30
STAGE1_SCORE_VOXEL = 0.005
TOP_K = 3
STAGE2_SCORE_VOXEL = 0.001

# Bewertung ("score") = mittlerer Abstand der Scanpunkte zum CAD, wobei
# einzelne Ausreisser bei SCORE_CAP gedeckelt werden (sonst wuerde ein
# Grat die ganze Bewertung versauen). Ab SCAN_WARN_SCORE gibt es eine
# Warnung, ab SCAN_REJECT_SCORE fliegt der Scan raus. (Alles in Metern.)
SCORE_CAP = 0.005
SCAN_WARN_SCORE = 0.0015
SCAN_REJECT_SCORE = 0.0030

# --- Konsens-Runde fuer wackelige Scans --------------------------------------
ENABLE_CONSENSUS = True
FIT_OK_THRESHOLD = 0.85   # "fit" = Anteil der Scanpunkte, die eine passende
#                            CAD-Stelle gefunden haben (0..1). Darunter gilt
#                            ein Scan als wackelig.
CONSENSUS_VOXEL = 0.002
CONSENSUS_MAX_TRIES = 60      # so oft darf zusaetzlich "geraten" werden
CONSENSUS_TIE_MM = 0.5        # Loesungen gelten als "gleich gut", wenn ihre
CONSENSUS_ANGLE_DEG = 45.0    # Scores naeher als das liegen, aber die Lagen
#                                staerker als dieser Winkel auseinander sind
CONSENSUS_TAU = 0.0015        # Punkt "trifft" den Anker bis zu diesem Abstand
CONSENSUS_MIN_OV = 50         # mind. so viele Treffer, sonst zaehlt es nicht

# --- Zusammenlegen & Ausreisser-Filter ----------------------------------------
VOXEL_FINE = 0.00001      # Ausduennung beim Zusammenlegen (0.01 mm =
#                            praktisch aus; nur exakte Doppelpunkte weg)
ENABLE_NOISE_FILTER = True
NOISE_MAX_DIST_FROM_CAD = 0.0015   # Punkte weiter als 1.5 mm vom CAD fliegen

# In den Schutzzonen (dort ERWARTEN wir ja Abweichungen wie Angussreste!)
# gilt stattdessen die groessere Zonen-Toleranz:
ZONE_RADIUS = 0.003                # 3 mm Wirkradius um jeden Zonen-Punkt
ZONE_MAX_DIST_FROM_CAD = 0.0030    # in Zonen sind bis 3 mm Abstand erlaubt

# Beim finalen gemeinsamen Feinzug braucht die Wolke Flaechennormalen.
# (Frueher war dieser Radius an VOXEL_FINE gekoppelt — faktisch 0.02 mm,
# zu klein fuer stabile Normalen. 2 mm ist ein robuster Wert.)
FINAL_PULL_NORMAL_RADIUS = 0.002

# ===========================================================================
# Ab hier ist normalerweise KEINE Anpassung noetig.
# ===========================================================================


def rot_angle_deg(T1, T2):
    """Winkel zwischen zwei Lagen (nur die Drehung, in Grad)."""
    R = T1[:3, :3].T @ T2[:3, :3]
    c = (np.trace(R) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


# === CAD & ZONEN LADEN ======================================================

def load_cad_resources(path):
    """Laedt das CAD. Ein Flaechennetz (Mesh) wird zu einer dichten
    Punktwolke abgetastet und liefert zusaetzlich eine 'Szene', mit der
    sich Abstaende zur echten Oberflaeche exakt messen lassen."""
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"CAD-Datei '{path}' nicht gefunden — Pfad in den "
            "EINSTELLUNGEN oder per --cad angeben.")

    m = o3d.io.read_triangle_mesh(path)
    if len(m.triangles) > 0:
        cprint("  [dim]CAD-Typ: Flaechennetz (Mesh) erkannt.[/]")
        m.remove_duplicated_vertices()
        m.remove_duplicated_triangles()
        m.remove_degenerate_triangles()
        m.remove_unreferenced_vertices()
        if CAD_SCALE != 1.0:
            m.vertices = o3d.utility.Vector3dVector(
                np.asarray(m.vertices) * CAD_SCALE)
        m.compute_vertex_normals()
        pcd = m.sample_points_uniformly(CAD_REG_POINTS)
        scene = o3d.t.geometry.RaycastingScene()
        scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(m))
    else:
        cprint("  [dim]CAD-Typ: reine Punktwolke erkannt.[/]")
        pcd = o3d.io.read_point_cloud(path)
        if CAD_SCALE != 1.0:
            pcd.points = o3d.utility.Vector3dVector(
                np.asarray(pcd.points) * CAD_SCALE)
        scene = None
    return pcd, scene


def load_exclusion_zones(filepath):
    """Sammelt alle Zonen-Punkte aus der JSON in eine Punktwolke. Ueber
    den Abstand dorthin erkennt der Ausreisser-Filter spaeter, ob ein
    Punkt in einer Schutzzone liegt (dort gilt mehr Toleranz)."""
    if not os.path.exists(filepath):
        cprint(f"  [dim]Keine Zonen-Datei unter '{filepath}' — "
               "Zonen-Schutz bleibt aus.[/]")
        return None
    try:
        with open(filepath, "r") as f:
            data = json.load(f)
        if isinstance(data, dict):
            for key in data.keys():
                if isinstance(data[key], list):
                    data = data[key]
                    break
        if not isinstance(data, list):
            data = [data]

        all_points = []
        for item in data:
            if "points" in item and len(item["points"]) > 0:
                all_points.extend(item["points"])
        if not all_points:
            return None

        zone_pcd = o3d.geometry.PointCloud()
        zone_pcd.points = o3d.utility.Vector3dVector(
            np.array(all_points) * CAD_SCALE)     # mm -> m, wie das CAD
        cprint(f"  [cyan]{len(all_points)}[/] Zonen-Punkte geladen "
               f"(Wirkradius {ZONE_RADIUS*1000:.1f} mm, Zonen-Toleranz "
               f"{ZONE_MAX_DIST_FROM_CAD*1000:.1f} mm).")
        return zone_pcd
    except Exception as e:
        cprint(f"  [red]Zonen-Datei konnte nicht gelesen werden: {e}[/]")
        return None


def build_target_pyramid(pcd, voxels):
    """CAD-Wolke in allen Grob-fein-Stufen vorbereiten (mit Normalen)."""
    pyramid = []
    for v in voxels:
        d = pcd.voxel_down_sample(v)
        d.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=v * 2, max_nn=30))
        pyramid.append((d, v))
    return pyramid


def score_mesh(scan_down, scene, cad_pcd, T, cap=SCORE_CAP):
    """Bewertung einer Lage: mittlerer Abstand der Scanpunkte zum CAD,
    Ausreisser bei 'cap' gedeckelt. Kleiner = besser."""
    p = copy.deepcopy(scan_down).transform(T)
    if scene is not None:
        pts = np.asarray(p.points).astype(np.float32)
        d = scene.compute_distance(o3d.core.Tensor(pts)).numpy()
    else:
        d = np.asarray(p.compute_point_cloud_distance(cad_pcd))
    return float(np.mean(np.minimum(d, cap)))


def icp_step(src_down, tgt_pcd_normals, voxel, init, max_iter):
    """Ein Einpass-Schritt: schiebt/dreht den Scan Richtung CAD, bis der
    Abstand nicht mehr kleiner wird (klassisches ICP-Verfahren)."""
    res = o3d.pipelines.registration.registration_icp(
        src_down, tgt_pcd_normals, voxel * 2.0, init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=max_iter))
    return res.transformation, res.fitness, res.inlier_rmse


def icp_multiscale(src_pyramid, tgt_pyramid, init):
    """Einpassen von grob nach fein: erst mit den groben Wuerfelstufen die
    Lage finden, dann mit den feinen praezisieren."""
    T = init
    fit = rmse = 0.0
    for (src, _), (tgt, v) in zip(src_pyramid, tgt_pyramid):
        T, fit, rmse = icp_step(src, tgt, v, T, ICP_MAX_ITER)
    return T, fit, rmse


def principal_axes(pcd):
    """Schwerpunkt und Hauptrichtungen einer Wolke (laengste, mittlere,
    kuerzeste Ausdehnung)."""
    pts = np.asarray(pcd.points)
    c = pts.mean(axis=0)
    cov = np.cov((pts - c).T)
    w, V = np.linalg.eigh(cov)
    return c, V[:, np.argsort(w)[::-1]]


def pca_hypotheses(scan, cad_pcd):
    """Baut aus den Hauptrichtungen von Scan und CAD bis zu 24 moegliche
    Startlagen (alle Vertauschungen und Umklappungen der Achsen — nur
    echte Drehungen, keine Spiegelungen)."""
    cs, Vs = principal_axes(scan)
    ct, Vt = principal_axes(cad_pcd)
    hyps = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product([1, -1], repeat=3):
            Vp = (Vs[:, list(perm)]) * np.array(signs)
            R = Vt @ Vp.T
            if np.linalg.det(R) > 0.5:          # keine Spiegelung zulassen
                T = np.eye(4)
                T[:3, :3] = R
                T[:3, 3] = ct - R @ cs
                hyps.append(T)
    return hyps


# === AUSRICHTUNG EINES EINZELNEN SCANS ======================================

def align_scan_two_stage(scan, cad_pcd, cad_pyramid, scene):
    """Der Kern: 24 Startlagen -> kurz einrasten -> die 3 besten fein
    nachziehen -> Spiegel-Kontrolle. Liefert die beste Lage samt Guete."""
    cad_coarse, voxel_coarse = cad_pyramid[0]

    src_pyramid = [(scan.voxel_down_sample(v), v) for _, v in cad_pyramid]
    scan_coarse_score = scan.voxel_down_sample(STAGE1_SCORE_VOXEL)
    scan_fine_score = scan.voxel_down_sample(STAGE2_SCORE_VOXEL)

    inits = pca_hypotheses(scan, cad_pyramid[-1][0])

    # --- Stufe 1: alle Startlagen kurz anpassen, bewerten -------------------
    src_coarse, _ = src_pyramid[0]
    coarse_results = []
    for T0 in inits:
        T_c, fit, _ = icp_step(src_coarse, cad_coarse, voxel_coarse, T0,
                               STAGE1_MAX_ITER)
        sc = score_mesh(scan_coarse_score, scene, cad_pcd, T_c)
        coarse_results.append((T_c, fit, sc))
    coarse_results.sort(key=lambda x: x[2])
    top_k = coarse_results[:TOP_K]

    # --- Stufe 2: die Favoriten fein einpassen -------------------------------
    refined = []
    for T0, _, _ in top_k:
        T, fit, rmse = icp_multiscale(src_pyramid, cad_pyramid, T0)
        sc = score_mesh(scan_fine_score, scene, cad_pcd, T)
        refined.append((T, fit, rmse, sc))
    refined.sort(key=lambda x: x[3])
    best_T, best_fit, best_rmse, best_sc = refined[0]

    # --- Spiegel-Kontrolle: um jede Achse 180° drehen und nachpruefen --------
    # (Symmetrische Bauteile rasten gern verkehrt herum ein.)
    center = cad_pcd.get_center()
    flipped = False
    for axis in ([1, 0, 0], [0, 1, 0], [0, 0, 1]):
        R = o3d.geometry.get_rotation_matrix_from_axis_angle(
            np.array(axis) * np.pi)
        T_flip = np.eye(4)
        T_flip[:3, :3] = R
        T_flip[:3, 3] = center - R @ center
        T_alt, fit_alt, rmse_alt = icp_multiscale(
            src_pyramid, cad_pyramid, T_flip @ best_T)
        sc_alt = score_mesh(scan_fine_score, scene, cad_pcd, T_alt)
        if fit_alt > best_fit:
            best_T, best_fit, best_rmse, best_sc = (T_alt, fit_alt,
                                                    rmse_alt, sc_alt)
            flipped = True

    return best_T, best_fit, best_rmse, best_sc, flipped


# === KONSENS-RUNDE FUER WACKELIGE SCANS =====================================

def anchor_consistency(scan_small_pts, T, anchor_tree, tau):
    """Wie gut deckt sich der Scan in dieser Lage mit den bereits sicher
    ausgerichteten Scans (dem 'Anker')? Kleiner = besser."""
    q = (scan_small_pts @ T[:3, :3].T) + T[:3, 3]
    d, _ = anchor_tree.query(q, k=1)
    ov = d <= tau
    if int(ov.sum()) < CONSENSUS_MIN_OV:
        return float("inf")
    return float(np.mean(d[ov]))


def prep_fpfh(pcd, voxel):
    """Bereitet Form-Merkmale (FPFH) vor: eine Art Fingerabdruck der
    lokalen Oberflaechenform, ueber den sich zusammenpassende Stellen
    zwischen Scan und CAD finden lassen."""
    d = pcd.voxel_down_sample(voxel)
    d.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2, max_nn=30))
    f = o3d.pipelines.registration.compute_fpfh_feature(
        d, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 5,
                                                max_nn=100))
    return d, f


def ransac_global(src_d, src_f, tgt_d, tgt_f, voxel):
    """Grobe Lagesuche ueber Form-Merkmale: zufaellig Punktpaare mit
    aehnlichem Fingerabdruck verbinden und die Lage nehmen, die die
    meisten Paare gleichzeitig erklaert (RANSAC-Prinzip)."""
    return o3d.pipelines.registration.\
        registration_ransac_based_on_feature_matching(
            src_d, tgt_d, src_f, tgt_f, True, voxel * 1.5,
            o3d.pipelines.registration.
            TransformationEstimationPointToPoint(False), 3,
            [o3d.pipelines.registration.
             CorrespondenceCheckerBasedOnEdgeLength(0.9),
             o3d.pipelines.registration.
             CorrespondenceCheckerBasedOnDistance(voxel * 1.5)],
            o3d.pipelines.registration.
            RANSACConvergenceCriteria(200000, 0.999)).transformation


def consensus_align(scan, cad_pcd, cad_pyramid, scene, cad_down, cad_feat,
                    anchor_tree):
    """Zweite Chance fuer wackelige Scans: zusaetzliche Startlagen raten,
    bis eine gut sitzt. Gibt es mehrere fast gleich gute, aber stark
    verdrehte Loesungen, entscheidet die Deckung mit dem Anker."""
    src_pyramid = [(scan.voxel_down_sample(v), v) for _, v in cad_pyramid]
    scan_small = np.asarray(scan.voxel_down_sample(CONSENSUS_VOXEL).points)
    scan_fine_score = scan.voxel_down_sample(STAGE2_SCORE_VOXEL)
    scan_d, scan_f = prep_fpfh(scan, CONSENSUS_VOXEL)

    cands = []

    def consider(T0):
        T, fit, rmse = icp_multiscale(src_pyramid, cad_pyramid, T0)
        cands.append((T, fit, rmse,
                      score_mesh(scan_fine_score, scene, cad_pcd, T)))

    for T0 in pca_hypotheses(scan, cad_pyramid[-1][0]):
        consider(T0)

    def best_fit():
        return max(cands, key=lambda c: c[1])[1]

    tries = 0
    while best_fit() < FIT_OK_THRESHOLD and tries < CONSENSUS_MAX_TRIES:
        try:
            consider(ransac_global(scan_d, scan_f, cad_down, cad_feat,
                                   CONSENSUS_VOXEL))
        except Exception:
            pass                      # ein missglueckter Versuch ist egal
        tries += 1

    cands.sort(key=lambda c: c[3])
    best = cands[0]
    pool = [best]
    for c in cands[1:]:
        if c[3] - best[3] <= CONSENSUS_TIE_MM * 1e-3 and \
                rot_angle_deg(best[0], c[0]) > CONSENSUS_ANGLE_DEG:
            pool.append(c)
    if len(pool) > 1:
        best = min(pool, key=lambda c: anchor_consistency(
            scan_small, c[0], anchor_tree, CONSENSUS_TAU))
    return best[0], best[1], best[2], best[3]


# === AUSREISSER-FILTER MIT ZONEN-SCHUTZ =====================================

def noise_filter(merged, scene, default_max_dist, zone_pcd):
    """Entfernt Punkte, die zu weit vom CAD wegliegen (Messfehler, Reste
    vom Hintergrund). In den Schutzzonen gilt die groessere Toleranz,
    damit echte Angussreste & Co. NICHT mit weggefiltert werden."""
    if scene is None:
        return merged

    pts = np.asarray(merged.points).astype(np.float32)
    d_cad = scene.compute_distance(o3d.core.Tensor(pts)).numpy()

    # Grundregel: Punkte nahe am CAD bleiben.
    keep_mask = d_cad < default_max_dist

    valid_in_zone_mask = np.zeros(len(pts), dtype=bool)
    if zone_pcd is not None:
        # Abstand jedes Punkts zum naechsten Zonen-Punkt:
        d_zone = np.asarray(merged.compute_point_cloud_distance(zone_pcd))
        in_zone_mask = d_zone < ZONE_RADIUS
        # In der Zone gilt die groessere Zonen-Toleranz:
        valid_in_zone_mask = in_zone_mask & (d_cad < ZONE_MAX_DIST_FROM_CAD)
        keep_mask = keep_mask | valid_in_zone_mask

    keep = np.where(keep_mask)[0]
    pct = 100.0 * (len(pts) - len(keep)) / max(1, len(pts))
    out = merged.select_by_index(keep.tolist())
    cprint(f"    {len(pts):,} → [cyan]{len(out.points):,}[/] Punkte "
           f"(entfernt: {pct:.1f}%)")

    if zone_pcd is not None:
        saved_by_zone = int(np.sum(valid_in_zone_mask
                                   & ~(d_cad < default_max_dist)))
        if saved_by_zone > 0:
            cprint(f"    [dim]{saved_by_zone:,} Punkte (z. B. Anguss) durch "
                   "die Schutzzonen vor dem Filter gerettet.[/]")
    return out


# === HAUPTPROGRAMM ==========================================================

def main(argv=None):
    p = argparse.ArgumentParser(
        description="Richtet alle Einzelscans ans CAD aus und fuegt sie zu "
                    "einer Gesamtwolke im CAD-System zusammen.")
    p.add_argument("--cad", default=CAD_PATH, help="CAD-Datei (PLY/STL)")
    p.add_argument("--scans", default=SCANS_DIR,
                   help="Ordner mit den .ply-Scans aus capture.py")
    p.add_argument("--zonen", default=ZONES_PATH,
                   help="Zonen-JSON (Schutz vor dem Ausreisser-Filter)")
    p.add_argument("--ausgabe", default=OUTPUT_PATH,
                   help="Zieldatei der Gesamtwolke (Einheit: Meter)")
    args = p.parse_args(argv)

    t_total = time.time()
    if _RICH:
        _con.print(Panel.fit(
            "[bold]CAD-Ausrichtung[/]   ·   jeder Scan unabhaengig gegen "
            "das CAD", border_style="cyan", box=_box.ROUNDED))
    else:
        print("==== CAD-Ausrichtung ====")

    crule("1 · CAD & Zonen laden")
    t = time.time()
    cad_pcd, scene = load_cad_resources(args.cad)
    zone_pcd = load_exclusion_zones(args.zonen)
    cad_pyramid = build_target_pyramid(cad_pcd, ICP_VOXELS)
    cprint(f"  [cyan]{len(cad_pcd.points):,}[/] CAD-Punkte   ·   "
           f"[dim]{time.time()-t:.1f}s[/]")

    crule("2 · Scans laden")
    paths = sorted(glob.glob(os.path.join(args.scans, "*.ply")))
    if not paths:
        cprint(f"  [red]Keine .ply-Dateien in '{args.scans}' gefunden — "
               "erst mit capture.py scannen.[/]")
        return 1
    scans = [o3d.io.read_point_cloud(pth) for pth in paths]
    cprint(f"  [cyan]{len(scans)}[/] Scans   ·   [dim]{args.scans}[/]")

    crule(f"3 · Ausrichtung   (24 Startlagen → Top-{TOP_K} → fein)")
    poses, scores, fits, rmses, flips, rejected = [], [], [], [], [], []

    for i, scan in enumerate(scans):
        t = time.time()
        T, fit, rmse, sc, flipped = align_scan_two_stage(
            scan, cad_pcd, cad_pyramid, scene)
        dt = time.time() - t

        if sc > SCAN_REJECT_SCORE:
            status = "reject"
            rejected.append(i)
        elif sc > SCAN_WARN_SCORE:
            status = "warn"
        else:
            status = "ok"

        poses.append(T); scores.append(sc); fits.append(fit)
        rmses.append(rmse); flips.append(flipped)
        cprint(_scan_line(i + 1, os.path.basename(paths[i]), fit, rmse, sc,
                          status, fit < FIT_OK_THRESHOLD, flipped, dt))

    # --- Runde 2: Konsens fuer wackelige Scans -------------------------------
    if ENABLE_CONSENSUS:
        bad = [i for i in range(len(scans)) if fits[i] < FIT_OK_THRESHOLD]
        safe = [i for i in range(len(scans))
                if i not in bad and i not in rejected]

        if bad and safe:
            anchor = o3d.geometry.PointCloud()
            for i in safe:
                anchor += copy.deepcopy(scans[i]).transform(poses[i])
            anchor = anchor.voxel_down_sample(CONSENSUS_VOXEL)
            anchor_tree = cKDTree(np.asarray(anchor.points))
            cad_down, cad_feat = prep_fpfh(cad_pcd, CONSENSUS_VOXEL)

            crule("3b · Konsens (wackelige Scans klaeren)")
            cprint(f"  {len(bad)} Scan(s) brauchen Klaerung · Anker aus "
                   f"[cyan]{len(safe)}[/] sicheren Scans")

            for i in bad:
                t = time.time()
                T2, fit2, rmse2, sc2 = consensus_align(
                    scans[i], cad_pcd, cad_pyramid, scene, cad_down,
                    cad_feat, anchor_tree)
                if fit2 > fits[i]:
                    poses[i], fits[i], scores[i], rmses[i] = (T2, fit2,
                                                              sc2, rmse2)
                    if i in rejected and sc2 <= SCAN_REJECT_SCORE:
                        rejected.remove(i)
                    res = "[green]durch Anker geklaert[/]"
                else:
                    res = "[dim]verworfen (nicht besser)[/]"
                ok = ("[green]OK[/]" if fit2 >= FIT_OK_THRESHOLD
                      else "[yellow]weiter wackelig[/]")
                cprint(f"  [bold]{i+1:02d}[/] "
                       f"[dim]{os.path.basename(paths[i]):<26}[/] "
                       f"fit → [cyan]{fit2:.3f}[/] ({ok})  "
                       f"score [cyan]{sc2*1000:.3f}[/]mm  "
                       f"[dim]{time.time()-t:.1f}s[/]  {res}")

    # --- Zusammenlegen & finaler gemeinsamer Feinzug --------------------------
    crule("4 · Zusammenlegen & finaler Feinzug")
    merged = o3d.geometry.PointCloud()
    for i, T in enumerate(poses):
        if i in rejected:
            continue
        merged += copy.deepcopy(scans[i]).transform(T)
    merged = merged.voxel_down_sample(VOXEL_FINE)

    cprint("  Alle Scans gemeinsam noch einmal aufs CAD ziehen...")
    T_final = np.eye(4)
    if scene is not None:
        merged.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
            radius=FINAL_PULL_NORMAL_RADIUS, max_nn=30))
        final_res = o3d.pipelines.registration.registration_icp(
            merged, cad_pcd, VOXEL_FINE * 2.0, np.eye(4),
            o3d.pipelines.registration.
            TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=50))
        T_final = final_res.transformation
        merged.transform(T_final)
        shift = np.linalg.norm(T_final[:3, 3]) * 1000
        cprint(f"  [green]✓ Feinzug abgeschlossen[/] (Verschiebung: "
               f"[cyan]{shift:.3f}[/] mm)")

    final_score = score_mesh(merged.voxel_down_sample(STAGE2_SCORE_VOXEL),
                             scene, cad_pcd, np.eye(4))

    if ENABLE_NOISE_FILTER:
        cprint(f"  Ausreisser-Filter (normal > "
               f"[cyan]{NOISE_MAX_DIST_FROM_CAD*1000:.1f}[/] mm, in Zonen > "
               f"[cyan]{ZONE_MAX_DIST_FROM_CAD*1000:.1f}[/] mm):")
        merged = noise_filter(merged, scene, NOISE_MAX_DIST_FROM_CAD,
                              zone_pcd)

    o3d.io.write_point_cloud(args.ausgabe, merged)

    # Nachvollziehbarkeit: was ist mit jedem Scan passiert?
    info = {
        "cad": args.cad, "scans_ordner": args.scans,
        "ausgabe": args.ausgabe, "einheit": "m",
        "final_score_mm": round(final_score * 1000, 3),
        "final_pull": np.asarray(T_final).tolist(),
        "scans": [{
            "datei": os.path.basename(paths[i]),
            "verwendet": i not in rejected,
            "fit": round(float(fits[i]), 3),
            "rmse_mm": round(float(rmses[i]) * 1000, 3),
            "score_mm": round(float(scores[i]) * 1000, 3),
            "gespiegelt": bool(flips[i]),
            "transformation": np.asarray(poses[i]).tolist(),
        } for i in range(len(scans))],
    }
    info_pfad = os.path.splitext(args.ausgabe)[0] + "_info.json"
    with open(info_pfad, "w", encoding="utf-8") as f:
        json.dump(info, f, ensure_ascii=False, indent=2)

    # --- Zusammenfassung ---------------------------------------------------
    used = len(scans) - len(rejected)
    if _RICH:
        tbl = Table.grid(padding=(0, 3))
        tbl.add_column(style="dim")
        tbl.add_column(style="bold")
        tbl.add_row("Scans verwendet", f"{used} / {len(scans)}")
        tbl.add_row("Verworfen", f"{len(rejected)}" if rejected
                    else "[green]0[/]")
        tbl.add_row("Gesamtabstand zum CAD", f"{final_score*1000:.3f} mm")
        tbl.add_row("Punkte (final)", f"{len(merged.points):,}")
        tbl.add_row("Ausgabe", f"{args.ausgabe}  (+ {info_pfad})")
        tbl.add_row("Zeit gesamt", f"{time.time()-t_total:.1f} s")
        _con.print(Panel(tbl, title="[bold green]✓ Fertig[/]",
                         border_style="green", box=_box.ROUNDED))
    else:
        print("\n=== Fertig ===")
        print(f"  Scans verwendet : {used}/{len(scans)}")
        print(f"  Gesamtabstand   : {final_score*1000:.3f} mm")
        print(f"  Ausgabe         : {args.ausgabe}  (+ {info_pfad})")
    cprint("  [dim]Weiter mit Stufe 3:  python main.py --scan "
           f"{args.ausgabe} --scan-einheit m ...[/]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
