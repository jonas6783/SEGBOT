#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cad_compare.py — Soll-Ist-Vergleich (Stufe 3 der Pipeline)
==========================================================
Zonenweiser Soll-Ist-Vergleich plus Schleifpfad-Planung fuer den Cobot,
mit interaktivem Browser-Befund.

Der Ablauf (die Arbeit steckt in den Modulen daneben):
    konfig.py           alle Einstellungen an einer Stelle
    analyse.py          CAD/Zonen laden, Abstaende, Zuordnung, Bewertung
    pfadplanung.py      DBSCAN-Regionen, Mittellinie/Raster, Seiten-Split,
                        Reihenfolge-Optimierung (minimales Umorientieren)
    ausgaben.py         Punktwolke, CSV-Log, Report, lokaler Server
                        (oeffnet den Report automatisch im Browser)
    report_template.py  HTML-Vorlage inkl. Schleif-Animation: fester
                        Stift, bewegtes Bauteil — unsere invertierte
                        Kinematik, im Viewer vorweggenommen

Einfach starten mit:  python cad_compare.py
Die Pfade zu CAD (STL/PLY-Mesh!), Scan und Zonen stehen in konfig.py.

Farbschema: gruen = innerhalb der Toleranz, Zonenfarbe = Defekt.
Ausgaben in OUTPUT_DIR: report.html, pointcloud.ply, grind_path.json
(Wegpunkte + Normalen in CAD-mm, Reihenfolge bereits optimiert),
befund_log.csv und eine Kopie des CAD fuer den Viewer.
"""

import json
import os
import shutil
import sys

import numpy as np
import open3d as o3d

import analyse
import ausgaben
import konfig
import pfadplanung
from konfig import (CAD_PATH, CAD_SCALE, CORRECT_COLOR, GRIND_OPTIMIZE_ORDER,
                    GRIND_SIDE_ANGLE_DEG, MAX_DIST_FROM_CAD, OUTPUT_DIR,
                    PART_ID, SCAN_PLY_PATH, SCAN_VOXEL_M, SERVER_PORT,
                    STANDARD_ZONE, ZONES_JSON_PATH, ZONE_GROUPS,
                    ZONE_LATERAL_M)

def main():
    print("[1/6] Eingaben laden ...")
    for p in (CAD_PATH, SCAN_PLY_PATH, ZONES_JSON_PATH):
        if not os.path.exists(p):
            sys.exit(f"      Fehlt: {p}")
    cad_mesh, scene = analyse.load_cad(CAD_PATH, CAD_SCALE)
    scan = o3d.io.read_point_cloud(SCAN_PLY_PATH)
    if SCAN_VOXEL_M > 0:
        nb = len(scan.points)
        scan = scan.voxel_down_sample(SCAN_VOXEL_M)
        print(f"      Downsampling ({SCAN_VOXEL_M*1000:.2f}mm): "
              f"{nb:,} -> {len(scan.points):,}".replace(",", "."))
    scan_pts = np.asarray(scan.points)
    zones = analyse.load_zones(ZONES_JSON_PATH, CAD_SCALE, ZONE_GROUPS, STANDARD_ZONE)
    if not zones:
        sys.exit("      Keine Zonen geladen.")
    print(f"      Scan: {len(scan_pts):,} Punkte | Zonen: {len(zones)}".replace(",", "."))

    print("[2/6] Distanzen + Fusspunkte ...")
    distances_all = analyse.compute_signed_distance(scan_pts, scene)
    foot_all, foot_nrm_all = analyse.foot_points(scan_pts, scene)

    print(f"[3/6] Klassifikation (lateral {ZONE_LATERAL_M*1000:.1f}mm, "
          f"Filter {MAX_DIST_FROM_CAD*1000:.0f}mm universell) ...")
    assignment_all, keep = analyse.classify_points(foot_all, distances_all, zones,
                                           ZONE_LATERAL_M, MAX_DIST_FROM_CAD)
    n_dropped = int((~keep).sum())
    scan_pts, dist = scan_pts[keep], distances_all[keep]
    foot, foot_nrm, assignment = foot_all[keep], foot_nrm_all[keep], assignment_all[keep]
    print(f"      Verworfen (>{MAX_DIST_FROM_CAD*1000:.0f}mm): {n_dropped:,} "
          f"| verbleibend: {len(scan_pts):,}".replace(",", "."))

    print("[4/6] Auswertung pro Zone ...")
    results = analyse.analyze(scan_pts, dist, assignment, zones)
    for r in results:
        if r["pass"] is None:
            print(f"      {r['name']:12s}  -- keine Punkte --"); continue
        tag = "i.O." if r["pass"] else "N.i.O."
        print(f"      {r['name']:12s}  tol={r['tolerance_mm']:4.2f}  max={r['max_mm']:5.3f}  "
              f"p95={r['p95_mm']:5.3f}mm  ({r['max_signed_mm']:+5.3f})  "
              f"{r['n_points']:6d} pts  {tag}")

    print("[5/6] Schleifpfade ableiten ...")
    regions = pfadplanung.detect_grind_regions(scan_pts, dist, foot, foot_nrm, assignment, zones)
    reorient = None
    if GRIND_OPTIMIZE_ORDER and len(regions) >= 1:
        regions = pfadplanung.split_regions_by_side(regions, GRIND_SIDE_ANGLE_DEG)
        for k, r in enumerate(regions):
            r["id"] = k
        if len(regions) > 1:
            regions, reorient = pfadplanung.optimize_grind_order(regions)
            print(f"      Reihenfolge optimiert: Umorientierung "
                  f"{reorient[0]:.0f}deg -> {reorient[1]:.0f}deg "
                  f"(-{max(0.0, reorient[0]-reorient[1]):.0f}deg gespart)")
    for rg in regions:
        print(f"      Region {rg['id']}: {rg['zone']} ({rg['type']}) "
              f"{len(rg['waypoints'])} Wegpunkte, max Abtrag {rg['max_removal_mm']:.2f}mm")
    if not regions:
        print("      Keine Schleifregionen (kein Material ueber Toleranz).")

    print("[6/6] Ausgaben schreiben ...")
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    ausgaben.write_pointcloud(scan_pts, dist, assignment, zones, analyse.hex_to_rgb(CORRECT_COLOR),
                     os.path.join(OUTPUT_DIR, "pointcloud.ply"))
    cad_basename = os.path.basename(CAD_PATH)
    shutil.copy2(CAD_PATH, os.path.join(OUTPUT_DIR, cad_basename))

    quality = float(np.minimum(np.abs(dist), 0.005).mean() * 1000)
    part_id = PART_ID or os.path.splitext(os.path.basename(SCAN_PLY_PATH))[0]
    summary = ausgaben.build_summary(part_id, results, quality, len(regions))
    ausgaben.append_spc_log(os.path.join(OUTPUT_DIR, "befund_log.csv"), summary)

    grind = {"coordinate_frame": "CAD", "unit": "mm",
             "note": "Normale zeigt nach aussen; Werkzeug entlang -Normale anfahren. "
                     "Vor KUKA-Nutzung Transformation CAD->Roboterbasis davorhaengen.",
             "order_optimized": bool(GRIND_OPTIMIZE_ORDER and reorient is not None),
             "reorientation_deg": round(reorient[1], 1) if reorient else None,
             "part_id": part_id, "timestamp": summary["timestamp"], "regions": regions}
    with open(os.path.join(OUTPUT_DIR, "grind_path.json"), "w", encoding="utf-8") as f:
        json.dump(grind, f, indent=2, ensure_ascii=False)

    html = ausgaben.build_html(summary, results, regions,
                      analyse.enc_i32(assignment), analyse.enc_f32(dist * 1000.0),
                      cad_basename, CAD_SCALE, len(scan_pts), n_dropped, quality)
    with open(os.path.join(OUTPUT_DIR, "report.html"), "w", encoding="utf-8") as f:
        f.write(html)

    print(f"      Verdikt: {summary['verdict']} | Schleifregionen: {len(regions)} | "
          f"Ordner: {OUTPUT_DIR}/")
    if konfig.AUTO_OPEN_REPORT:
        ausgaben.serve_and_open(OUTPUT_DIR, "report.html", SERVER_PORT)
    else:
        print(f"\n      cd {OUTPUT_DIR} && python -m http.server 8000")



if __name__ == "__main__":
    main()
