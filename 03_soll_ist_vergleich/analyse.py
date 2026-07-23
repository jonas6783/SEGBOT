# -*- coding: utf-8 -*-
"""
analyse.py — Laden, Abstaende, Zonen-Zuordnung und Bewertung
============================================================
Kernstueck des Soll-Ist-Vergleichs: CAD-Mesh und Zonen laden, fuer jeden
Scanpunkt den vorzeichenbehafteten Abstand zur CAD-Oberflaeche messen
(+ = Material zu viel, - = fehlt) samt Fusspunkt und nach aussen
zeigender Flaechennormale, jeden Punkt lateral seiner Zone zuordnen
(der Auffangbereich "Standard" bekommt den Rest, Ausreisser jenseits des
universellen Filters fliegen ueberall raus) und je Zone bewerten:
bestanden, wenn die groesste Abweichung innerhalb der Toleranz bleibt.
"""

import base64
import json

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

def hex_to_rgb(h):
    h = h.lstrip("#")
    return np.array([int(h[i:i+2], 16) / 255.0 for i in (0, 2, 4)], dtype=np.float32)


def enc_f32(a): return base64.b64encode(np.asarray(a, np.float32).tobytes()).decode("ascii")
def enc_i32(a): return base64.b64encode(np.asarray(a, np.int32).tobytes()).decode("ascii")


def load_cad(path, scale):
    m = o3d.io.read_triangle_mesh(path)            # liest STL und PLY-Mesh (per Endung)
    if len(m.triangles) == 0:
        raise ValueError(
            f"'{path}' enthaelt keine Dreiecke. Als CAD-Referenz wird ein Flaechennetz "
            f"gebraucht (STL oder PLY-Mesh mit Faces). Ein reiner Punktwolken-PLY hat "
            f"keine Oberflaeche zum Messen und funktioniert hier nicht.")
    if scale != 1.0:
        m.vertices = o3d.utility.Vector3dVector(np.asarray(m.vertices) * scale)
    m.compute_vertex_normals()
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(m))
    return m, scene


def load_zones(path, scale, groups, standard):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    raw = {z["name"]: z for z in data["zones"]}
    out = []
    for g in groups:
        pts_list, found, missing = [], [], []
        for src in g["sources"]:
            z = raw.get(src)
            if z is None:
                missing.append(src); continue
            pts_list.append(np.asarray(z["points"], dtype=np.float64) * scale)
            found.append(src)
        if not pts_list:
            print(f"      WARNUNG: '{g['name']}' ohne Quellzone (fehlt: {missing}).")
            continue
        out.append({"name": g["name"], "tolerance_m": g["tolerance_mm"] * 1e-3,
                    "tolerance_mm": float(g["tolerance_mm"]), "color": g["color"],
                    "points": np.vstack(pts_list), "n_sources": len(found),
                    "source_names": found, "is_standard": False})
    if standard:
        out.append({"name": standard["name"], "tolerance_m": standard["tolerance_mm"] * 1e-3,
                    "tolerance_mm": float(standard["tolerance_mm"]), "color": standard["color"],
                    "points": np.empty((0, 3)), "n_sources": 0,
                    "source_names": ["(Auffangbereich)"], "is_standard": True})
    return out


def compute_signed_distance(pts, scene):
    return scene.compute_signed_distance(o3d.core.Tensor(pts.astype(np.float32))).numpy()


def foot_points(scan_pts, scene):
    """Fusspunkt + nach aussen orientierte Flaechennormale je Scan-Punkt."""
    res = scene.compute_closest_points(o3d.core.Tensor(scan_pts.astype(np.float32)))
    fp = res["points"].numpy()
    nrm = res["primitive_normals"].numpy()
    v = scan_pts - fp
    flip = np.sum(nrm * v, axis=1) < 0          # Normale soll Richtung Material zeigen
    nrm[flip] = -nrm[flip]
    return fp, nrm


def classify_points(foot, distances, zones, lateral, max_dist):
    """Universeller Filter + laterale Zonenzuordnung.

    Regeln:
      - |Distanz| > max_dist            -> verworfen (UEBERALL, auch in Zonen).
      - sonst lateral ueber einer Zone  -> diese Zone (Fusspunkt <= lateral).
      - sonst                            -> Standardzone.

    Die Hoehe der Zone nach aussen ist also durch max_dist begrenzt; lateral
    haben definierte Zonen Vorrang vor Standard. Rueckgabe (assignment, keep).
    """
    all_pts, all_zid = [], []
    for i, z in enumerate(zones):
        if len(z["points"]) == 0:
            continue
        all_pts.append(z["points"]); all_zid.append(np.full(len(z["points"]), i, np.int32))
    all_pts = np.vstack(all_pts); all_zid = np.concatenate(all_zid)
    tree = cKDTree(all_pts)

    nn_d, nn_i = tree.query(foot, k=1, distance_upper_bound=lateral)
    in_zone = np.isfinite(nn_d)
    zone_of = all_zid[np.where(in_zone, nn_i, 0)]

    N = len(foot)
    keep = np.abs(distances) <= max_dist
    out = np.full(N, -1, np.int32)
    out[keep & in_zone] = zone_of[keep & in_zone]

    std_idx = next((i for i, z in enumerate(zones) if z["is_standard"]), None)
    if std_idx is not None:
        out[keep & (~in_zone)] = std_idx
    return out, keep


def analyze(scan_pts, distances, assignment, zones):
    results = []
    for i, z in enumerate(zones):
        mask = assignment == i
        n = int(mask.sum())
        if n == 0:
            results.append({"name": z["name"], "tolerance_mm": z["tolerance_mm"],
                            "color": z["color"], "n_points": 0, "pass": None,
                            "max_mm": None, "mean_mm": None, "p95_mm": None,
                            "max_signed_mm": None, "worst_xyz": None,
                            "is_standard": z["is_standard"]})
            continue
        d_mm = distances[mask] * 1000.0; d_abs = np.abs(d_mm)
        wi = int(np.argmax(d_abs)); wp = scan_pts[mask][wi]
        results.append({"name": z["name"], "tolerance_mm": z["tolerance_mm"], "color": z["color"],
                        "n_points": n, "max_mm": float(d_abs.max()),
                        "mean_mm": float(d_abs.mean()), "p95_mm": float(np.percentile(d_abs, 95)),
                        "max_signed_mm": float(d_mm[wi]),
                        "worst_xyz": [float(wp[0]), float(wp[1]), float(wp[2])],
                        "pass": bool(d_abs.max() <= z["tolerance_mm"]),
                        "is_standard": z["is_standard"]})
    return results


def color_points(d_m, tol_m, zone_rgb, correct_rgb):
    """Innerhalb Toleranz = gruen (correct_rgb); ausserhalb = Zonenfarbe."""
    rgb = np.tile(correct_rgb, (len(d_m), 1)).astype(np.float32)
    bad = np.abs(d_m) > tol_m
    if bad.any():
        rgb[bad] = zone_rgb
    return rgb


