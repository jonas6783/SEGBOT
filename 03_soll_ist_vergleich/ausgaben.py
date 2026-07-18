# -*- coding: utf-8 -*-
"""
ausgaben.py — Alles, was den Rechner verlaesst
==============================================
Die eingefaerbte Punktwolke (gruen = i.O., Zonenfarbe = ausser Toleranz),
die Befund-Zusammenfassung samt fortlaufendem CSV-Log, der HTML-Report —
und der kleine lokale Webserver, der den Report automatisch im Browser
oeffnet (noetig, weil der Viewer Punktwolke und CAD nachlaedt; direkt
per Doppelklick blockt der Browser das aus Sicherheitsgruenden).
"""

import csv
import datetime
import http.server
import json
import os
import socketserver
import threading
import webbrowser

import numpy as np
import open3d as o3d

from analyse import color_points, hex_to_rgb
from konfig import (CORRECT_COLOR, MAX_DIST_FROM_CAD, MESH_OPACITY,
                    POINT_SIZE)
from report_template import HTML_TEMPLATE

def write_pointcloud(scan_pts, dist, assignment, zones, correct_rgb, path):
    colors = np.zeros((len(scan_pts), 3), np.float32)
    for zi, z in enumerate(zones):
        m = assignment == zi
        if m.any():
            colors[m] = color_points(dist[m], z["tolerance_m"], hex_to_rgb(z["color"]), correct_rgb)
    p = o3d.geometry.PointCloud()
    p.points = o3d.utility.Vector3dVector(scan_pts)
    p.colors = o3d.utility.Vector3dVector(colors)
    o3d.io.write_point_cloud(path, p)


def build_summary(part_id, results, quality, n_regions):
    failed = [r for r in results if r["pass"] is False]
    verdict = "i.O." if not failed else "N.i.O."
    if failed:
        worst = max(failed, key=lambda r: r["max_mm"] - r["tolerance_mm"])
        ws, mx, tol = worst["name"], round(worst["max_mm"], 3), worst["tolerance_mm"]
        over = round(worst["max_mm"] - worst["tolerance_mm"], 3)
    else:
        ws, mx, tol, over = "-", "", "", ""
    return {
        "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
        "part_id": part_id, "verdict": verdict, "zonen_nio": len(failed),
        "schlimmste_zone": ws, "max_mm": mx, "toleranz_mm": tol,
        "ueberschreitung_mm": over, "scan_qualitaet_mm": round(quality, 3),
        "schleif_regionen": n_regions,
    }


def append_spc_log(csv_path, summary):
    fields = ["timestamp", "part_id", "verdict", "zonen_nio", "schlimmste_zone",
              "max_mm", "toleranz_mm", "ueberschreitung_mm", "scan_qualitaet_mm",
              "schleif_regionen"]
    exists = os.path.exists(csv_path)
    with open(csv_path, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        if not exists:
            w.writeheader()
        w.writerow(summary)


def build_html(summary, results, regions, zone_idx_b64, dev_b64, cad_basename,
               cad_scale, n_total, n_dropped, quality):
    overall = summary["verdict"] == "i.O."
    vcls, vtxt = ("pass", "i.O.") if overall else ("fail", "N.i.O.")
    fail_zones = [r["name"] for r in results if r["pass"] is False]
    meta = f"{summary['part_id']} · {summary['timestamp']}"
    if fail_zones:
        meta += f" · Verletzt: {', '.join(fail_zones)}"
    zones_min = [{"name": r["name"], "color": r["color"], "tolerance_mm": r["tolerance_mm"],
                  "max_mm": r["max_mm"], "mean_mm": r["mean_mm"], "p95_mm": r["p95_mm"],
                  "pass": r["pass"], "n_points": r["n_points"], "worst_xyz": r["worst_xyz"]}
                 for r in results]
    repl = {
        "__VERDICT_CLASS__": vcls, "__VERDICT_TEXT__": vtxt, "__META_LINE__": meta,
        "__N_TOTAL__": f"{n_total:,}".replace(",", "."),
        "__N_DROPPED__": f"{n_dropped:,}".replace(",", "."),
        "__MAXDIST_MM__": f"{MAX_DIST_FROM_CAD*1000:.0f}",
        "__QUALITY_MM__": f"{quality:.3f}", "__N_REGIONS__": str(len(regions)),
        "__ZONES__": json.dumps(zones_min), "__REGIONS__": json.dumps(regions),
        "__ZONE_IDX__": json.dumps(zone_idx_b64), "__ZONE_DEV__": json.dumps(dev_b64),
        "__CAD_FILE__": cad_basename, "__CAD_SCALE__": str(cad_scale),
        "__CAD_FORMAT__": (os.path.splitext(cad_basename)[1].lstrip(".").lower() or "stl"),
        "__POINT_SIZE__": str(POINT_SIZE), "__MESH_OPACITY__": str(MESH_OPACITY),
        "__CORRECT_COLOR__": CORRECT_COLOR, "__PART_ID__": summary["part_id"],
    }
    html = HTML_TEMPLATE
    for k, v in repl.items():
        html = html.replace(k, v)
    return html


# === SERVER ===============================================================

def serve_and_open(directory, page="report.html", port_start=8765):
    abs_dir = os.path.abspath(directory); cwd = os.getcwd(); os.chdir(abs_dir)
    handler = http.server.SimpleHTTPRequestHandler
    httpd, port = None, port_start
    for _ in range(20):
        try:
            httpd = socketserver.TCPServer(("127.0.0.1", port), handler); break
        except OSError:
            port += 1
    if httpd is None:
        os.chdir(cwd); print(f"      Kein freier Port ab {port_start}."); return
    url = f"http://127.0.0.1:{port}/{page}"
    print(f"\n      Server: {url}\n      Strg+C zum Beenden")
    threading.Timer(0.6, lambda: webbrowser.open(url)).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n      Server beendet.")
    finally:
        httpd.server_close(); os.chdir(cwd)


