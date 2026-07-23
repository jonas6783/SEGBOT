#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
capture.py — Bauteil einscannen (Stufe 1 der Pipeline)
======================================================

Was dieses Programm macht, in einem Satz: Es nimmt mit der RealSense-Kamera
ein Tiefenbild des Bauteils auf, schneidet per KI (YOLO findet das Bauteil und segmentiert es,
SAM zeichnet darauf im sub mm bereich die genaue Silhouette) alles Drumherum weg und speichert das
Ergebnis als 3D-Punktwolke.

Bedienung (es öffnet sich ein Kamerafenster):
    Live-Modus    c = Aufnahme machen        q = Beenden
    Prüf-Modus    p = speichern              s = Screenshot
                  r = zurück zum Livebild    q = Beenden

Nach jeder Aufnahme seht ihr erst eine Vorschau (lila = das, was als
Bauteil erkannt wurde). Erst wenn ihr mit "p" speichert, entstehen vier
zusammengehörige Dateien mit derselben Nummer:

    pointclouds/pointcloud_000.ply   die 3D-Punktwolke (in METERN!)
    rgb/rgb_000.png                  das Farbbild dazu
    depth/depth_000.npy              das maskierte Tiefenbild (Rohwerte)
    meta/meta_000.json               Kameradaten (Brennweite usw.)

Die PLY-Dateien sind die Eingabe für Stufe 2 (align_to_cad.py).

Tipps für gute Scans:
    - Das Bauteil muss während der Aufnahme absolut still liegen — es
      werden mehrere Bilder übereinandergelegt (Median), das entfernt
      das Tiefen-Flackern der Kamera fast vollständig.
    - Abstand Kamera-Bauteil zwischen DEPTH_MIN und DEPTH_MAX halten
      (unten einstellbar). Die D405 ist eine Nahbereichskamera.
    - Glänzende Stellen ggf. mattieren (Kreidespray), sonst gibt es Löcher.

Voraussetzungen (einmalig installieren):
    pip install pyrealsense2 opencv-python numpy open3d torch ultralytics
"""

import argparse
import json
import os
import sys
import time
import warnings

import numpy as np

# --- Bibliotheken mit verstaendlicher Fehlermeldung laden -------------------
# So stuerzt das Programm nicht kryptisch ab, wenn etwas fehlt, sondern sagt
# genau, was zu installieren ist.
_FEHLT = []
try:
    import cv2
except ImportError:
    _FEHLT.append(("opencv-python", "Bildverarbeitung/Anzeigefenster"))
try:
    import open3d as o3d
except ImportError:
    _FEHLT.append(("open3d", "Punktwolken speichern"))
try:
    import pyrealsense2 as rs
except ImportError:
    _FEHLT.append(("pyrealsense2", "Ansteuerung der RealSense-Kamera"))
try:
    import torch
except ImportError:
    _FEHLT.append(("torch", "laesst die KI-Modelle laufen"))
try:
    from ultralytics import SAM, YOLO
except ImportError:
    _FEHLT.append(("ultralytics", "YOLO- und SAM-Modelle"))


# ===========================================================================
# EINSTELLUNGEN
# ===========================================================================

# --- KI-Modelle -------------------------------------------------------------
# best.pt ist unser trainiertes YOLO-Modell (erkennt das Gussbauteil).
YOLO_MODEL_PATH = "best.pt"
# SAM zeichnet die exakte Umrisslinie. Die Gewichtsdatei ist gross und
# liegt nicht im Repo — Ultralytics laedt sie beim ersten Start selbst
# herunter, oder ihr legt sie manuell daneben.
SAM_MODEL_PATH = "sam2.1_l.pt"

# Wie sicher muss YOLO sein, damit eine Erkennung zaehlt? (0..1)
CONF_THRESHOLD = 0.9
# Erkannte Silhouette um so viele Pixel nach aussen vergroessern (0 = aus): kann man erhöhen aber bisher nicht nötig gewesen
EXPAND_PIXELS = 0

# --- Aufnahmebereich ----------------------------------------------------------
# Alles ausserhalb dieses Abstandsfensters wird verworfen (Tisch,
# Hintergrund, Haende). An euren Aufbau anpassen!
DEPTH_MIN_METERS = 0.05   # naeher als 5 cm wird ignoriert
DEPTH_MAX_METERS = 0.18   # weiter als 18 cm wird ignoriert

# --- Kameraeinstellung (D405) --------------------------------------------------
# Voreinstellungen des Tiefensensors:
#   1 = Standard
#   3 = Hohe Genauigkeit (filtert Reflexe streng, dafuer mehr Loecher)
#   4 = Hohe Dichte (schliesst Loecher — unser Standard fuer Gussteile)
#   5 = Mittlere Dichte (Kompromiss)
VISUAL_PRESET_INDEX = 4

# Pro Aufnahme werden so viele Bilder gesammelt und pro Pixel der Median
# gebildet. Mehr Bilder = ruhigere Tiefe, aber laengere Aufnahme.
N_MEDIAN_FRAMES = 30

# --- Optionale Filter (mit dem Median meist unnoetig, zum Testen einschaltbar)
USE_HOLE_FILLING = False      # Kamera fuellt kleine Tiefenloecher pro Bild
USE_OUTLIER_REMOVAL = False   # entfernt einzelne "fliegende" Punkte
SOR_NB_NEIGHBORS = 20
SOR_STD_RATIO = 2.0

# --- Zuschnitt & Ausgabe --------------------------------------------------------
# Farbbild/Tiefe/Kameradaten auf die YOLO-Box zuschneiden (spart Platz,
# die Kameradaten werden passend mitverschoben):
CROP_TO_BOX = True
CROP_PAD_PIXELS = 2           # kleiner Rand um die Box (0 = exakt die Box)

OUTPUT_DIR = "pointclouds"    # Punktwolken (.ply) — Eingabe fuer Stufe 2
RGB_DIR = "rgb"               # Farbbilder
DEPTH_DIR = "depth"           # maskierte Tiefenbilder (Rohwerte, .npy)
META_DIR = "meta"             # Kameradaten (JSON)

# ===========================================================================
# Ab hier ist normalerweise KEINE Anpassung noetig.
# ===========================================================================


def pruefe_voraussetzungen():
    """Fehlende Pakete und Modell-Dateien verstaendlich melden."""
    if _FEHLT:
        print("Es fehlen Python-Pakete:")
        for paket, zweck in _FEHLT:
            print(f"  - {paket:<15s} ({zweck})")
        print("\nInstallieren mit:")
        print("  pip install " + " ".join(p for p, _ in _FEHLT))
        sys.exit(1)
    if not os.path.isfile(YOLO_MODEL_PATH):
        print(f"Das YOLO-Modell '{YOLO_MODEL_PATH}' wurde nicht gefunden.")
        print("Die Datei muss neben dem Skript liegen (oder Pfad oben in")
        print("den EINSTELLUNGEN anpassen).")
        sys.exit(1)


def naechster_freier_index(ordner, muster="pointcloud_"):
    """Beim Neustart nicht bei 000 anfangen und alte Scans ueberschreiben,
    sondern hinter der hoechsten vorhandenen Nummer weitermachen."""
    hoechste = -1
    for name in os.listdir(ordner):
        if name.startswith(muster) and name.endswith(".ply"):
            try:
                hoechste = max(hoechste, int(name[len(muster):-4]))
            except ValueError:
                pass
    return hoechste + 1


def expand_mask(mask, pixels):
    """Vergroessert die erkannte Silhouette um ein paar Pixel nach aussen."""
    if pixels <= 0:
        return mask
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                  (2 * pixels + 1, 2 * pixels + 1))
    return cv2.dilate(mask.astype(np.uint8), k, iterations=1).astype(bool)


def capture_median_depth(pipeline, align, hole_filter):
    """
    Sammelt N_MEDIAN_FRAMES Bilder und bildet pro Pixel den Median ueber
    die gueltigen Tiefenwerte. Das entfernt das zeitliche Rauschen der
    Kamera und einzelne Ausreisser-Pixel fast vollstaendig.
    Rueckgabe: (median_tiefe_uint16, farbbild, kameradaten) oder 3x None.
    """
    stack, last_color, last_intr = [], None, None
    grabbed, attempts = 0, 0
    max_attempts = N_MEDIAN_FRAMES * 4       # Sicherheitslimit

    while grabbed < N_MEDIAN_FRAMES and attempts < max_attempts:
        attempts += 1
        try:
            frames = pipeline.wait_for_frames(2000)     # 2 s Timeout
            aligned = align.process(frames)
            c = aligned.get_color_frame()
            d = aligned.get_depth_frame()
        except RuntimeError:
            continue
        if not c or not d:
            continue
        if USE_HOLE_FILLING:
            try:
                d = hole_filter.process(d)
            except RuntimeError:
                pass
        stack.append(np.asanyarray(d.get_data()))
        last_color = np.asanyarray(c.get_data())
        last_intr = c.profile.as_video_stream_profile().intrinsics
        grabbed += 1

    if grabbed == 0:
        return None, None, None

    arr = np.stack(stack).astype(np.float32)
    arr[arr == 0] = np.nan                   # 0 = "keine Messung" -> ignorieren
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", category=RuntimeWarning)
        med = np.nanmedian(arr, axis=0)
    med[np.isnan(med)] = 0
    return med.astype(np.uint16), last_color, last_intr


def mask_to_pointcloud(mask, depth_image, color_image, intrinsics,
                       depth_scale):
    """
    Rechnet die maskierten Tiefenpixel in echte 3D-Punkte um (Lochkamera-
    Modell: aus Pixelposition + Tiefe + Brennweite wird ein Punkt in
    Metern). Punkte ausserhalb des Abstandsfensters fliegen raus.
    """
    ys, xs = np.where(mask)
    if len(xs) == 0:
        return None

    z = depth_image[ys, xs].astype(np.float32) * depth_scale
    valid = (z > DEPTH_MIN_METERS) & (z < DEPTH_MAX_METERS)
    xs, ys, z = xs[valid], ys[valid], z[valid]
    if len(xs) == 0:
        return None

    fx, fy = intrinsics.fx, intrinsics.fy
    cx, cy = intrinsics.ppx, intrinsics.ppy
    x = (xs - cx) * z / fx
    y = (ys - cy) * z / fy
    points = np.stack([x, y, z], axis=-1)
    colors = color_image[ys, xs][:, ::-1].astype(np.float32) / 255.0  # BGR->RGB

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    if USE_OUTLIER_REMOVAL and len(pcd.points) > 0:
        pcd, _ = pcd.remove_statistical_outlier(
            nb_neighbors=SOR_NB_NEIGHBORS, std_ratio=SOR_STD_RATIO)
    return pcd


def run_inference(color_img, yolo_model, sam_model, device):
    """YOLO findet das Bauteil (Kaestchen), SAM zeichnet darin die exakte
    Silhouette. Rueckgabe: (maske, boxen, klassennamen, sicherheiten)."""
    yolo_res = yolo_model.predict(color_img, conf=CONF_THRESHOLD,
                                  verbose=False, device=device)
    if len(yolo_res[0].boxes) == 0:
        return None, None, None, None

    boxes = yolo_res[0].boxes.xyxy.cpu().numpy()
    confs = yolo_res[0].boxes.conf.cpu().numpy()
    cls_ids = yolo_res[0].boxes.cls.cpu().numpy().astype(int)
    names_map = yolo_res[0].names
    cls_names = [names_map[i] for i in cls_ids]

    sam_res = sam_model.predict(color_img, bboxes=boxes,
                                verbose=False, device=device)

    combined = np.zeros(color_img.shape[:2], dtype=bool)
    for r in sam_res:
        if r.masks is None:
            continue
        for mask in r.masks.data:
            m = mask.cpu().numpy().astype(bool)
            if m.shape != color_img.shape[:2]:
                m = cv2.resize(m.astype(np.uint8),
                               (color_img.shape[1], color_img.shape[0]),
                               interpolation=cv2.INTER_NEAREST).astype(bool)
            combined |= expand_mask(m, EXPAND_PIXELS)
    return combined, boxes, cls_names, confs


def render_overlay(color_img, mask, boxes, cls_names=None, confs=None):
    """Zeichnet Maske (lila) und Erkennungs-Kaestchen (gruen) ins Bild."""
    out = color_img.copy()
    if mask is not None and mask.any():
        overlay = np.zeros_like(out, dtype=np.uint8)
        overlay[mask] = [255, 0, 150]
        out = cv2.addWeighted(out, 1.0, overlay, 0.5, 0)
    if boxes is not None:
        for i, b in enumerate(boxes):
            x1, y1, x2, y2 = int(b[0]), int(b[1]), int(b[2]), int(b[3])
            cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
            if cls_names is not None and confs is not None:
                label = f"{cls_names[i]} {confs[i]:.2f}"
                (tw, th), _ = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                y_label = max(y1, th + 6)
                cv2.rectangle(out, (x1, y_label - th - 6),
                              (x1 + tw + 6, y_label), (0, 255, 0), -1)
                cv2.putText(out, label, (x1 + 3, y_label - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
    return out


def union_box(boxes, img_shape, pad=0):
    """Ein Kaestchen, das alle Erkennungs-Kaestchen umschliesst
    (an den Bildrand geklemmt)."""
    h, w = img_shape[:2]
    x1 = max(0, int(np.floor(boxes[:, 0].min())) - pad)
    y1 = max(0, int(np.floor(boxes[:, 1].min())) - pad)
    x2 = min(w, int(np.ceil(boxes[:, 2].max())) + pad)
    y2 = min(h, int(np.ceil(boxes[:, 3].max())) + pad)
    return x1, y1, x2, y2


def save_capture(idx, mask, depth, color, intrinsics, depth_scale, pcd,
                 boxes):
    """Speichert die vier zusammengehoerigen Dateien unter derselben Nummer.
    Beim Zuschnitt wird der Bildmittelpunkt in den Kameradaten passend
    mitverschoben, damit spaetere Rueckrechnungen stimmen."""
    ply_path = os.path.join(OUTPUT_DIR, f"pointcloud_{idx:03d}.ply")
    o3d.io.write_point_cloud(ply_path, pcd)

    masked_depth = np.where(mask, depth, 0).astype(np.uint16)

    if CROP_TO_BOX and boxes is not None and len(boxes) > 0:
        x1, y1, x2, y2 = union_box(boxes, color.shape, CROP_PAD_PIXELS)
    else:
        x1, y1, x2, y2 = 0, 0, color.shape[1], color.shape[0]

    cv2.imwrite(os.path.join(RGB_DIR, f"rgb_{idx:03d}.png"),
                color[y1:y2, x1:x2])
    np.save(os.path.join(DEPTH_DIR, f"depth_{idx:03d}.npy"),
            masked_depth[y1:y2, x1:x2])

    meta = {
        "fx": float(intrinsics.fx), "fy": float(intrinsics.fy),
        "ppx": float(intrinsics.ppx) - x1, "ppy": float(intrinsics.ppy) - y1,
        "width": int(x2 - x1), "height": int(y2 - y1),
        "crop_offset": [int(x1), int(y1)],
        "orig_width": int(intrinsics.width),
        "orig_height": int(intrinsics.height),
        "depth_scale": float(depth_scale),
        "depth_min_m": DEPTH_MIN_METERS, "depth_max_m": DEPTH_MAX_METERS,
        "n_median_frames": N_MEDIAN_FRAMES,
    }
    with open(os.path.join(META_DIR, f"meta_{idx:03d}.json"), "w") as f:
        json.dump(meta, f, indent=2)
    return ply_path


# ===========================================================================
# HAUPTPROGRAMM
# ===========================================================================

def main():
    argparse.ArgumentParser(description=__doc__.splitlines()[1]).parse_args()
    pruefe_voraussetzungen()
    for d in (OUTPUT_DIR, RGB_DIR, DEPTH_DIR, META_DIR):
        os.makedirs(d, exist_ok=True)

    # --- KI-Modelle laden ---------------------------------------------------
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Rechne auf: {device}"
          + ("" if device == "cuda" else "  (ohne Grafikkarte ist die "
             "Erkennung langsamer, funktioniert aber)"))
    yolo_model = YOLO(YOLO_MODEL_PATH).to(device)
    sam_model = SAM(SAM_MODEL_PATH).to(device)
    print("KI-Modelle geladen.")

    # --- Kamera starten -------------------------------------------------------
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 15)
    try:
        profile = pipeline.start(config)
    except RuntimeError as exc:
        print("Kamera konnte nicht gestartet werden:", exc)
        print("Ist die RealSense per USB-3-Kabel angeschlossen (blauer "
              "Stecker) und wird sie im RealSense-Viewer angezeigt?")
        sys.exit(1)

    depth_sensor = profile.get_device().first_depth_sensor()
    if depth_sensor.supports(rs.option.visual_preset):
        depth_sensor.set_option(rs.option.visual_preset, VISUAL_PRESET_INDEX)
        print(f"Kamera-Voreinstellung {VISUAL_PRESET_INDEX} gesetzt.")
    else:
        print("Hinweis: Dieser Sensor unterstuetzt keine Voreinstellungen.")

    depth_scale = depth_sensor.get_depth_scale()
    align = rs.align(rs.stream.color)      # Tiefe pixelgenau aufs Farbbild
    hole_filter = rs.hole_filling_filter()

    print("Live:  c = Aufnahme   q = Beenden")
    print("Pruefen: p = speichern   s = Screenshot   r = zurueck   q = Beenden")

    mode = "live"
    fps_counter, fps, last_time = 0, 0, time.time()
    screenshot_count = 0
    pcd_count = naechster_freier_index(OUTPUT_DIR)
    if pcd_count > 0:
        print(f"Es liegen schon Scans im Ordner — mache bei Nummer "
              f"{pcd_count:03d} weiter (nichts wird ueberschrieben).")

    snap_color = snap_depth = snap_intrinsics = None
    snap_mask = snap_boxes = snap_cls_names = snap_confs = None
    snap_overlay, snap_inference_ms = None, 0.0

    try:
        while True:
            if mode == "live":
                frames = pipeline.wait_for_frames()
                aligned = align.process(frames)
                color = aligned.get_color_frame()
                depth = aligned.get_depth_frame()
                if not color or not depth:
                    continue
                img = np.asanyarray(color.get_data())

                # Nur YOLO im Livebild (schnell) — SAM erst bei der Aufnahme.
                live_res = yolo_model.predict(img, conf=CONF_THRESHOLD,
                                              verbose=False, device=device)
                live_boxes = live_cls_names = live_confs = None
                n_live = 0
                if len(live_res[0].boxes) > 0:
                    live_boxes = live_res[0].boxes.xyxy.cpu().numpy()
                    live_confs = live_res[0].boxes.conf.cpu().numpy()
                    ids = live_res[0].boxes.cls.cpu().numpy().astype(int)
                    live_cls_names = [live_res[0].names[i] for i in ids]
                    n_live = len(live_boxes)

                display_img = render_overlay(img, None, live_boxes,
                                             live_cls_names, live_confs)
                fps_counter += 1
                if time.time() - last_time >= 1.0:
                    fps, fps_counter, last_time = fps_counter, 0, time.time()
                cv2.putText(display_img,
                            f"LIVE | FPS: {fps} | erkannt: {n_live} | "
                            f"c = Aufnahme",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (255, 255, 255), 2)
                cv2.imshow("Bauteil-Scanner (YOLO + SAM)", display_img)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('c'):
                    print(f"Aufnahme — sammle {N_MEDIAN_FRAMES} Bilder "
                          "(Bauteil bitte still halten)...")
                    snap_depth, snap_color, snap_intrinsics = \
                        capture_median_depth(pipeline, align, hole_filter)
                    if snap_depth is None:
                        print("Aufnahme fehlgeschlagen — keine Bilder "
                              "erhalten. Bitte erneut versuchen.")
                        continue
                    print("Erkennung laeuft...")
                    t0 = time.time()
                    snap_mask, snap_boxes, snap_cls_names, snap_confs = \
                        run_inference(snap_color, yolo_model, sam_model,
                                      device)
                    snap_inference_ms = (time.time() - t0) * 1000
                    snap_overlay = render_overlay(
                        snap_color, snap_mask, snap_boxes,
                        snap_cls_names, snap_confs)
                    ok = snap_mask is not None and snap_mask.any()
                    print(f"Erkennung fertig in {snap_inference_ms:.0f} ms — "
                          f"Bauteil {'gefunden' if ok else 'NICHT gefunden'}.")
                    mode = "review"

            else:  # Pruef-Modus: Aufnahme ansehen, dann speichern/verwerfen
                view = snap_overlay.copy()
                ok = snap_mask is not None and snap_mask.any()
                cv2.putText(view,
                            f"PRUEFEN | {snap_inference_ms:.0f} ms | "
                            f"Maske: {'ja' if ok else 'NEIN'} | "
                            f"p=speichern  s=Screenshot  r=zurueck",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 255, 255), 2)
                cv2.imshow("Bauteil-Scanner (YOLO + SAM)", view)

                key = cv2.waitKey(30) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('r'):
                    mode, last_time, fps_counter = "live", time.time(), 0
                elif key == ord('s'):
                    name = f"screenshot_{screenshot_count}.png"
                    cv2.imwrite(name, snap_overlay)
                    print(f"Screenshot gespeichert: {name}")
                    screenshot_count += 1
                elif key == ord('p'):
                    if snap_mask is None or not snap_mask.any():
                        print("Kein Bauteil in der Maske — nichts zu "
                              "speichern.")
                        continue
                    pcd = mask_to_pointcloud(snap_mask, snap_depth,
                                             snap_color, snap_intrinsics,
                                             depth_scale)
                    if pcd is None or len(pcd.points) == 0:
                        print("Punktwolke ist leer — Abstand pruefen "
                              f"({DEPTH_MIN_METERS*100:.0f} bis "
                              f"{DEPTH_MAX_METERS*100:.0f} cm).")
                        continue
                    ply_path = save_capture(
                        pcd_count, snap_mask, snap_depth, snap_color,
                        snap_intrinsics, depth_scale, pcd, snap_boxes)
                    print(f"Gespeichert: {ply_path} "
                          f"({len(pcd.points)} Punkte) + rgb/depth/meta")
                    pcd_count += 1
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
