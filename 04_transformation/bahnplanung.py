#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bahnplanung.py — Roboterbahn erzeugen (Stufe 4 der Pipeline)
============================================================

Uebersetzt die Schleif-/Markierpfade aus Stufe 3 (grind_path.json) in
eine Bewegungsfolge fuer den Roboter. Grundgedanke "invertierte
Kinematik": Der Roboter haelt das BAUTEIL und fuehrt es am ortsfesten
Stift vorbei — gerechnet wird also, wie der GREIFERFLANSCH stehen muss,
damit der jeweilige Bahnpunkt genau an der Stiftspitze liegt und die
Flaeche senkrecht zum Stift steht. Jeder Bahnpunkt bringt seine eigene
Flaechennormale mit; als Ausweichpunkt markierte Wegpunkte (RRT) werden
im Anfahrabstand ueberflogen statt beruehrt.

Eingaben:
    grind_path.json            aus Stufe 3 (Regionen mit Wegpunkten in
                               mm, Normalen und is_evasion-Kennzeichen).
                               Standard: ../03_soll_ist_vergleich/befund/
                               — fehlt der Ordner, greift automatisch
                               die beiliegende beispiel_grind_path.json.
    kalibrierung_ergebnis.json aus kalibrierung.py: wie das Bauteil im
                               Greifer sitzt (T_flange_part), die
                               eingemessene Stiftspitze und die Guete.

Ausgabe:
    robot_path.txt             Kopfzeilen (# SCHLUESSEL=WERT) und je
                               Zeile PHASE;X;Y;Z;A;B;C — das Format,
                               das der MarkierungExecutor (Stufe 5)
                               liest.

Bewegungsphasen:
    TRANSFER   grosser Sicherheitsabstand zwischen den Regionen
    APPROACH   Anfahrabstand / Ausweich-Ueberfluege
    INFEED     Zustellen bis zum Kontakt
    CONTACT    Markierfahrt (Stift auf dem Bauteil)
    RETRACT    Abheben

Aufruf:
    python bahnplanung.py                 (Standardpfade, s. o.)
    python bahnplanung.py --demo          (Mini-Beispiel ohne echte
                                           Daten)
"""

import argparse
import json
import os
import sys
from datetime import datetime

import numpy as np
from scipy.spatial.transform import Rotation

# ===========================================================================
# EINSTELLUNGEN
# ===========================================================================
GRIND_PATH_DATEI = "../03_soll_ist_vergleich/befund/grind_path.json"
FALLBACK_DATEI = "beispiel_grind_path.json"   # liegt in diesem Ordner
KALIB_DATEI = "kalibrierung_ergebnis.json"
AUSGABE = "robot_path.txt"

# --- Der ortsfeste Stift (Roboterbasis-System, mm) --------------------------
# Die Spitze wird, wenn vorhanden, aus kalibrierung_ergebnis.json
# uebernommen (Feld stift_spitze_welt); dieser Wert ist der Rueckfall:
PEN_TIP_WORLD = np.array([618.0, 14.0, 308.0])
# Richtung des Stifts, von der Spitze weg in den Halter.

PEN_AXIS_WORLD = np.array([0.0, 0.0, 1.0])

# Zylindermodell des Stifthalters (fuer die Kollisionspruefung): Die Mine
# ragt PEN_FREILAENGE frei heraus, dahinter beginnt der dicke Halter.
PEN_FREILAENGE_MM = 25.0
PEN_RADIUS_MM = 12.0
PEN_LAENGE_MM = 90.0
ANPRESSWEG_MM = 2.0            # Zustellung ueber den Kontakt (Federweg)

# --- Bewegung -----------------------------------------------------------------
SICHER_ABSTAND_MM = 60.0       # TRANSFER
APPROACH_ABSTAND_MM = 15.0     # APPROACH / Ausweich-Ueberfluege
GESCHWINDIGKEIT = {"TRANSFER": 80.0, "APPROACH": 20.0, "INFEED": 5.0,
                   "CONTACT": 10.0, "RETRACT": 30.0}     # mm/s

# --- Roboter --------------------------------------------------------------------
MAX_REICHWEITE_MM = 820.0      #

PHASEN = ("TRANSFER", "APPROACH", "INFEED", "CONTACT", "RETRACT")


# ===========================================================================
# EINGABEN
# ===========================================================================

def lade_befunde(pfad):
    """Liest grind_path.json aus Stufe 3: je Region die Wegpunkte (mm)
    mit Flaechennormale und Ausweich-Kennzeichen."""
    with open(pfad, "r", encoding="utf-8") as f:
        daten = json.load(f)
    meta = {"order_optimized": daten.get("order_optimized"),
            "reorientation_deg": daten.get("reorientation_deg"),
            "part_id": daten.get("part_id")}
    regionen = []
    for r in daten.get("regions", []):
        wps = r.get("waypoints", [])
        if not wps:
            continue
        regionen.append({
            "zone": r.get("zone", "?"),
            "punkte": np.array([w["xyz"] for w in wps], float),
            "normalen": np.array([w["normal"] for w in wps], float),
            "evasion": np.array([bool(w.get("is_evasion", False))
                                 for w in wps]),
        })
    if not regionen:
        sys.exit(f"Keine Regionen in '{pfad}' — erst Stufe 3 laufen "
                 "lassen (oder Datei pruefen).")
    print("  Regionen: " + ", ".join(
        f"{r['zone']} ({len(r['punkte'])} WP"
        f"{', ' + str(int(r['evasion'].sum())) + ' Ausweich' if r['evasion'].any() else ''})"
        for r in regionen))
    if meta.get("reorientation_deg") is not None:
        print(f"  Reihenfolge aus Stufe 3 optimiert — Rest-Umorientierung "
              f"{meta['reorientation_deg']:.0f} Grad.")
    return regionen, meta


def lade_kalibrierung(pfad):
    """Liest unser Kalibrier-Ergebnis: T_flange_part und die eingemessene
    Stiftspitze (Guete-Kennzahlen nur informativ)."""
    global PEN_TIP_WORLD
    with open(pfad, "r", encoding="utf-8") as f:
        k = json.load(f)
    T = np.asarray(k["T_flange_part"], float).reshape(4, 4)
    if "stift_spitze_welt" in k:
        PEN_TIP_WORLD = np.asarray(k["stift_spitze_welt"], float)
    info = (f"RMSE {k.get('rmse_mm', '?')} mm, "
            f"{k.get('anzahl_punkte', '?')} Punkte")
    print(f"  Kalibrierung geladen ({info}); Stiftspitze "
          f"[{PEN_TIP_WORLD[0]:.1f}, {PEN_TIP_WORLD[1]:.1f}, "
          f"{PEN_TIP_WORLD[2]:.1f}] mm")
    return T, info


# ===========================================================================
# FLANSCHPOSEN (invertierte Kinematik)
# ===========================================================================
# Konventionen:
#   T_wf: Flansch -> Roboterbasis (die Ausgabepose)
#   T_fp: Bauteil -> Flansch (aus der Kalibrierung)
#   T_wb = T_wf @ T_fp
# Je Bahnpunkt p mit Normale n (beide im Bauteil-System):
#   1) T_wb @ p = Spitze + offset * Achse
#      (Achse zeigt von der Spitze weg vom Bauteil, also:
#       offset < 0 = Luftabstand, offset > 0 = anpressen/einfedern)
#   2) R_wb @ n = -Achse  (Stift steht senkrecht auf der Flaeche)

def _rot_aus_z(z_soll, referenz):
    z = z_soll / np.linalg.norm(z_soll)
    x = referenz - np.dot(referenz, z) * z
    if np.linalg.norm(x) < 1e-6:
        x = np.array([0.0, 1.0, 0.0]) - z[1] * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def baue_posen(regionen, T_fp):
    achse = PEN_AXIS_WORLD / np.linalg.norm(PEN_AXIS_WORLD)
    kontakt_offset = +ANPRESSWEG_MM        # Zustellung ueber den Kontakt
    referenz = np.array([1.0, 0.0, 0.0])
    T_fp_inv = np.linalg.inv(T_fp)
    posen = []

    def pose(p, n, offset):
        nonlocal referenz
        R_wb = _rot_aus_z(-achse, referenz) @ \
            _rot_aus_z(n / np.linalg.norm(n),
                       np.array([1.0, 0.0, 0.0])).T
        referenz = R_wb[:, 0]                 # Orientierung bleibt stetig
        T_wb = np.eye(4)
        T_wb[:3, :3] = R_wb
        T_wb[:3, 3] = PEN_TIP_WORLD + offset * achse - R_wb @ p
        return T_wb @ T_fp_inv

    for r in regionen:
        P, N, E = r["punkte"], r["normalen"], r["evasion"]
        posen.append(("TRANSFER", pose(P[0], N[0], -SICHER_ABSTAND_MM)))
        posen.append(("APPROACH", pose(P[0], N[0], -APPROACH_ABSTAND_MM)))
        im_kontakt = False
        for i in range(len(P)):
            if E[i]:                          # Ausweichpunkt: ueberfliegen
                if im_kontakt:
                    posen.append(("RETRACT", pose(P[i - 1], N[i - 1],
                                                  -APPROACH_ABSTAND_MM)))
                    im_kontakt = False
                posen.append(("APPROACH", pose(P[i], N[i],
                                               -APPROACH_ABSTAND_MM)))
            else:                             # Kontaktpunkt
                if not im_kontakt:
                    posen.append(("INFEED", pose(P[i], N[i],
                                                 kontakt_offset)))
                    im_kontakt = True
                posen.append(("CONTACT", pose(P[i], N[i], kontakt_offset)))
        letzte = len(P) - 1
        if im_kontakt:
            posen.append(("RETRACT", pose(P[letzte], N[letzte],
                                          -APPROACH_ABSTAND_MM)))
        posen.append(("RETRACT", pose(P[letzte], N[letzte],
                                      -SICHER_ABSTAND_MM)))
        print(f"  {r['zone']:<22s} geplant")

    xyz = np.array([T[:3, 3] for _, T in posen])
    assert not np.any(np.isnan(xyz)), "NaN in den Posen"
    zaehl = {ph: sum(1 for p, _ in posen if p == ph) for ph in PHASEN}
    print("  Posen: " + " ".join(f"{k}:{v}" for k, v in zaehl.items())
          + f"  (gesamt {len(posen)})")
    return posen


# ===========================================================================
# PRUEFUNGEN & EXPORT
# ===========================================================================

def pruefe(posen, T_fp, bauteil_punkte):
    """Kollisions- und Reichweitenpruefung. Der Stifthalter ist ein
    Zylinder ab PEN_FREILAENGE hinter der Spitze; geprueft werden AXIALE
    Lage und RADIALER Abstand getrennt (reine Punktabstaende reichen fuer
    Werkzeuggeometrie nicht — Projekterkenntnis). Als Bauteilgeometrie
    dienen hier die Bahnpunkte selbst — eine grobe Naeherung; die feine
    Schaftpruefung ist bereits in Stufe 3 gelaufen."""
    achse = PEN_AXIS_WORLD / np.linalg.norm(PEN_AXIS_WORLD)
    p_h = np.hstack([bauteil_punkte, np.ones((len(bauteil_punkte), 1))])
    schlecht, zu_weit = [], []
    for i, (phase, T_wf) in enumerate(posen):
        welt = (T_wf @ T_fp @ p_h.T).T[:, :3] - PEN_TIP_WORLD
        axial = welt @ achse
        radial = np.linalg.norm(welt - np.outer(axial, achse), axis=1)
        if np.any((axial > PEN_FREILAENGE_MM) & (axial < PEN_LAENGE_MM)
                  & (radial < PEN_RADIUS_MM)):
            schlecht.append((i, phase))
        if np.linalg.norm(T_wf[:3, 3]) > MAX_REICHWEITE_MM:
            zu_weit.append(i)
    if schlecht:
        print(f"  !! {len(schlecht)} Posen mit Halter-Kollision "
              f"(zuerst Pose {schlecht[0][0]}, {schlecht[0][1]}).")
    if zu_weit:
        print(f"  !! {len(zu_weit)} Posen ausserhalb der groben "
              f"Reichweite ({MAX_REICHWEITE_MM:.0f} mm).")
    if not schlecht and not zu_weit:
        print(f"  Kollision (grob) und Reichweite: OK ({len(posen)} "
              "Posen). Achslimits zeigt erst der Luftlauf am Roboter!")
    return not schlecht and not zu_weit


def schreibe_robot_path(pfad, posen, kalib_info, teil, quelle, meta):
    with open(pfad, "w", encoding="utf-8", newline="\n") as f:
        f.write("# ROBOT_PATH V2\n")
        f.write(f"# ERSTELLT={datetime.now().isoformat(timespec='seconds')}\n")
        f.write(f"# TEIL={teil}\n")
        f.write(f"# QUELLE={os.path.basename(quelle)}\n")
        if meta.get("reorientation_deg") is not None:
            f.write(f"# REIHENFOLGE_OPTIMIERT="
                    f"{1 if meta.get('order_optimized') else 0}\n")
            f.write(f"# UMORIENTIERUNG_DEG="
                    f"{meta['reorientation_deg']:.1f}\n")
        f.write(f"# KALIBRIERUNG={kalib_info}\n")
        f.write(f"# POSEN={len(posen)}\n")
        for ph in PHASEN:
            f.write(f"# V_{ph}={GESCHWINDIGKEIT[ph]:.1f}\n")
        f.write("# FORMAT=PHASE;X_mm;Y_mm;Z_mm;A_deg;B_deg;C_deg\n")
        for phase, T in posen:
            x, y, z = T[:3, 3]
            a, b, c = Rotation.from_matrix(T[:3, :3]).as_euler(
                "ZYX", degrees=True)
            f.write(f"{phase};{x:.3f};{y:.3f};{z:.3f};"
                    f"{a:.3f};{b:.3f};{c:.3f}\n")
    print(f"  -> '{pfad}' ({len(posen)} Posen)")


# ===========================================================================
# HAUPTPROGRAMM & DEMO
# ===========================================================================

def main(argv=None):
    ap = argparse.ArgumentParser(description="Bahnplanung: grind_path -> "
                                             "robot_path.txt")
    ap.add_argument("--grind", default=None,
                    help="grind_path.json aus Stufe 3")
    ap.add_argument("--kalib", default=KALIB_DATEI)
    ap.add_argument("--ausgabe", default=AUSGABE)
    ap.add_argument("--teil-id", default="kopfstueck")
    ap.add_argument("--demo", action="store_true",
                    help="Mini-Beispiel mit guter Demo-Kalibrierung "
                         "(zeigt den Kontaktfall)")
    args = ap.parse_args(argv)

    print("== Bahnplanung ==")
    if args.demo:
        args.grind, args.kalib = demo_daten()
        args.ausgabe, args.teil_id = "robot_path_demo.txt", "DEMO"

    quelle = args.grind
    if quelle is None:
        if os.path.isfile(GRIND_PATH_DATEI):
            quelle = GRIND_PATH_DATEI
        else:
            quelle = FALLBACK_DATEI
            print(f"  Hinweis: '{GRIND_PATH_DATEI}' nicht gefunden — "
                  f"nutze die beiliegende '{FALLBACK_DATEI}'.")

    print("[1/4] Eingaben")
    regionen, meta = lade_befunde(quelle)
    if args.teil_id == "kopfstueck" and meta.get("part_id"):
        args.teil_id = meta["part_id"]
    T_fp, info = lade_kalibrierung(args.kalib)

    print("[2/4] Posen berechnen")
    posen = baue_posen(regionen, T_fp)

    print("[3/4] Pruefen")
    alle_punkte = np.vstack([r["punkte"] for r in regionen])
    if not pruefe(posen, T_fp, alle_punkte):
        print("  Abbruch — es wird keine Roboterdatei geschrieben.")
        return 2

    print("[4/4] Schreiben")
    schreibe_robot_path(args.ausgabe, posen, info, args.teil_id, quelle,
                        meta)
    print("\nFertig. Datei in den src-Ordner des Sunrise-Projekts "
          "kopieren.")
    return 0


def demo_daten():
    """Mini-grind_path (Zickzack-Flaeche + Linie mit einem Ausweichpunkt)
    und eine Beispiel-Kalibrierung."""
    os.makedirs("demo_daten", exist_ok=True)
    wp = []
    for zi, y in enumerate(np.linspace(-4, 4, 5)):
        xs = np.linspace(-8, 8, 7)
        if zi % 2:
            xs = xs[::-1]
        for x in xs:
            wp.append({"xyz": [round(x - 30, 3), round(float(y), 3), 0.0],
                       "normal": [0.0, 0.0, 1.0], "is_evasion": False})
    linie = [{"xyz": [round(20 + 10 * np.cos(t), 3),
                      round(10 * np.sin(t), 3), 0.0],
              "normal": [0.0, 0.0, 1.0],
              "is_evasion": bool(0.9 < t < 1.3)}      # kurzer Ueberflug
             for t in np.linspace(0, np.pi, 24)]
    gp = "demo_daten/grind_path_demo.json"
    with open(gp, "w", encoding="utf-8") as f:
        json.dump({"part": "DEMO", "regions": [
            {"zone": "AngussA", "waypoints": wp},
            {"zone": "Grat_A", "waypoints": linie},
        ]}, f)
    T = np.eye(4)
    T[2, 3] = 120.0
    kp = "demo_daten/kalibrierung_demo.json"
    with open(kp, "w", encoding="utf-8") as f:
        json.dump({"T_flange_part": T.tolist(),
                   "stift_spitze_welt": [600.0, 0.0, 300.0],
                   "rmse_mm": 0.30, "anzahl_punkte": 8}, f)
    print("  Demo-Daten -> demo_daten/")
    return gp, kp


if __name__ == "__main__":
    sys.exit(main())
