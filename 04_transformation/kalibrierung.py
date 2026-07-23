#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
kalibrierung.py — Wie sitzt das Bauteil im Greifer? (Stufe 4)
=============================================================

Der Roboter haelt das Bauteil, aber niemand weiss auf den Zehntelmilli-
meter genau, WIE es im Greifer sitzt. Das misst dieses Skript:

Ablauf am Roboter:
  1. Mit punkte_vorschlagen.py 8 gut verteilte Antastpunkte auf dem CAD
     auswaehlen (markante, eindeutig anfahrbare Stellen).
  2. Jeden Punkt vorsichtig an die ORTSFESTE Stiftspitze fahren, bis er
     sie gerade beruehrt. Dabei zwischen den Punkten die Orientierung
     des Flanschs VARIIEREN — sonst laesst sich die Stiftposition nicht
     mitbestimmen.
  3. Je Beruehrung notieren: welcher CAD-Punkt (mm, Bauteil-System) und
     die Flanschpose vom Smartpad (X Y Z in mm, A B C in Grad).
  4. Alles in messungen.json eintragen (Vorlage: --vorlage) und dieses
     Skript starten.

Gerechnet wird beides gleichzeitig:
    T_flange_part   wie das Bauteil im Flansch sitzt
    Stiftspitze     wo die Spitze im Roboterbasis-System steht

Vorher laeuft eine Plausibilitaetspruefung: Bei zwei Messungen mit
GLEICHER Flanschorientierung muss der Abstand der Flanschpositionen dem
CAD-Abstand der beiden Punkte entsprechen (starres Bauteil!). Passt das
nicht, ist meist eine Messung doppelt kopiert oder das Bauteil auf dem
Magneten verrutscht — beides ist uns im Projekt passiert.

Ausgabe: kalibrierung_ergebnis.json mit T_flange_part, Stiftspitze,
RMSE und Punktzahl. Ist der RMSE > 0,5 mm oder sind es < 6 Punkte,
steht nur_luftlauf=true — die Bahnplanung erzeugt dann automatisch
eine kontaktlose Bahn. (Unsere letzte echte Messung: RMSE 0,85 mm bei
4 Punkten — deshalb blieb es im Projekt beim Luftlauf.)

Aufruf:
    python kalibrierung.py                    (liest messungen.json)
    python kalibrierung.py --vorlage          (legt eine Vorlage an)
    python kalibrierung.py --demo             (rechnet mit erfundenen,
                                               aber exakt bekannten Daten
                                               und prueft sich selbst)
"""

import argparse
import json
import os
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

MESSUNGEN_DATEI = "messungen.json"
AUSGABE_DATEI = "kalibrierung_ergebnis.json"

RMSE_LIMIT_MM = 0.5        # schlechter -> nur_luftlauf
MIN_PUNKTE = 6             # weniger    -> nur_luftlauf
PLAUSI_TOLERANZ_MM = 1.0   # Starrkoerper-Pruefung (siehe oben)
GLEICHE_ORIENTIERUNG_DEG = 1.0


# ===========================================================================
# EINGABE
# ===========================================================================

def lade_messungen(pfad):
    """Liest messungen.json: je Eintrag der angetastete CAD-Punkt und die
    am Smartpad abgelesene Flanschpose."""
    if not os.path.isfile(pfad):
        sys.exit(f"'{pfad}' nicht gefunden. Mit --vorlage eine Vorlage "
                 "anlegen und die Messwerte eintragen.")
    with open(pfad, "r", encoding="utf-8") as f:
        daten = json.load(f)
    q, T_wf = [], []
    for i, m in enumerate(daten["messungen"], 1):
        q.append(np.asarray(m["cad_punkt"], float))
        x, y, z, a, b, c = [float(v) for v in m["flansch"]]
        T = np.eye(4)
        T[:3, :3] = Rotation.from_euler("ZYX", [a, b, c],
                                        degrees=True).as_matrix()
        T[:3, 3] = [x, y, z]
        T_wf.append(T)
    print(f"  {len(q)} Messungen aus '{pfad}' gelesen.")
    return np.array(q), T_wf


def schreibe_vorlage(pfad):
    vorlage = {
        "_hinweis": ("Je Messung: cad_punkt = angetasteter Punkt aus "
                     "punkte_vorschlagen.py (mm, Bauteil-System); "
                     "flansch = [X,Y,Z,A,B,C] vom Smartpad (mm/Grad). "
                     "Orientierung zwischen den Messungen variieren!"),
        "messungen": [
            {"cad_punkt": [0.0, 0.0, 0.0],
             "flansch": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
            {"cad_punkt": [0.0, 0.0, 0.0],
             "flansch": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
        ],
    }
    with open(pfad, "w", encoding="utf-8") as f:
        json.dump(vorlage, f, ensure_ascii=False, indent=2)
    print(f"Vorlage geschrieben: '{pfad}' — Werte eintragen und Skript "
          "erneut starten.")


# ===========================================================================
# PLAUSIBILITAETSPRUEFUNG (faengt Doppelmessungen & verrutschtes Teil)
# ===========================================================================

def plausibilitaet(q, T_wf):
    fehler = []
    for i in range(len(q)):
        for j in range(i + 1, len(q)):
            R_rel = T_wf[i][:3, :3].T @ T_wf[j][:3, :3]
            winkel = np.degrees(np.arccos(
                np.clip((np.trace(R_rel) - 1) / 2, -1, 1)))
            if winkel > GLEICHE_ORIENTIERUNG_DEG:
                continue          # nur Paare mit gleicher Orientierung
            d_flansch = np.linalg.norm(T_wf[i][:3, 3] - T_wf[j][:3, 3])
            d_cad = np.linalg.norm(q[i] - q[j])
            if abs(d_flansch - d_cad) > PLAUSI_TOLERANZ_MM:
                fehler.append((i + 1, j + 1, d_flansch, d_cad))
    if fehler:
        print("FEHLER — Messungen unplausibel (starres Bauteil verletzt):")
        for i, j, df, dc in fehler:
            print(f"  Messung {i} und {j}: Flansch-Abstand {df:.2f} mm, "
                  f"CAD-Abstand {dc:.2f} mm")
        print("Typische Ursachen: eine Messung doppelt kopiert (Abstand 0)")
        print("oder das Bauteil ist auf dem Magneten verrutscht.")
        print("Bitte die betroffenen Punkte neu messen.")
        sys.exit(1)
    print("  Plausibilitaetspruefung: OK")


# ===========================================================================
# LOESER: T_flange_part und Stiftspitze gleichzeitig
# ===========================================================================
# Fuer jede Beruehrung gilt:  T_wf @ T_fp @ q  =  Spitze
# Umgestellt:                 R_fp @ q + t_fp  =  R_wf^T (Spitze - t_wf)
# Unbekannt sind R_fp, t_fp (wie sitzt das Bauteil) und die Spitze —
# zusammen 9 Zahlen, geloest per Ausgleichsrechnung ueber alle Messungen.

def loese(q, T_wf):
    R_w = [T[:3, :3] for T in T_wf]
    t_w = [T[:3, 3] for T in T_wf]

    def residuen(x):
        R_fp = Rotation.from_rotvec(x[:3]).as_matrix()
        t_fp, spitze = x[3:6], x[6:9]
        r = [R_fp @ q[i] + t_fp - R_w[i].T @ (spitze - t_w[i])
             for i in range(len(q))]
        return np.concatenate(r)

    start = np.zeros(9)
    start[6:9] = np.mean(t_w, axis=0)       # Spitze grob: mittlere Flanschlage
    erg = least_squares(residuen, start, method="lm")
    res = erg.fun.reshape(-1, 3)
    rmse = float(np.sqrt(np.mean(np.sum(res ** 2, axis=1))))

    T_fp = np.eye(4)
    T_fp[:3, :3] = Rotation.from_rotvec(erg.x[:3]).as_matrix()
    T_fp[:3, 3] = erg.x[3:6]
    return T_fp, erg.x[6:9], rmse


# ===========================================================================
# HAUPTPROGRAMM & DEMO
# ===========================================================================

def main(argv=None):
    ap = argparse.ArgumentParser(description="Kalibrierung: Bauteil im "
                                             "Greifer + Stiftspitze")
    ap.add_argument("--messungen", default=MESSUNGEN_DATEI)
    ap.add_argument("--ausgabe", default=AUSGABE_DATEI)
    ap.add_argument("--vorlage", action="store_true",
                    help="leere messungen.json anlegen")
    ap.add_argument("--demo", action="store_true",
                    help="Selbsttest mit exakt bekannten Kunstdaten")
    args = ap.parse_args(argv)

    if args.vorlage:
        schreibe_vorlage(args.messungen)
        return 0

    print("== Kalibrierung ==")
    if args.demo:
        q, T_wf, wahr = demo_messungen()
    else:
        q, T_wf = lade_messungen(args.messungen)
        wahr = None
    if len(q) < 3:
        sys.exit("Mindestens 3 Messungen noetig (besser 8).")

    plausibilitaet(q, T_wf)
    T_fp, spitze, rmse = loese(q, T_wf)

    nur_luftlauf = rmse > RMSE_LIMIT_MM or len(q) < MIN_PUNKTE
    grund = []
    if rmse > RMSE_LIMIT_MM:
        grund.append(f"RMSE {rmse:.2f} mm > {RMSE_LIMIT_MM} mm")
    if len(q) < MIN_PUNKTE:
        grund.append(f"nur {len(q)} Punkte (< {MIN_PUNKTE})")

    print(f"  RMSE: {rmse:.3f} mm bei {len(q)} Punkten")
    print(f"  Stiftspitze (Basis-System): "
          f"[{spitze[0]:.2f}, {spitze[1]:.2f}, {spitze[2]:.2f}] mm")
    if nur_luftlauf:
        print("  !! Qualitaet unzureichend (" + "; ".join(grund) + ")")
        print("     -> nur_luftlauf=true, die Bahnplanung bleibt "
              "kontaktlos.")
    else:
        print("  Qualitaet OK — Kontaktfahrten freigegeben.")

    with open(args.ausgabe, "w", encoding="utf-8") as f:
        json.dump({
            "T_flange_part": T_fp.tolist(),
            "stift_spitze_welt": [round(float(v), 3) for v in spitze],
            "rmse_mm": round(rmse, 3),
            "anzahl_punkte": len(q),
            "nur_luftlauf": bool(nur_luftlauf),
        }, f, ensure_ascii=False, indent=2)
    print(f"  -> '{args.ausgabe}'")

    if wahr is not None:                       # Demo: gegen Wahrheit pruefen
        T_wahr, spitze_wahr = wahr
        dt = np.linalg.norm(T_fp[:3, 3] - T_wahr[:3, 3])
        dw = np.degrees(np.arccos(np.clip(
            (np.trace(T_fp[:3, :3].T @ T_wahr[:3, :3]) - 1) / 2, -1, 1)))
        ds = np.linalg.norm(spitze - spitze_wahr)
        print("\n  Demo-Kontrolle gegen die bekannte Wahrheit:")
        print(f"    Bauteillage: {dt:.3f} mm / {dw:.3f} Grad daneben")
        print(f"    Stiftspitze: {ds:.3f} mm daneben")
        assert dt < 0.2 and dw < 0.2 and ds < 0.2, "Loeser ungenau!"
        print("  Selbsttest bestanden.")
    return 0


def demo_messungen():
    """Erfundenes, aber exakt bekanntes Setup: 8 Antastungen mit
    variierender Orientierung und 0,05 mm Messrauschen."""
    rng = np.random.default_rng(4)
    T_fp = np.eye(4)
    T_fp[:3, :3] = Rotation.from_euler("ZYX", [12, -5, 3],
                                       degrees=True).as_matrix()
    T_fp[:3, 3] = [4.0, -2.0, 118.0]
    spitze = np.array([612.0, 22.0, 305.0])

    q = np.array([[30, 20, 0], [-30, 20, 0], [-30, -20, 0], [30, -20, 0],
                  [30, 20, 12], [-30, 20, 12], [-30, -20, 12],
                  [0, 0, 12]], float)
    T_wf = []
    for i, qi in enumerate(q):
        R_w = Rotation.from_euler(
            "ZYX", [20 * i, 8 * ((-1) ** i), 5 * (i % 3)],
            degrees=True).as_matrix()
        t_w = spitze - R_w @ (T_fp[:3, :3] @ qi + T_fp[:3, 3])
        T = np.eye(4)
        T[:3, :3] = R_w
        T[:3, 3] = t_w + rng.normal(0, 0.05, 3)
        T_wf.append(T)
    print("  Demo: 8 kuenstliche Antastungen erzeugt.")
    return q, T_wf, (T_fp, spitze)


if __name__ == "__main__":
    sys.exit(main())
