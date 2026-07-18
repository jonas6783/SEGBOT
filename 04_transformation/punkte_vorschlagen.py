#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
punkte_vorschlagen.py — Antastpunkte fuer die Kalibrierung (Stufe 4)
====================================================================

Schlaegt 8 Punkte auf dem CAD vor, die man fuer die Kalibrierung an die
Stiftspitze antastet. Gute Antastpunkte sind moeglichst WEIT ueber das
Bauteil verteilt — je weiter auseinander, desto stabiler wird die
Ausgleichsrechnung. Das laeuft zweistufig:

    Start ist der Punkt, der am weitesten vom Schwerpunkt liegt (meist
    eine Ecke). Danach kommt immer der Punkt dazu, der von allen schon
    gewaehlten am weitesten entfernt ist — bis 8 beisammen sind.

Beim Antasten selbst gilt: markante Stellen bevorzugen (Ecken, Kanten-
enden), die man am echten Teil eindeutig wiederfindet, und zwischen den
Messungen die Flanschorientierung variieren (siehe kalibrierung.py).

Aufruf:
    python punkte_vorschlagen.py --cad ../03_soll_ist_vergleich/Bauteil.ply
    python punkte_vorschlagen.py --demo     (Wuerfel als Beispiel)

Ausgabe: Liste auf der Konsole + antastpunkte.json (die cad_punkt-Werte
fuer messungen.json).
"""

import argparse
import json
import os
import sys

import numpy as np

ANZAHL = 8
AUSGABE = "antastpunkte.json"


def lade_cad(pfad):
    if not os.path.isfile(pfad):
        sys.exit(f"CAD-Datei '{pfad}' nicht gefunden.")
    ext = os.path.splitext(pfad)[1].lower()
    if ext == ".npy":
        return np.asarray(np.load(pfad), float)[:, :3]
    if ext in (".xyz", ".txt", ".asc"):
        return np.loadtxt(pfad, float)[:, :3]
    try:
        import open3d as o3d
    except ImportError:
        sys.exit(f"Fuer '{ext}' wird open3d gebraucht: pip install open3d")
    mesh = o3d.io.read_triangle_mesh(pfad)
    if len(mesh.triangles) > 0:
        return np.asarray(mesh.sample_points_uniformly(50_000).points, float)
    return np.asarray(o3d.io.read_point_cloud(pfad).points, float)


def fibonacci_richtungen(n=64):
    """n gleichmaessig ueber die Kugel verteilte Richtungen
    (Fibonacci-Spirale) — damit tasten wir das Bauteil "von allen
    Seiten" ab."""
    i = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * i / n)
    theta = np.pi * (1 + 5 ** 0.5) * i
    return np.column_stack([np.cos(theta) * np.sin(phi),
                            np.sin(theta) * np.sin(phi), np.cos(phi)])


def weiteste_punkte(punkte, anzahl):
    """Zweistufig: (1) In viele Richtungen jeweils den AEUSSERSTEN Punkt
    einsammeln — das sind markante Stellen wie Ecken und Kantenenden,
    die man am echten Teil eindeutig wiederfindet. (2) Daraus per
    Weitester-Punkt-Verfahren die 'anzahl' am besten verteilten waehlen
    (jeder neue Punkt maximiert den Abstand zur bisherigen Auswahl)."""
    p = np.asarray(punkte, float)
    mitte = p.mean(axis=0)
    kandidaten_idx = {int(np.argmax((p - mitte) @ d))
                      for d in fibonacci_richtungen()}
    k = p[sorted(kandidaten_idx)]
    if len(k) < anzahl:
        k = p                                  # Notnagel: alle Punkte
    start = int(np.argmax(np.linalg.norm(k - mitte, axis=1)))
    auswahl = [start]
    abstand = np.linalg.norm(k - k[start], axis=1)
    for _ in range(anzahl - 1):
        naechster = int(np.argmax(abstand))
        auswahl.append(naechster)
        abstand = np.minimum(abstand,
                             np.linalg.norm(k - k[naechster], axis=1))
    return k[auswahl]


def main(argv=None):
    ap = argparse.ArgumentParser(description="8 gut verteilte "
                                             "Antastpunkte vorschlagen")
    ap.add_argument("--cad", default="../03_soll_ist_vergleich/Bauteil.ply")
    ap.add_argument("--anzahl", type=int, default=ANZAHL)
    ap.add_argument("--ausgabe", default=AUSGABE)
    ap.add_argument("--demo", action="store_true",
                    help="mit einem 60-mm-Wuerfel als Beispiel laufen")
    args = ap.parse_args(argv)

    print("== Antastpunkte vorschlagen ==")
    if args.demo:
        g = np.linspace(-30, 30, 13)
        seiten = []
        for feste_achse in range(3):
            for wert in (-30.0, 30.0):
                a, b = np.meshgrid(g, g)
                fl = np.zeros((a.size, 3))
                fl[:, feste_achse] = wert
                fl[:, (feste_achse + 1) % 3] = a.ravel()
                fl[:, (feste_achse + 2) % 3] = b.ravel()
                seiten.append(fl)
        punkte = np.vstack(seiten)
        print("  Demo: 60-mm-Wuerfel (Antastpunkte sollten an den Ecken "
              "landen).")
    else:
        punkte = lade_cad(args.cad)
        print(f"  CAD: {len(punkte):,} Punkte")

    vorschlag = weiteste_punkte(punkte, args.anzahl)
    print(f"\n  {args.anzahl} Vorschlaege (mm, Bauteil-System):")
    for i, p in enumerate(vorschlag, 1):
        print(f"    {i}:  [{p[0]:8.2f}, {p[1]:8.2f}, {p[2]:8.2f}]")

    with open(args.ausgabe, "w", encoding="utf-8") as f:
        json.dump({"antastpunkte": np.round(vorschlag, 3).tolist()},
                  f, ensure_ascii=False, indent=2)
    print(f"\n  -> '{args.ausgabe}'  (Werte als cad_punkt in "
          "messungen.json uebernehmen)")

    if args.demo:
        ecken = np.array([[x, y, z] for x in (-30, 30)
                          for y in (-30, 30) for z in (-30, 30)], float)
        d = np.linalg.norm(vorschlag[:, None] - ecken[None], axis=2).min(1)
        print(f"  Kontrolle: groesster Abstand zur naechsten Wuerfelecke "
              f"= {d.max():.1f} mm")
        assert d.max() < 9.0, "Vorschlaege liegen nicht an den Ecken!"
        print("  Selbsttest bestanden.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
