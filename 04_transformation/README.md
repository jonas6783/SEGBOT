# Schritt 4 — Transformation ins Robotersystem

Übersetzt die Pfade aus Schritt 3 in eine Roboterbahn. Die Kette:

```
Schritt 3: befund/grind_path.json  ──►  bahnplanung.py  ──►  robot_path.txt
                                          ▲
                     kalibrierung.py ─────┘  (wie sitzt das Bauteil im
                     (aus messungen.json)     Greifer + Stiftspitze)
```

## Beiliegende Projektergebnisse — alles direkt ausführbar

| Datei | Inhalt |
| --- | --- |
| `messungen.json` | die 4 Antastmessungen unserer letzten Kalibrierrunde |
| `kalibrierung_ergebnis.json` | daraus berechnet: Bauteillage + Stiftspitze, **RMSE 0,85 mm bei 4 Punkten** → `nur_luftlauf=true` |
| `beispiel_grind_path.json` | Ausgabe eines Laufs von Schritt 3 (greift automatisch, solange dort kein frischer `befund/`-Ordner liegt) |

Die drei Kommandos reproduzieren die gesamte Kette — inklusive des
Ergebnisses, das in `05_roboter/robot_path.txt` eingecheckt ist:

```
python punkte_vorschlagen.py --demo   # so wurden Antastpunkte gewählt
python kalibrierung.py                # rechnet unsere 4 Messungen durch
python bahnplanung.py                 # erzeugt robot_path.txt (Luftlauf, 41 Posen)
```

`kalibrierung.py` meldet dabei genau das, was im Projekt passiert ist:
Qualität unzureichend → das Sicherheits-Gate schaltet auf
`NUR_LUFTLAUF=1`, und `bahnplanung.py` plant alle Kontaktposen im
15-mm-Anfahrabstand statt mit Berührung. Jedes Skript hat zusätzlich
einen `--demo`-Modus, der den **Kontaktfall** mit einer guten
Kunst-Kalibrierung zeigt.

## Warum dieser Schritt im Projekt nicht mehr geklappt hat

Die Bahnplanung selbst funktioniert — sie erzeugt vollständige,
geprüfte Bewegungsfolgen über die fünf Phasen TRANSFER → APPROACH →
INFEED → CONTACT → RETRACT, übernimmt je Wegpunkt die Flächennormale
aus Schritt 3 und schreibt dessen Reihenfolge-Metrik
(`UMORIENTIERUNG_DEG`) mit in den Dateikopf. Gescheitert ist die **Kalibrierung**:

- Unsere letzte Messrunde nutzte nur **4 statt der mindestens 6**
  nötigen Antastpunkte und erreichte einen **RMSE von 0,85 mm** — die
  Qualitätsschwelle liegt bei 0,5 mm.
- Zusätzlich steht die Richtung des Markierstifts (`PEN_AXIS_WORLD` in
  `bahnplanung.py`) noch auf dem Platzhalterwert `(0, 0, 1)` statt auf
  der echten Einbaurichtung — das führte beim Ausführen zu
  Achslimit-Fehlern.

Das Sicherheits-Gate hat dadurch zuverlässig verhindert, dass mit
falscher Kalibrierung markiert wird; die eingecheckte
`05_roboter/robot_path.txt` (41 Posen, `NUR_LUFTLAUF=1`) ist genau
dieser dokumentierte Stand. Die Plausibilitätsprüfung in
`kalibrierung.py` fängt außerdem die beiden Fehler ab, die uns beim
Messen passiert sind: doppelt kopierte Messwerte und ein auf dem
Magneten verrutschtes Bauteil.

## Was für eine funktionierende Markierung noch fehlt

1. Kalibrierung mit **8 gut verteilten Punkten** wiederholen
   (Vorschläge: `punkte_vorschlagen.py`), Flanschorientierung zwischen
   den Messungen variieren, Ziel RMSE ≤ 0,5 mm.
2. Die echte Stiftachse aus der Haltergeometrie bestimmen und in
   `bahnplanung.py` eintragen.
3. Erst im Luftlauf testen, dann Kontaktfahrten freigeben.
