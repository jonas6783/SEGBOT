# Schritt 4 — Transformation ins Robotersystem

Übersetzt die Wegpunkte aus Schritt 3 per invertierter Kinematik in
Flanschposen: Der Roboter hält das Bauteil und führt jeden Bahnpunkt an
die ortsfeste Stiftspitze, Fläche senkrecht zum Stift.

```
Schritt 3: befund/grind_path.json ──► bahnplanung.py ──► robot_path.txt
                                        ▲
                    kalibrierung.py ────┘  T_flange_part + Stiftspitze
                    (aus messungen.json)
```

| Datei | Inhalt |
| --- | --- |
| `messungen.json` | unsere 4 Antastmessungen |
| `kalibrierung_ergebnis.json` | daraus: Bauteillage im Greifer + Stiftspitze (RMSE 0,85 mm) |
| `beispiel_grind_path.json` | Ausgabe eines Laufs von Schritt 3 (greift automatisch, solange dort kein frischer `befund/`-Ordner liegt) |

Alles direkt ausführbar — reproduziert die eingecheckte
`05_roboter/robot_path.txt` (41 Posen):

```
python punkte_vorschlagen.py --demo   # so wurden Antastpunkte gewählt
python kalibrierung.py                # rechnet unsere Messungen durch
python bahnplanung.py                 # erzeugt robot_path.txt
```

Bewegungsphasen je Region: TRANSFER (60 mm Sicherheitsabstand) →
APPROACH (15 mm) → INFEED → CONTACT (2 mm Anpressweg über den
gefederten Stift) → RETRACT. Geschwindigkeiten je Phase stehen im
Dateikopf, ebenso die aus Schritt 3 übernommene Reihenfolge-Metrik
(`UMORIENTIERUNG_DEG`).

**Projektstand:** Gerechnet wird mit unserer letzten Kalibrierung
(4 Punkte, RMSE 0,85 mm). Die Stiftachse (`PEN_AXIS_WORLD` in
`bahnplanung.py`) steht noch auf dem Platzhalter `(0, 0, 1)` statt auf
der echten Einbaurichtung — auf dem Roboter führte das zu
Achslimit-Fehlern; für einen Neuaufbau die Achse aus der
Haltergeometrie bestimmen und mehr Antastpunkte verwenden
(`punkte_vorschlagen.py` schlägt 8 vor).
