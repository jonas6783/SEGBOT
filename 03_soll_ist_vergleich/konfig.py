# -*- coding: utf-8 -*-
"""
konfig.py — Alle Einstellungen des Soll-Ist-Vergleichs an einer Stelle
======================================================================
Dateipfade, Zonen mit Toleranzen und Farben, Filter, Darstellung und die
Schleifpfad-Strategie. Die anderen Module lesen von hier — wer etwas
anpassen will, muss nur in diese Datei schauen.
"""

CAD_PATH        = "Bauteil.stl"          # CAD-Referenz: STL ODER PLY-Mesh (mit Faces)
SCAN_PLY_PATH   = "merged10.ply"
ZONES_JSON_PATH = "Bauteil_Zones.json"
OUTPUT_DIR      = "befund"               # ALLE Ausgaben (inkl. Log) landen hier
PART_ID         = ""                     # leer = aus Scan-Dateiname

CAD_SCALE = 0.001                        # mm -> m

# ----------------------------------------------------------------------
# ZONEN: Toleranz (tolerance_mm) und Farbe (color) je Zone einstellen.
# Die Farbe markiert DEFEKT-Punkte (ausser Toleranz). Korrekte Punkte sind
# immer gruen (CORRECT_COLOR) -> keine Zonenfarbe gruen waehlen.
# ----------------------------------------------------------------------
ZONE_GROUPS = [
    {"name": "GratUnten", "tolerance_mm": 0.5, "color": "#f59e0b",   # orange
     "sources": ["GratUnten"]},
    {"name": "Grat",      "tolerance_mm": 0.4, "color": "#ef4444",   # rot
     "sources": ["Gratx_A", "Gratx_A_copy", "Gratx_A_copy_copy",
                 "Grat_B",  "Grat_B_copy",  "Grat_B_copy_copy"]},
    {"name": "AngussA",   "tolerance_mm": 1.0, "color": "#ec4899",   # pink
     "sources": ["AngussA", "AngussA_copy"]},
    {"name": "AngussB",   "tolerance_mm": 1.0, "color": "#06b6d4",   # cyan
     "sources": ["AngussB"]},
]
# Auffangzone fuer alles ausserhalb der definierten Zonen. None = deaktiviert.
STANDARD_ZONE = {"name": "Standard", "tolerance_mm": 1.5, "color": "#a855f7"}  # violett
CORRECT_COLOR = "#22c55e"                # Farbe fuer Punkte INNERHALB Toleranz (gruen)

# ----------------------------------------------------------------------
# FILTER + ZONENZUORDNUNG
# ----------------------------------------------------------------------
MAX_DIST_FROM_CAD = 0.003                # 3mm: |Abstand| groesser -> UEBERALL verworfen
ZONE_LATERAL_M    = 0.001                # 1mm: Fusspunkt-Naehe fuer Zonenzugehoerigkeit

# ----------------------------------------------------------------------
# DICHTE + DARSTELLUNG
# ----------------------------------------------------------------------
SCAN_VOXEL_M = 0.0                       # >0 = Scan downsamplen (z.B. 0.0005=0.5mm), 0=voll
POINT_SIZE   = 0.0001                    # Punktgroesse im 3D-Viewer (m)
MESH_OPACITY = 0.56                      # Transparenz des CAD-Mesh im Viewer (0..1)

# ----------------------------------------------------------------------
# SCHLEIFPFAD-PLANUNG
# ----------------------------------------------------------------------
GRIND_ENABLE           = True
GRIND_INCLUDE_STANDARD = False           # Standard-Zone nicht schleifen
GRIND_DBSCAN_EPS       = 0.002           # 2mm Clusterradius (groesser bei spaerlichen Punkten)
GRIND_DBSCAN_MIN       = 10              # DBSCAN Kern-Mindestpunkte
GRIND_MIN_REGION_PTS   = 30              # Cluster kleiner -> ignorieren (Rauschen)
GRIND_LINE_SPACING     = 0.002           # 2mm Wegpunktabstand entlang Grat
GRIND_AREA_STEPOVER    = 0.002           # 2mm Rasterabstand bei Anguss
GRIND_OPTIMIZE_ORDER   = True            # Regionen-Reihenfolge nach Flaechennormale
                                         # optimieren: gleich ausgerichtete Stellen
                                         # zusammen -> minimales Umorientieren des Teils
GRIND_SIDE_ANGLE_DEG   = 30              # Regionen (z.B. Grat um eine Kante) in Teilstuecke
                                         # mit aehnlicher Normale zerlegen -> seitenweises
                                         # Abarbeiten statt am Stueck quer ueber das Teil

AUTO_OPEN_REPORT = True
SERVER_PORT      = 8765



# ----------------------------------------------------------------------
# Trennebene fuer das seitenweise Abarbeiten (split_regions_by_side):
# Ein Strich wird auch dann geteilt, wenn er diese XZ-Ebene (Y = Wert,
# in mm) ueberquert. Y=0 halbiert UNSER Kopfstueck — bauteilspezifisch,
# bei einem anderen Teil bzw. anderer CAD-Lage anpassen!
# ----------------------------------------------------------------------
SPLIT_EBENE_Y = 0.0
