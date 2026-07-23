# Schritt 5 — Ausführung auf dem Roboter

`MarkierungExecutor.java` läuft auf dem KUKA LBR iiwa unter Sunrise.OS
(Sunrise Workbench 1.14, Java 1.6).

In diesem Ordner liegt neben dem Executor auch die
`robot_path.txt` unseres Stands (41 Posen) — erzeugt aus den
beiliegenden Projektergebnissen in Schritt 4
(`python bahnplanung.py` reproduziert sie).

So kommt die Bahn auf den Roboter:

1. Die Java-Datei in das Sunrise-Projekt übernehmen (Paketname oben in
   der Datei ggf. an euer Projekt anpassen, Standard: `application`).
2. Die `robot_path.txt` aus Schritt 4 in den **`src`-Ordner** des
   Sunrise-Projekts kopieren und synchronisieren — der Executor liest
   sie als Classpath-Ressource
   (`getClass().getResourceAsStream("/robot_path.txt")`).
3. Anwendung starten. Der Executor liest den Dateikopf (u. a. Geschwindigkeit
   je Bewegungsphase), fährt die erste Pose
   per PTP mit reduzierter Geschwindigkeit an und alle weiteren per LIN.

Sicherheit: Standardmäßig ist die **Einzelschritt-Bestätigung** aktiv
(jede Pose wird per Dialog freigegeben; „Rest ohne Nachfrage" schaltet
auf Durchlauf).

Hinweis: Die Datei wurde außerhalb der Sunrise-Umgebung geschrieben und
per Parser auf Syntax und Java-1.6-Verträglichkeit geprüft — beim
Übernehmen einmal im Sunrise-Projekt bauen, da die KUKA-API hier nicht
gegenkompiliert werden konnte.
