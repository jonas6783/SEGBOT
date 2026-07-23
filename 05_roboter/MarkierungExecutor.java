package application;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.inject.Inject;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

/**
 * MarkierungExecutor — Stufe 5 der Pipeline.
 *
 * Faehrt die Bahn aus robot_path.txt ab (erzeugt von Stufe 4,
 * bahnplanung.py). Die Datei liegt als Classpath-Ressource im
 * src-Ordner des Sunrise-Projekts.
 *
 * Aufbau der Datei:
 *   Kopfzeilen  "# SCHLUESSEL=WERT"  (u. a. die Geschwindigkeit je
 *               Phase, z. B. V_CONTACT=10.0)
 *   Posenzeilen "PHASE;X;Y;Z;A;B;C"  (mm / Grad, Dezimalpunkt)
 *
 * Sicherheit:
 *   - EINZELSCHRITT = true: jede Pose wird per Dialog freigegeben
 *     ("Rest ohne Nachfrage" schaltet auf Durchlauf um).
 *   - Erste Pose per PTP mit reduzierter Achsgeschwindigkeit, danach
 *     LIN mit der Bahngeschwindigkeit der jeweiligen Phase.
 */
public class MarkierungExecutor extends RoboticsAPIApplication {

    @Inject
    private LBR robot;

    /** Jede Pose einzeln bestaetigen (Standard: an). */
    private static final boolean EINZELSCHRITT = true;
    /** Name der Bahndatei im src-Ordner. */
    private static final String RESSOURCE = "/robot_path.txt";
    /** Achsgeschwindigkeit (relativ) fuer die erste PTP-Anfahrt. */
    private static final double PTP_REL = 0.2;

    /** Eine Zeile der Bahndatei: Phase + Zielpose. */
    private static class Pose {
        final String phase;
        final Frame frame;

        Pose(String phase, Frame frame) {
            this.phase = phase;
            this.frame = frame;
        }
    }

    public void run() {
        Map<String, String> kopf = new HashMap<String, String>();
        List<Pose> posen = new ArrayList<Pose>();
        try {
            leseBahn(kopf, posen);
        } catch (Exception e) {
            getLogger().error("robot_path.txt konnte nicht gelesen werden: "
                    + e.getMessage());
            return;
        }
        getLogger().info("Bahn geladen: " + posen.size() + " Posen, Teil="
                + wert(kopf, "TEIL", "?") + ", erstellt="
                + wert(kopf, "ERSTELLT", "?"));

        // Geschwindigkeit je Phase aus dem Kopf (mm/s):
        Map<String, Double> tempo = new HashMap<String, Double>();
        String[] phasen = { "TRANSFER", "APPROACH", "INFEED", "CONTACT",
                "RETRACT" };
        for (int i = 0; i < phasen.length; i++) {
            tempo.put(phasen[i],
                    Double.valueOf(wert(kopf, "V_" + phasen[i], "20.0")));
        }

        boolean einzelschritt = EINZELSCHRITT;
        for (int i = 0; i < posen.size(); i++) {
            Pose p = posen.get(i);
            if (einzelschritt) {
                int w = getApplicationUI().displayModalDialog(
                        ApplicationDialogType.QUESTION,
                        "Pose " + (i + 1) + " / " + posen.size() + "  ("
                                + p.phase + ")\nAnfahren?",
                        "Weiter", "Rest ohne Nachfrage", "Abbrechen");
                if (w == 1) {
                    einzelschritt = false;
                } else if (w == 2) {
                    getLogger().info("Abbruch durch Bediener bei Pose "
                            + (i + 1) + ".");
                    return;
                }
            }
            double v = tempo.get(p.phase).doubleValue();
            getLogger().info("Pose " + (i + 1) + "/" + posen.size() + "  "
                    + p.phase + "  v=" + v + " mm/s");
            if (i == 0) {
                robot.move(ptp(p.frame).setJointVelocityRel(PTP_REL));
            } else {
                robot.move(lin(p.frame).setCartVelocity(v));
            }
        }
        getLogger().info("Bahn vollstaendig abgefahren ("
+ "Markierung).");
    }

    /** Liest Kopfzeilen und Posen aus der Classpath-Ressource. */
    private void leseBahn(Map<String, String> kopf, List<Pose> posen)
            throws Exception {
        InputStream in = getClass().getResourceAsStream(RESSOURCE);
        if (in == null) {
            throw new Exception("Ressource " + RESSOURCE + " fehlt — Datei "
                    + "in den src-Ordner legen und Projekt synchronisieren.");
        }
        BufferedReader r = new BufferedReader(
                new InputStreamReader(in, "UTF-8"));
        try {
            String zeile;
            while ((zeile = r.readLine()) != null) {
                zeile = zeile.trim();
                if (zeile.length() == 0) {
                    continue;
                }
                if (zeile.startsWith("#")) {
                    int gleich = zeile.indexOf('=');
                    if (gleich > 0) {
                        kopf.put(zeile.substring(1, gleich).trim(),
                                zeile.substring(gleich + 1).trim());
                    }
                    continue;
                }
                // PHASE;X;Y;Z;A;B;C  — Double.parseDouble erwartet immer
                // den Dezimalpunkt, unabhaengig von der Systemsprache.
                String[] t = zeile.split(";");
                if (t.length != 7) {
                    throw new Exception("Ungueltige Posenzeile: " + zeile);
                }
                Frame f = new Frame(
                        Double.parseDouble(t[1]),
                        Double.parseDouble(t[2]),
                        Double.parseDouble(t[3]),
                        Math.toRadians(Double.parseDouble(t[4])),
                        Math.toRadians(Double.parseDouble(t[5])),
                        Math.toRadians(Double.parseDouble(t[6])));
                posen.add(new Pose(t[0], f));
            }
        } finally {
            r.close();
        }
        if (posen.isEmpty()) {
            throw new Exception("Keine Posen in " + RESSOURCE + " gefunden.");
        }
    }

    private static String wert(Map<String, String> kopf, String schluessel,
            String standard) {
        String v = kopf.get(schluessel);
        return v != null ? v : standard;
    }
}
