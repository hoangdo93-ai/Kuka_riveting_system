# 🛡️ SICHERHEITSARCHITEKTUR: KUKA A380 Riveting System

**Status:** Entwurfsdokument für Portfolio-Zwecke - kein Ersatz für eine reale Risikobeurteilung  
**Scope:** Software-Sicherheitsmassnahmen Phase 1-5, eingeordnet in den Gesamt-Sicherheitsrahmen einer Roboterzelle

---

## ⚠️ Wichtiger Vorbehalt zu Beginn

Dieses Dokument beschreibt, **wie die Software-Schicht dieses Projekts in eine normgerechte Sicherheitsarchitektur eingeordnet würde** - es ist **keine** durchgeführte Risikobeurteilung nach ISO 12100 und **kein** Ersatz für die Unterschrift eines Sicherheitsingenieurs. Für eine reale Inbetriebnahme ist eine vollständige, projektspezifische Risikobeurteilung durch qualifiziertes Fachpersonal zwingend erforderlich.

---

## 📐 Normative Referenzen (Begriffsklärung)

Das ursprüngliche Systemdesign (Phase 1, `01_SYSTEM_DESIGN.md`) verwendet den Begriff **"SIL2"**. Hier eine wichtige fachliche Präzisierung, die in einem Bewerbungsgespräch Substanz zeigt:

| Norm | Begriff | Typischer Einsatzbereich |
|------|---------|---------------------------|
| **IEC 61508 / IEC 62061** | SIL (Safety Integrity Level, 1-4) | Elektrische/elektronische Sicherheits-Steuerungen, Prozessindustrie, teils Maschinenbau |
| **ISO 13849-1** | PL (Performance Level, a-e) | **Standard-Rahmenwerk für Maschinensicherheit** - in der Robotik-Industrie (ISO 10218-1/2) der gebräuchlichere Massstab |
| **ISO 10218-1/2** | Sicherheitsanforderungen für Industrieroboter | Direkt anwendbar auf diese Roboterzelle |

**Näherungsweise Entsprechung** (gängige Querverweistabelle, keine exakte 1:1-Übersetzung): SIL2 ≈ PLd (Kategorie 3). Für dieses Projekt wird daher **PLd nach ISO 13849-1** als das fachlich passendere Zielniveau behandelt, bei Beibehaltung der "SIL2"-Bezeichnung aus der ursprünglichen Planungsphase als informelles Äquivalent.

**Weitere relevante Normen:**
- **ISO 12100** - Allgemeine Gestaltungsleitsätze, Risikobeurteilung
- **ISO 10218-2** - Integration von Robotersystemen und -zellen
- **IEC 60204-1** - Elektrische Ausrüstung von Maschinen (Not-Halt-Kategorien 0/1)
- **IEC 61800-5-2** - Safe Torque Off (STO), Safe Stop 1/2 (SS1/SS2) für Antriebe

---

## 🏗️ Sicherheitsarchitektur: Die drei Schichten

```
┌─────────────────────────────────────────────────────────────┐
│ SCHICHT 0: Zertifizierte Hardware-Sicherheit                │
│ (NICHT Teil dieses Codebases - durch Sicherheitsintegrator) │
│                                                               │
│  • Physische Not-Halt-Taster → sicherheitsgerichtete        │
│    Relais (Kat. 3/PLd oder SIL2 nach IEC 62061)              │
│  • Lichtschranken/Laserscanner mit Muting-Logik              │
│  • Schutzzaun-Verriegelung (Türkontaktschalter)              │
│  • Safe Torque Off (STO) direkt am Antrieb (IEC 61800-5-2)  │
│  • Diese Schicht funktioniert UNABHÄNGIG von jeglicher       │
│    Software - auch bei Totalausfall des Steuerungs-PCs       │
└───────────────────────────┬───────────────────────────────────┘
                             │
┌───────────────────────────▼───────────────────────────────────┐
│ SCHICHT 1: KRC4-Controller-eigene Sicherheit                 │
│ (Teil der KUKA-Systemsoftware - NICHT durch dieses Projekt    │
│  implementiert, sondern vorausgesetzt)                        │
│                                                                │
│  • SafeOperation: Achsbereichs-/Raumüberwachung               │
│  • Geschwindigkeitsbegrenzung in T1/T2-Betriebsarten          │
│  • Interne Diagnose & Fehlerreaktion                          │
└───────────────────────────┬───────────────────────────────────┘
                             │
┌───────────────────────────▼───────────────────────────────────┐
│ SCHICHT 2: Supervisory Software (DIESES PROJEKT, Phase 1-5)  │
│                                                                │
│  • Trajektorien-Validierung gegen Geschwindigkeits-/          │
│    Beschleunigungslimits (Phase 2)                            │
│  • Parameter-Validierung vor Planungsbeginn (Phase 3)         │
│  • Abbruch-Prüfung nach jeder Teilbewegung (Phase 4/5)        │
│  • Verbindungs-Gesundheitsprüfung, Timeouts (Phase 5)         │
│  • Software-Not-Halt-Signal über TCP (Phase 5)                │
│  • Exception-Handling gegen Prozessabsturz (Phase 6-Fix)      │
└─────────────────────────────────────────────────────────────┘
```

**Kernaussage:** Dieses Projekt implementiert **ausschliesslich Schicht 2**. Schicht 0 und 1 sind Voraussetzung, nicht Teil des Lieferumfangs dieses Codebases - das wird im gesamten Projekt konsequent so kommuniziert (siehe bereits Phase 5's Sicherheitshinweis zu `send_emergency_stop()`).

---

## 🔍 Gefahrenanalyse (vereinfachte HAZOP-Betrachtung)

| # | Gefährdung | Mögliche Ursache | Schwere | Schicht(en) der Risikominderung |
|---|------------|-------------------|---------|-----------------------------------|
| H1 | Quetschen/Stoss durch Roboterarm | Person im Arbeitsraum während Bewegung | Schwer (potenziell tödlich) | **Schicht 0** (Lichtschranke stoppt Bewegung physisch) |
| H2 | Stichverletzung durch Niet-/Bohrwerkzeug | Kontakt während Bohrzyklus | Schwer | **Schicht 0** (Zugangsschutz zum Arbeitsraum) |
| H3 | Unerwartete Bewegung nach Netzwerkausfall | TCP-Verbindungsabbruch zwischen Motion Control und KRC4 | Mittel-Schwer | **Schicht 1** (Controller geht in sicheren Zustand bei Kommando-Timeout) + **Schicht 2** (`is_connection_healthy()` erkennt Abbruch, bricht Ausführung ab) |
| H4 | Bahnabweichung durch fehlerhafte Trajektorienberechnung | Softwarefehler in Phase 2 | Mittel | **Schicht 2** (`validate_trajectory()` prüft gegen Limits vor Ausführung) + **Schicht 1** (Controller-interne Achsbereichsüberwachung) |
| H5 | Prozessabsturz während Ausführung (unkontrollierter Stopp) | Unbehandelte Exception im Steuerungs-Prozess | Mittel | **Schicht 2** (systematisches Exception-Handling, siehe Phase 6-Fix) - Roboter bleibt aber mechanisch in Bewegung bis Controller-Timeout greift → **Schicht 1** als Rückfallebene |
| H6 | Verzögerter Nothalt | Software-E-Stop-Signal kommt wegen Netzwerkproblem nicht an | Schwer | **Schicht 0 MUSS unabhängig funktionieren** - genau deshalb ist Software-E-Stop allein nicht ausreichend |
| H7 | Falsche Geometrie-Parameter führen zu Kollision mit Werkstückhalterung | Fehlkonfiguration (z.B. falsche Flap-Masse) | Mittel | **Schicht 2** (Parameter-Validierung, Grenzwert-Prüfung) - deckt aber keine Kollisionen mit realer Umgebungsgeometrie ab (siehe Einschränkungen unten) |

**Wichtige Lücke, explizit benannt:** Dieses System verfügt über **keine Kollisionserkennung mit der realen Umgebung** (keine 3D-Scan-/Vision-Integration). Die Trajektorien-Validierung prüft nur kinematische Limits (Geschwindigkeit/Beschleunigung), nicht geometrische Kollisionsfreiheit mit Werkstückhalterung, Gerüst o.ä. Das wäre in einer realen Zelle zusätzlich durch Offline-Programmier-Simulation (z.B. KUKA.Sim) und/oder Schicht-0/1-Massnahmen abzudecken.

---

## ✅ Was dieser Codebase liefert (Software-Sicherheitsmassnahmen im Detail)

| Massnahme | Implementiert in | Wirkung | Validiert |
|-----------|-------------------|---------|-----------|
| Trajektorien-Limit-Validierung | `TrajectoryGenerator::validate_trajectory()` (Phase 2) | Verwirft Bewegungen, die Geschwindigkeits-/Beschleunigungslimits überschreiten, vor Ausführung | ✅ Unit-getestet (Session 1) |
| Parameter-Validierung | `PathPlanningNode::validate_request_parameters()` (Phase 3) | Verwirft unplausible Geometrie (negative Masse, zu grosse Sicherheitszone etc.) | Code-Review |
| Exception-Robustheit | Try/catch um `generate_boring_points()` (Phase 3 + **Phase 6-Fix** in Phase 5) | Verhindert Prozessabsturz (`std::terminate`) bei degenerierten Eingaben | ✅ Durch E2E-Test in Phase 6 **tatsächlich aufgedeckt und verifiziert behoben** |
| Abbruch-Reaktionszeit (Simulation) | Cancellation-Check nach jeder Sub-Phase (Phase 4) | ≤2s Reaktionszeit auf Action-Cancel | ✅ Integrationstest (Phase 5) |
| Abbruch-Reaktionszeit (Hardware) | 100ms-Poll-Intervall in `move_to_and_wait()` (Phase 5) | ≤100-200ms Reaktionszeit | ✅ Integrationstest (Phase 5) |
| Verbindungs-Gesundheitsprüfung | `KRC4Interface::is_connection_healthy()` (Phase 5) | Erkennt stille Verbindungsabbrüche binnen 2s, bricht Ausführung kontrolliert ab | ✅ Code-Review, Protokoll getestet |
| Motion-Timeout | `motion_timeout_s` Parameter (Phase 5) | Verhindert endloses Warten auf nie eintreffendes Feedback | Code-Review |
| Software-Not-Halt-Signal | `send_emergency_stop()` (Phase 5) | Best-Effort-Stopp-Signal bei Abbruch - **explizit NICHT als alleinige Sicherheitsmassnahme deklariert** | ✅ Protokoll getestet |

---

## ❌ Was dieser Codebase explizit NICHT liefert (Grenzen)

Für Transparenz und um im Gespräch keine Fehlerwartung zu wecken:

- **Keine zertifizierte Hardware-Sicherheitsschaltung** (Schicht 0) - muss separat durch Sicherheitsintegrator realisiert werden
- **Keine Kollisionserkennung mit realer Umgebungsgeometrie** - nur kinematische Limit-Prüfung
- **Keine Redundanz/Diversität in der Software-Schicht** (Single-Point-of-Failure in der Steuerungssoftware ist akzeptabel, WEIL Schicht 0/1 die eigentliche Sicherheitsfunktion tragen - genau das ist der Punkt dieser Architektur)
- **Kein reales KUKA.EKI-Protokoll** - vereinfachtes Demo-Protokoll (siehe Phase 5-Dokumentation)
- **Keine formale Software-Sicherheitszertifizierung** (z.B. nach IEC 61508 Teil 3 für die Software selbst) - dieses Projekt ist ein Portfolio-/Demonstrationsprojekt, kein zertifiziertes Sicherheitsprodukt

---

## 📋 Empfohlener Pfad zu einer realen PLd/SIL2-konformen Zelle

Für eine tatsächliche Inbetriebnahme (ausserhalb des Portfolio-Kontexts) wären folgende Schritte nötig:

1. **Formale Risikobeurteilung nach ISO 12100** durch qualifizierte Sicherheitsfachkraft, projektspezifisch für die reale Zellen-Geometrie
2. **Sicherheits-Anforderungsspezifikation (SRS)** mit konkretem PL-Zielwert pro Sicherheitsfunktion (nicht pauschal "PLd für alles")
3. **Auswahl zertifizierter Komponenten**: Sicherheits-SPS, Lichtschranken/Laserscanner mit passendem PL, Not-Halt-Taster nach ISO 13850
4. **Validierung nach ISO 13849-2** (Messung/Prüfung der realisierten Performance Level)
5. **Dokumentierte Abnahme** mit Unterschrift durch Sicherheitsverantwortlichen vor Produktivbetrieb
6. **Wiederkehrende Prüfungen** gemäss Betriebssicherheitsverordnung/anwendbarem lokalem Recht

Diese Schritte liegen ausserhalb des Umfangs eines Software-Portfolio-Projekts, werden hier aber bewusst benannt, um zu zeigen: *das Projekt versteht die Grenze zwischen Software-Engineering und zertifizierter funktionaler Sicherheit.*

---

## 🎯 Zusammenfassung für das Bewerbungsgespräch

> "Die Software-Schicht, die ich gebaut habe, übernimmt Trajektorien-Validierung, Abbruch-Handling und Verbindungsüberwachung - das sind Schicht-2-Massnahmen im Sinne einer mehrschichtigen Sicherheitsarchitektur. Ich habe bewusst dokumentiert, wo die Grenze zur zertifizierten Hardware-Sicherheit (Schicht 0, z.B. Lichtschranken mit sicherheitsgerichteter SPS) verläuft, und dass ein Software-Not-Halt-Signal über TCP niemals diese Hardware-Schicht ersetzen darf. Bei der Fehlersuche in Phase 6 habe ich zudem eine unbehandelte Exception gefunden, die den gesamten Steuerungsprozess hätte abstürzen lassen können - genau die Art von Robustheitslücke, die in einer Sicherheitsarchitektur systematisch gesucht werden muss."

---

**Dokument-Status:** Portfolio-Sicherheitskonzept, Phase 6  
**Nächster Bezug:** DEPLOYMENT_CHECKLIST.md (operative Inbetriebnahme-Schritte)
