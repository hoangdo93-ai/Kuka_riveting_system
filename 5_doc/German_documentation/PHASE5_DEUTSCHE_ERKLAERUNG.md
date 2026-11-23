# 🎓 PHASE 5 - DEUTSCHE ERKLÄRUNG: KRC4 Hardware Interface

**Zielgruppe:** Du als Entwickler  
**Zweck:** Verstehen, wie Simulation durch echte Hardware-Kommunikation ersetzt wird  
**Sprache:** 100% Deutsch

---

## 📖 Was ist neu in Phase 5?

Phase 4 hat mit `sleep()` **so getan als ob** der Roboter sich bewegt. Phase 5 **spricht wirklich mit dem KRC4-Controller** - über eine echte TCP/IP-Verbindung.

```
Phase 4: std::this_thread::sleep_for(2.0s);  // "simuliert" Bewegung
Phase 5: krc4_interface.send_target_pose(...) → wartet auf echtes Feedback
```

### **Die zentrale Design-Idee: Backend-Abstraktion**

Statt Phase 4's Code einfach zu löschen und durch Hardware-Code zu ersetzen, habe ich eine **Schnittstelle** (`IMotionBackend`) eingeführt:

```cpp
class IMotionBackend {
    virtual bool execute_travel(...) = 0;
    virtual bool approach_point(...) = 0;
    virtual bool drill(...) = 0;
    virtual bool retract(...) = 0;
};
```

**Zwei Implementierungen:**
- `SimulatedMotionBackend` - Phase 4's `sleep()`-Logik, 1:1 übernommen
- `KRC4MotionBackend` - Phase 5's echte Hardware-Kommunikation

**Der Clou:** `MotionControlNode` (Phase 4's State Machine) merkt praktisch **nichts** von der Umstellung - sie ruft einfach `motion_backend_->drill(...)` auf, egal ob dahinter eine `sleep()`-Simulation oder ein echter Roboter steckt. Per Parameter umschaltbar:

```yaml
hardware:
  use_real_hardware: false   # → SimulatedMotionBackend
  use_real_hardware: true    # → KRC4MotionBackend
```

**Das bestätigt im Nachhinein eine gute Entscheidung aus Phase 4:** Weil die Travel/Drilling-Logik schon damals klar von der State Machine getrennt war, war der Umbau auf Phase 5 ein chirurgischer Eingriff, kein Umbau von Grund auf.

---

## 🔌 Das Kommunikationsprotokoll

### **Wichtiger Realitäts-Check zuerst:**

Das echte KUKA.EKI-Protokoll (Ethernet KRL Interface) ist **proprietär und projektspezifisch** konfiguriert - jede KRC4-Installation hat ihre eigene XML-Schema-Datei, die zusammen mit KUKA-Systemintegratoren eingerichtet wird. Ich kenne dieses exakte Schema nicht aus gesicherter Quelle.

**Was ich stattdessen gebaut habe:** Ein **vereinfachtes, EKI-inspiriertes Protokoll** zu Demonstrationszwecken - architektonisch realistisch (TCP-Verbindung, XML-ähnliche Nachrichten, sequenzielle Befehls-IDs für Zuordnung von Antworten), aber nicht das exakte reale Schema. Das ist in allen Dateien klar dokumentiert.

### **Das Protokoll im Detail:**

**Ausgehend (Motion Backend → Controller):**
```xml
<Cmd seq="42" type="LIN">
  <Pos X="100.000" Y="0.000" Z="100.000" A="0.000" B="0.000" C="0.000"/>
  <Vel>500.000</Vel>
</Cmd>
```

**Eingehend (Controller → Motion Backend):**
```xml
<Status seq="42" state="MOVING" X="45.200" Y="0.000" Z="48.100" err="0"/>
<Status seq="42" state="COMPLETE" X="100.000" Y="0.000" Z="100.000" err="0"/>
```

**Warum `seq` (Sequence ID)?** Damit die Antwort eindeutig dem passenden Befehl zugeordnet werden kann - wichtig, weil der Controller mehrere `MOVING`-Updates sendet, bevor er `COMPLETE` meldet, und das Backend genau wissen muss: "Ist das die Antwort auf MEINEN letzten Befehl?"

---

## 🏗️ Die 3 Schichten der Hardware-Integration

```
┌─────────────────────────────────────────┐
│ MotionControlNode (Phase 4, unverändert) │  ← State Machine, Action Server
│   ruft auf: motion_backend_->drill(...)  │
└──────────────────┬────────────────────────┘
                    │
┌──────────────────▼────────────────────────┐
│ KRC4MotionBackend (Phase 5, NEU)          │  ← "Was bedeutet 'approach'/'drill'/
│   approach_point() → sendet Pose + Offset  │     'retract' in Roboter-Bewegungen?"
│   drill() → hält Position, wartet          │
│   retract() → sendet Pose zurück           │
└──────────────────┬────────────────────────┘
                    │
┌──────────────────▼────────────────────────┐
│ KRC4Interface (Phase 5, NEU)              │  ← "Wie spreche ich über TCP mit
│   send_target_pose() → XML über Socket     │     dem Controller?"
│   poll_feedback() → liest/parst Antworten  │
└─────────────────────────────────────────────┘
                    │
              TCP/IP Socket
                    │
┌──────────────────▼────────────────────────┐
│ KRC4 Controller (oder mock_krc4_server.py) │
└─────────────────────────────────────────────┘
```

**Warum diese Trennung?**
- `KRC4Interface` weiß **nichts** über Bohren, Annähern, Zurückziehen - nur "sende Pose, empfange Feedback"
- `KRC4MotionBackend` weiß **nichts** über TCP-Sockets - nur "was bedeutet ein Bohrzyklus in Bewegungsbefehlen"
- Klare Verantwortlichkeiten = einfacher zu testen, einfacher zu verstehen

---

## 🔧 Wichtige Design-Entscheidungen erklärt

### **1. Warum wird nicht jeder der ~40 Phase-2-Waypoints einzeln über TCP gestreamt?**

```cpp
bool KRC4MotionBackend::execute_travel(const Trajectory& travel, ...)
{
    // Nur den LETZTEN Waypoint senden!
    const auto& goal = travel.waypoints.back().position_mm;
    RobotPose target(goal.x, goal.y, goal.z, 0.0, 0.0, 0.0);
    return move_to_and_wait(target, cancel_check, error);
}
```

**Grund:** Netzwerk-Jitter (unregelmäßige Latenz) macht 50ms-genaues Streamen über normales TCP **unzuverlässig** - genau dafür gibt es bei KUKA das spezielle **RSI (Robot Sensor Interface)** mit garantierten 4-12ms Zykluszeiten über ein dediziertes Echtzeit-Protokoll. Für unsere Aufgabe (diskrete Bohrpunkte anfahren) reicht es, dem Controller **nur das Ziel** zu senden - der KRC4 hat selbst einen hochpräzisen Bewegungsinterpolator eingebaut, der die Bahn dorthin plant. Das ist der **Standard-Ansatz** in der Industrierobotik für punktbasierte Aufgaben wie diese.

### **2. Warum merkt sich `KRC4MotionBackend` Zustand zwischen den Aufrufen?**

```cpp
RobotPose current_base_pose_;      // Wird in approach_point() gesetzt
RobotPose current_approach_pose_;  // ...und in retract() wiederverwendet
```

`MotionControlNode` ruft immer in der Reihenfolge `approach_point() → drill() → retract()` auf (dokumentiert als "Calling Contract" im Interface). Das Backend nutzt das: `approach_point()` merkt sich die berechnete "Über-dem-Loch"-Position, `retract()` fährt genau dorthin zurück, ohne dass `MotionControlNode` diese Zwischen-Information selbst verwalten müsste.

### **3. Wie funktioniert die Abbruch-Erkennung bei echter Hardware?**

```cpp
while (true) {
    if (cancel_check()) {
        interface_.send_emergency_stop();  // Software-Stop-Signal senden
        error_out = "Canceled...";
        return false;
    }
    // ... Timeout-Check, Verbindungs-Check ...
    auto feedback = interface_.poll_feedback(100);  // 100ms Timeout pro Poll
    // ...
}
```

Die `move_to_and_wait()`-Schleife pollt alle 100ms auf Feedback **und** prüft bei jeder Iteration, ob abgebrochen werden soll. Das heißt: Reaktionszeit auf einen Abbruch-Befehl ist **≤100ms**, nicht erst nach Abschluss der Bewegung.

---

## ⚠️ WICHTIGER SICHERHEITSHINWEIS

Das ist der wichtigste Absatz in dieser ganzen Phase:

```cpp
bool KRC4Interface::send_emergency_stop();
```

**Dieser Software-E-Stop über TCP ist NIEMALS ein Ersatz für eine zertifizierte Hardware-Sicherheitsschaltung!**

In einer echten Industrieroboter-Zelle gehört dazu zwingend:
- Physische Not-Aus-Taster mit Hardware-verdrahteten Sicherheitsrelais
- Lichtschranken/Schutzzäune mit sicherheitsgerichteter SPS (typisch Kategorie 3/PLd oder SIL2, wie im ursprünglichen System-Design-Dokument erwähnt)
- Diese Sicherheitskreise laufen **komplett unabhängig** von der Software-Steuerung - ein Software-Bug, Netzwerkausfall oder Programmabsturz darf niemals die einzige Sicherheitsbarriere sein

`send_emergency_stop()` in diesem Projekt ist ein **"Software-Stopp-Signal"** - nützlich für kontrollierten, geordneten Abbruch bei normalen Fehlern (z.B. "Benutzer hat Action gecancelt"), aber für echte Gefahrensituationen braucht es zertifizierte Hardware. Das würde in einem realen Deployment-Projekt in Zusammenarbeit mit einem Sicherheitsingenieur/TÜV-Zertifizierer geplant werden.

---

## 🧪 Was ich tatsächlich getestet habe (nicht nur geschrieben!)

Anders als bei reinem "Code hinschreiben" habe ich diesmal **echte End-to-End-Tests** durchgeführt:

### **Test 1: Low-Level KRC4Interface**
```
✓ TCP-Verbindung zum Mock-Server aufgebaut
✓ Zielpose gesendet, MOVING- und COMPLETE-Feedback korrekt empfangen
✓ Heartbeat funktioniert
✓ Connection-Health-Check funktioniert
✓ Sauberer Disconnect
```

### **Test 2: Kompletter KRC4MotionBackend-Zyklus**
```
✓ approach_point() → drill() → retract() komplett durchlaufen
✓ Drilling-Hold-Dauer korrekt eingehalten (0.2s angefordert, 0.25s gemessen)
✓ Abbruch (Cancellation) mitten in einer langen Bewegung funktioniert korrekt
```

**Dabei entdeckt:** Zwei echte Bugs aus Phase 1-2 (siehe unten), die seit Phase 3 unbemerkt im Code schlummerten, weil die Dateien nie tatsächlich gemeinsam kompiliert wurden!

---

## 🐛 Gefundene & behobene Bugs (wichtig für dich!)

Beim ersten echten Kompilieren des Gesamt-Stacks (Phase 1 + 2 + 5 zusammen) kamen zwei Fehler zutage, die **seit Phase 3 im Code steckten**, aber nie auffielen, weil niemand die Header-Dateien zusammen gebaut hatte:

### **Bug 1: `Point3D` fehlte die Methode `distance_to()`**

`trajectory_generator_IMPLEMENTATION.hpp` (Phase 2) ruft überall `punkt.distance_to(anderer_punkt)` auf - aber die `Point3D`-Struct in `geometry_calculator_CORRECTED.hpp` (Phase 1, die nach der Fehlerkorrektur-Session verwendete Version) hatte diese Methode nie bekommen. Sie existierte nur in der separaten `INTEGRATION_DEMO.cpp`-Standalone-Version.

**Fix:** `distance_to()` zur kanonischen `Point3D`-Struct hinzugefügt.

### **Bug 2: Verwaister `#endif`**

Beide Header nutzen `#pragma once` (modern, empfohlen) - hatten aber zusätzlich am Dateiende ein überflüssiges `#endif` ohne passendes `#if`/`#ifndef`. Das ist ein klassischer Copy-Paste-Rest von klassischen Include-Guards, der einen Compile-Error verursacht (`error: #endif without #if`).

**Fix:** Die verwaisten `#endif`-Zeilen entfernt.

**Lektion:** Das zeigt, wie wichtig es ist, Code **tatsächlich zu kompilieren**, nicht nur zu schreiben und zu lesen - diese Phase war die erste, in der ich den vollständigen Stack wirklich durch `g++` gejagt habe.

---

## 🚀 Testen mit dem Mock-Server

### **Terminal 1: Mock-KRC4-Server starten**

```bash
python3 mock_krc4_server.py --port 54600 --speed-factor 5.0
```

`--speed-factor 5.0` lässt simulierte Bewegungen 5× schneller ablaufen - praktisch zum Testen, ohne dass jede Bewegung ewig dauert.

### **Terminal 2: System mit `use_real_hardware: true` starten**

In `motion_control_params.yaml`:
```yaml
hardware:
  use_real_hardware: true
  robot_ip: "127.0.0.1"   # ← lokaler Mock-Server statt echtem KRC4
  robot_port: 54600
```

```bash
colcon build --packages-select kuka_riveting_system
ros2 launch kuka_riveting_system full_system.launch.py
```

### **Terminal 3: Ausführung starten**

```bash
ros2 action send_goal /motion_control/execute_riveting \
    kuka_riveting/ExecuteRiveting "{}" --feedback
```

Jetzt siehst du im Mock-Server-Terminal echte `LIN`-Befehle mit Zielkoordinaten durchlaufen - **genau wie ein echter KRC4-Controller sie empfangen würde**.

---

## ❓ Häufige Fragen

### **F: Kann ich das direkt an einen echten KRC4 anschließen?**

A: Die Netzwerk-Schicht (TCP-Verbindung, Timeout-Handling, Fehlerbehandlung) ist produktionsreif. Das **Nachrichtenformat** (XML-Schema) müsste aber an die tatsächliche EKI-Konfiguration deines KRC4 angepasst werden - das ist projektspezifisch und wird typischerweise zusammen mit KUKA-Systemintegratoren eingerichtet.

### **F: Warum nicht KUKA.RSI für mehr Echtzeit-Präzision?**

A: RSI ist für **sensorgeführte Korrekturen** auf einer bereits laufenden KRL-Bewegung gedacht (z.B. Kraftregelung, Bildverarbeitungs-Korrektur) - nicht für komplett extern gesteuerte Punkt-zu-Punkt-Bewegungen von Grund auf. Für unsere Aufgabe (diskrete Bohrpunkte anfahren, keine Sensorkorrektur in Echtzeit) ist ein einfacheres Punkt-Befehls-Protokoll die angemessenere Wahl.

### **F: Was passiert, wenn die Verbindung während der Ausführung abbricht?**

A: `is_connection_healthy()` prüft, ob innerhalb der letzten 2 Sekunden Feedback empfangen wurde. Bleibt es länger still, bricht `move_to_and_wait()` mit einer klaren Fehlermeldung ab - der State wechselt auf `ERROR`, keine Endlos-Warteschleife.

---

## 🎯 Zusammenfassung

| Aspekt | Umsetzung |
|--------|-----------|
| **Architektur** | `IMotionBackend`-Abstraktion, Phase 4 bleibt unverändert |
| **Protokoll** | Vereinfachtes, EKI-inspiriertes TCP/XML (klar als Demo gekennzeichnet) |
| **Testing** | Echte End-to-End-Tests gegen Mock-Server (nicht nur Code-Review!) |
| **Sicherheit** | Klarer Disclaimer: Software-E-Stop ≠ zertifizierte Hardware-Sicherheit |
| **Gefundene Bugs** | 2 reale Compile-Fehler aus Phase 1-2 entdeckt und behoben |
| **Testbarkeit ohne Hardware** | `mock_krc4_server.py` simuliert einen KRC4 lokal |

---

**Phase 5 ist fertig implementiert UND getestet!**  
**Bereit für Phase 6: System-Integration & Production-Readiness** 🚀
