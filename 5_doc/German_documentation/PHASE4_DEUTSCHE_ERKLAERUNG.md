# 🎓 PHASE 4 - DEUTSCHE ERKLÄRUNG: Motion Control Node

**Zielgruppe:** Du als Entwickler  
**Zweck:** Verstehen, wie State Machine + Action Server zusammenspielen  
**Sprache:** 100% Deutsch  

---

## 📖 Was ist neu in Phase 4?

Phase 3 hat **geplant**. Phase 4 **führt aus**.

```
Phase 3: "Hier ist der Plan: 42 Punkte, 41 Bewegungen, 150 Sekunden"
Phase 4: "OK, ich bohre jetzt Punkt 1... Punkt 2... Punkt 3..."
```

### **Warum ROS2 Action statt Service?**

Das ist die wichtigste Design-Entscheidung in Phase 4:

| | **Service** (Phase 3) | **Action** (Phase 4) |
|---|---|---|
| **Dauer** | Kurz (150ms) | Lang (~3 Minuten) |
| **Feedback** | Nein, nur Endergebnis | Ja, laufende Updates |
| **Abbrechbar** | Nein | Ja (Notaus!) |
| **Pattern** | Request → Response | Goal → Feedback×N → Result |

Ein Bohrvorgang dauert Minuten und muss **jederzeit abbrechbar** sein (Sicherheit!). Dafür ist ein Service ungeeignet - eine Action genau richtig.

---

## 🏗️ Die zwei State Machines

### **1. Top-Level: MotionState**

```cpp
enum class MotionState {
    IDLE,                 // Wartet auf Auftrag
    PLANNING_TRAJECTORY,  // Fragt Phase 3 Service an
    EXECUTING,            // Bohrt gerade
    COMPLETED,            // Fertig!
    ERROR                 // Etwas ging schief
};
```

```
IDLE ──(Goal erhalten)──> PLANNING_TRAJECTORY
                                  │
                        (Plan gültig?)
                           │           │
                          Ja          Nein
                           │           │
                           ▼           ▼
                       EXECUTING     ERROR ──> IDLE
                           │
                   (alle Punkte fertig?)
                           │           │
                          Ja          Nein (Fehler/Abbruch)
                           │           │
                           ▼           ▼
                       COMPLETED     ERROR
                           │           │
                           └─────┬─────┘
                                 ▼
                               IDLE (bereit für nächsten Auftrag)
```

### **2. Sub-State: ExecutionPhase (pro Bohrpunkt)**

```cpp
enum class ExecutionPhase {
    TRAVELING,    // Fährt zum nächsten Punkt
    APPROACHING,  // Positioniert sich fein über dem Loch
    DRILLING,     // Bohrt gerade
    RETRACTING    // Zieht Werkzeug zurück
};
```

```
Für JEDEN der 42 Punkte wiederholt sich:

TRAVELING (von vorherigem Punkt)
    ↓
APPROACHING (2.0s - Standard-Timing)
    ↓
DRILLING (1.5s)
    ↓
RETRACTING (1.0s)
    ↓
[nächster Punkt, oder fertig]
```

**Warum zwei State Machines?** Die äußere (`MotionState`) beschreibt "wo stehen wir im großen Ganzen", die innere (`ExecutionPhase`) beschreibt "was passiert gerade bei diesem einen Punkt". Das hält den Code übersichtlich.

---

## 🔧 Die wichtigsten Funktionen erklärt

### **`execute()` - Der Dirigent**

Das ist die Hauptfunktion, die in einem **eigenen Thread** läuft (wichtig! Nicht im ROS2-Executor-Thread, sonst würde alles blockieren):

```cpp
void MotionControlNode::execute(goal_handle)
{
    set_state(PLANNING_TRAJECTORY);
    
    // Schritt 1: Plan validieren (ruft Phase 3 Service auf)
    if (!request_trajectory_plan(error)) {
        // Fehler → ERROR state, Goal "abort"
        return;
    }
    
    set_state(EXECUTING);
    
    // Schritt 2: Alle 42 Punkte abarbeiten
    bool success = execute_point_sequence(goal_handle, points_completed);
    
    // Schritt 3: Ergebnis zurückmelden
    if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);   // Benutzer hat abgebrochen
    } else if (success) {
        goal_handle->succeed(result);    // Alles geklappt!
    } else {
        goal_handle->abort(result);      // Fehler aufgetreten
    }
    
    set_state(IDLE);  // Bereit für nächsten Auftrag
}
```

### **`request_trajectory_plan()` - Verbindung zu Phase 3**

Hier zeigt sich die Integration zwischen Phase 3 und Phase 4:

```cpp
bool MotionControlNode::request_trajectory_plan(error_message)
{
    // (1) Warte bis Path Planning Service verfügbar ist
    plan_client_->wait_for_service(5s);
    
    // (2) Rufe /path_planning/plan auf (wie in Phase 3 getestet!)
    auto future = plan_client_->async_send_request(request);
    auto response = future.get();  // Warte auf Antwort
    
    if (!response->success) {
        error_message = "Path Planning failed";
        return false;
    }
    
    // (3) WICHTIG: Regeneriere Punkte LOKAL
    geometry_calculator_.generate_boring_points(geometry_params_);
    
    return true;
}
```

**Warum regeneriert Motion Control die Punkte nochmal lokal, statt die vom Service zu nehmen?**

Das ist eine bewusste Design-Entscheidung:
- Phase 3's Service-Antwort enthält **alle 1640 Waypoints als flaches Array** (gut für Visualisierung, z.B. RViz)
- Motion Control braucht aber die **42 einzelnen Bohrpunkte** als strukturierte `Point3D`-Liste (für die Drilling-Zyklen)
- Da `GeometryCalculator::generate_boring_points()` **deterministisch** ist (gleiche Parameter → immer gleiche Punkte), kann Motion Control sie einfach selbst nochmal berechnen
- Der Service-Call bleibt trotzdem sinnvoll: Er **validiert**, dass der Plan überhaupt machbar ist (alle Trajectories innerhalb der Limits), bevor wir anfangen zu bohren

```
Service-Call (Phase 3)  →  "Ist der Plan überhaupt gültig?" (Pre-Flight-Check)
Lokale Regenerierung    →  "Welche konkreten Punkte bohre ich?" (Execution-Daten)
```

### **`execute_point_sequence()` - Die Haupt-Schleife**

```cpp
bool MotionControlNode::execute_point_sequence(goal_handle, points_completed)
{
    const auto& boring_points = geometry_calculator_.get_boring_points();  // 42 Punkte
    
    for (size_t i = 0; i < boring_points.size(); ++i) {
        
        // Abbruch-Check VOR jedem Punkt
        if (goal_handle->is_canceling()) return false;
        
        // TRAVELING: Fahre zum Punkt (außer beim allerersten)
        if (i > 0) {
            Trajectory travel = trajectory_generator_.generate_point_to_point(
                boring_points[i-1], boring_points[i]);   // Phase 2!
            simulate_travel_phase(travel, goal_handle);
        }
        
        // APPROACH → DRILL → RETRACT
        simulate_drilling_cycle(boring_points[i], goal_handle, i, total);
        
        points_completed++;
        
        // Feedback an den Client senden
        publish_feedback(goal_handle, i, total, boring_points[i], elapsed);
    }
    
    return true;
}
```

**Das Schöne:** Hier wird **Phase 2 (TrajectoryGenerator)** direkt wiederverwendet, genau wie in Phase 3 - aber diesmal nicht für die Planung, sondern um die Bewegung **zeitlich korrekt zu simulieren** (jeder der ~40 Waypoints pro Bewegung wird mit der echten 50ms-Taktung durchlaufen).

### **`simulate_drilling_cycle()` - Eine einzelne Bohrung**

```cpp
bool MotionControlNode::simulate_drilling_cycle(point, goal_handle, idx, total)
{
    set_phase(APPROACHING);
    publish_status("Point 5/42: approaching drill position");
    sleep(approach_duration_s_);    // Standard: 2.0s
    if (goal_handle->is_canceling()) return false;  // Abbruch-Check!
    
    set_phase(DRILLING);
    publish_status("Point 5/42: drilling");
    sleep(drilling_duration_s_);    // Standard: 1.5s
    if (goal_handle->is_canceling()) return false;
    
    set_phase(RETRACTING);
    publish_status("Point 5/42: retracting tool");
    sleep(retract_duration_s_);     // Standard: 1.0s
    if (goal_handle->is_canceling()) return false;
    
    return true;
}
```

**Wichtig:** Nach **jeder** Phase wird geprüft, ob abgebrochen werden soll. So reagiert das System innerhalb von maximal 2 Sekunden auf einen Notaus-Befehl, statt erst nach dem kompletten Zyklus.

---

## 🧵 Threading - Ein wichtiges Detail

### **Warum ein separater Thread für `execute()`?**

```cpp
void MotionControlNode::handle_accepted(goal_handle)
{
    std::thread{
        std::bind(&MotionControlNode::execute, this, _1),
        goal_handle
    }.detach();   // ← Läuft im Hintergrund weiter
}
```

**Das Problem ohne separaten Thread:**
```
ROS2 Executor Thread
    │
    ├─ Verarbeitet Service-Calls
    ├─ Verarbeitet Topic-Nachrichten
    ├─ Verarbeitet Action-Goals
    │
    └─ Wenn execute() hier direkt liefe:
       └─ 3 Minuten Bohren = Executor komplett blockiert
          └─ Keine Cancel-Requests kommen mehr durch!
          └─ Keine Service-Antworten können verarbeitet werden!
```

**Mit separatem Thread:**
```
ROS2 Executor Thread (MultiThreadedExecutor)
    │
    ├─ Bleibt frei für: Cancel-Requests, neue Anfragen, Service-Antworten
    │
    └─ execute() läuft parallel in eigenem std::thread
       └─ Kann 3 Minuten dauern, blockiert aber nichts
       └─ goal_handle->is_canceling() wird regelmäßig geprüft
```

**Warum `MultiThreadedExecutor` statt `SingleThreadedExecutor`?**

In `request_trajectory_plan()` blockieren wir mit `future.wait_for(10s)` auf die Service-Antwort. Diese Antwort muss aber vom ROS2-Executor verarbeitet werden - und zwar während unser `execute()`-Thread wartet! Mit `MultiThreadedExecutor` kann der Executor weiterarbeiten (die Service-Antwort verarbeiten), auch während ein anderer Teil des Systems blockiert wartet.

---

## 📡 Live-Feedback verstehen

### **Was der Client während der Ausführung sieht:**

```bash
ros2 action send_goal /motion_control/execute_riveting \
    kuka_riveting/ExecuteRiveting "{}" --feedback
```

**Feedback-Nachrichten (alle paar Sekunden):**
```yaml
feedback:
  current_point_index: 4
  total_points: 42
  current_state: "EXECUTING"
  current_phase: "DRILLING"
  progress_percent: 11.9
  current_position: {x: 100.0, y: 0.0, z: 2100.0}
  elapsed_time_s: 23.4
```

**Das Besondere:** Der Benutzer sieht in Echtzeit:
- Welcher Punkt gerade bearbeitet wird (4 von 42)
- Was gerade genau passiert (DRILLING, nicht nur "EXECUTING")
- Wo sich das Werkzeug befindet (x, y, z)
- Wie viel Prozent schon fertig sind

---

## 🔗 Zusammenspiel Phase 1-4 (Gesamtbild)

```
┌─────────────────────────────────────────────────────┐
│ PHASE 4: Motion Control Node                        │
│                                                       │
│  execute()                                           │
│    ├─ request_trajectory_plan()                      │
│    │    └─ Service Call ──────────┐                  │
│    │                              ▼                  │
│    │                    ┌──────────────────┐         │
│    │                    │ PHASE 3:         │         │
│    │                    │ Path Planning    │         │
│    │                    │ Node             │         │
│    │                    │  ├─ Phase 1      │         │
│    │                    │  │  (Geometry)   │         │
│    │                    │  └─ Phase 2      │         │
│    │                    │     (Trajectory) │         │
│    │                    └──────────────────┘         │
│    │                                                  │
│    └─ execute_point_sequence()                        │
│         ├─ geometry_calculator_ (Phase 1, LOKAL)     │
│         │    └─ generate_boring_points() → 42 Punkte │
│         │                                             │
│         └─ FOR jeden Punkt:                           │
│              ├─ trajectory_generator_ (Phase 2, LOKAL)│
│              │    └─ generate_point_to_point()        │
│              │       → simulate_travel_phase()        │
│              └─ simulate_drilling_cycle()              │
│                   → approach/drill/retract             │
└─────────────────────────────────────────────────────┘
```

**Wichtige Erkenntnis:** Motion Control Node nutzt Phase 1 & 2 **zweimal auf unterschiedliche Weise**:
1. **Indirekt** über den Service-Call zu Phase 3 (zur Validierung)
2. **Direkt** durch eigene Instanzen von `GeometryCalculator` und `TrajectoryGenerator` (für die echte Ausführungssteuerung)

---

## 🚀 Testen der Implementierung

### **Terminal 1: Beide Nodes starten**

```bash
ros2 launch kuka_riveting_system full_system.launch.py
```

### **Terminal 2: Action-Goal senden**

```bash
ros2 action send_goal /motion_control/execute_riveting \
    kuka_riveting/ExecuteRiveting "{}" --feedback
```

**Erwarteter Ablauf:**
```
Goal accepted
Feedback: point 1/42, APPROACHING, 2.4%
Feedback: point 1/42, DRILLING, 2.4%
Feedback: point 2/42, TRAVELING, 4.8%
...
Result: success=true, points_completed=42, total_time=189.3s
```

### **Terminal 3: Status mitverfolgen**

```bash
ros2 topic echo /motion_control/status
```

### **Abbruch testen (Strg+C in Terminal 2, oder):**

```bash
ros2 action cancel /motion_control/execute_riveting
```

---

## ❓ Häufige Fragen

### **F: Was passiert, wenn Path Planning Node nicht läuft?**

A: `request_trajectory_plan()` wartet 5 Sekunden auf den Service, dann schlägt es fehl mit einer klaren Fehlermeldung. Der State geht auf `ERROR`, dann zurück zu `IDLE`. Kein Absturz.

### **F: Kann ich während der Ausführung einen zweiten Auftrag starten?**

A: Nein - `handle_goal()` prüft `current_state_ != IDLE` und lehnt neue Goals ab, solange ein Auftrag läuft. Das verhindert, dass zwei Bohrvorgänge parallel laufen.

### **F: Was, wenn ich die Bohrzeiten für Tests verkürzen will?**

A: In `motion_control_params.yaml` die auskommentierten "Fast testing values" aktivieren (0.2s/0.15s/0.1s statt 2.0s/1.5s/1.0s) - dann dauert der komplette Durchlauf statt ~3 Minuten nur ~20 Sekunden.

### **F: Warum nicht die Waypoints direkt vom Service verwenden?**

A: Siehe oben - Design-Entscheidung für Entkopplung. Könnte man in Phase 5/6 refactoren, falls gewünscht (z.B. Service-Antwort um Marker-Indizes für Bohrpunkte erweitern).

---

## 🎯 Zusammenfassung

| Aspekt | Umsetzung |
|--------|-----------|
| **Kommunikationsmuster** | ROS2 Action Server (statt Service) |
| **State Machine** | 2-stufig: MotionState + ExecutionPhase |
| **Threading** | Separater std::thread für execute(), MultiThreadedExecutor |
| **Phase 3 Integration** | Service-Client zur Plan-Validierung |
| **Phase 1-2 Integration** | Direkte lokale Wiederverwendung für Ausführung |
| **Abbruch-Fähigkeit** | Check nach jeder Phase (max. 2s Reaktionszeit) |
| **Feedback** | Live-Updates: Punkt, Phase, Position, Fortschritt % |

---

**Phase 4 ist fertig implementiert!**  
**Bereit für Phase 5: KUKA Hardware Interface** 🚀
