# Trajectory Generator Implementierung - Vollständige Anleitung

**Modul:** Motion Planning & Control  
**Datei:** trajectory_generator.hpp  
**Sprache:** C++17  
**ROS2:** Humble  
**Status:** Production-Ready Implementation

---

## 📋 Inhaltsverzeichnis

1. [Mathematische Grundlagen](#mathematische-grundlagen)
2. [Algorithmus-Design](#algorithmus-design)
3. [Datenstrukturen & Schnittstellen](#datenstrukturen--schnittstellen)
4. [Implementierung Schritt-für-Schritt](#implementierung-schritt-für-schritt)
5. [Validierung & Testing](#validierung--testing)
6. [Integration mit Geometry Calculator](#integration-mit-geometry-calculator)

---

## 🎯 Ziel des Trajectory Generators

**Aufgabe:** Generiere sichere, zeitoptimale Bewegungen zwischen zwei Bohrpunkten

**Eingabe:**
- Start-Position (Point3D) - wo der Roboter gerade ist
- Goal-Position (Point3D) - wohin der Roboter soll
- Bewegungsparameter (max Geschwindigkeit, Beschleunigung)
- Zeit-Beschränkung (optional)

**Ausgabe:**
- Trajektorie mit Waypoints (Position, Geschwindigkeit, Beschleunigung pro Zeitschritt)
- Validierung gegen Grenzen
- Performance-Metriken

---

## 📐 Mathematische Grundlagen

### **1. Cubic Polynomial Motion Profile**

#### Problem:
Wie bewegen wir den Roboter von Position A nach Position B, ohne ruckartig zu werden?

#### Lösung: Kubisches Polynom

```
q(t) = q₀ + 3(qf - q₀)(t/T)² - 2(qf - q₀)(t/T)³

wobei:
  q₀ = Start-Position
  qf = End-Position
  t = aktuelle Zeit [0, T]
  T = Gesamtdauer
  τ = t/T = normalisierte Zeit [0, 1]
```

#### Eigenschaften:

```
✅ q(0) = q₀          (Start-Position)
✅ q(T) = qf          (End-Position)
✅ dq/dt(0) = 0       (Start-Geschwindigkeit = 0)
✅ dq/dt(T) = 0       (End-Geschwindigkeit = 0)
✅ d²q/dt²(0) ≠ 0     (Beschleunigung ist möglich)
✅ d²q/dt²(T) ≠ 0     (Beschleunigung ist möglich)

⚠️  d³q/dt³ = konstant (Ruckbegrenzung)
```

#### Visuell - "Bell-Shaped" Geschwindigkeitsprofil:

```
     v(t)
      ▲
      │      ╱╲
      │     ╱  ╲
      │    ╱    ╲
  v_max├───      ───
      │
      │
      └─────────────── t
      0       T/2      T

Start → Beschleunigung → Konstante Geschwindigkeit → Verzögerung → Stop
```

### **2. Geschwindigkeit des Kubischen Polynoms**

Erste Ableitung:

```
dq/dt = d/dt[q₀ + 3(qf-q₀)τ² - 2(qf-q₀)τ³]

      = 3(qf-q₀) · d(τ²)/dt - 2(qf-q₀) · d(τ³)/dt
      
      = 3(qf-q₀) · 2τ · dτ/dt - 2(qf-q₀) · 3τ² · dτ/dt
      
      = [6(qf-q₀)τ - 6(qf-q₀)τ²] · (1/T)
      
      = 6(qf-q₀)/T · (τ - τ²)

Vereinfacht:
v(t) = 6(qf-q₀)/T² · t - 6(qf-q₀)/T³ · t²
```

**Eigenschaften:**
- Maximum bei t = T/2
- v(0) = 0
- v(T) = 0

### **3. Beschleunigung des Kubischen Polynoms**

Zweite Ableitung:

```
d²q/dt² = d/dt[6(qf-q₀)/T² · t - 6(qf-q₀)/T³ · t²]
        = 6(qf-q₀)/T² - 12(qf-q₀)/T³ · t

a(t) = 6(qf-q₀)/T² - 12(qf-q₀)/T³ · t
```

**Eigenschaften:**
- Ändert sich linear mit t
- a(0) = 6(qf-q₀)/T² (positive Beschleunigung)
- a(T) = -6(qf-q₀)/T² (negative Beschleunigung = Verzögerung)
- a(T/2) = 0 (in der Mitte)

---

## 🔧 Algorithmus-Design

### **Ablauf der Trajectory Generation**

```
┌─────────────────────────────────────────────┐
│ INPUT: Start, Goal, MotionProfile           │
└──────────────┬──────────────────────────────┘
               │
               ▼
    ┌──────────────────────────┐
    │ 1. BERECHNE DISTANZ      │
    │    distance = Start      │
    │            .distance_to  │
    │            (Goal)        │
    └──────────────┬───────────┘
                   │
                   ▼
    ┌──────────────────────────────────────┐
    │ 2. BERECHNE AUSFÜHRUNGSZEIT          │
    │    Falls nicht vorgegeben:           │
    │    T = f(distance, v_max, a_max)     │
    │                                      │
    │    Dreiecks-Profil (nur Accel):      │
    │    T = 2 * √(distance / a_max)       │
    │                                      │
    │    Trapez-Profil (Accel+Const+Decel)│
    │    T = 2 * a_max + (dist - 2*d_accel)/v_max
    └──────────────┬───────────────────────┘
                   │
                   ▼
    ┌──────────────────────────────────────┐
    │ 3. GENERIERE WAYPOINTS               │
    │    FOR t = 0 TO T STEP dt:           │
    │      τ = t/T                         │
    │      q(t) = q₀ + ...                 │
    │      v(t) = 6(qf-q₀)/T² * (τ - τ²)  │
    │      a(t) = 6(qf-q₀)/T² - ...        │
    │      Speichere Waypoint              │
    └──────────────┬───────────────────────┘
                   │
                   ▼
    ┌──────────────────────────────────────┐
    │ 4. VALIDIERE TRAJEKTORIE             │
    │    FOR each waypoint:                │
    │      IF v > v_max:  FEHLER!          │
    │      IF a > a_max:  FEHLER!          │
    │      IF pos out of bounds: FEHLER!   │
    └──────────────┬───────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────┐
│ OUTPUT: Trajectory mit Waypoints            │
│         Duration, MaxVel, MaxAccel, Valid   │
└─────────────────────────────────────────────┘
```

---

## 📦 Datenstrukturen & Schnittstellen

### **1. MotionProfile Struktur**

```cpp
struct MotionProfile {
    // Geschwindigkeitslimits
    double max_linear_velocity_mms;       // [mm/s]
    double max_joint_velocity_degs;       // [deg/s]
    
    // Beschleunigungslimits
    double max_linear_acceleration_mms2;  // [mm/s²]
    double max_angular_acceleration_degs2; // [deg/s²]
    
    // Sicherheit
    double jerk_limit_mms3;               // [mm/s³]
    double collision_margin_mm;           // [mm]
    
    // Diskretisierung
    double time_step_ms;                  // [ms] Abtastrate
};
```

**Beispiel - Standard-Profil (konservativ):**
```cpp
MotionProfile profile;
profile.max_linear_velocity_mms = 500.0;         // 0.5 m/s
profile.max_linear_acceleration_mms2 = 2000.0;   // 2 m/s²
profile.time_step_ms = 50.0;                     // 20Hz
```

**Beispiel - Schnelles Profil:**
```cpp
MotionProfile fast_profile;
fast_profile.max_linear_velocity_mms = 800.0;    // 0.8 m/s
fast_profile.max_linear_acceleration_mms2 = 3500.0;
fast_profile.time_step_ms = 50.0;
```

### **2. Waypoint Struktur**

```cpp
struct Trajectory::Waypoint {
    double time_s;              // Zeitstempel [s]
    Point3D position_mm;        // (x, y, z) Position [mm]
    Point3D velocity_mms;       // (vx, vy, vz) [mm/s]
    Point3D acceleration_mms2;  // (ax, ay, az) [mm/s²]
};
```

**Beispiel:**
```
Waypoint 0:  t=0.00s,  pos=(100, 0, 100),    vel=(0, 0, 0),      acc=(200, 0, 200)
Waypoint 1:  t=0.05s,  pos=(102.5, 0, 102.5), vel=(50, 0, 50),    acc=(190, 0, 190)
Waypoint 2:  t=0.10s,  pos=(107.4, 0, 107.4), vel=(99, 0, 99),    acc=(160, 0, 160)
...
```

### **3. Trajectory Struktur**

```cpp
struct Trajectory {
    vector<Waypoint> waypoints;    // Alle Zwischenpunkte
    Point3D start;                 // Start-Position
    Point3D goal;                  // End-Position
    double duration_s;             // Gesamtdauer [s]
    double max_velocity_mms;       // Peak-Geschwindigkeit [mm/s]
    double max_acceleration_mms2;  // Peak-Beschleunigung [mm/s²]
    
    bool is_valid() const;         // Validierungsprüfung
};
```

---

## 🔨 Implementierung Schritt-für-Schritt

### **SCHRITT 1: Cubic Polynomial Berechnung**

```cpp
/**
 * @brief Kubisches Polynom: q(t) = q₀ + 3(qf-q₀)τ² - 2(qf-q₀)τ³
 * 
 * @param q0 Start-Position
 * @param qf End-Position
 * @param t Aktuelle Zeit [0, T]
 * @param T Gesamtdauer
 * @return Position zum Zeitpunkt t
 */
static double cubic_polynomial(double q0, double qf, double t, double T)
{
    // τ = t/T (normalisierte Zeit, 0 ≤ τ ≤ 1)
    double tau = t / T;
    
    // q(τ) = q₀ + 3(qf - q₀)τ² - 2(qf - q₀)τ³
    return q0 + 3.0 * (qf - q0) * tau * tau 
             - 2.0 * (qf - q0) * tau * tau * tau;
}

// BEISPIEL:
// q0 = 100 mm (Start)
// qf = 1100 mm (End)
// T = 2.0 s
// t = 1.0 s (Mitte der Bewegung)

// τ = 1.0 / 2.0 = 0.5
// q(0.5) = 100 + 3×1000×0.5² - 2×1000×0.5³
//        = 100 + 3×1000×0.25 - 2×1000×0.125
//        = 100 + 750 - 250
//        = 600 mm ✅
```

### **SCHRITT 2: Geschwindigkeit Berechnung**

```cpp
/**
 * @brief Erste Ableitung (Geschwindigkeit)
 * dq/dt = 6(qf-q₀)/T² × (t/T) - 6(qf-q₀)/T³ × (t/T)²
 */
static double cubic_polynomial_velocity(double q0, double qf, double t, double T)
{
    double tau = t / T;
    double coeff = (qf - q0) / (T * T);
    
    // v(t) = 6×coeff×τ - 6×coeff×τ²
    return 6.0 * coeff * tau - 6.0 * coeff * tau * tau;
}

// BEISPIEL mit denselben Werten:
// coeff = (1100 - 100) / (2.0 × 2.0) = 1000 / 4 = 250
// τ = 0.5
// v(0.5) = 6×250×0.5 - 6×250×0.5²
//        = 750 - 375
//        = 375 mm/s ✅ (Maximum bei τ=0.5)
```

### **SCHRITT 3: Beschleunigung Berechnung**

```cpp
/**
 * @brief Zweite Ableitung (Beschleunigung)
 * d²q/dt² = 6(qf-q₀)/T² - 12(qf-q₀)/T³ × (t/T)
 */
static double cubic_polynomial_acceleration(double q0, double qf, double t, double T)
{
    double coeff = (qf - q0) / (T * T);
    double tau = t / T;
    
    // a(t) = 6×coeff - 12×coeff×τ
    return 6.0 * coeff - 12.0 * coeff * tau;
}

// BEISPIEL:
// coeff = 250
// τ = 0.5
// a(0.5) = 6×250 - 12×250×0.5
//        = 1500 - 1500
//        = 0 mm/s² ✅ (Keine Beschleunigung in der Mitte)
```

### **SCHRITT 4: Minimale Ausführungszeit Berechnen**

```cpp
/**
 * @brief Berechnet minimale Ausführungszeit unter Berücksichtigung
 *        von Geschwindigkeit und Beschleunigungslimits
 * 
 * Zwei Fälle:
 * 1. Dreiecks-Profil: Nur Beschleunigung, kein konstantes v
 * 2. Trapez-Profil: Beschleunigung → Konstante v → Verzögerung
 */
double calculate_minimum_time(double distance, double v_max, double a_max)
{
    // SCHRITT 1: Dreiecks-Profil Berechnung
    // Wenn distance = 0.5 × a × T², dann T = √(distance / a_max)
    double t_triangle = std::sqrt(distance / a_max);
    
    // SCHRITT 2: Prüfe ob Dreiecks-Profil ausreichend ist
    // Wenn Peak-Geschwindigkeit ≤ v_max, verwende Dreiecks-Profil
    if (2 * v_max >= a_max * t_triangle) {
        return 2.0 * t_triangle;  // Dreieck ausreichend
    }
    
    // SCHRITT 3: Trapez-Profil notwendig
    double t_accel = v_max / a_max;  // Zeit zum Beschleunigen
    
    // Distanz während Beschleunigung: d = 0.5 × a × t²
    double distance_accel = 0.5 * a_max * t_accel * t_accel;
    
    if (distance <= 2.0 * distance_accel) {
        // Distanz zu kurz, Dreiecks-Profil genug
        return 2.0 * t_triangle;
    } else {
        // Trapez-Profil: Accel + Const + Decel
        double distance_const_vel = distance - 2.0 * distance_accel;
        double t_const_vel = distance_const_vel / v_max;
        return 2.0 * t_accel + t_const_vel;
    }
}

// BEISPIEL:
// distance = 1000 mm (1 m)
// v_max = 500 mm/s
// a_max = 2000 mm/s²

// t_triangle = √(1000 / 2000) = √0.5 = 0.707 s
// 2 × v_max = 1000 mm/s
// a_max × t_triangle = 2000 × 0.707 = 1414 mm/s
// 1000 < 1414 ✓ → Dreiecks-Profil ausreichend
// T = 2 × 0.707 = 1.414 s
```

### **SCHRITT 5: Waypoints Generieren (Nested Loop)**

```cpp
Trajectory generate_point_to_point(const Point3D& start, 
                                    const Point3D& goal,
                                    double duration = 0.0)
{
    Trajectory traj;
    traj.start = start;
    traj.goal = goal;
    
    // Berechne Strecke
    double distance = start.distance_to(goal);
    
    // Berechne oder verwende vorgegebene Dauer
    if (duration <= 0.0) {
        duration = calculate_minimum_time(distance, v_max, a_max);
    }
    traj.duration_s = duration;
    
    // Generiere Waypoints
    double dt = time_step_ms / 1000.0;  // z.B. 0.05 s
    int num_steps = static_cast<int>(duration / dt) + 1;
    
    for (int i = 0; i < num_steps; ++i) {
        double t = std::min(static_cast<double>(i) * dt, duration);
        
        Waypoint wp;
        wp.time_s = t;
        
        // Position für jede Achse
        wp.position_mm.x = cubic_polynomial(start.x, goal.x, t, duration);
        wp.position_mm.y = cubic_polynomial(start.y, goal.y, t, duration);
        wp.position_mm.z = cubic_polynomial(start.z, goal.z, t, duration);
        
        // Geschwindigkeit für jede Achse
        wp.velocity_mms.x = cubic_polynomial_velocity(start.x, goal.x, t, duration);
        wp.velocity_mms.y = cubic_polynomial_velocity(start.y, goal.y, t, duration);
        wp.velocity_mms.z = cubic_polynomial_velocity(start.z, goal.z, t, duration);
        
        // Beschleunigung für jede Achse
        wp.acceleration_mms2.x = cubic_polynomial_acceleration(start.x, goal.x, t, duration);
        wp.acceleration_mms2.y = cubic_polynomial_acceleration(start.y, goal.y, t, duration);
        wp.acceleration_mms2.z = cubic_polynomial_acceleration(start.z, goal.z, t, duration);
        
        // Tracking für Peak-Werte
        double vel_mag = wp.velocity_mms.distance_to(Point3D(0, 0, 0));
        traj.max_velocity_mms = std::max(traj.max_velocity_mms, vel_mag);
        
        double accel_mag = wp.acceleration_mms2.distance_to(Point3D(0, 0, 0));
        traj.max_acceleration_mms2 = std::max(traj.max_acceleration_mms2, accel_mag);
        
        traj.waypoints.push_back(wp);
    }
    
    return traj;
}

// BEISPIEL-AUSGABE mit dt=0.05s und T=2.0s:
// Waypoint  0: t=0.00s, pos=(100.00, 0, 100), vel=(0, 0, 0),       acc=(300, 0, 300)
// Waypoint  1: t=0.05s, pos=(100.75, 0, 100.75), vel=(29.8, 0, 29.8), acc=(298, 0, 298)
// Waypoint  2: t=0.10s, pos=(102.98, 0, 102.98), vel=(59.1, 0, 59.1), acc=(291, 0, 291)
// ...
// Waypoint 20: t=1.00s, pos=(600.00, 0, 600), vel=(300, 0, 300),   acc=(0, 0, 0)
// ...
// Waypoint 40: t=2.00s, pos=(1100.0, 0, 1100), vel=(0, 0, 0),      acc=(-300, 0, -300)
```

### **SCHRITT 6: Validierung**

```cpp
bool validate_trajectory(const Trajectory& traj) const
{
    for (const auto& wp : traj.waypoints) {
        // Prüfe Geschwindigkeit
        double vel_mag = wp.velocity_mms.distance_to(Point3D(0, 0, 0));
        if (vel_mag > max_linear_velocity_mms * 1.01) {  // 1% Toleranz
            return false;
        }
        
        // Prüfe Beschleunigung
        double accel_mag = wp.acceleration_mms2.distance_to(Point3D(0, 0, 0));
        if (accel_mag > max_linear_acceleration_mms2 * 1.01) {
            return false;
        }
    }
    return true;
}

// BEISPIEL:
// traj.max_velocity_mms = 375 mm/s
// profile.max_linear_velocity_mms = 500 mm/s
// 375 < 500 ✓ → VALID

// traj.max_acceleration_mms2 = 300 mm/s²
// profile.max_linear_acceleration_mms2 = 2000 mm/s²
// 300 < 2000 ✓ → VALID
```

---

## 🧪 Validierung & Testing

### **Test 1: Punkt-zu-Punkt Bewegung**

```cpp
void test_point_to_point()
{
    TrajectoryGenerator gen(MotionProfile::create_default());
    
    Point3D start(0, 0, 0);
    Point3D goal(1000, 0, 0);  // 1m horizontal
    
    Trajectory traj = gen.generate_point_to_point(start, goal, 2.0);
    
    // Validierungen
    assert(traj.is_valid());
    assert(traj.waypoints.size() > 10);  // Mindestens 10 Waypoints
    assert(traj.duration_s == 2.0);
    
    // Prüfe Start-Position
    assert(abs(traj.waypoints[0].position_mm.x - 0.0) < 1.0);
    
    // Prüfe End-Position
    assert(abs(traj.waypoints.back().position_mm.x - 1000.0) < 1.0);
    
    // Prüfe Start/End Geschwindigkeit
    assert(abs(traj.waypoints[0].velocity_mms.distance_to(Point3D(0,0,0))) < 10.0);
    assert(abs(traj.waypoints.back().velocity_mms.distance_to(Point3D(0,0,0))) < 10.0);
    
    std::cout << "✓ Test point_to_point PASSED\n";
}
```

### **Test 2: Geschwindigkeit-Profil**

```cpp
void test_velocity_profile()
{
    TrajectoryGenerator gen(MotionProfile::create_default());
    
    Point3D start(0, 0, 0);
    Point3D goal(2000, 0, 0);
    
    Trajectory traj = gen.generate_point_to_point(start, goal, 2.0);
    
    // Finde Maximum Geschwindigkeit
    double max_vel = 0;
    size_t max_vel_idx = 0;
    
    for (size_t i = 0; i < traj.waypoints.size(); ++i) {
        double vel = traj.waypoints[i].velocity_mms.distance_to(Point3D(0,0,0));
        if (vel > max_vel) {
            max_vel = vel;
            max_vel_idx = i;
        }
    }
    
    // Maximum sollte etwa in der Mitte sein (t ≈ T/2)
    size_t middle_idx = traj.waypoints.size() / 2;
    
    assert(abs(static_cast<int>(max_vel_idx) - static_cast<int>(middle_idx)) < 3);
    
    std::cout << "✓ Test velocity_profile PASSED\n";
    std::cout << "  Max velocity: " << max_vel << " mm/s at t=" 
              << traj.waypoints[max_vel_idx].time_s << "s\n";
}
```

### **Test 3: Limit-Validierung**

```cpp
void test_limit_validation()
{
    auto profile = MotionProfile::create_default();
    TrajectoryGenerator gen(profile);
    
    Point3D start(0, 0, 0);
    Point3D goal(500, 0, 0);
    
    Trajectory traj = gen.generate_point_to_point(start, goal, 1.0);
    
    // Alle Waypoints sollten Limits einhalten
    assert(gen.validate_trajectory(traj));
    
    // Max-Werte sollten bekannt sein
    assert(traj.max_velocity_mms <= profile.max_linear_velocity_mms);
    assert(traj.max_acceleration_mms2 <= profile.max_linear_acceleration_mms2);
    
    std::cout << "✓ Test limit_validation PASSED\n";
    std::cout << "  Max velocity: " << traj.max_velocity_mms << " / " 
              << profile.max_linear_velocity_mms << " mm/s\n";
    std::cout << "  Max acceleration: " << traj.max_acceleration_mms2 << " / "
              << profile.max_linear_acceleration_mms2 << " mm/s²\n";
}
```

---

## 🔗 Integration mit Geometry Calculator

### **Kompletter Workflow**

```cpp
#include "geometry_calculator.hpp"
#include "trajectory_generator.hpp"

int main()
{
    // ===== PHASE 1: Bohrpunkt-Generierung =====
    GeometryParams geo_params;
    GeometryCalculator geo_calc(geo_params);
    
    size_t num_points = geo_calc.generate_boring_points();
    const auto& boring_points = geo_calc.get_boring_points();
    
    std::cout << "Generated " << num_points << " boring points\n";
    
    // ===== PHASE 2: Trajectory Planning =====
    MotionProfile motion_profile = MotionProfile::create_default();
    TrajectoryGenerator traj_gen(motion_profile);
    
    std::vector<Trajectory> all_trajectories;
    
    // Generiere Trajectories zwischen aufeinanderfolgenden Punkten
    for (size_t i = 0; i < boring_points.size() - 1; ++i) {
        const Point3D& current = boring_points[i];
        const Point3D& next = boring_points[i + 1];
        
        try {
            Trajectory traj = traj_gen.generate_point_to_point(current, next);
            
            if (traj_gen.validate_trajectory(traj)) {
                all_trajectories.push_back(traj);
                
                std::cout << "Trajectory " << i << ": "
                          << traj.duration_s << "s, "
                          << "v_max=" << traj.max_velocity_mms << " mm/s\n";
            }
        } catch (const std::exception& e) {
            std::cerr << "Error generating trajectory " << i << ": " 
                      << e.what() << "\n";
        }
    }
    
    std::cout << "\nGenerated " << all_trajectories.size() << " trajectories\n";
    
    return 0;
}

// ERWARTETE AUSGABE:
// Generated 42 boring points
// Trajectory 0: 1.50s, v_max=425.3 mm/s
// Trajectory 1: 2.15s, v_max=350.2 mm/s
// Trajectory 2: 2.15s, v_max=350.2 mm/s
// ... (insgesamt 41 Trajectories)
// Generated 41 trajectories
```

---

## 📋 Zusammenfassung

| Komponente | Funktion | Input | Output |
|-----------|----------|-------|--------|
| **Cubic Polynomial** | Berechnet Position | q₀, qf, t, T | q(t) |
| **Velocity** | Berechnet Geschwindigkeit | q₀, qf, t, T | v(t) |
| **Acceleration** | Berechnet Beschleunigung | q₀, qf, t, T | a(t) |
| **Min Time** | Berechnet optimale Dauer | distance, v_max, a_max | T |
| **Waypoint Loop** | Generiert Zwischenpunkte | start, goal, T | Trajektorie |
| **Validation** | Prüft Limits | Trajektorie | bool |

---

## 🚀 Nächste Schritte

1. ✅ Geometry Calculator implementiert (42 Punkte)
2. → **Trajectory Generator implementieren** (JETZT)
3. → Path Planning ROS2 Node
4. → Motion Control State Machine
5. → Gesamtintegration & Testing

---

**Status:** 📖 Detaillierte Erklärung mit Code-Beispielen  
**Nächste Datei:** trajectory_generator_IMPLEMENTATION.hpp
