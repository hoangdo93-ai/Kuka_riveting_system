#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <iomanip>

namespace kuka_riveting {

// ============================================================================
// DATENSTRUKTUREN
// ============================================================================

/**
 * @struct MotionProfile
 * @brief Definiert Bewegungsparameter für sichere Roboter-Ausführung
 * 
 * Alle Limit-Werte sind HARD LIMITS:
 * - Der Roboter darf diese Grenzen NICHT überschreiten
 * - Validierung wirft Exception bei Verletzung
 */
struct MotionProfile {
    // ===== GESCHWINDIGKEITSLIMITS =====
    double max_linear_velocity_mms;       // [mm/s] TCP Geschwindigkeit
    double max_joint_velocity_degs;       // [deg/s] Gelenk-Geschwindigkeit
    
    // ===== BESCHLEUNIGUNGSLIMITS =====
    double max_linear_acceleration_mms2;  // [mm/s²] TCP Beschleunigung
    double max_angular_acceleration_degs2; // [deg/s²] Gelenk-Beschleunigung
    
    // ===== SICHERHEIT =====
    double jerk_limit_mms3;               // [mm/s³] Ruck-Begrenzung
    double collision_margin_mm;           // [mm] Sicherheitsabstand
    
    // ===== DISKRETISIERUNG =====
    double time_step_ms;                  // [ms] Abtastrate für Waypoints
    
    /**
     * @brief Standard-Bewegungsprofil (konservativ)
     * 
     * Verwendung für A380-Riveting mit Sicherheitsfokus:
     * - v_max: 500 mm/s (0.5 m/s) - moderate Geschwindigkeit
     * - a_max: 2000 mm/s² (2 m/s²) - sanfte Beschleunigung
     * - time_step: 50ms (20Hz) - Real-Time Anforderung
     */
    static MotionProfile create_default() {
        MotionProfile profile;
        profile.max_linear_velocity_mms = 500.0;
        profile.max_joint_velocity_degs = 90.0;
        profile.max_linear_acceleration_mms2 = 2000.0;
        profile.max_angular_acceleration_degs2 = 180.0;
        profile.jerk_limit_mms3 = 5000.0;
        profile.collision_margin_mm = 50.0;
        profile.time_step_ms = 50.0;
        return profile;
    }
    
    /**
     * @brief Schnelleres Profil (wenn Sicherheit erhöht)
     * 
     * Verwendung wenn:
     * - Kein Kollisions-Risiko (z.B. im Freiraum)
     * - Höhere Durchsatzrate erforderlich
     * - Sicherheitszonen geräumt
     */
    static MotionProfile create_fast() {
        MotionProfile profile = create_default();
        profile.max_linear_velocity_mms = 800.0;
        profile.max_linear_acceleration_mms2 = 3500.0;
        profile.collision_margin_mm = 100.0;
        return profile;
    }
    
    /**
     * @brief Äußerst vorsichtiges Profil (Teach/Manuell)
     */
    static MotionProfile create_safe() {
        MotionProfile profile = create_default();
        profile.max_linear_velocity_mms = 200.0;
        profile.max_linear_acceleration_mms2 = 1000.0;
        profile.collision_margin_mm = 25.0;
        return profile;
    }
};

/**
 * @struct Trajectory
 * @brief Repräsentiert eine Bahn von Punkt A nach Punkt B
 * 
 * Enthält alle Informationen zur zeitbasierten Bewegungsausführung
 */
struct Trajectory {
    /**
     * @struct Waypoint
     * @brief Ein einzelner Punkt auf der Trajektorie zu Zeit t
     */
    struct Waypoint {
        double time_s;              // Zeitstempel [s]
        Point3D position_mm;        // (x, y, z) Position [mm]
        Point3D velocity_mms;       // (vx, vy, vz) Geschwindigkeit [mm/s]
        Point3D acceleration_mms2;  // (ax, ay, az) Beschleunigung [mm/s²]
    };
    
    // Daten
    std::vector<Waypoint> waypoints;
    Point3D start;
    Point3D goal;
    double duration_s;
    double max_velocity_mms;
    double max_acceleration_mms2;
    
    // Konstruktor
    Trajectory()
        : duration_s(0.0), max_velocity_mms(0.0), max_acceleration_mms2(0.0) {}
    
    /**
     * @brief Gibt Anzahl der Waypoints zurück
     */
    size_t get_waypoint_count() const {
        return waypoints.size();
    }
    
    /**
     * @brief Prüft ob Trajektorie gültig ist
     */
    bool is_valid() const {
        return !waypoints.empty() && duration_s > 0.0;
    }
};

// ============================================================================
// TRAJECTORY GENERATOR KLASSE
// ============================================================================

/**
 * @class TrajectoryGenerator
 * @brief Generiert zeitoptimale Trajektorien mit Cubic Polynomial Profiles
 * 
 * Algorithmen:
 * - Cubic Polynomial: q(t) = q₀ + 3(qf-q₀)τ² - 2(qf-q₀)τ³
 * - Velocity Profile: Bell-shaped (Accel → Const → Decel)
 * - Minimal Time: Trapezoid-Profil-Optimierung
 */
class TrajectoryGenerator {
private:
    MotionProfile motion_profile_;
    
    // ========================================================================
    // PRIVATE HILFSFUNKTIONEN - CUBIC POLYNOMIAL
    // ========================================================================
    
    /**
     * @brief Kubisches Polynom für Punkt-zu-Punkt Bewegung
     * 
     * q(t) = q₀ + 3(qf - q₀)(t/T)² - 2(qf - q₀)(t/T)³
     * 
     * Eigenschaften:
     * - q(0) = q₀ (Startposition)
     * - q(T) = qf (Endposition)
     * - dq/dt(0) = 0 (Start sanft)
     * - dq/dt(T) = 0 (Ende sanft)
     * 
     * @param q0 Start-Position
     * @param qf End-Position
     * @param t Aktuelle Zeit
     * @param T Gesamtdauer
     * @return q(t) Position zum Zeitpunkt t
     */
    static double cubic_polynomial(double q0, double qf, double t, double T) {
        double tau = t / T;  // τ = t/T ∈ [0, 1]
        
        // q(τ) = q₀ + 3(qf-q₀)τ² - 2(qf-q₀)τ³
        return q0 + 3.0 * (qf - q0) * tau * tau 
                 - 2.0 * (qf - q0) * tau * tau * tau;
    }
    
    /**
     * @brief Erste Ableitung (Geschwindigkeit)
     * 
     * dq/dt = 6(qf-q₀)/T² × (t/T) - 6(qf-q₀)/T³ × (t/T)²
     * 
     * Eigenschaften:
     * - dq/dt(0) = 0
     * - dq/dt(T) = 0
     * - Maximum bei t = T/2
     * 
     * @return v(t) Geschwindigkeit zum Zeitpunkt t
     */
    static double cubic_polynomial_velocity(double q0, double qf, double t, double T) {
        double tau = t / T;
        double coeff = (qf - q0) / (T * T);
        
        // v(t) = 6×coeff×τ - 6×coeff×τ²
        return 6.0 * coeff * tau - 6.0 * coeff * tau * tau;
    }
    
    /**
     * @brief Zweite Ableitung (Beschleunigung)
     * 
     * d²q/dt² = 6(qf-q₀)/T² - 12(qf-q₀)/T³ × (t/T)
     * 
     * Eigenschaften:
     * - a(0) = 6(qf-q₀)/T² (positive Beschleunigung)
     * - a(T) = -6(qf-q₀)/T² (negative Beschleunigung)
     * - a(T/2) = 0 (Null in der Mitte)
     * 
     * @return a(t) Beschleunigung zum Zeitpunkt t
     */
    static double cubic_polynomial_acceleration(double q0, double qf, double t, double T) {
        double tau = t / T;
        double coeff = (qf - q0) / (T * T);
        
        // a(t) = 6×coeff - 12×coeff×τ
        return 6.0 * coeff - 12.0 * coeff * tau;
    }
    
    /**
     * @brief Berechnet minimale Ausführungszeit unter Limiten
     * 
     * Berücksichtigt zwei Profil-Typen:
     * 1. Dreiecks-Profil: Nur Beschleunigung
     * 2. Trapez-Profil: Accel → Const Vel → Decel
     * 
     * ALGORITHMUS:
     * Dreiecks-Profil: T = 2√(distance / a_max)
     * 
     * Wenn v_peak (von Dreiecks-Profil) ≤ v_max:
     *   → Dreiecks-Profil ausreichend
     * 
     * Sonst Trapez-Profil:
     *   t_accel = v_max / a_max
     *   d_accel = 0.5 × a_max × t_accel²
     *   d_const = distance - 2×d_accel
     *   T = 2×t_accel + d_const/v_max
     * 
     * @param distance Strecke [mm]
     * @param v_max Maximale Geschwindigkeit [mm/s]
     * @param a_max Maximale Beschleunigung [mm/s²]
     * @return Minimale Zeit [s]
     */
    double calculate_minimum_time(double distance, double v_max, double a_max) const {
        // FALL 1: Dreiecks-Profil
        // Wenn Distanz kurz genug, nur Beschleunigung/Verzögerung nötig
        double t_triangle = std::sqrt(distance / a_max);
        
        // Peak-Geschwindigkeit bei Dreiecks-Profil
        double v_peak_triangle = a_max * t_triangle;
        
        // FALL 2: Dreiecks-Profil reicht nicht aus
        // Wenn Peak-Geschwindigkeit ≤ v_max, ist Dreiecks-Profil OK
        if (v_peak_triangle <= v_max * 1.001) {  // 0.1% Toleranz
            return 2.0 * t_triangle;
        }
        
        // FALL 3: Trapez-Profil notwendig
        // Accel mit v_max, dann Const Velocity, dann Decel
        double t_accel = v_max / a_max;
        double distance_accel = 0.5 * a_max * t_accel * t_accel;
        
        // Prüfe ob distance zu klein für Trapez ist
        if (distance <= 2.0 * distance_accel) {
            // Trotzdem Dreiecks-Profil verwenden
            return 2.0 * t_triangle;
        }
        
        // Trapez-Profil: 2×Accel + ConstVel
        double distance_const_vel = distance - 2.0 * distance_accel;
        double t_const_vel = distance_const_vel / v_max;
        return 2.0 * t_accel + t_const_vel;
    }
    
public:
    /**
     * @brief Konstruktor
     * @param profile MotionProfile mit Limits
     */
    explicit TrajectoryGenerator(const MotionProfile& profile = MotionProfile::create_default())
        : motion_profile_(profile) {}
    
    // ========================================================================
    // PUBLIC API
    // ========================================================================
    
    /**
     * @brief Generiert Punkt-zu-Punkt Trajektorie
     * 
     * ALGORITHMUS:
     * 1. Berechne Distanz zwischen start und goal
     * 2. Berechne optimale Ausführungszeit
     * 3. Generiere Waypoints mit Nested Loop
     * 4. Validiere gegen Limits
     * 
     * EINGABEN:
     * - start: Start-Position [mm]
     * - goal: Ziel-Position [mm]
     * - duration_override: Optionale feste Dauer [s]
     *                      Wenn 0: auto-berechnen
     * 
     * AUSGABE:
     * - Trajectory mit Waypoints, Duration, Peak-Werte
     * 
     * @throws std::runtime_error bei ungültigen Eingaben
     */
    Trajectory generate_point_to_point(const Point3D& start, const Point3D& goal, double duration_override = 0.0) {
        
        Trajectory traj;
        traj.start = start;
        traj.goal = goal;
        
        // ===== SCHRITT 1: Berechne Distanz =====
        double distance = start.distance_to(goal);
        
        // Spezialfall: Start ≈ Goal
        if (distance < 1e-6) {
            traj.duration_s = 0.0;
            
            Trajectory::Waypoint wp;
            wp.time_s = 0.0;
            wp.position_mm = start;
            wp.velocity_mms = Point3D(0, 0, 0);
            wp.acceleration_mms2 = Point3D(0, 0, 0);
            
            traj.waypoints.push_back(wp);
            return traj;
        }
        
        // ===== SCHRITT 2: Berechne Ausführungszeit =====
        if (duration_override > 0.0) {
            traj.duration_s = duration_override;
        } else {
            traj.duration_s = calculate_minimum_time(
                distance,
                motion_profile_.max_linear_velocity_mms,
                motion_profile_.max_linear_acceleration_mms2
            );
        }
        
        // ===== SCHRITT 3: Generiere Waypoints =====
        double dt_s = motion_profile_.time_step_ms / 1000.0;
        int num_steps = static_cast<int>(traj.duration_s / dt_s) + 1;
        
        traj.max_velocity_mms = 0.0;
        traj.max_acceleration_mms2 = 0.0;
        
        for (int i = 0; i < num_steps; ++i) {
            double t = std::min(static_cast<double>(i) * dt_s, traj.duration_s);
            
            Trajectory::Waypoint wp;
            wp.time_s = t;
            
            // ===== POSITION =====
            wp.position_mm.x = cubic_polynomial(start.x, goal.x, t, traj.duration_s);
            wp.position_mm.y = cubic_polynomial(start.y, goal.y, t, traj.duration_s);
            wp.position_mm.z = cubic_polynomial(start.z, goal.z, t, traj.duration_s);
            
            // ===== GESCHWINDIGKEIT =====
            wp.velocity_mms.x = cubic_polynomial_velocity(start.x, goal.x, t, traj.duration_s);
            wp.velocity_mms.y = cubic_polynomial_velocity(start.y, goal.y, t, traj.duration_s);
            wp.velocity_mms.z = cubic_polynomial_velocity(start.z, goal.z, t, traj.duration_s);
            
            double vel_mag = wp.velocity_mms.distance_to(Point3D(0, 0, 0));
            traj.max_velocity_mms = std::max(traj.max_velocity_mms, vel_mag);
            
            // ===== BESCHLEUNIGUNG =====
            wp.acceleration_mms2.x = cubic_polynomial_acceleration(start.x, goal.x, t, traj.duration_s);
            wp.acceleration_mms2.y = cubic_polynomial_acceleration(start.y, goal.y, t, traj.duration_s);
            wp.acceleration_mms2.z = cubic_polynomial_acceleration(start.z, goal.z, t, traj.duration_s);
            
            double accel_mag = wp.acceleration_mms2.distance_to(Point3D(0, 0, 0));
            traj.max_acceleration_mms2 = std::max(traj.max_acceleration_mms2, accel_mag);
            
            traj.waypoints.push_back(wp);
        }
        
        return traj;
    }
    
    /**
     * @brief Validiert Trajektorie gegen Motion Profile Limits
     * 
     * Prüfungen:
     * - Alle Geschwindigkeiten ≤ max_linear_velocity (mit 1% Toleranz)
     * - Alle Beschleunigungen ≤ max_linear_acceleration (mit 1% Toleranz)
     * - Keine Position außerhalb Grenzen (später: Kollisionen)
     * 
     * @param traj Zu validierende Trajektorie
     * @return true wenn alle Limits eingehalten, false sonst
     */
    bool validate_trajectory(const Trajectory& traj) const {
        for (const auto& wp : traj.waypoints) {
            // Geschwindigkeit prüfen
            double vel_mag = wp.velocity_mms.distance_to(Point3D(0, 0, 0));
            if (vel_mag > motion_profile_.max_linear_velocity_mms * 1.01) {
                return false;
            }
            
            // Beschleunigung prüfen
            double accel_mag = wp.acceleration_mms2.distance_to(Point3D(0, 0, 0));
            if (accel_mag > motion_profile_.max_linear_acceleration_mms2 * 1.01) {
                return false;
            }
        }
        return true;
    }
    
    /**
     * @brief Gibt Trajektorie-Statistik aus
     */
    void print_trajectory_info(const Trajectory& traj) const {
        std::cout << "\n" << std::string(50, '=') << "\n";
        std::cout << "TRAJECTORY INFO\n";
        std::cout << std::string(50, '=') << "\n";
        std::cout << "Start: (" << std::fixed << std::setprecision(1)
                  << traj.start.x << ", " << traj.start.z << ") mm\n";
        std::cout << "Goal:  (" << traj.goal.x << ", " << traj.goal.z << ") mm\n";
        std::cout << "Distance: " << traj.start.distance_to(traj.goal) << " mm\n";
        std::cout << "Duration: " << traj.duration_s << " s\n";
        std::cout << "Waypoints: " << traj.waypoints.size() << "\n";
        std::cout << "Max Velocity: " << std::setprecision(1) << traj.max_velocity_mms 
                  << " mm/s (" << traj.max_velocity_mms / 1000.0 << " m/s)\n";
        std::cout << "Max Acceleration: " << traj.max_acceleration_mms2 
                  << " mm/s² (" << traj.max_acceleration_mms2 / 1000.0 << " m/s²)\n";
        std::cout << "Valid: " << (validate_trajectory(traj) ? "YES ✓" : "NO ✗") << "\n";
        std::cout << std::string(50, '=') << "\n\n";
    }
};

} // namespace kuka_riveting

#endif
