#pragma once

#include <vector>
#include <cmath>
#include <stdexcept>
#include <iostream>
#include <iomanip>

namespace kuka_riveting {

// ============================================================================
// DATENSTRUKTUREN
// ============================================================================

/**
 * @struct Point3D
 * @brief Dreidimensionaler Punkt im kartesischen Koordinatensystem
 * 
 * Darstellung: (X, Y, Z) in Millimetern
 * - X: Schienen-Achse (horizontal, längs der 24m-Breite)
 * - Y: Radiale Komponente (vom Roboter, = 0 für flat surface)
 * - Z: Vertikale Komponente (gegen Schwerkraft, längs der 3.5m-Höhe)
 */
struct Point3D {
    double x;  // [mm]
    double y;  // [mm]
    double z;  // [mm]
    
    Point3D() : x(0.0), y(0.0), z(0.0) {}
    Point3D(double x_val, double y_val, double z_val)
        : x(x_val), y(y_val), z(z_val) {}
		
	/**
     * @brief Euclidean distance to another point [mm]
     *
     * Required by TrajectoryGenerator (Phase 2) for distance-based motion
     * timing, and used throughout Phase 3-5 for velocity/acceleration
     * magnitude calculations.
     */
    double distance_to(const Point3D& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        double dz = z - other.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
};

/**
 * @struct GeometryParams
 * @brief Konfigurationsparameter für die Geometrie-Berechnung
 * 
 * WICHTIG: Alle Maße in Millimetern!
 */
struct GeometryParams {
    // A380 Landeklappe Dimensionen
    double flap_width_mm;           // [mm] Horizontale Ausdehnung (Standard: 24000mm)
    double flap_height_mm;          // [mm] Vertikale Ausdehnung (Standard: 3500mm)
    
    // Rasterabstände
    double vertical_pitch_mm;       // [mm] Abstand Z-Richtung (Standard: 500mm)
    double horizontal_pitch_mm;     // [mm] Abstand X-Richtung (Standard: 4000mm)
    
    // Sicherheitszonen
    double safety_margin_mm;        // [mm] Rand-Offset (Standard: 100mm)
    
    // Konstruktor mit Standard-Werten
    GeometryParams()
        : flap_width_mm(24000.0),
          flap_height_mm(3500.0),
          vertical_pitch_mm(500.0),
          horizontal_pitch_mm(4000.0),
          safety_margin_mm(100.0) {}
};

// ============================================================================
// GEOMETRY CALCULATOR KLASSE
// ============================================================================

/**
 * @class GeometryCalculator
 * @brief Berechnet alle Bohrpunkte basierend auf Gitter-Geometrie
 */
class GeometryCalculator {
private:
    GeometryParams params_;
    std::vector<Point3D> boring_points_;  // Speicherung aller Bohrpunkte
    
    /**
     * @brief Validiert einen Punkt gegen Arbeitsbereich-Constraints
     * @param point Zu validierender Punkt
     * @return true wenn Punkt im gültigen Bereich liegt
     */
    bool is_point_valid(const Point3D& point) const {
        // X-Bereichsprüfung
        if (point.x < params_.safety_margin_mm ||
            point.x > params_.flap_width_mm - params_.safety_margin_mm) {
            return false;
        }
        
        // Z-Bereichsprüfung
        if (point.z < params_.safety_margin_mm ||
            point.z > params_.flap_height_mm - params_.safety_margin_mm) {
            return false;
        }
        
        return true;
    }
    
public:
    /**
     * @brief Konstruktor mit Custom-Parametern
     */
    explicit GeometryCalculator(const GeometryParams& params = GeometryParams())
        : params_(params), boring_points_() {}
    
    // ========================================================================
    // FUNKTION 1: generate_boring_points()
    // ========================================================================
    
    /**
     * @brief Generiert alle Bohrpunkte im Raster-Format
     * 
     * ALGORITHMUS:
     * 1. Berechne Anzahl der Punkte pro Achse (horizontal & vertikal)
     * 2. Nested-Loop über X-Achse und Z-Achse
     * 3. Generiere Point3D für jeden (X, Z) Gitter-Schnittpunkt
     * 4. Validiere Punkt gegen Constraints
     * 5. Speichere in Vector boring_points_
     * 
     * MATHEMATIK:
     * Anzahl X: floor((Width - 2×Margin) / PitchX) + 1
     * Anzahl Z: floor((Height - 2×Margin) / PitchZ) + 1
     * 
     * Mit den Standard-Parametern:
     * X: (24000 - 200) / 4000 + 1 = 5.95 + 1 = 6 Punkte
     * Z: (3500 - 200) / 500 + 1 = 6.6 + 1 = 7 Punkte
     * Total: 6 × 7 = 42 Bohrpunkte
     * 
     * @return Anzahl der generierten Bohrpunkte
     * @throws std::runtime_error wenn keine Punkte generiert werden
     */
    size_t generate_boring_points() {
        // 1. BERECHNUNG: Anzahl Punkte pro Achse
        // =====================================
        
        // Berechne verfügbare Breite (abzüglich Sicherheitszonen)
        double available_width = params_.flap_width_mm - 2 * params_.safety_margin_mm;
        
        // Berechne verfügbare Höhe (abzüglich Sicherheitszonen)
        double available_height = params_.flap_height_mm - 2 * params_.safety_margin_mm;
        
        // Anzahl Abstände = verfügbare Distanz / Abstand
        // Anzahl Punkte = Anzahl Abstände + 1 (Startpunkt zählt mit!)
        size_t num_points_x = static_cast<size_t>(available_width / params_.horizontal_pitch_mm) + 1;
        size_t num_points_z = static_cast<size_t>(available_height / params_.vertical_pitch_mm) + 1;
        
        // Gesamtanzahl
        size_t total_expected = num_points_x * num_points_z;
        
        std::cout << "Expected points: " << num_points_x << " (X) × " 
                  << num_points_z << " (Z) = " << total_expected << "\n";
        
        // 2. GENERIERUNG: Iteriere über Gitter
        // ===================================
        boring_points_.clear();  // Lösche alte Punkte
        
        // Nested Loop: Extern über X-Achse, Intern über Z-Achse
        for (double x = params_.safety_margin_mm; x <= params_.flap_width_mm - params_.safety_margin_mm; x += params_.horizontal_pitch_mm) {
            for (double z = params_.safety_margin_mm; z <= params_.flap_height_mm - params_.safety_margin_mm; z += params_.vertical_pitch_mm) {
                // Erstelle Point3D
                // Y = 0.0 (konstante radiale Position vom Roboter)
                Point3D point(x, 0.0, z);
                
                // 3. VALIDIERUNG: Prüfe Punkt
                // ===========================
                if (is_point_valid(point)) {
                    boring_points_.push_back(point);
                }
            }
        }
        
        // 4. FEHLERBEHANDLUNG
        // ===================
        if (boring_points_.empty()) {
            throw std::runtime_error(
                "GeometryCalculator: Keine gültigen Bohrpunkte generiert. "
                "Prüfe Geometrie-Parameter und Sicherheitszonen."
            );
        }
        
        return boring_points_.size();
    }
    
    // ========================================================================
    // FUNKTION 2: get_boring_points()
    // ========================================================================
    
    /**
     * @brief Gibt alle generierten Bohrpunkte zurück
     * 
     * ZWECK:
     * - Bereitstellung der generierten Punkte für nachgelagerte Module
     * - Schnittstelle zu Motion Planning Komponenten
     * - Lesezugriff (const reference) für Sicherheit
     * 
     * VERWENDUNG:
     * const auto& points = calc.get_boring_points();  // Alle Punkte holen
     * for (const auto& point : points) {
     *     std::cout << "Point: (" << point.x << ", " << point.z << ")\n";
     * }
     * 
     * @return const-Referenz zum Vector der Bohrpunkte
     * @throws std::runtime_error wenn noch keine Punkte generiert wurden
     * 
     * VORAUSSETZUNG:
     * - generate_boring_points() muss vorher aufgerufen worden sein
     * - boring_points_ Vector darf nicht leer sein
     */
    const std::vector<Point3D>& get_boring_points() const {
        // Fehlerprüfung: Wurden Punkte generiert?
        if (boring_points_.empty()) {
            throw std::runtime_error(
                "GeometryCalculator: Noch keine Bohrpunkte generiert. "
                "Rufe generate_boring_points() auf."
            );
			boring_points_ = GeometryCalculator().generate_boring_points();
        }
        
        // Rückgabe als const-Referenz (keine Kopie!)
        return boring_points_;
    }
    
    /**
     * @brief Gibt einen einzelnen Bohrpunkt zurück
     * @param index Index im Bohrpunkt-Array (0-basiert)
     * @return const-Referenz zum Punkt
     * @throws std::out_of_range bei ungültigem Index
     */
    const Point3D& get_boring_point(size_t index) const {
        if (index >= boring_points_.size()) {
            throw std::out_of_range(
                "Index " + std::to_string(index) + 
                " außerhalb Bereich [0, " + std::to_string(boring_points_.size() - 1) + "]"
            );
        }
        return boring_points_[index];
    }
    
    /**
     * @brief Gibt die Anzahl der generierten Bohrpunkte zurück
     */
    size_t get_point_count() const {
        return boring_points_.size();
    }
    
    // ========================================================================
    // WEITERE HILFSFUNKTIONEN
    // ========================================================================
    
    /**
     * @brief Gibt Statistik über das Bohrpunkt-Raster aus
     */
    void print_statistics() const {
        if (boring_points_.empty()) {
            std::cout << "Keine Bohrpunkte vorhanden.\n";
            return;
        }
        
        std::cout << "\n" << std::string(50, '=') << "\n";
        std::cout << "BOHRPUNKT-STATISTIK\n";
        std::cout << std::string(50, '=') << "\n";
        std::cout << "Gesamtanzahl: " << boring_points_.size() << " Punkte\n";
        
        // Finde Min/Max
        double min_x = boring_points_[0].x, max_x = boring_points_[0].x;
        double min_z = boring_points_[0].z, max_z = boring_points_[0].z;
        
        for (const auto& p : boring_points_) {
            min_x = std::min(min_x, p.x);
            max_x = std::max(max_x, p.x);
            min_z = std::min(min_z, p.z);
            max_z = std::max(max_z, p.z);
        }
        
        std::cout << "X-Bereich: " << std::fixed << std::setprecision(1)
                  << min_x << " - " << max_x << " mm\n";
        std::cout << "Z-Bereich: " << min_z << " - " << max_z << " mm\n";
        std::cout << std::string(50, '=') << "\n\n";
    }
};

} // namespace kuka_riveting

#endif
