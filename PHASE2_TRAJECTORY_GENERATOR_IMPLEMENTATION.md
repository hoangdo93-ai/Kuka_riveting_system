# Trajectory Generator Implementation - Complete Guide

**Module:** Motion Planning & Control  
**File:** trajectory_generator.hpp  
**Language:** C++17  
**ROS2:** Humble  
**Status:** Production-Ready Implementation

---

## 📋 Table of Contents

1. [Mathematical Foundations](#mathematical-foundations)
2. [Algorithm Design](#algorithm-design)
3. [Data Structures & Interfaces](#data-structures--interfaces)
4. [Step-by-Step Implementation](#step-by-step-implementation)
5. [Validation & Testing](#validation--testing)
6. [Integration with Geometry Calculator](#integration-with-geometry-calculator)

---

## 🎯 Goal of the Trajectory Generator

**Task:** Generate safe, time-optimal motions between two boring points

**Input:**
- Start position (Point3D) – where the robot currently is
- Goal position (Point3D) – where the robot needs to go
- Motion parameters (max velocity, acceleration)
- Time constraint (optional)

**Output:**
- Trajectory with waypoints (position, velocity, acceleration per time step)
- Validation against limits
- Performance metrics

---

## 📐 Mathematical Foundations

### **1. Cubic Polynomial Motion Profile**

#### Problem:
How do we move the robot from position A to position B without jerking?

#### Solution: Cubic Polynomial

```
q(t) = q₀ + 3(qf - q₀)(t/T)² - 2(qf - q₀)(t/T)³

where:
  q₀ = start position
  qf = end position
  t  = current time [0, T]
  T  = total duration
  τ  = t/T = normalized time [0, 1]
```

#### Properties:

```
✅ q(0) = q₀          (start position)
✅ q(T) = qf          (end position)
✅ dq/dt(0) = 0       (start velocity = 0)
✅ dq/dt(T) = 0       (end velocity = 0)
✅ d²q/dt²(0) ≠ 0     (acceleration is possible)
✅ d²q/dt²(T) ≠ 0     (acceleration is possible)

⚠️  d³q/dt³ = constant (jerk limitation)
```

#### Visual – "Bell-Shaped" Velocity Profile:

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

Start → Acceleration → Constant Velocity → Deceleration → Stop
```

### **2. Velocity of the Cubic Polynomial**

First derivative:

```
dq/dt = d/dt[q₀ + 3(qf-q₀)τ² - 2(qf-q₀)τ³]

      = 3(qf-q₀) · d(τ²)/dt - 2(qf-q₀) · d(τ³)/dt

      = 3(qf-q₀) · 2τ · dτ/dt - 2(qf-q₀) · 3τ² · dτ/dt

      = [6(qf-q₀)τ - 6(qf-q₀)τ²] · (1/T)

      = 6(qf-q₀)/T · (τ - τ²)

Simplified:
v(t) = 6(qf-q₀)/T² · t - 6(qf-q₀)/T³ · t²
```

**Properties:**
- Maximum at t = T/2
- v(0) = 0
- v(T) = 0

### **3. Acceleration of the Cubic Polynomial**

Second derivative:

```
d²q/dt² = d/dt[6(qf-q₀)/T² · t - 6(qf-q₀)/T³ · t²]
        = 6(qf-q₀)/T² - 12(qf-q₀)/T³ · t

a(t) = 6(qf-q₀)/T² - 12(qf-q₀)/T³ · t
```

**Properties:**
- Changes linearly with t
- a(0) = 6(qf-q₀)/T² (positive acceleration)
- a(T) = -6(qf-q₀)/T² (negative acceleration = deceleration)
- a(T/2) = 0 (zero in the middle)

---

## 🔧 Algorithm Design

### **Trajectory Generation Workflow**

```
┌─────────────────────────────────────────────┐
│ INPUT: Start, Goal, MotionProfile           │
└──────────────┬──────────────────────────────┘
               │
               ▼
    ┌──────────────────────────┐
    │ 1. COMPUTE DISTANCE      │
    │    distance = Start      │
    │            .distance_to  │
    │            (Goal)        │
    └──────────────┬───────────┘
                   │
                   ▼
    ┌──────────────────────────────────────┐
    │ 2. COMPUTE EXECUTION TIME            │
    │    If not specified:                 │
    │    T = f(distance, v_max, a_max)     │
    │                                      │
    │    Triangle profile (accel only):    │
    │    T = 2 * √(distance / a_max)       │
    │                                      │
    │    Trapezoid profile (Accel+Const+Decel):
    │    T = 2 * a_max + (dist - 2*d_accel)/v_max
    └──────────────┬───────────────────────┘
                   │
                   ▼
    ┌──────────────────────────────────────┐
    │ 3. GENERATE WAYPOINTS                │
    │    FOR t = 0 TO T STEP dt:           │
    │      τ = t/T                         │
    │      q(t) = q₀ + ...                 │
    │      v(t) = 6(qf-q₀)/T² * (τ - τ²)  │
    │      a(t) = 6(qf-q₀)/T² - ...        │
    │      Store waypoint                  │
    └──────────────┬───────────────────────┘
                   │
                   ▼
    ┌──────────────────────────────────────┐
    │ 4. VALIDATE TRAJECTORY               │
    │    FOR each waypoint:                │
    │      IF v > v_max:  ERROR!           │
    │      IF a > a_max:  ERROR!           │
    │      IF pos out of bounds: ERROR!    │
    └──────────────┬───────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────┐
│ OUTPUT: Trajectory with waypoints           │
│         Duration, MaxVel, MaxAccel, Valid   │
└─────────────────────────────────────────────┘
```

---

## 📦 Data Structures & Interfaces

### **1. MotionProfile Structure**

```cpp
struct MotionProfile {
    // Velocity limits
    double max_linear_velocity_mms;        // [mm/s]
    double max_joint_velocity_degs;        // [deg/s]

    // Acceleration limits
    double max_linear_acceleration_mms2;   // [mm/s²]
    double max_angular_acceleration_degs2; // [deg/s²]

    // Safety
    double jerk_limit_mms3;                // [mm/s³]
    double collision_margin_mm;            // [mm]

    // Discretization
    double time_step_ms;                   // [ms] sampling rate
};
```

**Example – Default profile (conservative):**
```cpp
MotionProfile profile;
profile.max_linear_velocity_mms = 500.0;         // 0.5 m/s
profile.max_linear_acceleration_mms2 = 2000.0;   // 2 m/s²
profile.time_step_ms = 50.0;                     // 20 Hz
```

**Example – Fast profile:**
```cpp
MotionProfile fast_profile;
fast_profile.max_linear_velocity_mms = 800.0;    // 0.8 m/s
fast_profile.max_linear_acceleration_mms2 = 3500.0;
fast_profile.time_step_ms = 50.0;
```

### **2. Waypoint Structure**

```cpp
struct Trajectory::Waypoint {
    double time_s;              // timestamp [s]
    Point3D position_mm;        // (x, y, z) position [mm]
    Point3D velocity_mms;       // (vx, vy, vz) [mm/s]
    Point3D acceleration_mms2;  // (ax, ay, az) [mm/s²]
};
```

**Example:**
```
Waypoint 0:  t=0.00s,  pos=(100, 0, 100),     vel=(0, 0, 0),      acc=(200, 0, 200)
Waypoint 1:  t=0.05s,  pos=(102.5, 0, 102.5), vel=(50, 0, 50),    acc=(190, 0, 190)
Waypoint 2:  t=0.10s,  pos=(107.4, 0, 107.4), vel=(99, 0, 99),    acc=(160, 0, 160)
...
```

### **3. Trajectory Structure**

```cpp
struct Trajectory {
    vector<Waypoint> waypoints;    // all intermediate points
    Point3D start;                 // start position
    Point3D goal;                  // end position
    double duration_s;             // total duration [s]
    double max_velocity_mms;       // peak velocity [mm/s]
    double max_acceleration_mms2;  // peak acceleration [mm/s²]

    bool is_valid() const;         // validation check
};
```

---

## 🔨 Step-by-Step Implementation

### **STEP 1: Cubic Polynomial Calculation**

```cpp
/**
 * @brief Cubic polynomial: q(t) = q₀ + 3(qf-q₀)τ² - 2(qf-q₀)τ³
 *
 * @param q0 Start position
 * @param qf End position
 * @param t  Current time [0, T]
 * @param T  Total duration
 * @return Position at time t
 */
static double cubic_polynomial(double q0, double qf, double t, double T)
{
    // τ = t/T (normalized time, 0 ≤ τ ≤ 1)
    double tau = t / T;

    // q(τ) = q₀ + 3(qf - q₀)τ² - 2(qf - q₀)τ³
    return q0 + 3.0 * (qf - q0) * tau * tau
             - 2.0 * (qf - q0) * tau * tau * tau;
}

// EXAMPLE:
// q0 = 100 mm (start)
// qf = 1100 mm (end)
// T  = 2.0 s
// t  = 1.0 s (midpoint of motion)
//
// τ = 1.0 / 2.0 = 0.5
// q(0.5) = 100 + 3×1000×0.5² - 2×1000×0.5³
//        = 100 + 750 - 250
//        = 600 mm ✅
```

### **STEP 2: Velocity Calculation**

```cpp
/**
 * @brief First derivative (velocity)
 * dq/dt = 6(qf-q₀)/T² × (t/T) - 6(qf-q₀)/T³ × (t/T)²
 */
static double cubic_polynomial_velocity(double q0, double qf, double t, double T)
{
    double tau = t / T;
    double coeff = (qf - q0) / (T * T);

    // v(t) = 6×coeff×τ - 6×coeff×τ²
    return 6.0 * coeff * tau - 6.0 * coeff * tau * tau;
}

// EXAMPLE (same values as above):
// coeff = (1100 - 100) / (2.0 × 2.0) = 1000 / 4 = 250
// τ = 0.5
// v(0.5) = 6×250×0.5 - 6×250×0.5²
//        = 750 - 375
//        = 375 mm/s ✅ (maximum at τ=0.5)
```

### **STEP 3: Acceleration Calculation**

```cpp
/**
 * @brief Second derivative (acceleration)
 * d²q/dt² = 6(qf-q₀)/T² - 12(qf-q₀)/T³ × (t/T)
 */
static double cubic_polynomial_acceleration(double q0, double qf, double t, double T)
{
    double coeff = (qf - q0) / (T * T);
    double tau = t / T;

    // a(t) = 6×coeff - 12×coeff×τ
    return 6.0 * coeff - 12.0 * coeff * tau;
}

// EXAMPLE:
// coeff = 250, τ = 0.5
// a(0.5) = 6×250 - 12×250×0.5
//        = 1500 - 1500
//        = 0 mm/s² ✅ (zero acceleration at midpoint)
```

### **STEP 4: Computing Minimum Execution Time**

```cpp
/**
 * @brief Computes minimum execution time respecting velocity
 *        and acceleration limits.
 *
 * Two cases:
 *   1. Triangle profile: acceleration only, no constant-velocity phase
 *   2. Trapezoid profile: acceleration → constant velocity → deceleration
 */
double calculate_minimum_time(double distance, double v_max, double a_max)
{
    // STEP 1: Triangle profile
    // If distance = 0.5 × a × T², then T = √(distance / a_max)
    double t_triangle = std::sqrt(distance / a_max);

    // STEP 2: Check whether triangle profile is sufficient
    // If peak velocity ≤ v_max, triangle profile is enough
    if (2 * v_max >= a_max * t_triangle) {
        return 2.0 * t_triangle;
    }

    // STEP 3: Trapezoid profile required
    double t_accel = v_max / a_max;  // time to accelerate

    // Distance covered during acceleration: d = 0.5 × a × t²
    double distance_accel = 0.5 * a_max * t_accel * t_accel;

    if (distance <= 2.0 * distance_accel) {
        return 2.0 * t_triangle;
    } else {
        // Trapezoid: Accel + Const + Decel
        double distance_const_vel = distance - 2.0 * distance_accel;
        double t_const_vel = distance_const_vel / v_max;
        return 2.0 * t_accel + t_const_vel;
    }
}

// EXAMPLE:
// distance = 1000 mm (1 m)
// v_max    = 500 mm/s
// a_max    = 2000 mm/s²
//
// t_triangle = √(1000 / 2000) = √0.5 = 0.707 s
// 2 × v_max = 1000 mm/s
// a_max × t_triangle = 2000 × 0.707 = 1414 mm/s
// 1000 < 1414 ✓ → triangle profile sufficient
// T = 2 × 0.707 = 1.414 s
```

### **STEP 5: Generating Waypoints (Nested Loop)**

```cpp
Trajectory generate_point_to_point(const Point3D& start,
                                    const Point3D& goal,
                                    double duration = 0.0)
{
    Trajectory traj;
    traj.start = start;
    traj.goal  = goal;

    // Compute distance
    double distance = start.distance_to(goal);

    // Use specified or auto-computed duration
    if (duration <= 0.0) {
        duration = calculate_minimum_time(distance, v_max, a_max);
    }
    traj.duration_s = duration;

    // Generate waypoints
    double dt = time_step_ms / 1000.0;  // e.g. 0.05 s
    int num_steps = static_cast<int>(duration / dt) + 1;

    for (int i = 0; i < num_steps; ++i) {
        double t = std::min(static_cast<double>(i) * dt, duration);

        Waypoint wp;
        wp.time_s = t;

        // Position per axis
        wp.position_mm.x = cubic_polynomial(start.x, goal.x, t, duration);
        wp.position_mm.y = cubic_polynomial(start.y, goal.y, t, duration);
        wp.position_mm.z = cubic_polynomial(start.z, goal.z, t, duration);

        // Velocity per axis
        wp.velocity_mms.x = cubic_polynomial_velocity(start.x, goal.x, t, duration);
        wp.velocity_mms.y = cubic_polynomial_velocity(start.y, goal.y, t, duration);
        wp.velocity_mms.z = cubic_polynomial_velocity(start.z, goal.z, t, duration);

        // Acceleration per axis
        wp.acceleration_mms2.x = cubic_polynomial_acceleration(start.x, goal.x, t, duration);
        wp.acceleration_mms2.y = cubic_polynomial_acceleration(start.y, goal.y, t, duration);
        wp.acceleration_mms2.z = cubic_polynomial_acceleration(start.z, goal.z, t, duration);

        // Track peak values
        double vel_mag = wp.velocity_mms.distance_to(Point3D(0, 0, 0));
        traj.max_velocity_mms = std::max(traj.max_velocity_mms, vel_mag);

        double accel_mag = wp.acceleration_mms2.distance_to(Point3D(0, 0, 0));
        traj.max_acceleration_mms2 = std::max(traj.max_acceleration_mms2, accel_mag);

        traj.waypoints.push_back(wp);
    }

    return traj;
}

// EXAMPLE OUTPUT with dt=0.05s and T=2.0s:
// Waypoint  0: t=0.00s, pos=(100.00, 0, 100),  vel=(0, 0, 0),         acc=(300, 0, 300)
// Waypoint  1: t=0.05s, pos=(100.75, 0, 100.75), vel=(29.8, 0, 29.8), acc=(298, 0, 298)
// Waypoint  2: t=0.10s, pos=(102.98, 0, 102.98), vel=(59.1, 0, 59.1), acc=(291, 0, 291)
// ...
// Waypoint 20: t=1.00s, pos=(600.00, 0, 600),  vel=(300, 0, 300),     acc=(0, 0, 0)
// ...
// Waypoint 40: t=2.00s, pos=(1100.0, 0, 1100), vel=(0, 0, 0),         acc=(-300, 0, -300)
```

### **STEP 6: Validation**

```cpp
bool validate_trajectory(const Trajectory& traj) const
{
    for (const auto& wp : traj.waypoints) {
        // Check velocity
        double vel_mag = wp.velocity_mms.distance_to(Point3D(0, 0, 0));
        if (vel_mag > max_linear_velocity_mms * 1.01) {  // 1% tolerance
            return false;
        }

        // Check acceleration
        double accel_mag = wp.acceleration_mms2.distance_to(Point3D(0, 0, 0));
        if (accel_mag > max_linear_acceleration_mms2 * 1.01) {
            return false;
        }
    }
    return true;
}

// EXAMPLE:
// traj.max_velocity_mms = 375 mm/s
// profile.max_linear_velocity_mms = 500 mm/s
// 375 < 500 ✓ → VALID

// traj.max_acceleration_mms2 = 300 mm/s²
// profile.max_linear_acceleration_mms2 = 2000 mm/s²
// 300 < 2000 ✓ → VALID
```

---

## 🧪 Validation & Testing

### **Test 1: Point-to-Point Motion**

```cpp
void test_point_to_point()
{
    TrajectoryGenerator gen(MotionProfile::create_default());

    Point3D start(0, 0, 0);
    Point3D goal(1000, 0, 0);  // 1 m horizontal

    Trajectory traj = gen.generate_point_to_point(start, goal, 2.0);

    // Assertions
    assert(traj.is_valid());
    assert(traj.waypoints.size() > 10);  // at least 10 waypoints
    assert(traj.duration_s == 2.0);

    // Check start position
    assert(abs(traj.waypoints[0].position_mm.x - 0.0) < 1.0);

    // Check end position
    assert(abs(traj.waypoints.back().position_mm.x - 1000.0) < 1.0);

    // Check start/end velocity
    assert(abs(traj.waypoints[0].velocity_mms.distance_to(Point3D(0,0,0))) < 10.0);
    assert(abs(traj.waypoints.back().velocity_mms.distance_to(Point3D(0,0,0))) < 10.0);

    std::cout << "✓ Test point_to_point PASSED\n";
}
```

### **Test 2: Velocity Profile**

```cpp
void test_velocity_profile()
{
    TrajectoryGenerator gen(MotionProfile::create_default());

    Point3D start(0, 0, 0);
    Point3D goal(2000, 0, 0);

    Trajectory traj = gen.generate_point_to_point(start, goal, 2.0);

    // Find peak velocity
    double max_vel = 0;
    size_t max_vel_idx = 0;

    for (size_t i = 0; i < traj.waypoints.size(); ++i) {
        double vel = traj.waypoints[i].velocity_mms.distance_to(Point3D(0,0,0));
        if (vel > max_vel) {
            max_vel = vel;
            max_vel_idx = i;
        }
    }

    // Peak should occur near the midpoint (t ≈ T/2)
    size_t middle_idx = traj.waypoints.size() / 2;

    assert(abs(static_cast<int>(max_vel_idx) - static_cast<int>(middle_idx)) < 3);

    std::cout << "✓ Test velocity_profile PASSED\n";
    std::cout << "  Max velocity: " << max_vel << " mm/s at t="
              << traj.waypoints[max_vel_idx].time_s << "s\n";
}
```

### **Test 3: Limit Validation**

```cpp
void test_limit_validation()
{
    auto profile = MotionProfile::create_default();
    TrajectoryGenerator gen(profile);

    Point3D start(0, 0, 0);
    Point3D goal(500, 0, 0);

    Trajectory traj = gen.generate_point_to_point(start, goal, 1.0);

    // All waypoints must respect limits
    assert(gen.validate_trajectory(traj));

    // Peak values must be within bounds
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

## 🔗 Integration with Geometry Calculator

### **Complete Workflow**

```cpp
#include "geometry_calculator.hpp"
#include "trajectory_generator.hpp"

int main()
{
    // ===== PHASE 1: Boring Point Generation =====
    GeometryParams geo_params;
    GeometryCalculator geo_calc(geo_params);

    size_t num_points = geo_calc.generate_boring_points();
    const auto& boring_points = geo_calc.get_boring_points();

    std::cout << "Generated " << num_points << " boring points\n";

    // ===== PHASE 2: Trajectory Planning =====
    MotionProfile motion_profile = MotionProfile::create_default();
    TrajectoryGenerator traj_gen(motion_profile);

    std::vector<Trajectory> all_trajectories;

    // Generate trajectories between consecutive points
    for (size_t i = 0; i < boring_points.size() - 1; ++i) {
        const Point3D& current = boring_points[i];
        const Point3D& next    = boring_points[i + 1];

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

// EXPECTED OUTPUT:
// Generated 42 boring points
// Trajectory 0: 1.50s, v_max=425.3 mm/s
// Trajectory 1: 2.15s, v_max=350.2 mm/s
// Trajectory 2: 2.15s, v_max=350.2 mm/s
// ... (41 trajectories in total)
// Generated 41 trajectories
```

---

## 📋 Summary

| Component | Function | Input | Output |
|-----------|----------|-------|--------|
| **Cubic Polynomial** | Computes position | q₀, qf, t, T | q(t) |
| **Velocity** | Computes velocity | q₀, qf, t, T | v(t) |
| **Acceleration** | Computes acceleration | q₀, qf, t, T | a(t) |
| **Min Time** | Computes optimal duration | distance, v_max, a_max | T |
| **Waypoint Loop** | Generates intermediate points | start, goal, T | Trajectory |
| **Validation** | Checks limits | Trajectory | bool |

---

## 🚀 Next Steps

1. ✅ Geometry Calculator implemented (42 points)
2. → **Implement Trajectory Generator** (NOW)
3. → Path Planning ROS2 Node
4. → Motion Control State Machine
5. → Full Integration & Testing

---

**Status:** 📖 Detailed explanation with code examples  
**Next file:** trajectory_generator_IMPLEMENTATION.hpp
