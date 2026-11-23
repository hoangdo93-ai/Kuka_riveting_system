
// Kommandos für Kompilierungen
/*

cd ~/kuka_ws

# Vollständiger Build
colcon build --packages-select kuka_riveting_system

# Mit Compiler-Flags
colcon build --packages-select kuka_riveting_system \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# Nur Tests bauen
colcon build --packages-select kuka_riveting_system --cmake-target test
*/

// Kommandos für Unit Tests
/*

# Setup
cd ~/kuka_ws
source install/setup.bash

# Standalone Test-Executable
./install/kuka_riveting_system/lib/kuka_riveting_system/test_riveting_system

# Oder mit ROS2 Test-Framework
colcon test --packages-select kuka_riveting_system
colcon test-result --verbose
*/

//[TEST SUITE 1: GEOMETRY CALCULATOR]
Point3D::distance_to()
GeometryCalculator::initialize()
GeometryCalculator::generate_boring_points() (42 points)
GeometryCalculator::point_coordinates
GeometryCalculator::get_approach_position()

//[TEST SUITE 2: TRAJECTORY GENERATOR]
TrajectoryGenerator::generate_point_to_point() (41 waypoints)
Trajectory::start_position_accuracy()
Trajectory::end_position_accuracy()
Trajectory::velocity_profile(bell_shaped)
TrajectoryGenerator::validate_trajectory()

//[TEST SUITE 3: INTEGRATION & PERFORMANCE]
// Full System: Geometry + Trajectory Planning (2 trajectories)
// Performance: Geometry Calculation (0.45ms per calc)
// Performance: Trajectory Generation (3.12ms per traj)

// Pass Rate: 100.0%