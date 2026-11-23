#pragma once

// Standalone end-to-end pipeline runner, mirroring
// MotionControlNode::execute_point_sequence() (Phase 4/5) exactly, but
// without the ROS2 layer, so the full Phase 1 -> 2 -> {backend} chain can
// be compiled and run directly in CI or a local shell. Used by both
// test_end_to_end_pipeline.cpp (SimulatedMotionBackend) and
// test_end_to_end_hardware.cpp (KRC4MotionBackend), so a regression in
// either backend's integration with the core pipeline shows up in both.

#include "kuka_riveting/geometry_calculator.hpp"
#include "kuka_riveting/trajectory_generator.hpp"
#include "kuka_riveting/motion_backend_interface.hpp"

#include <string>
#include <chrono>
#include <functional>
#include <iostream>
#include <iomanip>

namespace kuka_riveting {
namespace testing {

struct PipelineResult {
    bool success = false;
    int points_completed = 0;
    int total_points = 0;
    double total_time_s = 0.0;
    std::string failure_reason;
};

/**
 * @brief Runs the full A380 riveting pipeline end-to-end against a given
 *        motion backend: Phase 1 geometry -> Phase 2 trajectories -> the
 *        backend's execute_travel/approach_point/drill/retract calls.
 */
inline PipelineResult run_full_pipeline(
    const GeometryParams& geo_params,
    const MotionProfile& motion_profile,
    MotionBackendPtr backend,
    const std::function<bool()>& cancel_check,
    bool verbose)
{
    PipelineResult result;

    GeometryCalculator geometry_calculator;
    TrajectoryGenerator trajectory_generator(motion_profile);

    // --- Phase 1: Geometry ---
    // GeometryCalculator throws std::runtime_error for degenerate
    // parameters (e.g. a zero-size flap yielding no valid points).
    // Mirrors the try/catch now present in both ROS2 nodes (Phase 3's
    // path_planning_node, and Phase 5's motion_control_node after the
    // Phase 6 robustness fix) so this harness reports a clean failure
    // instead of crashing the test binary.
    size_t num_points = 0;
    try {
        num_points = geometry_calculator.generate_boring_points(geo_params);
    } catch (const std::exception& e) {
        result.failure_reason = std::string("Geometry generation failed: ") + e.what();
        result.total_points = 0;
        return result;
    }
    const auto& boring_points = geometry_calculator.get_boring_points();
    result.total_points = static_cast<int>(num_points);

    if (verbose) {
        std::cout << "[Phase 1] Generated " << num_points << " boring points\n";
    }

    // --- Phase 5: Backend initialization ---
    std::string backend_error;
    if (!backend->initialize(backend_error)) {
        result.failure_reason = "Backend init failed: " + backend_error;
        return result;
    }
    if (verbose) {
        std::cout << "[Backend] Initialized: " << backend->backend_name() << "\n";
    }

    auto start = std::chrono::steady_clock::now();

    for (size_t i = 0; i < boring_points.size(); ++i) {
        if (cancel_check()) {
            result.failure_reason = "Canceled before point " + std::to_string(i + 1);
            backend->shutdown();
            result.total_time_s = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start).count();
            return result;
        }

        // --- Phase 2: Travel trajectory (skip for the first point) ---
        if (i > 0) {
            Trajectory travel = trajectory_generator.generate_point_to_point(
                boring_points[i - 1], boring_points[i]);

            if (!trajectory_generator.validate_trajectory(travel)) {
                result.failure_reason =
                    "Trajectory to point " + std::to_string(i + 1) + " exceeds motion limits";
                backend->shutdown();
                return result;
            }

            if (!backend->execute_travel(travel, cancel_check)) {
                result.failure_reason =
                    "Travel to point " + std::to_string(i + 1) + " failed or was canceled";
                backend->shutdown();
                result.total_time_s = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - start).count();
                return result;
            }
        }

        // --- Drilling cycle ---
        if (!backend->approach_point(boring_points[i], cancel_check)) {
            result.failure_reason = "Approach at point " + std::to_string(i + 1) + " failed";
            backend->shutdown();
            return result;
        }
        if (!backend->drill(cancel_check)) {
            result.failure_reason = "Drill at point " + std::to_string(i + 1) + " failed";
            backend->shutdown();
            return result;
        }
        if (!backend->retract(boring_points[i], cancel_check)) {
            result.failure_reason = "Retract at point " + std::to_string(i + 1) + " failed";
            backend->shutdown();
            return result;
        }

        result.points_completed++;

        if (verbose && ((i + 1) % 10 == 0 || i + 1 == boring_points.size())) {
            std::cout << "  Progress: " << (i + 1) << "/" << num_points << " ("
                      << std::fixed << std::setprecision(1)
                      << (100.0 * (i + 1) / num_points) << "%)\n";
        }
    }

    backend->shutdown();
    result.total_time_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
    result.success = true;
    return result;
}

} // namespace testing
} // namespace kuka_riveting
