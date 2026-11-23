// End-to-end integration test: Phase 1 (Geometry) -> Phase 2 (Trajectory)
// -> Phase 5 (SimulatedMotionBackend). Validates the complete software
// pipeline a MotionControlNode::execute() call would drive, without the
// ROS2 layer, so it can be compiled and run directly.

#include "kuka_riveting/e2e_pipeline_runner.hpp"
#include "kuka_riveting/simulated_motion_backend.hpp"

#include <atomic>
#include <thread>
#include <iostream>

using namespace kuka_riveting;
using namespace kuka_riveting::testing;

int main() {
    std::cout << "======================================================\n";
    std::cout << "  E2E TEST 1: Full A380 Riveting Pipeline (Simulated)\n";
    std::cout << "======================================================\n\n";

    // --- TEST A: Full run, all 42 points, must succeed ---------------------
    std::cout << "--- Test A: Full execution (all points, default A380 flap) ---\n";

    GeometryParams geo_params;  // default A380: 24000x3500mm, 4000x500 pitch, 100mm margin
    MotionProfile motion_profile = MotionProfile::create_default();

    // Fast timing for test runtime; production defaults are 2.0/1.5/1.0s
    // (see motion_control_params.yaml)
    auto backend_a = std::make_shared<SimulatedMotionBackend>(
        motion_profile.time_step_ms, /*approach_s=*/0.01, /*drill_s=*/0.01, /*retract_s=*/0.01);

    auto never_cancel = []() { return false; };
    auto result_a = run_full_pipeline(geo_params, motion_profile, backend_a, never_cancel, true);

    std::cout << "\nResult: success=" << result_a.success
              << ", points=" << result_a.points_completed << "/" << result_a.total_points
              << ", time=" << result_a.total_time_s << "s\n";

    bool test_a_ok = result_a.success
        && result_a.points_completed == result_a.total_points
        && result_a.total_points == 42;  // expected for default A380 geometry

    std::cout << "Test A: " << (test_a_ok ? "PASS" : "FAIL") << "\n\n";

    // --- TEST B: Cancellation mid-execution --------------------------------
    std::cout << "--- Test B: Cancellation partway through execution ---\n";

    auto backend_b = std::make_shared<SimulatedMotionBackend>(
        motion_profile.time_step_ms, 0.02, 0.02, 0.02);

    std::atomic<bool> cancel_flag{false};
    auto cancel_check_b = [&cancel_flag]() { return cancel_flag.load(); };

    std::thread canceler([&cancel_flag]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        cancel_flag.store(true);
    });

    auto result_b = run_full_pipeline(geo_params, motion_profile, backend_b, cancel_check_b, false);
    canceler.join();

    std::cout << "Result: success=" << result_b.success
              << ", points=" << result_b.points_completed << "/" << result_b.total_points
              << ", reason=\"" << result_b.failure_reason << "\"\n";

    bool test_b_ok = !result_b.success
        && result_b.points_completed > 0
        && result_b.points_completed < result_b.total_points;

    std::cout << "Test B: " << (test_b_ok ? "PASS" : "FAIL") << "\n\n";

    // --- TEST C: Invalid geometry handled gracefully (no crash) -----------
    std::cout << "--- Test C: Degenerate geometry (zero-size flap) ---\n";

    GeometryParams bad_params;
    bad_params.flap_width_mm = 0.0;
    bad_params.flap_height_mm = 0.0;

    auto backend_c = std::make_shared<SimulatedMotionBackend>(
        motion_profile.time_step_ms, 0.01, 0.01, 0.01);

    auto result_c = run_full_pipeline(bad_params, motion_profile, backend_c, never_cancel, false);

    std::cout << "Result: total_points=" << result_c.total_points
              << ", success=" << result_c.success << "\n";

    // A zero-size flap should yield zero (or a minimal) point set and the
    // pipeline must not crash either way - this is the actual assertion.
    bool test_c_ok = true;  // reaching this line without a crash already proves it
    std::cout << "Test C: " << (test_c_ok ? "PASS (no crash on degenerate input)" : "FAIL") << "\n\n";

    // --- Summary -------------------------------------------------------------
    bool all_pass = test_a_ok && test_b_ok && test_c_ok;
    std::cout << "======================================================\n";
    std::cout << "  OVERALL: " << (all_pass ? "ALL TESTS PASSED" : "SOME TESTS FAILED") << "\n";
    std::cout << "======================================================\n";

    return all_pass ? 0 : 1;
}
