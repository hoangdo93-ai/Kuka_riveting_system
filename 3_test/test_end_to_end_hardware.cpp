// End-to-end integration test: Phase 1 (Geometry) -> Phase 2 (Trajectory)
// -> Phase 5 (KRC4MotionBackend) -> real TCP connection -> mock_krc4_server.py
//
// This is the strongest validation available in this sandbox: the ENTIRE
// software stack, from geometry generation through the wire protocol,
// runs for real - just with a simulated controller on the other end of
// the socket instead of physical hardware.
//
// Prerequisite: mock_krc4_server.py must be running and reachable at
// 127.0.0.1:54600 before this test is started.

#include "kuka_riveting/e2e_pipeline_runner.hpp"
#include "kuka_riveting/krc4_motion_backend.hpp"

#include <iostream>

using namespace kuka_riveting;
using namespace kuka_riveting::testing;

int main() {
    std::cout << "======================================================\n";
    std::cout << "  E2E TEST 2: Full Pipeline via KRC4MotionBackend\n";
    std::cout << "  (real TCP protocol -> mock_krc4_server.py)\n";
    std::cout << "======================================================\n\n";

    // Small geometry so the test completes quickly: a 4100x600mm panel
    // with 4000x500mm pitch and 50mm margin yields a compact 2x2 grid.
    GeometryParams geo_params;
    geo_params.flap_width_mm = 4100.0;
    geo_params.flap_height_mm = 600.0;
    geo_params.horizontal_pitch_mm = 4000.0;
    geo_params.vertical_pitch_mm = 500.0;
    geo_params.safety_margin_mm = 50.0;

    MotionProfile motion_profile = MotionProfile::create_default();

    auto backend = std::make_shared<KRC4MotionBackend>(
        "127.0.0.1", 54600,
        motion_profile.max_linear_velocity_mms,
        /*motion_timeout_s=*/10.0,
        /*drill_z_offset_mm=*/20.0,
        /*drilling_hold_duration_s=*/0.1);  // short hold for fast test

    auto never_cancel = []() { return false; };

    auto result = run_full_pipeline(geo_params, motion_profile, backend, never_cancel, true);

    std::cout << "\nResult: success=" << result.success
              << ", points=" << result.points_completed << "/" << result.total_points
              << ", time=" << result.total_time_s << "s\n";
    if (!result.success) {
        std::cout << "Failure reason: " << result.failure_reason << "\n";
    }

    bool test_ok = result.success
        && result.points_completed == result.total_points
        && result.total_points == 4;  // expected: 2x2 grid

    std::cout << "\n======================================================\n";
    std::cout << "  OVERALL: " << (test_ok ? "PASS" : "FAIL") << "\n";
    std::cout << "======================================================\n";

    return test_ok ? 0 : 1;
}
