#pragma once

#include <string>
#include <cstdint>

#include "kuka_riveting/motion_backend_interface.hpp"
#include "kuka_riveting/krc4_interface.hpp"

namespace kuka_riveting {

/**
 * @class KRC4MotionBackend
 * @brief Real-hardware motion backend, driving a KUKA KRC4 via KRC4Interface
 *
 * Execution strategy:
 * - execute_travel(): sends only the FINAL waypoint of the given Phase 2
 *   trajectory as a single LIN target - the KRC4's own motion interpolator
 *   handles the path in between. Streaming every 50ms Phase-2 waypoint
 *   over TCP is deliberately avoided (see PHASE5 documentation: network
 *   jitter makes this unsuitable for real-time correction without RSI,
 *   and the discrete boring-point task does not need it).
 * - approach_point() / drill() / retract(): sent as three separate LIN
 *   moves (up, down, up again) around a Z-offset above the panel. A real
 *   deployment would trigger an external rivet-tool I/O signal during
 *   drill() rather than relying on a fixed hold time.
 *
 * Relies on the calling contract documented in IMotionBackend: for a
 * given point, approach_point() is always called before drill(), which
 * is always called before retract().
 */
class KRC4MotionBackend : public IMotionBackend {
public:
    KRC4MotionBackend(
        std::string robot_ip,
        uint16_t robot_port,
        double velocity_mms,
        double motion_timeout_s,
        double drill_z_offset_mm,
        double drilling_hold_duration_s);

    bool initialize(std::string& error_message) override;
    void shutdown() override;

    bool execute_travel(
        const Trajectory& travel,
        const std::function<bool()>& cancel_check) override;

    bool approach_point(
        const Point3D& target_point,
        const std::function<bool()>& cancel_check) override;

    bool drill(const std::function<bool()>& cancel_check) override;

    bool retract(
        const Point3D& target_point,
        const std::function<bool()>& cancel_check) override;

    std::string backend_name() const override { return "KRC4MotionBackend"; }

private:
    std::string robot_ip_;
    uint16_t robot_port_;
    double velocity_mms_;
    double motion_timeout_s_;
    double drill_z_offset_mm_;
    double drilling_hold_duration_s_;

    KRC4Interface interface_;
    uint32_t next_sequence_id_;

    // Remembered between approach_point() -> drill() -> retract() for the
    // current point, per the IMotionBackend calling contract.
    RobotPose current_base_pose_;
    RobotPose current_approach_pose_;

    /// Sends one target pose and polls feedback until MOTION_COMPLETE,
    /// FAULT/ESTOP, cancellation, or timeout. Returns true only on a
    /// clean MOTION_COMPLETE matching the sent sequence id.
    bool move_to_and_wait(
        const RobotPose& target,
        const std::function<bool()>& cancel_check,
        std::string& error_out);
};

} // namespace kuka_riveting
