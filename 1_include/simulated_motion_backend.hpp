#pragma once

#include <thread>
#include <chrono>

#include "kuka_riveting/motion_backend_interface.hpp"

namespace kuka_riveting {

/**
 * @class SimulatedMotionBackend
 * @brief sleep()-based motion simulation (Phase 4 behavior, extracted)
 *
 * Used for development, demos, and testing without access to real
 * hardware. Timing matches Phase 4 exactly: travel is stepped at the
 * trajectory's native time_step_ms cadence, and the drilling cycle holds
 * for the configured approach/drilling/retract durations.
 */
class SimulatedMotionBackend : public IMotionBackend {
public:
    SimulatedMotionBackend(
        double time_step_ms,
        double approach_duration_s,
        double drilling_duration_s,
        double retract_duration_s)
        : time_step_ms_(time_step_ms),
          approach_duration_s_(approach_duration_s),
          drilling_duration_s_(drilling_duration_s),
          retract_duration_s_(retract_duration_s)
    {
    }

    bool initialize(std::string& /*error_message*/) override
    {
        return true;  // nothing to connect to
    }

    void shutdown() override
    {
        // no-op
    }

    bool execute_travel(
        const Trajectory& travel,
        const std::function<bool()>& cancel_check) override
    {
        const auto step = std::chrono::milliseconds(static_cast<int>(time_step_ms_));
        for (size_t i = 0; i < travel.waypoints.size(); ++i) {
            if (cancel_check()) return false;
            std::this_thread::sleep_for(step);
        }
        return true;
    }

    bool approach_point(
        const Point3D& /*target_point*/,
        const std::function<bool()>& cancel_check) override
    {
        std::this_thread::sleep_for(std::chrono::duration<double>(approach_duration_s_));
        return !cancel_check();
    }

    bool drill(const std::function<bool()>& cancel_check) override
    {
        std::this_thread::sleep_for(std::chrono::duration<double>(drilling_duration_s_));
        return !cancel_check();
    }

    bool retract(
        const Point3D& /*target_point*/,
        const std::function<bool()>& cancel_check) override
    {
        std::this_thread::sleep_for(std::chrono::duration<double>(retract_duration_s_));
        return !cancel_check();
    }

    std::string backend_name() const override
    {
        return "SimulatedMotionBackend";
    }

private:
    double time_step_ms_;
    double approach_duration_s_;
    double drilling_duration_s_;
    double retract_duration_s_;
};

} // namespace kuka_riveting
