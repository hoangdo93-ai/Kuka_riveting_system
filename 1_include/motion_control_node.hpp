#pragma once

#include <memory>
#include <vector>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"

// Phase 1 & 2 modules (reused directly)
#include "kuka_riveting/geometry_calculator.hpp"
#include "kuka_riveting/trajectory_generator.hpp"

// Forward declarations for ROS2 generated types
namespace kuka_riveting_msgs {
namespace action {
struct ExecuteRiveting;
} // namespace action
namespace srv {
struct PlanTrajectory;
} // namespace srv
} // namespace kuka_riveting_msgs

namespace kuka_riveting {

/**
 * @enum MotionState
 * @brief Top-level state machine states for the Motion Control Node
 *
 * Flow: IDLE -> PLANNING_TRAJECTORY -> EXECUTING -> COMPLETED
 *       Any state -> ERROR on failure
 */
enum class MotionState {
    IDLE,                 ///< Waiting for an execution goal
    PLANNING_TRAJECTORY,  ///< Requesting/validating the trajectory plan
    EXECUTING,            ///< Actively moving through boring points
    COMPLETED,            ///< All points successfully riveted
    ERROR                 ///< Unrecoverable failure occurred
};

/**
 * @enum ExecutionPhase
 * @brief Sub-state within EXECUTING, repeated for every boring point
 *
 * Cycle per point: TRAVELING -> APPROACHING -> DRILLING -> RETRACTING
 */
enum class ExecutionPhase {
    TRAVELING,    ///< Moving from previous point toward current point
    APPROACHING,  ///< Fine positioning above the drill point
    DRILLING,     ///< Rivet hole drilling in progress
    RETRACTING    ///< Withdrawing tool before moving to next point
};

/// Human-readable state name (used in logs, status topic, feedback messages)
std::string to_string(MotionState state);

/// Human-readable phase name (used in logs, status topic, feedback messages)
std::string to_string(ExecutionPhase phase);

/**
 * @class MotionControlNode
 * @brief ROS2 Action Server that executes the A380 riveting sequence
 *
 * Responsibilities:
 * - Expose an Action server for long-running, cancelable execution with feedback
 * - Call the Path Planning service (Phase 3) to validate the plan is feasible
 * - Independently regenerate boring points (Phase 1) for deterministic execution
 * - Drive a per-point state machine: travel, approach, drill, retract
 * - Publish live status and action feedback throughout execution
 */
class MotionControlNode : public rclcpp::Node {
public:
    using ExecuteRiveting = kuka_riveting_msgs::action::ExecuteRiveting;
    using GoalHandleExecuteRiveting = rclcpp_action::ServerGoalHandle<ExecuteRiveting>;
    using PlanTrajectory = kuka_riveting_msgs::srv::PlanTrajectory;

    explicit MotionControlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~MotionControlNode() override;

private:
    // ======================================================================
    // ROS2 Components
    // ======================================================================

    /// Action server: motion_control/execute_riveting
    rclcpp_action::Server<ExecuteRiveting>::SharedPtr action_server_;

    /// Service client to Path Planning Node (Phase 3): /path_planning/plan
    rclcpp::Client<PlanTrajectory>::SharedPtr plan_client_;

    /// Publisher: motion_control/status
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;

    // ======================================================================
    // Reused Modules (Phase 1 & 2)
    // ======================================================================

    GeometryCalculator geometry_calculator_;
    TrajectoryGenerator trajectory_generator_;

    // ======================================================================
    // Parameters
    // ======================================================================

    GeometryParams geometry_params_;
    MotionProfile motion_profile_;

    /// Drilling cycle timing [s] - configurable via parameter server
    double approach_duration_s_;
    double drilling_duration_s_;
    double retract_duration_s_;

    // ======================================================================
    // State Machine (atomics: read from action thread, written from executor)
    // ======================================================================

    std::atomic<MotionState> current_state_;
    std::atomic<ExecutionPhase> current_phase_;

    // ======================================================================
    // Action Server Callbacks
    // ======================================================================

    /// Decide whether to accept an incoming goal (rejects if not IDLE)
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const ExecuteRiveting::Goal> goal);

    /// Decide whether to accept a cancellation request (always accepts)
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleExecuteRiveting> goal_handle);

    /// Spawns the execution thread once a goal has been accepted
    void handle_accepted(const std::shared_ptr<GoalHandleExecuteRiveting> goal_handle);

    /// Main execution routine - runs in its own detached thread
    void execute(const std::shared_ptr<GoalHandleExecuteRiveting> goal_handle);

    // ======================================================================
    // Helper Functions
    // ======================================================================

    void load_parameters();

    /// Calls Path Planning service to validate feasibility of the current parameters
    bool request_trajectory_plan(std::string& error_message);

    /// Iterates over all boring points: travel + drill cycle, with feedback
    bool execute_point_sequence(
        const std::shared_ptr<GoalHandleExecuteRiveting>& goal_handle,
        int& points_completed);

    /// Simulates smooth travel motion along a Phase 2 trajectory
    bool simulate_travel_phase(
        const Trajectory& travel,
        const std::shared_ptr<GoalHandleExecuteRiveting>& goal_handle);

    /// Simulates the approach -> drill -> retract cycle at one boring point
    bool simulate_drilling_cycle(
        const Point3D& target_point,
        const std::shared_ptr<GoalHandleExecuteRiveting>& goal_handle,
        size_t point_index,
        size_t total_points);

    /// Publishes action feedback (progress, position, current state/phase)
    void publish_feedback(
        const std::shared_ptr<GoalHandleExecuteRiveting>& goal_handle,
        size_t point_index,
        size_t total_points,
        const Point3D& position,
        double elapsed_time_s);

    void publish_status(const std::string& message);
    void set_state(MotionState state);
    void set_phase(ExecutionPhase phase);
};

} // namespace kuka_riveting
