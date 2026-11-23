#include <sstream>
#include <iomanip>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace kuka_riveting {

// ============================================================================
// State / Phase to string helpers
// ============================================================================

std::string to_string(MotionState state)
{
    switch (state) {
        case MotionState::IDLE:                return "IDLE";
        case MotionState::PLANNING_TRAJECTORY: return "PLANNING_TRAJECTORY";
        case MotionState::EXECUTING:           return "EXECUTING";
        case MotionState::COMPLETED:           return "COMPLETED";
        case MotionState::ERROR:               return "ERROR";
    }
    return "UNKNOWN";
}

std::string to_string(ExecutionPhase phase)
{
    switch (phase) {
        case ExecutionPhase::TRAVELING:   return "TRAVELING";
        case ExecutionPhase::APPROACHING: return "APPROACHING";
        case ExecutionPhase::DRILLING:    return "DRILLING";
        case ExecutionPhase::RETRACTING:  return "RETRACTING";
    }
    return "UNKNOWN";
}

// ============================================================================
// Constructor
// ============================================================================

MotionControlNode::MotionControlNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("motion_control_node", options),
      geometry_calculator_(),
      trajectory_generator_(MotionProfile::create_default()),
      current_state_(MotionState::IDLE),
      current_phase_(ExecutionPhase::TRAVELING)
{
    RCLCPP_INFO(get_logger(), "Initializing Motion Control Node...");

    load_parameters();

    // Status publisher
    status_publisher_ = create_publisher<std_msgs::msg::String>(
        "motion_control/status", 10);

    // Service client to Phase 3 (Path Planning Node)
    plan_client_ = create_client<PlanTrajectory>("/path_planning/plan");

    // Action server for the riveting execution goal
    action_server_ = rclcpp_action::create_server<ExecuteRiveting>(
        this,
        "motion_control/execute_riveting",
        std::bind(&MotionControlNode::handle_goal, this, _1, _2),
        std::bind(&MotionControlNode::handle_cancel, this, _1),
        std::bind(&MotionControlNode::handle_accepted, this, _1));

    publish_status("Motion Control Node initialized successfully");
    RCLCPP_INFO(get_logger(),
        "Motion Control Node ready. Action server: motion_control/execute_riveting");
}

// ============================================================================
// Destructor
// ============================================================================

MotionControlNode::~MotionControlNode()
{
    RCLCPP_INFO(get_logger(), "Motion Control Node shutting down");
}

// ============================================================================
// Parameter Loading
// ============================================================================

void MotionControlNode::load_parameters()
{
    RCLCPP_INFO(get_logger(), "Loading parameters from parameter server...");

    // Geometry parameters (must match Path Planning Node defaults for
    // deterministic, reproducible boring point generation)
    declare_parameter("geometry.flap_width_mm", 24000.0);
    declare_parameter("geometry.flap_height_mm", 3500.0);
    declare_parameter("geometry.vertical_pitch_mm", 500.0);
    declare_parameter("geometry.horizontal_pitch_mm", 4000.0);
    declare_parameter("geometry.safety_margin_mm", 100.0);

    geometry_params_.flap_width_mm = get_parameter("geometry.flap_width_mm").as_double();
    geometry_params_.flap_height_mm = get_parameter("geometry.flap_height_mm").as_double();
    geometry_params_.vertical_pitch_mm = get_parameter("geometry.vertical_pitch_mm").as_double();
    geometry_params_.horizontal_pitch_mm = get_parameter("geometry.horizontal_pitch_mm").as_double();
    geometry_params_.safety_margin_mm = get_parameter("geometry.safety_margin_mm").as_double();

    // Motion profile parameters
    declare_parameter("motion.max_linear_velocity_mms", 500.0);
    declare_parameter("motion.max_linear_acceleration_mms2", 2000.0);
    declare_parameter("motion.time_step_ms", 50.0);

    motion_profile_.max_linear_velocity_mms = get_parameter("motion.max_linear_velocity_mms").as_double();
    motion_profile_.max_linear_acceleration_mms2 = get_parameter("motion.max_linear_acceleration_mms2").as_double();
    motion_profile_.time_step_ms = get_parameter("motion.time_step_ms").as_double();

    trajectory_generator_ = TrajectoryGenerator(motion_profile_);

    // Drilling cycle timing
    declare_parameter("drilling.approach_duration_s", 2.0);
    declare_parameter("drilling.drilling_duration_s", 1.5);
    declare_parameter("drilling.retract_duration_s", 1.0);

    approach_duration_s_ = get_parameter("drilling.approach_duration_s").as_double();
    drilling_duration_s_ = get_parameter("drilling.drilling_duration_s").as_double();
    retract_duration_s_  = get_parameter("drilling.retract_duration_s").as_double();

    RCLCPP_INFO(get_logger(),
        "Drilling cycle: approach=%.2fs, drill=%.2fs, retract=%.2fs",
        approach_duration_s_, drilling_duration_s_, retract_duration_s_);
}

// ============================================================================
// Action Server Callbacks
// ============================================================================

rclcpp_action::GoalResponse MotionControlNode::handle_goal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const ExecuteRiveting::Goal> /*goal*/)
{
    RCLCPP_INFO(get_logger(), "Received riveting execution goal request");

    // Only accept a new goal if the node is currently idle
    if (current_state_.load() != MotionState::IDLE) {
        RCLCPP_WARN(get_logger(), "Rejecting goal: node is busy (current state: %s)", to_string(current_state_.load()).c_str() );
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionControlNode::handle_cancel(
    const std::shared_ptr<GoalHandleExecuteRiveting> /*goal_handle*/)
{
    RCLCPP_INFO(get_logger(), "Received cancel request");
    // Always accept cancellation; the execution loop checks is_canceling()
    // between every drilling phase and every travel waypoint.
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionControlNode::handle_accepted(
    const std::shared_ptr<GoalHandleExecuteRiveting> goal_handle)
{
    // Execution must not block the executor thread, so run it separately.
    std::thread{ std::bind(&MotionControlNode::execute, this, std::placeholders::_1), goal_handle }.detach();
}

// ============================================================================
// Main Execution Routine
// ============================================================================

void MotionControlNode::execute(
    const std::shared_ptr<GoalHandleExecuteRiveting> goal_handle)
{
    auto exec_start_time = std::chrono::steady_clock::now();

    // --- STATE: VALIDATE PLANNING_TRAJECTORY --------------------------------
    set_state(MotionState::PLANNING_TRAJECTORY);
    publish_status("Requesting trajectory plan from Path Planning service...");

    std::string plan_error;
    if (!request_trajectory_plan(plan_error)) {
        set_state(MotionState::ERROR);

        auto result = std::make_shared<ExecuteRiveting::Result>();
        result->success = false;
        result->message = plan_error;
        result->points_completed = 0;
        result->total_points = 0;
        result->total_execution_time_s = 0.0;

        RCLCPP_ERROR(get_logger(), "Planning failed: %s", plan_error.c_str());
        goal_handle->abort(result);

        set_state(MotionState::IDLE);
        return;
    }

    // --- STATE: EXECUTING ---------------------------------------------------
    set_state(MotionState::EXECUTING);
    publish_status("Trajectory plan validated. Beginning execution.");

    int points_completed = 0;
    bool execution_success = execute_point_sequence(goal_handle, points_completed);

    auto exec_end_time = std::chrono::steady_clock::now();
    double total_time_s = std::chrono::duration<double>(
        exec_end_time - exec_start_time).count();

    auto result = std::make_shared<ExecuteRiveting::Result>();
    result->points_completed = points_completed;
    result->total_points = static_cast<int32_t>(geometry_calculator_.get_boring_points().size());
    result->total_execution_time_s = total_time_s;

    // Cancellation takes priority over success/failure
    if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Execution canceled by client request";
        publish_status(result->message);
        goal_handle->canceled(result);
        set_state(MotionState::IDLE);
        return;
    }

    if (execution_success) {
        result->success = true;
        std::ostringstream oss;
        oss << "Riveting execution completed successfully: "
            << points_completed << "/" << result->total_points
            << " points in " << std::fixed << std::setprecision(1)
            << total_time_s << " seconds";
        result->message = oss.str();

        set_state(MotionState::COMPLETED);
        publish_status(result->message);
        RCLCPP_INFO(get_logger(), "%s", result->message.c_str());
        goal_handle->succeed(result);

        // Return to IDLE so the node can accept the next goal
        set_state(MotionState::IDLE);
    } else {
        result->success = false;
        result->message = "Execution failed before completing all points";

        set_state(MotionState::ERROR);
        publish_status(result->message);
        RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
        goal_handle->abort(result);

        set_state(MotionState::IDLE);
    }
}

// ============================================================================
// Trajectory Plan Request (Service Client to Phase 3)
// ============================================================================

bool MotionControlNode::request_trajectory_plan(std::string& error_message)
{
    // Wait for the Path Planning service to become available
    if (!plan_client_->wait_for_service(5s)) {
        error_message = "Path Planning service '/path_planning/plan' not available "
                         "(is the Path Planning Node running?)";
        return false;
    }

    auto request = std::make_shared<PlanTrajectory::Request>();
    // Fields left at zero -> Path Planning Node falls back to its own
    // parameter-server defaults, which this node assumes match its own
    // geometry_params_ / motion_profile_ for deterministic local execution.

    auto future = plan_client_->async_send_request(request);

    // This call runs on a dedicated std::thread (spawned in handle_accepted),
    // while the MultiThreadedExecutor spins on the main thread and services
    // the pending future in the background.
    auto status = future.wait_for(10s);
    if (status != std::future_status::ready) {
        error_message = "Path Planning service call timed out after 10 seconds";
        return false;
    }

    auto response = future.get();
    if (!response->success) {
        error_message = "Path Planning reported failure: " + response->status_message;
        return false;
    }

    RCLCPP_INFO(get_logger(),
        "Plan validated: %d waypoints, %d trajectories, %.1f s total duration",
        response->waypoint_count, response->trajectory_count,
        response->total_duration_s);

    // Regenerate the same boring points locally and deterministically.
    // Same geometry parameters always yield the same point set (Phase 1),
    // so this gives the execution loop direct Point3D access without
    // needing to parse the flattened waypoint array from the service response.
    geometry_calculator_.generate_boring_points(geometry_params_);

    return true;
}

// ============================================================================
// Point-by-Point Execution Loop
// ============================================================================

bool MotionControlNode::execute_point_sequence(
    const std::shared_ptr<GoalHandleExecuteRiveting>& goal_handle,
    int& points_completed)
{
    const auto& boring_points = geometry_calculator_.get_boring_points();
    const size_t total_points = boring_points.size();
    points_completed = 0;

    if (total_points == 0) {
        publish_status("No boring points to execute");
        return false;
    }

    auto exec_start = std::chrono::steady_clock::now();

    for (size_t i = 0; i < total_points; ++i) {
        if (goal_handle->is_canceling()) {
            publish_status("Execution canceled during point sequence");
            return false;
        }

        // TRAVELING: move from the previous point to this one (skip for
        // the very first point - the tool is assumed already positioned there)
        if (i > 0) {
            Trajectory travel = trajectory_generator_.generate_point_to_point(boring_points[i - 1], boring_points[i]);

            if (!trajectory_generator_.validate_trajectory(travel)) {
                std::ostringstream oss;
                oss << "Travel trajectory to point " << i << " exceeds motion limits";
                publish_status(oss.str());
                RCLCPP_ERROR(get_logger(), "%s", oss.str().c_str());
                return false;
            }

            if (!simulate_travel_phase(travel, goal_handle)) {
                return false;  // canceled mid-travel
            }
        }

        // APPROACH -> DRILL -> RETRACT cycle at this boring point
        if (!simulate_drilling_cycle(boring_points[i], goal_handle, i, total_points)) {
            return false;  // canceled mid-cycle
        }

        points_completed++;

        double elapsed_s = std::chrono::duration<double>( std::chrono::steady_clock::now() - exec_start ).count();
        publish_feedback(goal_handle, i, total_points, boring_points[i], elapsed_s);
    }

    return true;
}

// ============================================================================
// Travel Simulation (between boring points)
// ============================================================================

bool MotionControlNode::simulate_travel_phase(
    const Trajectory& travel,
    const std::shared_ptr<GoalHandleExecuteRiveting>& goal_handle)
{
    set_phase(ExecutionPhase::TRAVELING);

    // Step through the trajectory at its native sampling rate (time_step_ms),
    // matching the cadence Phase 2 used when generating the waypoints.
    const auto step_duration = std::chrono::milliseconds(
        static_cast<int>(motion_profile_.time_step_ms));

    for (size_t w = 0; w < travel.waypoints.size(); ++w) {
        if (goal_handle->is_canceling()) {
            return false;
        }
        std::this_thread::sleep_for(step_duration);
    }

    return true;
}

// ============================================================================
// Drilling Cycle Simulation (at a single boring point)
// ============================================================================

bool MotionControlNode::simulate_drilling_cycle(
    const Point3D& target_point,
    const std::shared_ptr<GoalHandleExecuteRiveting>& goal_handle,
    size_t point_index,
    size_t total_points)
{
    std::ostringstream prefix;
    prefix << "Point " << (point_index + 1) << "/" << total_points;

    // --- APPROACHING ---
    set_phase(ExecutionPhase::APPROACHING);
    publish_status(prefix.str() + ": approaching drill position");
    std::this_thread::sleep_for(std::chrono::duration<double>(approach_duration_s_));
    if (goal_handle->is_canceling()) return false;

    // --- DRILLING ---
    set_phase(ExecutionPhase::DRILLING);
    publish_status(prefix.str() + ": drilling");
    std::this_thread::sleep_for(std::chrono::duration<double>(drilling_duration_s_));
    if (goal_handle->is_canceling()) return false;

    // --- RETRACTING ---
    set_phase(ExecutionPhase::RETRACTING);
    publish_status(prefix.str() + ": retracting tool");
    std::this_thread::sleep_for(std::chrono::duration<double>(retract_duration_s_));
    if (goal_handle->is_canceling()) return false;

    RCLCPP_INFO(get_logger(), "%s: rivet hole complete at (%.1f, %.1f, %.1f) mm",
        prefix.str().c_str(), target_point.x, target_point.y, target_point.z);

    return true;
}

// ============================================================================
// Feedback Publishing
// ============================================================================

void MotionControlNode::publish_feedback(
    const std::shared_ptr<GoalHandleExecuteRiveting>& goal_handle,
    size_t point_index,
    size_t total_points,
    const Point3D& position,
    double elapsed_time_s)
{
    auto feedback = std::make_shared<ExecuteRiveting::Feedback>();

    feedback->current_point_index = static_cast<int32_t>(point_index);
    feedback->total_points = static_cast<int32_t>(total_points);
    feedback->current_state = to_string(current_state_.load());
    feedback->current_phase = to_string(current_phase_.load());
    feedback->progress_percent =
        100.0 * static_cast<double>(point_index + 1) / static_cast<double>(total_points);

    feedback->current_position.x = position.x;
    feedback->current_position.y = position.y;
    feedback->current_position.z = position.z;
    feedback->elapsed_time_s = elapsed_time_s;

    goal_handle->publish_feedback(feedback);
}

// ============================================================================
// Status Publishing & State Setters
// ============================================================================

void MotionControlNode::publish_status(const std::string& message)
{
    auto msg = std_msgs::msg::String();
    msg.data = message;
    status_publisher_->publish(msg);
}

void MotionControlNode::set_state(MotionState state)
{
    current_state_.store(state);
    RCLCPP_DEBUG(get_logger(), "State -> %s", to_string(state).c_str());
}

void MotionControlNode::set_phase(ExecutionPhase phase)
{
    current_phase_.store(phase);
    RCLCPP_DEBUG(get_logger(), "Phase -> %s", to_string(phase).c_str());
}

} // namespace kuka_riveting

// ============================================================================
// Main Entry Point
// ============================================================================

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<kuka_riveting::MotionControlNode>();

    // MultiThreadedExecutor: the action's execute() callback runs in its own
    // detached std::thread, but the executor still needs free threads to
    // process the Path Planning service response, goal/cancel requests, and
    // feedback publishing concurrently with that execution thread.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    RCLCPP_INFO(node->get_logger(), "Starting Motion Control Node main loop...");
    executor.spin();

    rclcpp::shutdown();
    return 0;
}