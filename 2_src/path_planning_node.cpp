#include "kuka_riveting/path_planning_node.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <functional>

namespace kuka_riveting {

// ============================================================================
// Constructor
// ============================================================================

PathPlanningNode::PathPlanningNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("path_planning_node", options),
      geometry_calculator_(),
      trajectory_generator_(MotionProfile::create_default())
{
    RCLCPP_INFO(get_logger(), "Initializing Path Planning Node...");

    // Load parameters from parameter server
    load_parameters();

    // Create status publisher
    status_publisher_ = create_publisher<std_msgs::msg::String>(
        "path_planning/status", 10);

    // Create the planning service (Phase 6 fix: this call was previously
    // missing entirely, so "/path_planning/plan" never actually existed
    // for clients - including motion_control_node's plan_client_ - to call)
    plan_service_ = create_service<PlanTrajectory>(
        "path_planning/plan",
        std::bind(&PathPlanningNode::handle_plan_request, this,
                  std::placeholders::_1, std::placeholders::_2));

    publish_status("Node initialized successfully");

    RCLCPP_INFO(get_logger(), "Path Planning Node ready for requests");
}

// ============================================================================
// Destructor
// ============================================================================

PathPlanningNode::~PathPlanningNode()
{
    RCLCPP_INFO(get_logger(), "Path Planning Node shutting down");
}

// ============================================================================
// Parameter Loading
// ============================================================================

void PathPlanningNode::load_parameters()
{
    RCLCPP_INFO(get_logger(), "Loading parameters from parameter server...");

    // Declare and read geometry parameters
    declare_parameter("geometry.flap_width_mm", 24000.0);
    declare_parameter("geometry.flap_height_mm", 3500.0);
    declare_parameter("geometry.vertical_pitch_mm", 500.0);
    declare_parameter("geometry.horizontal_pitch_mm", 4000.0);
    declare_parameter("geometry.safety_margin_mm", 100.0);

    geometry_params_.flap_width_mm = 
        get_parameter("geometry.flap_width_mm").as_double();
    geometry_params_.flap_height_mm = 
        get_parameter("geometry.flap_height_mm").as_double();
    geometry_params_.vertical_pitch_mm = 
        get_parameter("geometry.vertical_pitch_mm").as_double();
    geometry_params_.horizontal_pitch_mm = 
        get_parameter("geometry.horizontal_pitch_mm").as_double();
    geometry_params_.safety_margin_mm = 
        get_parameter("geometry.safety_margin_mm").as_double();

    // Declare and read motion profile parameters
    declare_parameter("motion.max_linear_velocity_mms", 500.0);
    declare_parameter("motion.max_linear_acceleration_mms2", 2000.0);
    declare_parameter("motion.time_step_ms", 50.0);

    motion_profile_.max_linear_velocity_mms = 
        get_parameter("motion.max_linear_velocity_mms").as_double();
    motion_profile_.max_linear_acceleration_mms2 = 
        get_parameter("motion.max_linear_acceleration_mms2").as_double();
    motion_profile_.time_step_ms = 
        get_parameter("motion.time_step_ms").as_double();

    // Re-initialize trajectory generator with loaded profile
    trajectory_generator_ = TrajectoryGenerator(motion_profile_);

    std::ostringstream oss;
    oss << "Parameters loaded:\n"
        << "  Flap: " << geometry_params_.flap_width_mm << "×" 
        << geometry_params_.flap_height_mm << " mm\n"
        << "  Pitch: " << geometry_params_.horizontal_pitch_mm << "×"
        << geometry_params_.vertical_pitch_mm << " mm\n"
        << "  Velocity: " << motion_profile_.max_linear_velocity_mms << " mm/s";
    
    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
}

// ============================================================================
// Parameter Validation
// ============================================================================

bool PathPlanningNode::validate_request_parameters(
    const kuka_riveting_msgs::srv::PlanTrajectory::Request& /*request*/,
    std::string& error_message)
{
    // Validates the working geometry_params_/motion_profile_, which
    // apply_parameter_overrides() has already updated in-place with any
    // non-zero fields from `request` - so the request itself needs no
    // further inspection here. Kept as a parameter to match the header's
    // documented signature and for future per-field request validation.

    // Validate dimensions
    if (geometry_params_.flap_width_mm <= 0.0 ||
        geometry_params_.flap_height_mm <= 0.0) {
        error_message = "Flap dimensions must be positive";
        return false;
    }

    // Validate pitches
    if (geometry_params_.horizontal_pitch_mm <= 0.0 ||
        geometry_params_.vertical_pitch_mm <= 0.0) {
        error_message = "Pitch distances must be positive";
        return false;
    }

    // Validate safety margin
    if (geometry_params_.safety_margin_mm < 0.0 ||
        geometry_params_.safety_margin_mm >=
            std::min(geometry_params_.flap_width_mm,
                    geometry_params_.flap_height_mm) / 2.0) {
        error_message = "Safety margin invalid (too large or negative)";
        return false;
    }

    // Validate motion profile
    if (motion_profile_.max_linear_velocity_mms <= 0.0 ||
        motion_profile_.max_linear_acceleration_mms2 <= 0.0) {
        error_message = "Motion limits must be positive";
        return false;
    }

    return true;
}

// ============================================================================
// Parameter Override
// ============================================================================

void PathPlanningNode::apply_parameter_overrides(
    const kuka_riveting_msgs::srv::PlanTrajectory::Request& request)
{
    // Request fields default to 0.0 when left unset by the client, which
    // is never a valid value for any of these quantities - so "non-zero"
    // is used as the "this field was explicitly provided" signal.
    bool any_override = false;

    if (request.flap_width_mm > 0.0) {
        geometry_params_.flap_width_mm = request.flap_width_mm;
        any_override = true;
    }
    if (request.flap_height_mm > 0.0) {
        geometry_params_.flap_height_mm = request.flap_height_mm;
        any_override = true;
    }
    if (request.horizontal_pitch_mm > 0.0) {
        geometry_params_.horizontal_pitch_mm = request.horizontal_pitch_mm;
        any_override = true;
    }
    if (request.vertical_pitch_mm > 0.0) {
        geometry_params_.vertical_pitch_mm = request.vertical_pitch_mm;
        any_override = true;
    }
    if (request.safety_margin_mm > 0.0) {
        geometry_params_.safety_margin_mm = request.safety_margin_mm;
        any_override = true;
    }
    if (request.max_linear_velocity_mms > 0.0) {
        motion_profile_.max_linear_velocity_mms = request.max_linear_velocity_mms;
        any_override = true;
    }
    if (request.max_linear_acceleration_mms2 > 0.0) {
        motion_profile_.max_linear_acceleration_mms2 = request.max_linear_acceleration_mms2;
        any_override = true;
    }

    if (any_override) {
        // Motion profile fields may have changed - rebuild the generator
        trajectory_generator_ = TrajectoryGenerator(motion_profile_);
        RCLCPP_INFO(get_logger(), "Applied request parameter overrides");
    } else {
        RCLCPP_DEBUG(get_logger(), "No parameter overrides in this request");
    }
}

// ============================================================================
// Trajectory Generation (Core Logic)
// ============================================================================

bool PathPlanningNode::generate_complete_plan(
    std::vector<Trajectory>& out_trajectories,
    std::string& error_message)
{
    publish_status("Generating geometry...");

    // Step 1: Generate boring points using Phase 1 module
    try {
        size_t num_points = geometry_calculator_.generate_boring_points(geometry_params_);
        
        std::ostringstream oss;
        oss << "Generated " << num_points << " boring points";
        publish_status(oss.str());
        
        RCLCPP_INFO(get_logger(), "Generated %zu boring points", num_points);
    } catch (const std::exception& e) {
        error_message = std::string("Geometry generation failed: ") + e.what();
        RCLCPP_ERROR(get_logger(), "%s", error_message.c_str());
        return false;
    }

    const auto& boring_points = geometry_calculator_.get_boring_points();

    // Step 2: Generate trajectories between consecutive points
    publish_status("Planning trajectories...");
    out_trajectories.clear();

    for (size_t i = 0; i < boring_points.size() - 1; ++i) {
        try {
            const Point3D& current = boring_points[i];
            const Point3D& next = boring_points[i + 1];

            // Generate trajectory using Phase 2 module
            Trajectory traj = trajectory_generator_.generate_point_to_point(current, next);

            // Validate trajectory against motion limits
            if (!trajectory_generator_.validate_trajectory(traj)) {
                std::ostringstream oss;
                oss << "Trajectory " << i << " exceeds motion limits";
                error_message = oss.str();
                RCLCPP_WARN(get_logger(), "%s", error_message.c_str());
                continue;
            }

            out_trajectories.push_back(traj);

            // Progress update every 5 trajectories
            if ((i + 1) % 5 == 0) {
                std::ostringstream oss;
                oss << "Planned trajectory " << (i + 1) << "/" 
                    << (boring_points.size() - 1);
                publish_status(oss.str());
            }

        } catch (const std::exception& e) {
            std::ostringstream oss;
            oss << "Trajectory generation failed at index " << i 
                << ": " << e.what();
            error_message = oss.str();
            RCLCPP_ERROR(get_logger(), "%s", error_message.c_str());
            return false;
        }
    }

    if (out_trajectories.empty()) {
        error_message = "No valid trajectories generated";
        return false;
    }

    std::ostringstream oss;
    oss << "Planning complete. Generated " << out_trajectories.size() 
        << " trajectories";
    publish_status(oss.str());
    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());

    return true;
}

// ============================================================================
// Response Packing
// ============================================================================

void PathPlanningNode::pack_response(
    const std::vector<Trajectory>& trajectories,
    kuka_riveting_msgs::srv::PlanTrajectory::Response& response)
{
    response.waypoints.clear();
    response.timestamps.clear();
    response.velocities_mms.clear();
    response.total_duration_s = 0.0;

    for (const auto& traj : trajectories) {
        for (const auto& waypoint : traj.waypoints) {
            // Convert Point3D to ROS2 geometry_msgs::Point
            geometry_msgs::msg::Point pt;
            pt.x = waypoint.position_mm.x;
            pt.y = waypoint.position_mm.y;
            pt.z = waypoint.position_mm.z;
            response.waypoints.push_back(pt);

            // Store timestamp and velocity magnitude
            response.timestamps.push_back(waypoint.time_s);
            double vel_mag = waypoint.velocity_mms.distance_to(Point3D(0, 0, 0));
            response.velocities_mms.push_back(vel_mag);
        }

        response.total_duration_s += traj.duration_s;
    }

    response.waypoint_count = static_cast<int32_t>(response.waypoints.size());
    response.trajectory_count = static_cast<int32_t>(trajectories.size());

    RCLCPP_INFO(get_logger(),
        "Packed %d waypoints, total duration: %.2f seconds",
        response.waypoint_count, response.total_duration_s);
}

// ============================================================================
// Status Publishing
// ============================================================================

void PathPlanningNode::publish_status(const std::string& message)
{
    auto msg = std_msgs::msg::String();
    msg.data = message;
    status_publisher_->publish(msg);
}

// ============================================================================
// Service Handler (Main Entry Point)
// ============================================================================

void PathPlanningNode::handle_plan_request(
    const std::shared_ptr<kuka_riveting_msgs::srv::PlanTrajectory::Request> request,
    std::shared_ptr<kuka_riveting_msgs::srv::PlanTrajectory::Response> response)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    RCLCPP_INFO(get_logger(), "Received planning request");
    response->header.stamp = now();

    // Step 1: Apply any parameter overrides from the request
    apply_parameter_overrides(*request);

    // Step 2: Validate parameters
    std::string validation_error;
    if (!validate_request_parameters(*request, validation_error)) {
        RCLCPP_ERROR(get_logger(), "Validation failed: %s", validation_error.c_str());
        response->success = false;
        response->status_message = validation_error;
        return;
    }

    // Step 3: Generate complete trajectory plan
    std::vector<Trajectory> trajectories;
    std::string generation_error;

    if (!generate_complete_plan(trajectories, generation_error)) {
        RCLCPP_ERROR(get_logger(), "Generation failed: %s", generation_error.c_str());
        response->success = false;
        response->status_message = generation_error;
        return;
    }

    // Step 4: Pack trajectories directly into the response message
    pack_response(trajectories, *response);

    // Step 5: Build success status message
    std::ostringstream response_ss;
    response_ss << "SUCCESS: Generated " << trajectories.size()
                << " trajectories with " << response->waypoints.size()
                << " total waypoints. Duration: " << std::fixed
                << std::setprecision(1) << response->total_duration_s << " seconds";
    response->success = true;
    response->status_message = response_ss.str();

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);

    RCLCPP_INFO(get_logger(), "Planning completed in %ld ms", duration.count());
    publish_status(response->status_message);
}

} // namespace kuka_riveting

// ============================================================================
// Main Entry Point
// ============================================================================

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    // Create and run the node
    auto node = std::make_shared<kuka_riveting::PathPlanningNode>();
    
    // Simple single-threaded executor for Phase 3
    // In production, could use MultiThreadedExecutor for concurrent requests
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    
    RCLCPP_INFO(node->get_logger(), "Starting Path Planning Node main loop...");
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
