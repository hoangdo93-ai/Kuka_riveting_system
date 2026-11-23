#pragma once

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"

// Include our Phase 1 & 2 modules
#include "kuka_riveting/geometry_calculator.hpp"
#include "kuka_riveting/trajectory_generator.hpp"

// ROS2 Service & Message types will be auto-generated
// We forward-declare them here
namespace kuka_riveting_msgs {
namespace srv {
struct PlanTrajectory;
} // namespace srv
} // namespace kuka_riveting_msgs

namespace kuka_riveting {

/**
 * @class PathPlanningNode
 * @brief ROS2 Node for A380 riveting trajectory planning
 * 
 * Responsibilities:
 * - Wrap GeometryCalculator (Phase 1) and TrajectoryGenerator (Phase 2)
 * - Provide /plan service for trajectory planning requests
 * - Publish status updates on /status topic
 * - Manage ROS2 parameters and lifecycle
 */
class PathPlanningNode : public rclcpp::Node {
public:
    /// Constructor: Initialize node, setup services, load parameters
    explicit PathPlanningNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /// Destructor
    ~PathPlanningNode() override;

private:
    // ======================================================================
    // ROS2 Components
    // ======================================================================
    
    /// Service server: /path_planning/plan
    rclcpp::Service<kuka_riveting_msgs::srv::PlanTrajectory>::SharedPtr plan_service_;
    
    /// Publisher: /path_planning/status
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;

    // ======================================================================
    // Our Modules (Phase 1 & 2)
    // ======================================================================
    
    /// Geometry calculator from Phase 1
    GeometryCalculator geometry_calculator_;
    
    /// Trajectory generator from Phase 2
    TrajectoryGenerator trajectory_generator_;

    // ======================================================================
    // Parameters
    // ======================================================================
    
    /// Geometry parameters loaded from parameter server
    GeometryParams geometry_params_;
    
    /// Motion profile loaded from parameter server
    MotionProfile motion_profile_;

    // ======================================================================
    // Service Handler
    // ======================================================================
    
    /**
     * @brief Callback for /plan service requests
     * 
     * Workflow:
     * 1. Receive request with optional parameter overrides
     * 2. Validate all parameters
     * 3. Generate boring points using GeometryCalculator
     * 4. Generate trajectories using TrajectoryGenerator
     * 5. Package response with all waypoints
     */
    void handle_plan_request(
        const std::shared_ptr<kuka_riveting_msgs::srv::PlanTrajectory::Request> request,
        std::shared_ptr<kuka_riveting_msgs::srv::PlanTrajectory::Response> response);

    // ======================================================================
    // Helper Functions
    // ======================================================================
    
    /**
     * @brief Load parameters from ROS2 parameter server
     * 
     * Reads geometry and motion parameters, handles defaults
     */
    void load_parameters();

    /**
     * @brief Validate planning request parameters
     * 
     * Checks:
     * - Positive dimensions
     * - Reasonable pitch distances
     * - Safety margins not larger than flap
     * - Velocity/acceleration limits positive
     */
    bool validate_request_parameters(
        const kuka_riveting_msgs::srv::PlanTrajectory::Request& request,
        std::string& error_message);

    /**
     * @brief Apply request parameter overrides to working parameters
     * 
     * If request has non-zero values, override the defaults
     * Otherwise keep parameter server values
     */
    void apply_parameter_overrides(
        const kuka_riveting_msgs::srv::PlanTrajectory::Request& request);

    /**
     * @brief Generate complete trajectory plan
     * 
     * High-level orchestration:
     * 1. Generate boring points
     * 2. For each adjacent pair: generate trajectory
     * 3. Collect all waypoints
     * 4. Return success status
     */
    bool generate_complete_plan(
        std::vector<Trajectory>& out_trajectories,
        std::string& error_message);

    /**
     * @brief Pack trajectories into ROS2 response message
     * 
     * Flattens trajectory waypoints into arrays suitable for
     * ROS2 message passing
     */
    void pack_response(
        const std::vector<Trajectory>& trajectories,
        kuka_riveting_msgs::srv::PlanTrajectory::Response& response);

    /**
     * @brief Publish status message to /status topic
     * 
     * Used for progress updates during planning
     */
    void publish_status(const std::string& message);
};

} // namespace kuka_riveting
