#pragma once

#include <string>
#include <optional>
#include <cstdint>
#include <chrono>

namespace kuka_riveting {

/// Connection-level state of the TCP link to the KRC4 controller
enum class RobotConnectionState {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    ERROR
};

/// Motion-level state reported by the KRC4 controller per command
enum class RobotMotionState {
    IDLE,
    MOVING,
    MOTION_COMPLETE,
    FAULT,
    ESTOP_ACTIVE
};

/**
 * @struct RobotPose
 * @brief Cartesian target pose in KUKA convention (X,Y,Z position + A,B,C orientation)
 *
 * A,B,C follow the standard KUKA Euler angle convention (rotation about
 * Z, Y, X respectively, in degrees). This project only ever commands a
 * fixed tool orientation (perpendicular to the flap surface), so A,B,C
 * default to 0.0 throughout.
 */
struct RobotPose {
    double x, y, z;  // [mm]
    double a, b, c;  // [deg]

    RobotPose(double x = 0.0, double y = 0.0, double z = 0.0,
              double a = 0.0, double b = 0.0, double c = 0.0)
        : x(x), y(y), z(z), a(a), b(b), c(c) {}
};

/// One feedback message received from the controller
struct RobotFeedback {
    uint32_t sequence_id;
    RobotMotionState state;
    RobotPose actual_pose;
    int error_code;
};

/**
 * @class KRC4Interface
 * @brief TCP/IP communication layer to a KUKA KRC4 robot controller
 *
 * Implements a simplified, EKI-inspired (Ethernet KRL Interface) wire
 * protocol: target poses are sent as compact pseudo-XML messages over a
 * persistent TCP connection, and the controller reports motion state +
 * actual position on the same connection.
 *
 * IMPORTANT: The exact message schema here is a demonstrative
 * simplification for this portfolio project, NOT the official KUKA.EKI
 * XML schema. A production deployment requires matching the actual EKI
 * configuration file defined on the KRC4 (KUKA System Software), which
 * is project-specific and set up together with KUKA system integrators.
 * See mock_krc4_server.py for a reference implementation of this
 * simplified protocol's server side, used for local testing.
 *
 * Thread-safety: NOT thread-safe. Intended to be owned and driven by a
 * single calling thread (see KRC4MotionBackend).
 */
class KRC4Interface {
public:
    KRC4Interface();
    ~KRC4Interface();

    // ======================================================================
    // Connection Management
    // ======================================================================

    /// Opens a TCP connection to the controller. Blocks up to timeout_ms.
    bool connect(const std::string& ip_address, uint16_t port, int timeout_ms = 5000);

    void disconnect();
    bool is_connected() const;
    RobotConnectionState connection_state() const;

    // ======================================================================
    // Motion Commands
    // ======================================================================

    /// Sends a single LIN (linear) target pose command, tagged with sequence_id
    /// so the corresponding feedback can be correlated to this specific command.
    bool send_target_pose(const RobotPose& target, double velocity_mms, uint32_t sequence_id);

    /// Sends a software emergency-stop command.
    /// NOT a substitute for a certified hardware E-stop circuit - see
    /// PHASE5 documentation's safety disclaimer.
    bool send_emergency_stop();

    /// Sends a keep-alive / connection health check message
    bool send_heartbeat();

    // ======================================================================
    // Feedback
    // ======================================================================

    /// Waits up to timeout_ms for one feedback message; returns nullopt on
    /// timeout or if the connection is not established.
    std::optional<RobotFeedback> poll_feedback(int timeout_ms = 100);

    /// True if a feedback message was received within max_silence_ms
    bool is_connection_healthy(int max_silence_ms = 1000) const;

private:
    int socket_fd_;
    RobotConnectionState connection_state_;
    std::chrono::steady_clock::time_point last_feedback_time_;

    std::string encode_target_pose_message(
        const RobotPose& target, double velocity_mms, uint32_t seq_id) const;
    std::string encode_estop_message() const;
    std::string encode_heartbeat_message() const;
    std::optional<RobotFeedback> decode_feedback_message(const std::string& raw) const;

    bool send_raw(const std::string& message);
    std::optional<std::string> receive_raw(int timeout_ms);
};

} // namespace kuka_riveting
