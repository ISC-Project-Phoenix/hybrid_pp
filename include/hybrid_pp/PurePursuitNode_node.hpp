#pragma once

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

/// Pure pursuit command result, with components
struct CommandCalcResult {
    ackermann_msgs::msg::AckermannDrive command;
    /// Distance to the point on the path used
    float look_ahead_distance{};
    /// Radius of the arc being followed in the command
    double turning_radius{};
};

/// Pure pursuit command result, with components
struct PathCalcResult {
    /// Point on the path we are moving to
    geometry_msgs::msg::PoseStamped intersection_point{};
    /// Distance to the point on the path used
    float look_ahead_distance{};
};

class PurePursuitNode : public rclcpp::Node {
private:
    /// Standard point (0,0), this acts as the center of the rear axle
    const geometry_msgs::msg::Point zero{};

    /// Current speed from odom.
    float current_speed;

    // Parameters
    float min_look_ahead_distance;
    float max_look_ahead_distance;
    float max_speed;
    float wheel_base;
    float gravity_constant;
    std::string rear_axle_frame;
    /// Constant to multiply by speed for look ahead distance calculation.
    float k_dd;
    /// Publishes visualisations if true
    bool debug;

    // Multithreading
    /// Set to true to kill thread
    std::atomic_bool stop_token{false};
    /// Thread that performs the main loop
    std::thread work_thread;
    /// Current path to follow. Should always be in the rear axel frame
    std::optional<nav_msgs::msg::Path::SharedPtr> path;
    /// Node frequency
    rclcpp::WallRate rate;

    // TF
    std::shared_ptr<tf2_ros::TransformListener> transform_listener;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    // Pub/Sub
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr nav_ack_vel_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_vis_marker_pub;

    static float distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
        return std::hypot((float)p1.x - (float)p2.x, (float)p1.y - (float)p2.y);
    }

    /// Publishes a zero move command
    void publish_stop_command() {
        ackermann_msgs::msg::AckermannDrive stop{};
        this->nav_ack_vel_pub->publish(stop);
    }

    std::optional<PathCalcResult> get_path_point();

public:
    PurePursuitNode(const rclcpp::NodeOptions& options);

    /// Callback for receiving paths
    void path_cb(nav_msgs::msg::Path::SharedPtr msg);
    /// Callback for getting current speed from odom.
    void odom_speed_cb(nav_msgs::msg::Odometry::SharedPtr msg);
    /// Publishes markers visualising the pure pursuit geometry.
    void publish_visualisation(float look_ahead_distance, float steering_angle, double distance_to_icr);
    /// Calculates the command to reach the given point.
    CommandCalcResult calculate_command_to_point(const geometry_msgs::msg::PoseStamped& target_point,
                                                 float look_ahead_distance) const;

    ~PurePursuitNode() override { this->stop_token.store(true); }
};
