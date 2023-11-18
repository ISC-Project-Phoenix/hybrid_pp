#pragma once

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <map>
#include <mutex>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

/// Pure pursuit command result, with components
struct CommandCalcResult {
    ackermann_msgs::msg::AckermannDrive command;
    /// Distance to the point on the path used
    float look_ahead_distance{};
    /// Radius of the arc being followed in the command
    double turning_radius{};
};

class PurePursuitNode : public rclcpp::Node {
private:
    /// Current speed from odom.
    float current_speed;
    // Standard point (0,0), this acts as the center of the rear axle
    geometry_msgs::msg::Point zero;
    // The command to be published
    CommandCalcResult command;

    // Parameters
    tf2::Duration transform_tolerance;
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
    /// Current path to follow
    std::optional<nav_msgs::msg::Path::SharedPtr> path;  // TODO make path
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
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    }

    template <class PoseType>
    static float distance_pose(const PoseType& p1, const PoseType& p2) {
        return std::sqrt(std::pow(p1.pose.position.x - p2.pose.position.x, 2) +
                         std::pow(p1.pose.position.y - p2.pose.position.y, 2));
    }

    /// Publishes a zero move command
    void publish_stop_command() {
        ackermann_msgs::msg::AckermannDrive zero{};
        this->nav_ack_vel_pub->publish(zero);
    }

    geometry_msgs::msg::PoseStamped::SharedPtr get_path_point(const nav_msgs::msg::Path::SharedPtr path);

public:
    PurePursuitNode(const rclcpp::NodeOptions& options);

    /// Callback for sending ackerman data after calculating ackerman angle.
    void ackerman_cb(nav_msgs::msg::Path::SharedPtr msg);
    /// Callback for getting current speed from odom.
    void odom_speed_cb(nav_msgs::msg::Odometry::SharedPtr msg);
    /// Publishes markers visualising the pure pursuit geometry.
    void publish_visualisation(float look_ahead_distance, float steering_angle, double distance_to_icr);
    /// Calculates the command to reach the given point.
    CommandCalcResult calculate_command_to_point(geometry_msgs::msg::PoseStamped::SharedPtr target_point) const;

    ~PurePursuitNode() override { this->stop_token.store(true); }
};
