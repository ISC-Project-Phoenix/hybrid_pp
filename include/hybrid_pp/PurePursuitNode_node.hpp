#pragma once

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <mutex>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "laser_geometry/laser_geometry/laser_geometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
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

struct VisualizationComponents {
    /// Point on the path we are moving to
    geometry_msgs::msg::PoseStamped intersection_point{};
    /// Distance to the point on the path used
    float look_ahead_distance{};
    /// Radius of the arc being followed in the command
    double turning_radius{};
    /// A list of positions of the objects detected by the lidar
    std::vector<geometry_msgs::msg::PoseStamped> objects;
    /// The interpreted set of points between the points given from the planner
    std::optional<std::vector<geometry_msgs::msg::PoseStamped>> path_spline;
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
    /// Radius for obstacle avoidance
    float avoidance_radius;
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
    /// Node frequency
    rclcpp::WallRate rate;
    /// Path lock
    std::mutex spline_mtx;
    /// Odom lock
    std::mutex odom_mtx;
    /// Objects lock
    std::mutex obj_mtx;

    /// The interpreted set of points between the points given from the planner
    std::optional<std::vector<geometry_msgs::msg::PoseStamped>> path_spline;
    /// A list of positions of the objects detected by the lidar
    std::vector<geometry_msgs::msg::PoseStamped> objects;

    // TF
    std::shared_ptr<tf2_ros::TransformListener> transform_listener;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    // Pub/Sub
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_scan_sub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr nav_ack_vel_pub;

    // Visualization publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr travel_path_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr look_ahead_vis_marker_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr intersection_point_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr planner_path_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr object_vis_pub;

    static float distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
        return std::hypot((float)p1.x - (float)p2.x, (float)p1.y - (float)p2.y);
    }

    /// Publishes a zero move command
    void publish_stop_command() {
        ackermann_msgs::msg::AckermannDrive stop{};
        this->nav_ack_vel_pub->publish(stop);
    }

    /// Transforms a path into a given target frame.
    void transform_path(nav_msgs::msg::Path& path, std::string target_frame) {
        auto trans = this->tf_buffer->lookupTransform(target_frame, path.header.frame_id, tf2::TimePointZero);

        for (auto& pose : path.poses) {
            tf2::doTransform(pose, pose, trans);
            pose.header.frame_id = target_frame;
        }
        path.header.frame_id = target_frame;
    }

    /// Transforms a spline into a given target frame.
    void transform_path(std::vector<geometry_msgs::msg::PoseStamped>& path, std::string target_frame) {
        if (path.empty()) {
            return;
        }

        auto trans = this->tf_buffer->lookupTransform(target_frame, path[0].header.frame_id, tf2::TimePointZero);

        for (auto& pose : path) {
            tf2::doTransform(pose, pose, trans);
            pose.header.frame_id = target_frame;
        }
    }

public:
    PurePursuitNode(const rclcpp::NodeOptions& options);

    /// Callback for receiving paths
    void path_cb(nav_msgs::msg::Path::SharedPtr msg);
    /// Callback for getting current speed from odom.
    void odom_speed_cb(nav_msgs::msg::Odometry::SharedPtr msg);
    /// Callback for getting getting and pre-prossesing LIDAR scans
    void lidar_scan_cb(sensor_msgs::msg::LaserScan::SharedPtr msg);
    /// Publishes markers visualising the pure pursuit geometry.
    void publish_visualisation(VisualizationComponents vis_compoenets);
    /// Calculates the command to reach the given point.
    CommandCalcResult calculate_command_to_point(const geometry_msgs::msg::PoseStamped& target_point,
                                                 float look_ahead_distance) const;
    /// Gets a lits of grouped point clusters to be used as an object to avoid
    std::vector<geometry_msgs::msg::PoseStamped> get_objects_from_scan(
        std::shared_ptr<sensor_msgs::msg::LaserScan>& laser_scan);
    /// Get the point where the look ahead distance intersects the path
    std::optional<PathCalcResult> get_path_point(const std::vector<geometry_msgs::msg::PoseStamped>& path_spline);
    /// Creates a spline from the path recived from path planner and also checks if it needs to avoid objects
    std::vector<geometry_msgs::msg::PoseStamped> get_path_spline(const nav_msgs::msg::Path& path);

    ~PurePursuitNode() override { this->stop_token.store(true); }
};
