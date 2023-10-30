#include "hybrid_pp/PurePursuitNode_node.hpp"

// For _1
using namespace std::placeholders;

// For 1000ms
using namespace std::chrono_literals;

PurePursuitNode::PurePursuitNode(const rclcpp::NodeOptions& options)
    : Node("PurePursuitNode", options), rate(this->declare_parameter<float>("frequency", 20)) {
    // Params
    min_look_ahead_distance = this->declare_parameter<float>("min_look_ahead_distance",
                                                             3.85);  // This is set to the min turning radius of phnx
    max_look_ahead_distance = this->declare_parameter<float>("max_look_ahead_distance", 10.0);
    k_dd = this->declare_parameter<float>("k_dd", 0.7);
    max_speed = this->declare_parameter<float>("max_speed", 6.7056);
    rear_axle_frame = this->declare_parameter<std::string>("rear_axle_frame", "rear_axle");
    wheel_base = this->declare_parameter<float>("wheel_base", 1.08);
    gravity_constant = this->declare_parameter<float>("gravity_constant", 9.81);
    debug = this->declare_parameter<bool>("debug", true);

    // Var init
    current_speed = 1;

    // Pub Sub
    goal_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 1, std::bind(&PurePursuitNode::ackerman_cb, this, _1));
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom_can", 1,
                                                                  std::bind(&PurePursuitNode::odom_speed_cb, this, _1));
    nav_ack_vel_pub = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("/nav_ack_vel", 1);
    path_vis_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 1);

    // TF
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    this->work_thread = std::thread{[this]() {
        RCLCPP_INFO(this->get_logger(), "Beginning pure pursuit loop...");
        while (this->rate.sleep()) {
            // Break if node is dying
            if (this->stop_token.load()) {
                break;
            }

            // Wait until we have path
            if (!this->path.has_value()) {
                continue;
            }

            // TODO to make this work with paths, we first need to select a point on the path to use below
            // We also need to add stopping behavior if there are no reachable points

            //TODO remove when we use path, since PP algo removes need for this
            // For now this considers the waypoint reached if its under our steering radius
            {
                auto trans = this->tf_buffer->lookupTransform(this->rear_axle_frame, (*this->path)->header.frame_id,
                                                              tf2::TimePointZero);
                geometry_msgs::msg::PoseStamped transformed_goal_pose{};
                tf2::doTransform(**this->path, transformed_goal_pose, trans);

                RCLCPP_INFO(this->get_logger(), "Distance to goal %f",
                            distance_pose(transformed_goal_pose, geometry_msgs::msg::PoseStamped{}));

                // Stop tracking if too close
                if (distance_pose(transformed_goal_pose, geometry_msgs::msg::PoseStamped{}) < 3.85) {
                    this->path = std::nullopt;
                    this->publish_stop_command();
                    continue;
                }
            }

            // Calculate command to point on path
            auto command = this->calculate_command_to_point(*this->path);

            nav_ack_vel_pub->publish(command.command);

            if (debug) {
                publish_visualisation(command.look_ahead_distance, command.command.steering_angle,
                                      command.turning_radius);
            }
        }
    }};
}

void PurePursuitNode::ackerman_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->path = msg; }

CommandCalcResult PurePursuitNode::calculate_command_to_point(
    geometry_msgs::msg::PoseStamped::SharedPtr target_point) const {
    // Transform goal pose to rear_axle frame
    auto trans =
        this->tf_buffer->lookupTransform(this->rear_axle_frame, target_point->header.frame_id, tf2::TimePointZero);
    geometry_msgs::msg::PoseStamped transformed_goal_pose{};
    tf2::doTransform(*target_point, transformed_goal_pose, trans);

    // Calc look ahead distance
    float look_ahead_distance = std::clamp(k_dd * current_speed, min_look_ahead_distance, max_look_ahead_distance);

    // Calculate the angle between the robot and the goal.
    float alpha = std::atan2(transformed_goal_pose.pose.position.y, transformed_goal_pose.pose.position.x);

    // Set the desired steering angle and set it to the ackerman message
    float steering_angle = std::atan(2.0 * wheel_base * std::sin(alpha) / look_ahead_distance);
    ackermann_msgs::msg::AckermannDrive ack_msg;
    ack_msg.steering_angle = steering_angle;

    // Gets the distance to the instantaneous center of rotation from the center of the rear axle (turning radius)
    float distance_to_icr = wheel_base / std::tan(steering_angle);

    // Set the speed based off the eq v = sqrt(static_friction * gravity * turn_radius) with static friction being 1.
    // This finds the fastest speed that can be taken without breaking friction and slipping.
    float set_speed = std::clamp(std::sqrt(this->gravity_constant * std::abs(distance_to_icr)), 0.1f, this->max_speed);
    ack_msg.speed = set_speed;

    CommandCalcResult out{ack_msg, look_ahead_distance, distance_to_icr};
    return out;
}

void PurePursuitNode::publish_visualisation(float look_ahead_distance, float steering_angle, double distance_to_icr) {
    visualization_msgs::msg::Marker path_prediction_marker{};
    // Declares the path prediction marker in the rear axle frame
    path_prediction_marker.header.frame_id = rear_axle_frame;
    path_prediction_marker.header.stamp = get_clock()->now();
    path_prediction_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_prediction_marker.action = visualization_msgs::msg::Marker::ADD;
    path_prediction_marker.ns = "hybrid_pp_ns";
    path_prediction_marker.id = 0;
    path_prediction_marker.pose.orientation.w = 1.0;
    path_prediction_marker.scale.x = 0.1;
    path_prediction_marker.color.a = 1.0;
    path_prediction_marker.color.g = 1.0;

    visualization_msgs::msg::Marker look_ahead_distance_marker{};
    // Declares the look ahead marker in the rear axle frame
    look_ahead_distance_marker.header.frame_id = rear_axle_frame;
    look_ahead_distance_marker.header.stamp = get_clock()->now();
    look_ahead_distance_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    look_ahead_distance_marker.action = visualization_msgs::msg::Marker::ADD;
    look_ahead_distance_marker.ns = "hybrid_pp_ns";
    look_ahead_distance_marker.id = 1;
    look_ahead_distance_marker.pose.orientation.w = 1.0;
    look_ahead_distance_marker.scale.x = 0.1;
    look_ahead_distance_marker.color.a = 1.0;
    look_ahead_distance_marker.color.r = 1.0;

    // Standard point (0,0), this acts as the center of the rear axle
    geometry_msgs::msg::Point zero;

    // Position used for plotting the visualization markers
    geometry_msgs::msg::Point path_graph_point;
    geometry_msgs::msg::Point look_ahead_graph_point;

    // If our steering angle is negative, graphing math is a little different
    if (steering_angle < 0) {
        // Loop runs until the point generated hits the look ahead distance raduis
        for (double index = 0; distance(path_graph_point, zero) - look_ahead_distance <= 0; index += 0.001) {
            // Sets the x and y coords using polar coodiants the the ICR as the origin
            path_graph_point.x = distance_to_icr * -sin(index);
            path_graph_point.y = distance_to_icr * -cos(index) + distance_to_icr;

            // Appends the generated point to the markers list of points
            path_prediction_marker.points.push_back(path_graph_point);
        }
    } else {
        // Loop runs until the point generated hits the look ahead distance raduis
        for (double index = -3.14 / 2; distance(path_graph_point, zero) - look_ahead_distance <= 0; index += 0.001) {
            // Sets the x and y coords using ploar coodiants the the ICR as the origin
            path_graph_point.x = distance_to_icr * cos(index);
            path_graph_point.y = distance_to_icr * sin(index) + distance_to_icr;

            // Appends the generated point to the markers list of points
            path_prediction_marker.points.push_back(path_graph_point);
        }
    }

    // Appends a list of points in a circle to the look ahead distance marker using look ahead distance as radius
    for (double index = 0; index <= 2 * 3.14; index += 0.01) {
        look_ahead_graph_point.x = look_ahead_distance * cos(index);
        look_ahead_graph_point.y = look_ahead_distance * sin(index);

        look_ahead_distance_marker.points.push_back(look_ahead_graph_point);
    }

    // Publish the markers to the visualization_marker topic
    path_vis_marker_pub->publish(path_prediction_marker);
    path_vis_marker_pub->publish(look_ahead_distance_marker);
}

void PurePursuitNode::odom_speed_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_speed = msg->twist.twist.linear.x;
}
