#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "../include/MotionControl.hpp"

MotionControlNode::MotionControlNode() : 
    rclcpp::Node("motion_control_node"),
    yaw_error_(0.0),
    collision_threshold_normal_(0.3),
    collision_threshold_rotating_(0.5),
    collision_threshold_(0.3)  // Initialize base threshold if needed
{
    // Parameter declarations
    this->declare_parameter<double>("max_linear_speed", 0.3);
    this->declare_parameter<double>("max_angular_speed", 0.5);
    this->declare_parameter<double>("lookahead_distance", 0.3);
    this->declare_parameter<double>("k_p", 0.2);
    this->declare_parameter<double>("collision_threshold_normal", 0.3);
    this->declare_parameter<double>("collision_threshold_rotating", 0.5);

    // Parameter assignments
    max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
    max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
    lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
    k_p_ = this->get_parameter("k_p").as_double();
    collision_threshold_normal_ = this->get_parameter("collision_threshold_normal").as_double();
    collision_threshold_rotating_ = this->get_parameter("collision_threshold_rotating").as_double();

        // Subscribers
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry", 10, std::bind(&MotionControlNode::odomCallback, this, std::placeholders::_1));
 
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MotionControlNode::lidarCallback, this, std::placeholders::_1));

        // Publisher
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Client for path planning
        plan_client_ = this->create_client<nav_msgs::srv::GetPlan>("/plan_path");

        // Action server
        nav_server_ = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
           this,
            "go_to_goal",
            std::bind(&MotionControlNode::navHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MotionControlNode::navHandleCancel, this, std::placeholders::_1),
            std::bind(&MotionControlNode::navHandleAccepted, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Motion control node started.");
    }

void MotionControlNode::checkCollision() {
    if (laser_scan_.ranges.empty()) return;

    // Adjust threshold based on robot state
    double current_threshold = (fabs(yaw_error_) > 0.2) ? 
                             collision_threshold_rotating_ : 
                             collision_threshold_normal_;

    // Check front sector of the laser scan
    size_t center_index = laser_scan_.ranges.size() / 2;
    size_t sector_size = laser_scan_.ranges.size() / 8; // Check 1/8th of the scan in front
    
    for (size_t i = center_index - sector_size/2; i < center_index + sector_size/2; ++i) {
        if (laser_scan_.ranges[i] < current_threshold) {
            geometry_msgs::msg::Twist stop;
            twist_publisher_->publish(stop);
            
            if (goal_handle_) {
                auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
                goal_handle_->abort(result);
                RCLCPP_WARN(get_logger(), "Emergency stop activated due to obstacle!");
            }
            return;
        }
    }
}

void MotionControlNode::updateTwist() {
    // Return immediately if no path or no active goal
    if (path_.poses.empty() || !goal_handle_) return;

    // 1. Find the immediate target point on the path
    geometry_msgs::msg::PointStamped target_point;
    double min_approach_dist = 0.3; // meters - minimum distance to start following
    bool found = false;
    
    // Find first path point that's at least min_approach_dist away
    for (size_t i = 0; i < path_.poses.size(); ++i) {
        double dx = path_.poses[i].pose.position.x - current_pose_.pose.position.x;
        double dy = path_.poses[i].pose.position.y - current_pose_.pose.position.y;
        double dist = sqrt(dx*dx + dy*dy);
        
        if (dist >= min_approach_dist) {
            target_point.point = path_.poses[i].pose.position;
            target_point.header = path_.poses[i].header;
            found = true;
            break;
        }
    }
    
    // If no point found, use the final goal point
    if (!found && !path_.poses.empty()) {
        target_point.point = path_.poses.back().pose.position;
        target_point.header = path_.poses.back().header;
    }

    // 2. Calculate current and target orientations
    // Get current robot yaw from quaternion
    tf2::Quaternion q(
        current_pose_.pose.orientation.x,
        current_pose_.pose.orientation.y,
        current_pose_.pose.orientation.z,
        current_pose_.pose.orientation.w);
    double roll, pitch, current_yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, current_yaw);
    
    // Calculate desired yaw toward target point
    double target_dx = target_point.point.x - current_pose_.pose.position.x;
    double target_dy = target_point.point.y - current_pose_.pose.position.y;
    double target_yaw = atan2(target_dy, target_dx);
    
    // Calculate and normalize yaw error
    yaw_error_ = target_yaw - current_yaw;
    while (yaw_error_ > M_PI) yaw_error_ -= 2*M_PI;
    while (yaw_error_ < -M_PI) yaw_error_ += 2*M_PI;

    // 3. Generate appropriate twist command
    geometry_msgs::msg::Twist twist;
    
    // Phase 1: Orient toward path (rotate in place if angle error is large)
    if (fabs(yaw_error_) > 0.2) { // ~11.5 degree threshold
        twist.linear.x = 0.0; // No forward motion
        twist.angular.z = std::clamp(k_p_ * yaw_error_, 
                                   -max_angular_speed_, 
                                    max_angular_speed_);
    } 
    // Phase 2: Move forward while following path
    else {
        // Pure pursuit calculations
        // Transform target point to robot frame
        double x = target_dx * cos(current_yaw) + target_dy * sin(current_yaw);
        double y = -target_dx * sin(current_yaw) + target_dy * cos(current_yaw);
        
        // Calculate desired curvature
        double curvature = 2.0 * y / (x*x + y*y);
        
        // Calculate forward speed (reduces when turning sharply)
        double forward_speed = std::min(max_linear_speed_, 
                                      max_linear_speed_ * (1.0 - 0.5*std::abs(curvature)));
        
        // Calculate angular speed (proportional to curvature)
        double angular_speed = curvature * forward_speed * k_p_;
        
        // Set the twist commands
        twist.linear.x = forward_speed;
        twist.angular.z = std::clamp(angular_speed, 
                                   -max_angular_speed_, 
                                    max_angular_speed_);
    }

    // 4. Publish the twist command
    twist_publisher_->publish(twist);
}

rclcpp_action::GoalResponse MotionControlNode::navHandleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal) {
    (void)uuid;
    goal_pose_ = goal->pose;
    RCLCPP_INFO(get_logger(), "Received new navigation goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionControlNode::navHandleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionControlNode::navHandleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    goal_handle_ = goal_handle;
    
    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    request->start = current_pose_;
    request->goal = goal_pose_;  // Changed from goal_pose_.pose to goal_pose_
    request->tolerance = 0.1;
    
    auto future = plan_client_->async_send_request(
        request, std::bind(&MotionControlNode::pathCallback, this, std::placeholders::_1));
}

void MotionControlNode::execute() {
    rclcpp::Rate loop_rate(10);
    auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();

    try {
        while (rclcpp::ok() && goal_handle_ && goal_handle_->is_active()) {
            if (goal_handle_->is_canceling()) {
                goal_handle_->canceled(result);
                RCLCPP_INFO(get_logger(), "Goal canceled");
                return;
            }

            // Check if we have valid path and pose data
            if (path_.poses.empty() || current_pose_.header.frame_id.empty()) {
                RCLCPP_WARN(get_logger(), "Missing path or pose data");
                goal_handle_->abort(result);
                return;
            }

            // Calculate distance to goal
            double dx = goal_pose_.pose.position.x - current_pose_.pose.position.x;
            double dy = goal_pose_.pose.position.y - current_pose_.pose.position.y;
            double distance = sqrt(dx*dx + dy*dy);

            // Publish feedback
            auto feedback = std::make_shared<nav2_msgs::action::NavigateToPose::Feedback>();
            feedback->distance_remaining = distance;
            goal_handle_->publish_feedback(feedback);

            // Check if goal reached
            if (distance < 0.1) {  // 10cm threshold
                goal_handle_->succeed(result);
                RCLCPP_INFO(get_logger(), "Goal reached");
                return;
            }

            loop_rate.sleep();
        }
    } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Exception in execute: %s", e.what());
        if (goal_handle_ && goal_handle_->is_active()) {
            goal_handle_->abort(result);
        }
    }
}

void MotionControlNode::pathCallback(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future) {
    try {
        auto response = future.get();
        
        if (!response) {
            RCLCPP_ERROR(get_logger(), "Service call failed");
            if (goal_handle_) {
                auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
                goal_handle_->abort(result);
            }
            return;
        }

        if (response->plan.poses.empty()) {
            RCLCPP_ERROR(get_logger(), "Received empty path");
            if (goal_handle_) {
                auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
                goal_handle_->abort(result);
            }
            return;
        }

        path_ = response->plan;
        RCLCPP_INFO(get_logger(), "Received path with %ld points", path_.poses.size());
        
        // Start execution in a new thread
        if (goal_handle_ && goal_handle_->is_active()) {
            std::thread(&MotionControlNode::execute, this).detach();
        }
    } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Exception in path callback: %s", e.what());
        if (goal_handle_) {
            auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
            goal_handle_->abort(result);
        }
    }
}

void MotionControlNode::odomCallback(const nav_msgs::msg::Odometry & msg) {
    current_pose_.header = msg.header;
    current_pose_.pose = msg.pose.pose;
    
    checkCollision();
    updateTwist();
}

void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan & msg) {
    laser_scan_ = msg;
}