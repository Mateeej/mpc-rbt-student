#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "../include/MotionControl.hpp"

MotionControlNode::MotionControlNode() :
    rclcpp::Node("motion_control_node") {
        // Parameters
        this->declare_parameter<double>("max_linear_speed", 0.1);
        this->declare_parameter<double>("max_angular_speed", 0.5);
        this->declare_parameter<double>("lookahead_distance", 0.5);
        this->declare_parameter<double>("collision_threshold", 0.5);
        this->declare_parameter<double>("k_p", 0.5);

        max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        collision_threshold_ = this->get_parameter("collision_threshold").as_double();
        k_p_ = this->get_parameter("k_p").as_double();

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

    // Check front sector of the laser scan
    size_t center_index = laser_scan_.ranges.size() / 2;
    size_t sector_size = laser_scan_.ranges.size() / 8; // Check 1/8th of the scan in front
    
    for (size_t i = center_index - sector_size/2; i < center_index + sector_size/2; ++i) {
        if (laser_scan_.ranges[i] < collision_threshold_) {
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
    if (path_.poses.empty() || !goal_handle_) return;

    // Pure pursuit algorithm
    // Find the lookahead point
    geometry_msgs::msg::PointStamped lookahead_point;
    bool found = false;
    
    for (size_t i = path_.poses.size()-1; i > 0; --i) {
        double dx = path_.poses[i].pose.position.x - current_pose_.pose.position.x;
        double dy = path_.poses[i].pose.position.y - current_pose_.pose.position.y;
        double dist = sqrt(dx*dx + dy*dy);
        
        if (dist <= lookahead_distance_) {
            lookahead_point.point = path_.poses[i].pose.position;
            lookahead_point.header = path_.poses[i].header;
            found = true;
            break;
        }
    }



    if (!found && !path_.poses.empty()) {
        lookahead_point.point = path_.poses.back().pose.position;
        lookahead_point.header = path_.poses.back().header;
    }

    // Transform lookahead point to robot frame
    try {
        // Calculate relative position
        double target_dx = lookahead_point.point.x - current_pose_.pose.position.x;
        double target_dy = lookahead_point.point.y - current_pose_.pose.position.y;
        double target_yaw = atan2(target_dy, target_dx);


        
        // Get current yaw from quaternion
        tf2::Quaternion q(
            current_pose_.pose.orientation.x,
            current_pose_.pose.orientation.y,
            current_pose_.pose.orientation.z,
            current_pose_.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, current_yaw;
        m.getRPY(roll, pitch, current_yaw);

        // Calculate and normalize yaw error
        double yaw_error_ = target_yaw - current_yaw;
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
    } catch (...) {
        RCLCPP_ERROR(get_logger(), "Failed to calculate twist command");
    }
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
    rclcpp::Rate loop_rate(10); // 10 Hz
    
    auto feedback = std::make_shared<nav2_msgs::action::NavigateToPose::Feedback>();
    auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
    
    while (rclcpp::ok()) {
        if (goal_handle_->is_canceling()) {
            goal_handle_->canceled(result);
            RCLCPP_INFO(get_logger(), "Goal canceled");
            return;
        }
        
        // Calculate distance to goal
        double dx = goal_pose_.pose.position.x - current_pose_.pose.position.x;
        double dy = goal_pose_.pose.position.y - current_pose_.pose.position.y;
        double distance = sqrt(dx*dx + dy*dy);
        
        feedback->distance_remaining = distance;
        goal_handle_->publish_feedback(feedback);
        
        if (distance < 0.1) {
            goal_handle_->succeed(result);
            RCLCPP_INFO(get_logger(), "Goal reached");
            return;
        }
        
        loop_rate.sleep();
    }
}

void MotionControlNode::pathCallback(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future) {
    auto response = future.get();
    
    if (response && !response->plan.poses.empty()) {
        path_ = response->plan;
        std::thread(&MotionControlNode::execute, this).detach();
    } else {
        auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
        goal_handle_->abort(result);
        RCLCPP_ERROR(get_logger(), "Failed to get path plan");
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