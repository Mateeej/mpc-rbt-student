#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "mpc_rbt_solution/MotionControl.hpp"

MotionControlNode::MotionControlNode() :
    rclcpp::Node("motion_control_node") {

        // Subscribers for odometry and laser scans
        // add code here
        
        // Publisher for robot control
        // add code here

        // Client for path planning
        // add code here

        // Action server
        // add code here

        RCLCPP_INFO(get_logger(), "Motion control node started.");

        // Connect to path planning service server
        // add code here
    }

void MotionControlNode::checkCollision() {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    if (laser_scan_.ranges[i] < thresh) {
        geometry_msgs::msg::Twist stop;
        twist_publisher_->publish(stop);
    }
    */
}

void MotionControlNode::updateTwist() {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    geometry_msgs::msg::Twist twist;
    twist.angular.z = P * xte;
    twist.linear.x = v_max;

    twist_publisher_->publish(twist);
    */
}

rclcpp_action::GoalResponse MotionControlNode::navHandleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal) {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    (void)uuid;
    ...
    return ...;
    */
}

rclcpp_action::CancelResponse MotionControlNode::navHandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    (void)goal_handle;
    ...
    return ...;
    */
}

void MotionControlNode::navHandleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    ...
    auto future = plan_client_->async_send_request(request,
        std::bind(&MotionControlNode::pathCallback, this, std::placeholders::_1));
    */
}

void MotionControlNode::execute() {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    rclcpp::Rate loop_rate(1.0); // 1 Hz

    while (rclcpp::ok()) {

        if (goal_handle_->is_canceling()) {
            ...
            return;
        }

        ...

        goal_handle_->publish_feedback(feedback);

        loop_rate.sleep();
    }
    */
}

void MotionControlNode::pathCallback(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future) {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    if (response && response->plan.poses.size() > 0) {
        goal_handle_->execute();
        std::thread(&MotionControlNode::execute, this).detach();
    }
    */
}

void MotionControlNode::odomCallback(const nav_msgs::msg::Odometry & msg) {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    checkCollision();
    updateTwist();
    */
}

void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan & msg) {
    // add code here
}
