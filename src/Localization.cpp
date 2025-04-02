#include "mpc_rbt_simulator/RobotConfig.hpp"

LocalizationNode::LocalizationNode() : 
    rclcpp::Node("localization_node"), 
    last_time_(this->get_clock()->now()) {

    // Odometry message initialization
    odometry_.header.frame_id = "map";
    odometry_.child_frame_id = "base_link";
    // add code here

    // Subscriber for joint_states
    // add code here

    // Publisher for odometry
    // add code here

    // tf_briadcaster 
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "Localization node started.");
}

void LocalizationNode::jointCallback(const sensor_msgs::msg::JointState & msg) {
    // add code here


    // ********
    // * Help *
    // ********
    /*
    auto current_time = this->get_clock()->now();

    updateOdometry(msg.velocity[0], msg.velocity[1], dt);
    publishOdometry();
    publishTransform();
    */
}

void LocalizationNode::updateOdometry(double left_wheel_vel, double right_wheel_vel, double dt) {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    double linear =  ;
    double angular = ;  //robot_config::HALF_DISTANCE_BETWEEN_WHEELS

    tf2::Quaternion tf_quat;
    tf2::fromMsg(odometry_.pose.pose.orientation, tf_quat);
    double roll, pitch, theta;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, theta);

    theta = std::atan2(std::sin(theta), std::cos(theta));

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    */
}

void LocalizationNode::publishOdometry() {
    // add code here
}

void LocalizationNode::publishTransform() {
    // add code here
    
    // ********
    // * Help *
    // ********
    //tf_broadcaster_->sendTransform(t);
}
