#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "../include/Localization.hpp"

LocalizationNode::LocalizationNode() : 
    rclcpp::Node("localization_node"), 
    x_(0.0), y_(0.0), theta_(0.0),
    last_time_(this->get_clock()->now()) {

    // Odometry message initialization
    odometry_.header.frame_id = "map";
    odometry_.child_frame_id = "base_link";
    // add code here

    // Subscriber for joint_states
    //ros::Subscriber joint_states_sub = nh.subscribe("/joint_states", 10, jointStatesCallback);

    // Subscriber pro /joint_states
    joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&LocalizationNode::jointCallback, this, std::placeholders::_1));

    // Publisher for odometry
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry", 10);

    // tf_briadcaster 
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "Localization node started.");
}

void LocalizationNode::jointCallback(const sensor_msgs::msg::JointState & msg) {
    
    //std::ostringstream oss;
    //oss << "Position: [";
    //printf("daco prislo");

    auto current_time = this->get_clock()->now();
    auto dt = current_time.seconds() - last_time_.seconds();
    last_time_ = current_time;

    RCLCPP_INFO(get_logger(), "DeltaTime is %f", dt);


    updateOdometry(msg.velocity[0], msg.velocity[1], dt);
    publishOdometry();
    publishTransform();

    
    // Debugging – vypíšeme přijímané data
    RCLCPP_INFO(get_logger(), "Received joint states message");
    RCLCPP_INFO(get_logger(), "Joint names:");
    
    // Vypíšeme všechny názvy kloubů
    for (const auto &name : msg.name) {
        RCLCPP_INFO(get_logger(), " - %s", name.c_str());
    }

    // Vypíšeme pozice kloubů
    for (size_t i = 0; i < msg.position.size(); ++i) {
        RCLCPP_INFO(get_logger(), "Joint %s position: %f", msg.name[i].c_str(), msg.position[i]);
    }

    // Vypíšeme rychlosti kloubů
    for (size_t i = 0; i < msg.velocity.size(); ++i) {
        RCLCPP_INFO(get_logger(), "Joint %s velocity: %f", msg.name[i].c_str(), msg.velocity[i]);
    }
    
}

void LocalizationNode::updateOdometry(double left_wheel_vel, double right_wheel_vel, double dt) {
    // Výpočet lineární a úhlové rychlosti
    double linear = (left_wheel_vel*robot_config::WHEEL_RADIUS + right_wheel_vel*robot_config::WHEEL_RADIUS) / 2.0;
    double angular = (left_wheel_vel*robot_config::WHEEL_RADIUS - right_wheel_vel*robot_config::WHEEL_RADIUS) / (robot_config::HALF_DISTANCE_BETWEEN_WHEELS*2.0);

    // Získání aktuální orientace robota
    tf2::Quaternion tf_quat;
    tf2::fromMsg(odometry_.pose.pose.orientation, tf_quat);
    double roll, pitch, theta;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, theta);

    // Korigování orientace na interval (-pi, pi)
    theta = std::atan2(std::sin(theta), std::cos(theta));

    // Výpočet změny v pozici a orientaci
    double delta_x = linear * dt * cos(theta);  // změna v ose X
    double delta_y = linear * dt * sin(theta);  // změna v ose Y
    double delta_theta = angular * dt;          // změna v orientaci (theta)

    // Aktualizace celkové pozice a orientace robota
    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;

    // Oprava orientace na interval (-pi, pi)
    //theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

    // Aktualizace odometrie v odometrickém zprávě
    odometry_.pose.pose.position.x = x_;
    odometry_.pose.pose.position.y = y_;
    //odometry_.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, sin(theta_ / 2), cos(theta_ / 2)));

    // Výpočet rychlosti robota (lineární a úhlová)
    odometry_.twist.twist.linear.x = linear;
    odometry_.twist.twist.angular.z = angular;

    // Publikace odometrie
    publishOdometry();
    publishTransform();

    // ********
    // * Help *
    // ********
    /*
    double linear = (left_wheel_vel + right_wheel_vel)/2;
    double angular = (right_wheel_vel - left_wheel_vel)/robot_config::HALF_DISTANCE_BETWEEN_WHEELS;

    tf2::Quaternion tf_quat;
    tf2::fromMsg(odometry_.pose.pose.orientation, tf_quat);
    double roll, pitch, theta;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, theta);

    theta = std::atan2(std::sin(theta), std::cos(theta));
    */
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    q.normalize();
    odometry_.pose.pose.orientation = tf2::toMsg(q);
    
}

void LocalizationNode::publishOdometry() {
    // add code here
    // Nastavení časového razítka a rámce pro odometrické zprávy
    odometry_.header.stamp = this->get_clock()->now();
    
    // Publikování odometrie na topiku "odom"
    odometry_publisher_->publish(odometry_);
}

void LocalizationNode::publishTransform() {
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = this->get_clock()->now();
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "base_link";

    // Nastavení translace
    transformStamped.transform.translation.x = x_;
    transformStamped.transform.translation.y = y_;
    transformStamped.transform.translation.z = 0.0;

    // Nastavení rotace
    transformStamped.transform.rotation = tf2::toMsg(tf2::Quaternion(0, 0, sin(theta_ / 2), cos(theta_ / 2)));

    // Posílání transformace
    tf_broadcaster_->sendTransform(transformStamped);
    
    // ********
    // * Help *
    // ********
    //tf_broadcaster_->sendTransform(t);
}
