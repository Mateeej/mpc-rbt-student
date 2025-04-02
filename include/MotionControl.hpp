#ifndef MOTIONCTRL_HPP
#define MOTIONCTRL_HPP

#include <vector>
#include <math.h>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

class MotionControlNode : public rclcpp::Node {
    public:
        MotionControlNode();
    
    private:
        // Parameters
        // TO DO

        // Methods
        void checkCollision();
        void updateTwist();
        void execute();
            
        // Callbacks
        rclcpp_action::GoalResponse navHandleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal);
        rclcpp_action::CancelResponse navHandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle);
        void navHandleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle);
        
        void pathCallback(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture);
        void odomCallback(const nav_msgs::msg::Odometry & msg);
        void lidarCallback(const sensor_msgs::msg::LaserScan & msg);
    
        // Clients
        rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr plan_client_;    
    
        // Actions
        rclcpp_action::Server<nav2_msgs::action::NavigateToPose>::SharedPtr nav_server_;
    
        // Publishers
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;

        // Subscribers
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
        
        // Handles
        std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle_;

        // Data
        nav_msgs::msg::Path path_;
        geometry_msgs::msg::PoseStamped current_pose_;
        geometry_msgs::msg::PoseStamped goal_pose_;
        sensor_msgs::msg::LaserScan laser_scan_;
    };



#endif // MOTIONCTRL_HPP
