#ifndef KEYBOARDCONTROL_HPP
#define KEYBOARDCONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <termios.h>

class KeyboardControlNode : public rclcpp::Node {
    public:

        KeyboardControlNode();

        ~KeyboardControlNode();

    private:

        void timerCallback();

        //Publishers
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;

        rclcpp::TimerBase::SharedPtr timer_;

        struct termios old_termios_;
};

#endif // KEYBOARDCONTROL_HPP