#ifndef ELECTROLYSIS_NODE_HPP
#define ELECTROLYSIS_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "demo_nova_sanctum/msg/water.hpp"
#include "std_msgs/msg/float64.hpp"

class ElectrolysisNode : public rclcpp::Node {
public:
    ElectrolysisNode();

private:
    void waterCallback(const demo_nova_sanctum::msg::Water::SharedPtr msg);
    void performElectrolysis();
    void recirculationLoop();

    // Water parameters
    double water_level_;
    double pressure_;
    double temperature_;
    bool water_available_;

    // ROS Parameters
    double efficiency_factor_;
    double cooling_rate_;
    double required_pressure_;
    double depletion_factor_;

    // ROS 2 Interfaces
    rclcpp::Subscription<demo_nova_sanctum::msg::Water>::SharedPtr water_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr hydrogen_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr oxygen_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr temp_pub_;
    rclcpp::TimerBase::SharedPtr electrolysis_timer_;
};

#endif // ELECTROLYSIS_NODE_HPP
