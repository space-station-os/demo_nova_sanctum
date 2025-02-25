#ifndef URINE_TANK_HPP
#define URINE_TANK_HPP

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp"
#include "demo_nova_sanctum/srv/upa.hpp"
#include <chrono>
#include <memory>
#include <string>
#include <vector>

class UrineTank : public rclcpp::Node {
private:
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr water_subscriber_;
    rclcpp::Client<demo_nova_sanctum::srv::Upa>::SharedPtr upa_client_;
    
    double tank_capacity;   // Max urine tank capacity before processing
    double current_volume;  // Track urine accumulation
    double flush_volume;    // Water used per flush
    double processing_threshold;  // When to send urine to UPA

public:
    UrineTank();
    void water_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void process_urine();
};

#endif // URINE_TANK_HPP
