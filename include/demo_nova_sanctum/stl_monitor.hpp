#ifndef STL_MONITOR_HPP
#define STL_MONITOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "demo_nova_sanctum/msg/air_data.hpp"  // Make sure this matches your custom message location

class STLMonitor : public rclcpp::Node
{
public:
    STLMonitor();
    ~STLMonitor();

private:
    void collectorCallback(const demo_nova_sanctum::msg::AirData::SharedPtr msg);
    void desiccantCallback(const demo_nova_sanctum::msg::AirData::SharedPtr msg);
    void adsorbentCallback(const demo_nova_sanctum::msg::AirData::SharedPtr msg);
    
    bool checkBounds(double value, double a, double b);

    rclcpp::Subscription<demo_nova_sanctum::msg::AirData>::SharedPtr collector_sub_;
    rclcpp::Subscription<demo_nova_sanctum::msg::AirData>::SharedPtr desiccant_sub_;
    rclcpp::Subscription<demo_nova_sanctum::msg::AirData>::SharedPtr adsorbent_sub_;

    // Optimized Bounds
    double collector_a_, collector_b_;
    double desiccant_a_, desiccant_b_;
    double adsorbent_a_, adsorbent_b_;
};

#endif // STL_MONITOR_HPP
