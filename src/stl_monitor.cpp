#include "demo_nova_sanctum/stl_monitor.hpp"

STLMonitor::STLMonitor() : Node("stl_monitor")
{
    // Setting optimized bounds
    collector_a_ = 19.784776;      // Lower bound for /collector_air_quality
    collector_b_ = 332.612543;     // Upper bound for /collector_air_quality

    desiccant_a_ = 38.461645;      // Lower bound for /desiccant_air_quality
    desiccant_b_ = 351.864317;     // Upper bound for /desiccant_air_quality

    adsorbent_a_ = 1.36371;        // Lower bound for /adsorbent_air_quality
    adsorbent_b_ = 188.288461;     // Upper bound for /adsorbent_air_quality

    collector_sub_ = this->create_subscription<demo_nova_sanctum::msg::AirData>(
        "/collector_air_quality", 10,
        std::bind(&STLMonitor::collectorCallback, this, std::placeholders::_1));

    desiccant_sub_ = this->create_subscription<demo_nova_sanctum::msg::AirData>(
        "/desiccant_air_quality", 10,
        std::bind(&STLMonitor::desiccantCallback, this, std::placeholders::_1));

    adsorbent_sub_ = this->create_subscription<demo_nova_sanctum::msg::AirData>(
        "/adsorbent_air_quality", 10,
        std::bind(&STLMonitor::adsorbentCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "STL Monitor Node Initialized.");
}

STLMonitor::~STLMonitor() {}

bool STLMonitor::checkBounds(double value, double a, double b)
{
    return value >= a && value <= b;
}

void STLMonitor::collectorCallback(const demo_nova_sanctum::msg::AirData::SharedPtr msg)
{
    bool in_bounds = checkBounds(msg->co2_mass, collector_a_, collector_b_);
    if (in_bounds)
    {
        RCLCPP_INFO(this->get_logger(), "COLLECTOR: Signal within range [%.2f, %.2f]. Value: %.2f", collector_a_, collector_b_, msg->co2_mass);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "COLLECTOR: Signal out of range! Value: %.2f", msg->co2_mass);
    }
}

void STLMonitor::desiccantCallback(const demo_nova_sanctum::msg::AirData::SharedPtr msg)
{
    bool in_bounds = checkBounds(msg->co2_mass, desiccant_a_, desiccant_b_);
    if (in_bounds)
    {
        RCLCPP_INFO(this->get_logger(), "DESICCANT: Signal within range [%.2f, %.2f]. Value: %.2f", desiccant_a_, desiccant_b_, msg->co2_mass);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "DESICCANT: Signal out of range! Value: %.2f", msg->co2_mass);
    }
}

void STLMonitor::adsorbentCallback(const demo_nova_sanctum::msg::AirData::SharedPtr msg)
{
    bool in_bounds = checkBounds(msg->co2_mass, adsorbent_a_, adsorbent_b_);
    if (in_bounds)
    {
        RCLCPP_INFO(this->get_logger(), "ADSORBENT: Signal within range [%.2f, %.2f]. Value: %.2f", adsorbent_a_, adsorbent_b_, msg->co2_mass);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "ADSORBENT: Signal out of range! Value: %.2f", msg->co2_mass);
    }
}



int main(int argc, char *argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a node
    auto node = std::make_shared<STLMonitor>();

    // Spin the node (run the event loop)
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
