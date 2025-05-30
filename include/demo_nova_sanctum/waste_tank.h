#ifndef WHC_WASTE_TANK_HPP
#define WHC_WASTE_TANK_HPP

#include <rclcpp/rclcpp.hpp>
#include "demo_nova_sanctum/srv/upa.hpp"
#include "std_msgs/msg/float64.hpp"  
#include "demo_nova_sanctum/msg/water_crew.hpp"

class WHCWasteTank : public rclcpp::Node {
public:
    WHCWasteTank();

private:
    // Service client
    rclcpp::Client<demo_nova_sanctum::srv::Upa>::SharedPtr upa_client_;

    // Timers
    rclcpp::TimerBase::SharedPtr retry_timer_;
    rclcpp::TimerBase::SharedPtr urine_collection_timer_;
    rclcpp::Publisher<demo_nova_sanctum::msg::WaterCrew>::SharedPtr waste_status_pub_;
     rclcpp::TimerBase::SharedPtr dashboard_timer_;
    // New Sabatier water subscriber
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sabatier_water_sub_; 

    // State variables
    double urine_volume_;
    double pretreatment_volume_;
    double flush_volume_;
    double total_water_volume_;
    double tank_capacity_;
    double processing_threshold_;
    bool upa_available_;

    // Methods
    void simulate_urine_collection();
    void process_waste_transfer();
    void process_urine_response(rclcpp::Client<demo_nova_sanctum::srv::Upa>::SharedFuture future);
    void retry_process_waste_transfer();

    void publish_status();
    void receive_sabatier_water(const std_msgs::msg::Float64::SharedPtr msg);
};

#endif // WHC_WASTE_TANK_HPP
