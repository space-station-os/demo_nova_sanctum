#ifndef WATER_SERVICE_HPP
#define WATER_SERVICE_HPP

#include "rclcpp/rclcpp.hpp"
#include "demo_nova_sanctum/srv/water.hpp"

class WaterService : public rclcpp::Node {
public:
    WaterService();

private:
    void water_accumulation();
    void handle_water_request(
        const std::shared_ptr<demo_nova_sanctum::srv::Water::Request> request,
        std::shared_ptr<demo_nova_sanctum::srv::Water::Response> response);
    
    void send_deionization_request();
    void process_deionization_response(rclcpp::Client<demo_nova_sanctum::srv::Water>::SharedFuture future);

    // ROS Components
    rclcpp::Service<demo_nova_sanctum::srv::Water>::SharedPtr water_service_;
    rclcpp::Client<demo_nova_sanctum::srv::Water>::SharedPtr deionization_client_;
    rclcpp::TimerBase::SharedPtr accumulation_timer_;

    // Water Properties
    double water_level_;
    double contaminants_level_;
    double iodine_level_;
    double max_tank_capacity_;
    double accumulation_rate_;
};

#endif // WATER_SERVICE_HPP
