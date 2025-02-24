#ifndef WATER_SERVICE_HPP
#define WATER_SERVICE_HPP

#include <rclcpp/rclcpp.hpp>
#include "demo_nova_sanctum/srv/water.hpp"
#include "demo_nova_sanctum/msg/water.hpp"

class WaterService : public rclcpp::Node {
public:
    WaterService();

private:
    rclcpp::Subscription<demo_nova_sanctum::msg::Water>::SharedPtr tank_status_subscriber_;
    rclcpp::Service<demo_nova_sanctum::srv::Water>::SharedPtr water_service_;
    rclcpp::Client<demo_nova_sanctum::srv::Water>::SharedPtr deionization_client_;
    rclcpp::TimerBase::SharedPtr accumulation_timer_;

    double water_level_;
    double contaminants_level_;
    double iodine_level_;
    double gas_bubbles_;
    double pressure_;
    double temperature_;

    double max_tank_capacity_;
    double accumulation_rate_;
    double threshold_level_;

    void tank_status_callback(const demo_nova_sanctum::msg::Water::SharedPtr msg);
    void water_accumulation();
    void handle_water_request(
        const std::shared_ptr<demo_nova_sanctum::srv::Water::Request> request,
        std::shared_ptr<demo_nova_sanctum::srv::Water::Response> response);
    void send_deionization_request();
    void process_deionization_response(rclcpp::Client<demo_nova_sanctum::srv::Water>::SharedFuture future);
};

#endif // WATER_SERVICE_HPP
