#ifndef WPA_PRODUCT_WATER_TANK_HPP
#define WPA_PRODUCT_WATER_TANK_HPP

#include <rclcpp/rclcpp.hpp>
#include "demo_nova_sanctum/srv/wpa_product_water_tank.hpp"
#include "demo_nova_sanctum/srv/wpa_dispense_water.hpp"
#include "demo_nova_sanctum/msg/wpa_tank_status.hpp"

class ProductWaterTank : public rclcpp::Node {
public:
    ProductWaterTank();

private:
    rclcpp::Service<demo_nova_sanctum::srv::Water>::SharedPtr water_tank_service_;
    rclcpp::Service<demo_nova_sanctum::srv::PotableWater>::SharedPtr dispense_water_service_;
    rclcpp::Publisher<demo_nova_sanctum::msg::Water>::SharedPtr tank_status_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    double current_water_volume_;
    double current_gas_bubbles_;
    double current_contaminants_;
    double current_iodine_level_;
    double current_pressure_;
    double current_temperature_;

    void handle_water_tank(
        const std::shared_ptr<demo_nova_sanctum::srv::Water::Request> request,
        std::shared_ptr<demo_nova_sanctum::srv::Water::Response> response);

    void handle_dispense_water(
        const std::shared_ptr<demo_nova_sanctum::srv::PotableWater::Request> request,
        std::shared_ptr<demo_nova_sanctum::srv::PotableWater::Response> response);

    void publish_tank_status();
};

#endif // WPA_PRODUCT_WATER_TANK_HPP
