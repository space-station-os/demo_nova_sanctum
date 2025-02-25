#include "demo_nova_sanctum/water_processor_assembly/product_water_tank.h"

ProductWaterTank::ProductWaterTank() : Node("wpa_product_water_tank_server"), current_water_volume_(0.0) {
    water_tank_service_ = this->create_service<demo_nova_sanctum::srv::Water>(
        "/wpa/product_water_tank",
        std::bind(&ProductWaterTank::handle_water_tank, this, std::placeholders::_1, std::placeholders::_2)
    );

    dispense_water_service_ = this->create_service<demo_nova_sanctum::srv::CleanWater>(
        "/wpa/dispense_water",
        std::bind(&ProductWaterTank::handle_dispense_water, this, std::placeholders::_1, std::placeholders::_2)
    );

    tank_status_publisher_ = this->create_publisher<demo_nova_sanctum::msg::WaterCrew>(
        "/wpa/tank_status", 10
    );

    publish_timer_ = this->create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&ProductWaterTank::publish_tank_status, this)
    );

    RCLCPP_INFO(this->get_logger(), "Product Water Tank Server Ready...");
}

void ProductWaterTank::handle_water_tank(
    const std::shared_ptr<demo_nova_sanctum::srv::Water::Request> request,
    std::shared_ptr<demo_nova_sanctum::srv::Water::Response> response) 
{
    double incoming_water = request->water;
    double available_space = 100.0 - current_water_volume_;

    if (incoming_water > available_space) {
        response->success = false;
        response->message = "Tank full! Can only accept " + std::to_string(available_space) + "L more.";
        RCLCPP_WARN(this->get_logger(), "Water tank near full! Accepting %.2fL and rejecting %.2fL.", available_space, incoming_water - available_space);
        incoming_water = available_space;  // Accept only available space
    }

    current_water_volume_ += incoming_water;
    current_gas_bubbles_ = request->gas_bubbles;
    current_contaminants_ = request->contaminants;
    current_iodine_level_ = request->iodine_level;
    current_pressure_ = request->pressure;
    current_temperature_ = request->temperature;

    response->success = true;
    response->message = "Water successfully stored in tank.";

    RCLCPP_INFO(this->get_logger(), "Tank filled with %.2f liters. Current volume: %.2fL", incoming_water, current_water_volume_);
}

void ProductWaterTank::handle_dispense_water(
    const std::shared_ptr<demo_nova_sanctum::srv::CleanWater::Request> request,
    std::shared_ptr<demo_nova_sanctum::srv::CleanWater::Response> response) 
{
    double requested_water = request->water;

    if (requested_water > current_water_volume_) {
        response->success = false;
        response->message = "Not enough water available.";
        RCLCPP_WARN(this->get_logger(), "Water request exceeds available amount. Request: %.2fL, Available: %.2fL",
                    requested_water, current_water_volume_);
        return;
    }

    if (current_iodine_level_ < 0.2) {
        response->success = false;
        response->message = "Water iodine level too low for safe consumption!";
        RCLCPP_WARN(this->get_logger(), "Water cannot be dispensed! Iodine level (%.2f mg/L) is below safe threshold.", current_iodine_level_);
        return;
    }

    current_water_volume_ -= requested_water;
    response->success = true;
    response->message = "Water successfully dispensed.";

    RCLCPP_INFO(this->get_logger(), "Dispensed %.2f liters of water. Remaining in tank: %.2fL", requested_water, current_water_volume_);
}

void ProductWaterTank::publish_tank_status() {
    demo_nova_sanctum::msg::WaterCrew msg;
    msg.water = current_water_volume_;
    msg.gas_bubbles = current_gas_bubbles_ * 0.9;  // Gas bubbles reduce over time
    msg.contaminants = current_contaminants_;
    msg.iodine_level = current_iodine_level_;
    msg.pressure = current_pressure_;
    msg.temperature = current_temperature_;

    tank_status_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Tank Status Published: Water = %.2fL | Iodine = %.2fmg/L", 
                current_water_volume_, current_iodine_level_);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProductWaterTank>());
    rclcpp::shutdown();
    return 0;
}
