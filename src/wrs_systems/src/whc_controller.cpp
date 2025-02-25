#include "demo_nova_sanctum/whc_controller.h"

WasteHygieneCompartment::WasteHygieneCompartment() 
    : Node("whc_controller"), urination_detected(false), urine_volume(0.3), pretreatment_volume(0.05), flush_volume(1.0) {
    
    // Subscribe to ultrasound sensor detecting urine flow
    ultrasound_subscriber_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/urine_sensor", 10, std::bind(&WasteHygieneCompartment::urine_callback, this, std::placeholders::_1)
    );

    // Publisher for total water volume to tank
    water_volume_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/whc/water_volume", 10);

    RCLCPP_INFO(this->get_logger(), "Waste Hygiene Compartment Node Initialized");
}

void WasteHygieneCompartment::urine_callback(const sensor_msgs::msg::Range::SharedPtr msg) {
    if (msg->range < 0.1) { // Assuming <0.1m means urine flow stopped
        RCLCPP_INFO(this->get_logger(), "Urination completed. Adding pretreatment and flushing...");

        // Add Pretreatment
        add_pretreatment();

        // Flush System
        flush();

        // Publish total accumulated water
        publish_total_water();
    }
}

void WasteHygieneCompartment::add_pretreatment() {
    RCLCPP_INFO(this->get_logger(), "Adding %.2f liters of pretreatment...", pretreatment_volume);
    total_water_volume += pretreatment_volume;
}

void WasteHygieneCompartment::flush() {
    RCLCPP_INFO(this->get_logger(), "Flushing with %.2f liters of water...", flush_volume);
    total_water_volume += flush_volume;
}

void WasteHygieneCompartment::publish_total_water() {
    total_water_volume += urine_volume;  // Include urine into the total count
    std_msgs::msg::Float64 msg;
    msg.data = total_water_volume;
    water_volume_publisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Total accumulated water: %.2f liters", total_water_volume);

    // Reset total water volume after publishing (assuming cycle restarts)
    total_water_volume = 0.0;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WasteHygieneCompartment>());
    rclcpp::shutdown();
    return 0;
}
