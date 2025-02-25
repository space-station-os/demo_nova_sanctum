#include "demo_nova_sanctum/waste_tank.h"

UrineTank::UrineTank() : Node("urine_tank"), current_volume(0.0) {
    // Declare and get parameters (allow changes via param server)
    this->declare_parameter("tank_capacity", 22.0);
    this->declare_parameter("processing_threshold", 20.0);
    this->declare_parameter("flush_volume", 1.0);
    
    tank_capacity = this->get_parameter("tank_capacity").as_double();
    processing_threshold = this->get_parameter("processing_threshold").as_double();
    flush_volume = this->get_parameter("flush_volume").as_double();

    // Subscriber to urine accumulation
    water_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
        "/whc/water_volume", 10, std::bind(&UrineTank::water_callback, this, std::placeholders::_1)
    );

    // Service client to send urine to UPA
    upa_client_ = this->create_client<demo_nova_sanctum::srv::Upa>("/upa/process_urine");

    RCLCPP_INFO(this->get_logger(), "Urine Tank Node Initialized");
}

void UrineTank::water_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    current_volume += msg->data;
    RCLCPP_INFO(this->get_logger(), "Tank received %.2f liters. Current volume: %.2f liters", msg->data, current_volume);

    // If tank reaches limit, send urine to UPA
    if (current_volume >= processing_threshold) {
        process_urine();
    }
}

void UrineTank::process_urine() {
    if (!upa_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "UPA service unavailable");
        return;
    }

    auto request = std::make_shared<demo_nova_sanctum::srv::Upa::Request>();
    request->urine = current_volume;  // Send current tank urine to UPA

    auto future_result = upa_client_->async_send_request(request);
    
    // Handle response asynchronously
    future_result.wait();
    auto response = future_result.get();

    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "UPA processed urine successfully: %s", response->message.c_str());
        current_volume = 0.0;  // Empty tank after processing
    } else {
        RCLCPP_WARN(this->get_logger(), "UPA failed to process urine: %s", response->message.c_str());
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UrineTank>());
    rclcpp::shutdown();
    return 0;
}
