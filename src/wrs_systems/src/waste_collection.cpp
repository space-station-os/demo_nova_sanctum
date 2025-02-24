#include "demo_nova_sanctum/waste_tank.h"

WHCWasteTank::WHCWasteTank() 
    : Node("whc_waste_tank"),
      urine_volume_(0.3), 
      pretreatment_volume_(0.05), 
      flush_volume_(1.0),
      total_water_volume_(0.0),
      upa_available_(true) { // Initially assume UPA is available

    // Declare parameters
    this->declare_parameter("tank_capacity", 22.0);
    this->declare_parameter("processing_threshold", 10.0);  // Lower than full capacity for safety

    tank_capacity_ = this->get_parameter("tank_capacity").as_double();
    processing_threshold_ = this->get_parameter("processing_threshold").as_double();

    // Service client for UPA
    upa_client_ = this->create_client<demo_nova_sanctum::srv::Upa>("/upa/process_urine");

    // Retry mechanism for UPA
    retry_timer_ = this->create_wall_timer(
        std::chrono::seconds(5), std::bind(&WHCWasteTank::retry_process_waste_transfer, this)
    );

    RCLCPP_INFO(this->get_logger(), "WHC Waste Tank Node Initialized");

    // Simulate urine collection periodically
    urine_collection_timer_ = this->create_wall_timer(
        std::chrono::seconds(3), std::bind(&WHCWasteTank::simulate_urine_collection, this)
    );
}

void WHCWasteTank::simulate_urine_collection() {
    // **Stop urine collection if UPA is unavailable**
    if (!upa_available_) {
        RCLCPP_WARN(this->get_logger(), "[WHC] Urine collection paused! UPA is unavailable.");
        return;
    }

    // Simulate urination completed and add pretreatment + flush
    double new_waste = urine_volume_ + pretreatment_volume_ + flush_volume_;
    total_water_volume_ += new_waste;

    RCLCPP_INFO(this->get_logger(), "[WHC] Urination cycle complete. New waste: %.2f liters. Total waste collected: %.2f liters",
                new_waste, total_water_volume_);

    // Check if it's time to transfer to UPA
    if (total_water_volume_ >= processing_threshold_) {
        process_waste_transfer();
    }
}

void WHCWasteTank::process_waste_transfer() {
    // **Check if UPA is available**
    if (!upa_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_FATAL(this->get_logger(), "UPA service unavailable! Cannot process waste. Retrying in 5 seconds...");
        upa_available_ = false; // Mark UPA as unavailable
        return;
    }

    // **UPA is available again → Resume collection**
    if (!upa_available_) {
        RCLCPP_INFO(this->get_logger(), "UPA service restored! Resuming urine collection...");
        upa_available_ = true;
    }

    // Create service request
    auto request = std::make_shared<demo_nova_sanctum::srv::Upa::Request>();
    request->urine = total_water_volume_;

    RCLCPP_INFO(this->get_logger(), "Sending %.2f liters of waste to UPA...", total_water_volume_);

    // Send request asynchronously
    auto future_result = upa_client_->async_send_request(request,
        std::bind(&WHCWasteTank::process_urine_response, this, std::placeholders::_1));
}

void WHCWasteTank::process_urine_response(rclcpp::Client<demo_nova_sanctum::srv::Upa>::SharedFuture future) {
    try {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "UPA processed urine successfully: %s", response->message.c_str());
            total_water_volume_ = 0.0;  // Empty tank after processing
        } else {
            RCLCPP_WARN(this->get_logger(), "UPA failed to process waste: %s. Retrying in 5 seconds...", response->message.c_str());
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception while calling UPA service: %s", e.what());
    }
}

void WHCWasteTank::retry_process_waste_transfer() {
    if (total_water_volume_ >= processing_threshold_) {
        RCLCPP_INFO(this->get_logger(), "Retrying waste transfer to UPA...");
        process_waste_transfer();
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WHCWasteTank>());
    rclcpp::shutdown();
    return 0;
}
