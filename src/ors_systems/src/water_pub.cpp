#include "demo_nova_sanctum/water_status.h"

using namespace std::chrono_literals;

WaterService::WaterService()
    : Node("water_service"),
      water_level_(this->declare_parameter<double>("initial_water_level", 0.0)),
      contaminants_level_(this->declare_parameter<double>("initial_contaminants_level", 0.0)),
      iodine_level_(this->declare_parameter<double>("initial_iodine_level", 0.0)),
      max_tank_capacity_(this->declare_parameter<double>("max_tank_capacity", 100.0)),
      accumulation_rate_(this->declare_parameter<double>("accumulation_rate", 5.0)) {

    // Service to provide water when requested
    water_service_ = this->create_service<demo_nova_sanctum::srv::Water>(
        "/water_request",
        std::bind(&WaterService::handle_water_request, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Client to send water to deionization chamber
    deionization_client_ = this->create_client<demo_nova_sanctum::srv::Water>("/deionization_chamber");

    // Timer to simulate water accumulation over time
    accumulation_timer_ = this->create_wall_timer(
        2s, std::bind(&WaterService::water_accumulation, this)
    );

    RCLCPP_INFO(this->get_logger(), "Water service node initialized. Monitoring tank levels...");
}

void WaterService::water_accumulation() {
    if (water_level_ >= max_tank_capacity_) {
        RCLCPP_WARN(this->get_logger(), "Tank full (%.2f L)! Sending request to deionization chamber.", water_level_);
        send_deionization_request();
        return;
    }

    // Increment water level, contaminants, and iodine concentration
    water_level_ += accumulation_rate_;
    contaminants_level_ += accumulation_rate_ * 0.2;  // 20% of accumulation rate
    iodine_level_ += accumulation_rate_ * 0.05;       // 5% of accumulation rate

    if (water_level_ > max_tank_capacity_) {
        water_level_ = max_tank_capacity_;
    }

    RCLCPP_INFO(this->get_logger(), "Tank filling: Water = %.2f L, Contaminants = %.2f ppm, Iodine = %.2f ppm",
                water_level_, contaminants_level_, iodine_level_);
}

void WaterService::handle_water_request(
    const std::shared_ptr<demo_nova_sanctum::srv::Water::Request> request,
    std::shared_ptr<demo_nova_sanctum::srv::Water::Response> response) {

    RCLCPP_INFO(this->get_logger(), "Received external water request for %.2f L", request->water);

    if (water_level_ < request->water) {
        response->success = false;
        response->message = "Not enough water available!";
        RCLCPP_ERROR(this->get_logger(), "Request failed: Insufficient water.");
        return;
    }

    send_deionization_request();
    response->success = true;
    response->message = "Water successfully transferred to deionization chamber.";
}

void WaterService::send_deionization_request() {
    if (!deionization_client_->wait_for_service(5s)) {
        RCLCPP_ERROR(this->get_logger(), "Deionization chamber service is not available!");
        return;
    }

    auto request = std::make_shared<demo_nova_sanctum::srv::Water::Request>();
    request->water = water_level_;
    request->contaminants = contaminants_level_;
    request->iodine_level = iodine_level_;
    request->gas_bubbles = 0.0;  
    request->pressure = 14.7;
    request->temperature = 25.0;

    RCLCPP_INFO(this->get_logger(), "Sending water to deionization chamber: Water = %.2f L, Contaminants = %.2f ppm, Iodine = %.2f ppm",
                request->water, request->contaminants, request->iodine_level);

    // Send request asynchronously with callback
    auto future = deionization_client_->async_send_request(request,
        std::bind(&WaterService::process_deionization_response, this, std::placeholders::_1));
}

void WaterService::process_deionization_response(rclcpp::Client<demo_nova_sanctum::srv::Water>::SharedFuture future) {
    try {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Deionization successful. Emptying tank...");
            water_level_ = 0.0;
            contaminants_level_ = 0.0;
            iodine_level_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "Tank emptied successfully.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Deionization failed: %s", response->message.c_str());
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception while calling deionization service: %s", e.what());
    }
}

// Main Function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaterService>());
    rclcpp::shutdown();
    return 0;
}
