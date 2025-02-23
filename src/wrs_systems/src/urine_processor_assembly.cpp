#include "demo_nova_sanctum/urine_processor_assembly.h"

UrineProcessorAssembly::UrineProcessorAssembly() : Node("upa_service_server") {
    upa_service_ = this->create_service<demo_nova_sanctum::srv::Upa>(
        "/upa/process_urine",
        std::bind(&UrineProcessorAssembly::handle_urine_request, this, std::placeholders::_1, std::placeholders::_2)
    );

    wpa_client_ = this->create_client<demo_nova_sanctum::srv::Distillation>("/wpa/process_water");

    RCLCPP_INFO(this->get_logger(), "Urine Processor Assembly Server Ready...");
}

void UrineProcessorAssembly::handle_urine_request(
    const std::shared_ptr<demo_nova_sanctum::srv::Upa::Request> request,
    std::shared_ptr<demo_nova_sanctum::srv::Upa::Response> response) 
{
    double urine_volume = request->urine;
    if (urine_volume <= 0) {
        response->message = "Invalid urine volume!";
        response->success = false;
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Received %.2f liters of urine from Waste Collection Tank...", urine_volume);

    double distilled_urine = 0.0;
    double contaminants = 0.0;

    // **Simulate the distillation process in batches**
    simulate_distillation(urine_volume, distilled_urine, contaminants);

    response->success = true;
    response->message = "Distillation complete! Sending to Purge Pump.";

    // **Simulate the Purge Pump**
    double temperature = 100.0;
    simulate_purge_pump(distilled_urine, temperature);

    // **Send processed water and contaminants to Water Processor Assembly**
    send_to_wpa(distilled_urine, contaminants);
}

void UrineProcessorAssembly::simulate_distillation(double urine_volume, double &distilled_urine, double &contaminants) {
    RCLCPP_INFO(this->get_logger(), "Starting Distillation Process...");
    
    double batch_size = urine_volume / 2.0; 
    for (int i = 0; i < 2; i++) {
        double temp_contaminants = batch_size * 0.1;  
        double temp_distilled = batch_size - temp_contaminants;

        RCLCPP_INFO(this->get_logger(), "Batch %d: Heated to 100°C - %0.2f L of clean water extracted.", i + 1, temp_distilled);
        rclcpp::sleep_for(std::chrono::seconds(2));

        contaminants += temp_contaminants;
        distilled_urine += temp_distilled;
    }

    RCLCPP_INFO(this->get_logger(), "Distillation Complete! Total clean water: %.2f L, Contaminants: %.2f L", distilled_urine, contaminants);
}

void UrineProcessorAssembly::simulate_purge_pump(double &distilled_urine, double &temperature) {
    RCLCPP_INFO(this->get_logger(), "Purge Pump Activated: Cooling down water...");

    for (int i = 0; i < 5; i++) {
        temperature -= 10.0;
        RCLCPP_INFO(this->get_logger(), "Cooling... Current Temperature: %.2f°C", temperature);
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    RCLCPP_INFO(this->get_logger(), "Condensation complete! %.2f L clean water is ready for WPA.", distilled_urine);
}

void UrineProcessorAssembly::send_to_wpa(double processed_water, double contaminants) {
    if (!wpa_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Water Processor Assembly (WPA) Service Unavailable!");
        return;
    }

    auto request = std::make_shared<demo_nova_sanctum::srv::Distillation::Request>();
    request->urine = processed_water;
    request->contaminants = contaminants;

    auto future_result = wpa_client_->async_send_request(request);
    future_result.wait();

    auto response = future_result.get();
    RCLCPP_INFO(this->get_logger(), "WPA Response: %s", response->message.c_str());
}
    
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UrineProcessorAssembly>());
    rclcpp::shutdown();
    return 0;
}
