#include "demo_nova_sanctum/water_processor_assembly/filteration_unit.h"

WaterProcessorAssembly::WaterProcessorAssembly() : Node("wpa_service_server") {
    wpa_service_ = this->create_service<demo_nova_sanctum::srv::Distillation>(
        "/wpa/process_water",
        std::bind(&WaterProcessorAssembly::handle_water_processing, this, std::placeholders::_1, std::placeholders::_2)
    );

    next_stage_client_ = this->create_client<demo_nova_sanctum::srv::Filteration>("/wpa/filtered_water");

    RCLCPP_INFO(this->get_logger(), "Water Processor Assembly Server Ready...");
}

void WaterProcessorAssembly::handle_water_processing(
    const std::shared_ptr<demo_nova_sanctum::srv::Distillation::Request> request,
    std::shared_ptr<demo_nova_sanctum::srv::Distillation::Response> response) 
{
    double urine_volume = request->urine;
    double contaminants = request->contaminants;

    if (urine_volume <= 0) {
        response->message = "Invalid water volume!";
        response->success = false;
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }

    // **Simulated Initial Contaminant Load from Urine**
    double initial_organics = 30.0;  // High organic content
    double initial_ammonia = 50.0;   // Urine contains high ammonia

    RCLCPP_INFO(this->get_logger(), "Received %.2f liters of urine water with %.2f%% contaminants.", urine_volume, contaminants);
    RCLCPP_INFO(this->get_logger(), "Initial organic load: %.2f%% | Initial ammonia load: %.2f%%", initial_organics, initial_ammonia);

    // **Step 1: External Filter Assembly (EFA) - Remove 20% contaminants**
    double contaminants_after_efa = contaminants * 0.8;
    initial_organics *= 0.8;  // 20% of organics removed
    initial_ammonia *= 0.9;   // Only 10% ammonia removed

    RCLCPP_INFO(this->get_logger(), "EFA Filter: Removed 20%% contaminants. Remaining: %.2f%%", contaminants_after_efa);
    RCLCPP_INFO(this->get_logger(), "EFA Filter: Organics Remaining: %.2f%% | Ammonia Remaining: %.2f%%", initial_organics, initial_ammonia);

    // **Step 2: Particulate Filter - Remove 50% contaminants**
    double contaminants_after_particulate = contaminants_after_efa * 0.5;  // Removes 50% of the remaining contaminants
    initial_organics *= 0.7;  // 30% more organics removed
    initial_ammonia *= 0.8;   // 20% ammonia removed

    RCLCPP_INFO(this->get_logger(), "Particulate Filter: Removed 50%% contaminants. Remaining: %.2f%%", contaminants_after_particulate);
    RCLCPP_INFO(this->get_logger(), "Particulate Filter: Organics Remaining: %.2f%% | Ammonia Remaining: %.2f%%", initial_organics, initial_ammonia);

    response->success = true;
    response->message = "Filtration complete! Sending filtered water to next stage.";

    // **Send filtered water to next stage**
    send_to_next_stage(urine_volume, contaminants_after_particulate, initial_organics, initial_ammonia);
}

void WaterProcessorAssembly::send_to_next_stage(double filtered_water, double remaining_contaminants, double organics, double ammonia) {
    if (!next_stage_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Next stage filtration service unavailable!");
        return;
    }

    auto request = std::make_shared<demo_nova_sanctum::srv::Filteration::Request>();
    request->filtered_water = filtered_water;
    request->contaminants = remaining_contaminants;
    request->organics = organics;
    request->ammonia = ammonia;

    auto future_result = next_stage_client_->async_send_request(request);
    future_result.wait();

    auto response = future_result.get();
    RCLCPP_INFO(this->get_logger(), "Next Stage Filtration Response: %s", response->message.c_str());
}
    
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaterProcessorAssembly>());
    rclcpp::shutdown();
    return 0;
}
