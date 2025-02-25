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
    double initial_organics = 30.0;  // Organic material in urine
    double initial_ammonia = 50.0;   // Urine contains high ammonia

    RCLCPP_INFO(this->get_logger(), "Received %.2f liters of urine water with %.2f%% contaminants.", urine_volume, contaminants);
    RCLCPP_INFO(this->get_logger(), "Initial Organic Load: %.2f%% | Initial Ammonia Load: %.2f%%", initial_organics, initial_ammonia);

    // **Step 1: External Filter Assembly (EFA) - Remove 25% contaminants**
    double contaminants_after_efa = contaminants * 0.75;  // 25% contaminants removed
    initial_organics *= 0.75;  // 25% of organics removed
    initial_ammonia *= 0.85;   // 15% ammonia removed

    RCLCPP_INFO(this->get_logger(), "EFA Filter: Removed 25%% contaminants. Remaining: %.2f%%", contaminants_after_efa);
    RCLCPP_INFO(this->get_logger(), "EFA Filter: Organics Remaining: %.2f%% | Ammonia Remaining: %.2f%%", initial_organics, initial_ammonia);

    // **Step 2: Particulate Filter - Remove 60% contaminants**
    double contaminants_after_particulate = contaminants_after_efa * 0.40;  // 60% of the remaining contaminants removed
    initial_organics *= 0.60;  // 40% more organics removed
    initial_ammonia *= 0.75;   // 25% ammonia removed

    RCLCPP_INFO(this->get_logger(), "Particulate Filter: Removed 60%% contaminants. Remaining: %.2f%%", contaminants_after_particulate);
    RCLCPP_INFO(this->get_logger(), "Particulate Filter: Organics Remaining: %.2f%% | Ammonia Remaining: %.2f%%", initial_organics, initial_ammonia);

    response->success = true;
    response->message = "Recieved distilled water, Onto next stage of filtration...";

    // **Send filtered water to next stage**
    send_to_next_stage(urine_volume, contaminants_after_particulate, initial_organics, initial_ammonia);
}

void WaterProcessorAssembly::send_to_next_stage(double filtered_water, double remaining_contaminants, double organics, double ammonia) {
    if (!next_stage_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_FATAL(this->get_logger(), "Next stage filtration service unavailable! Retaining processed water.");
        return;
    }

    auto request = std::make_shared<demo_nova_sanctum::srv::Filteration::Request>();
    request->filtered_water = filtered_water;
    request->contaminants = remaining_contaminants;
    request->organics = organics;
    request->ammonia = ammonia;

    RCLCPP_INFO(this->get_logger(), "Sending %.2f L filtered water with %.2f%% contaminants, %.2f%% organics, %.2f%% ammonia to next stage...",
                filtered_water, remaining_contaminants, organics, ammonia);

    // **Send request asynchronously**
    auto future_result = next_stage_client_->async_send_request(request,
        std::bind(&WaterProcessorAssembly::handle_next_stage_response, this, std::placeholders::_1));
}

void WaterProcessorAssembly::handle_next_stage_response(rclcpp::Client<demo_nova_sanctum::srv::Filteration>::SharedFuture future) {
    try {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Next Stage Filtration successfully completed: %s", response->message.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Next Stage Filtration failed: %s", response->message.c_str());
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception while calling Next Stage Filtration service: %s", e.what());
    }
}
    
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaterProcessorAssembly>());
    rclcpp::shutdown();
    return 0;
}
