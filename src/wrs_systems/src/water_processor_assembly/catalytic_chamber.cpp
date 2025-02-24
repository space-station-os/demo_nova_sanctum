#include "demo_nova_sanctum/water_processor_assembly/catalytic_chamber.h"

MultiFiltrationProcessor::MultiFiltrationProcessor() : Node("wpa_multi_filtration_server"), unprocessed_water_(0.0) {
    multi_filtration_service_ = this->create_service<demo_nova_sanctum::srv::Filteration>(
        "/wpa/filtered_water",
        std::bind(&MultiFiltrationProcessor::handle_multi_filtration, this, std::placeholders::_1, std::placeholders::_2)
    );

    catalytic_reactor_client_ = this->create_client<demo_nova_sanctum::srv::IonBed>("/wpa/catalytic_reactor");

    RCLCPP_INFO(this->get_logger(), "Multi-Filtration Processing Server Ready...");
}

void MultiFiltrationProcessor::handle_multi_filtration(
    const std::shared_ptr<demo_nova_sanctum::srv::Filteration::Request> request,
    std::shared_ptr<demo_nova_sanctum::srv::Filteration::Response> response) 
{
    double filtered_water = request->filtered_water;
    double contaminants = request->contaminants;
    double organics = request->organics;
    double ammonia = request->ammonia;

    if (filtered_water <= 0) {
        response->message = "Invalid water volume!";
        response->success = false;
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Received %.2f liters of filtered water.", filtered_water);
    RCLCPP_INFO(this->get_logger(), "Initial Contaminants: %.2f%% | Organics: %.2f%% | Ammonia: %.2f%%", contaminants, organics, ammonia);

    // **Step 1: Multifiltration Bed - Remove ammonia and organics**
    remove_ammonia_and_organics(filtered_water, contaminants, organics, ammonia);

    response->success = true;
    response->message = "Multi-Filtration complete! Sending to Catalytic Reactor.";

    // **Send purified water to the Catalytic Reactor**
    send_to_catalytic_reactor(filtered_water, contaminants, organics);
}

void MultiFiltrationProcessor::remove_ammonia_and_organics(double &filtered_water, double &contaminants, double &organics, double &ammonia) {
    // **Simulating removal process**
    organics *= 0.50;  // First stage removes 50% of organics
    ammonia *= 0.60;   // First stage removes 40% of ammonia
    contaminants *= 0.70;  // 30% of contaminants removed

    RCLCPP_INFO(this->get_logger(), "Multifiltration Bed: Removed 50%% Organics, 40%% Ammonia.");
    RCLCPP_INFO(this->get_logger(), "Remaining Organics: %.2f%% | Remaining Ammonia: %.2f%%", organics, ammonia);
}

void MultiFiltrationProcessor::send_to_catalytic_reactor(double filtered_water, double remaining_contaminants, double organics) {
    if (!catalytic_reactor_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_FATAL(this->get_logger(), "Catalytic Reactor Service Unavailable! Retaining processed water.");
        unprocessed_water_ = filtered_water; // Store water for later processing
        return;
    }

    auto request = std::make_shared<demo_nova_sanctum::srv::IonBed::Request>();
    request->filtered_water = filtered_water;
    request->contaminants = remaining_contaminants;
    

    RCLCPP_INFO(this->get_logger(), "Sending %.2f L filtered water with %.2f%% contaminants...",
                filtered_water, remaining_contaminants);

    auto future_result = catalytic_reactor_client_->async_send_request(request,
        std::bind(&MultiFiltrationProcessor::handle_catalytic_reactor_response, this, std::placeholders::_1));
}

void MultiFiltrationProcessor::handle_catalytic_reactor_response(rclcpp::Client<demo_nova_sanctum::srv::IonBed>::SharedFuture future) {
    try {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Catalytic Reactor successfully processed water: %s", response->message.c_str());
            unprocessed_water_ = 0.0; // Clear stored water
        } else {
            RCLCPP_WARN(this->get_logger(), "Catalytic Reactor failed: %s. Retaining processed water.", response->message.c_str());
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception while calling Catalytic Reactor service: %s", e.what());
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiFiltrationProcessor>());
    rclcpp::shutdown();
    return 0;
}
