#include "demo_nova_sanctum/water_processor_assembly/catalytic_chamber.h"

MultiFiltrationProcessor::MultiFiltrationProcessor() : Node("wpa_multi_filtration_server") {
    multi_filtration_service_ = this->create_service<demo_nova_sanctum::srv::Filteration >(
        "/wpa/filtered_water",
        std::bind(&MultiFiltrationProcessor::handle_multi_filtration, this, std::placeholders::_1, std::placeholders::_2)
    );

    catalytic_reactor_client_ = this->create_client<demo_nova_sanctum::srv::IonBed>("/wpa/catalytic_reactor");

    RCLCPP_INFO(this->get_logger(), "Multi-Filtration Processing Server Ready...");
}

void MultiFiltrationProcessor::handle_multi_filtration(
    const std::shared_ptr<demo_nova_sanctum::srv::Filteration ::Request> request,
    std::shared_ptr<demo_nova_sanctum::srv::Filteration ::Response> response) 
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

    // **Step 2: Catalytic Reactor - Oxidize Organics & Kill Microbes**
    simulate_catalytic_reactor(filtered_water, contaminants);

    // **Send purified water to the next processing stage**
    send_to_catalytic_reactor(filtered_water, contaminants);
}

void MultiFiltrationProcessor::remove_ammonia_and_organics(double &filtered_water, double &contaminants, double &organics, double &ammonia) {
    // **Simulating removal process**
    organics *= 0.2;  // Removes 80% of organics
    ammonia *= 0.3;   // Removes 70% of ammonia

    RCLCPP_INFO(this->get_logger(), "Multifiltration Bed: Removed 80%% Organics, 70%% Ammonia.");
    RCLCPP_INFO(this->get_logger(), "Remaining Organics: %.2f%% | Remaining Ammonia: %.2f%%", organics, ammonia);
}

void MultiFiltrationProcessor::simulate_catalytic_reactor(double &filtered_water, double &contaminants) {
    RCLCPP_INFO(this->get_logger(), "Catalytic Reactor Activated: Oxidizing volatile organics & killing microbes...");

    // **Simulated oxidation & sterilization process**
    for (int i = 0; i < 3; i++) {
        RCLCPP_INFO(this->get_logger(), "Heat cycle %d: Reactor at %d°C - Organic oxidation in progress.", i + 1, 150);
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    contaminants *= 0.33;  // Removes 67% of remaining contaminants
    RCLCPP_INFO(this->get_logger(), "Catalytic Reactor: Microbial sterilization complete. Remaining Contaminants: %.2f%%", contaminants);
}

void MultiFiltrationProcessor::send_to_catalytic_reactor(double filtered_water, double remaining_contaminants) {
    if (!catalytic_reactor_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Catalytic Reactor Service Unavailable!");
        return;
    }

    auto request = std::make_shared<demo_nova_sanctum::srv::IonBed::Request>();
    request->filtered_water = filtered_water;
    request->contaminants = remaining_contaminants;

    auto future_result = catalytic_reactor_client_->async_send_request(request);
    future_result.wait();

    auto response = future_result.get();
    RCLCPP_INFO(this->get_logger(), "Catalytic Reactor Response: %s", response->message.c_str());
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiFiltrationProcessor>());
    rclcpp::shutdown();
    return 0;
}
