#include "demo_nova_sanctum/water_processor_assembly/ionization_bed.h"

CatalyticReactorProcessor::CatalyticReactorProcessor() : Node("wpa_catalytic_reactor_server") {
    catalytic_reactor_service_ = this->create_service<demo_nova_sanctum::srv::IonBed>(
        "/wpa/catalytic_reactor",
        std::bind(&CatalyticReactorProcessor::handle_catalytic_reactor, this, std::placeholders::_1, std::placeholders::_2)
    );

    product_water_tank_client_ = this->create_client<demo_nova_sanctum::srv::Water>("/wpa/product_water_tank");

    RCLCPP_INFO(this->get_logger(), "Catalytic Reactor Processing Server Ready...");
}

void CatalyticReactorProcessor::handle_catalytic_reactor(
    const std::shared_ptr<demo_nova_sanctum::srv::IonBed::Request> request,
    std::shared_ptr<demo_nova_sanctum::srv::IonBed::Response> response) 
{
    double filtered_water = request->filtered_water;
    double contaminants = request->contaminants;

    if (filtered_water <= 0) {
        response->message = "Invalid water volume!";
        response->success = false;
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Received %.2f liters of filtered water.", filtered_water);
    RCLCPP_INFO(this->get_logger(), "Initial Contaminants: %.2f%%", contaminants);

    double iodine_level = 0.0;
    double gas_bubbles = 0.05; // Initial gas bubble presence

    // **Step 1: Ion Exchange Bed - Remove contaminants & add iodine**
    process_ion_exchange(filtered_water, contaminants, iodine_level, gas_bubbles);

    response->success = true;
    response->message = "Ion Exchange Complete! Sending to Product Water Tank.";

    // **Send purified water to the Product Water Tank**
    send_to_product_water_tank(filtered_water, gas_bubbles, contaminants, iodine_level, 1.0, 25.0);
}

void CatalyticReactorProcessor::process_ion_exchange(double &filtered_water, double &contaminants, double &iodine_level, double &gas_bubbles) {
    // **Simulating ion exchange purification process**
    contaminants *= 0.2;  // Removes 80% of remaining contaminants
    iodine_level = 0.5;   // Adds iodine (mg/L) for microbial control
    gas_bubbles += 0.02;  // Slight increase in gas bubbles

    RCLCPP_INFO(this->get_logger(), "Ion Exchange Bed: Removed 80%% Contaminants.");
    RCLCPP_INFO(this->get_logger(), "Final Contaminants: %.2f%% | Iodine Level: %.2f mg/L | Gas Bubbles: %.2f%%", 
                contaminants, iodine_level, gas_bubbles);
}

void CatalyticReactorProcessor::send_to_product_water_tank(double water, double gas_bubbles, double contaminants, double iodine_level, double pressure, double temperature) {
    if (!product_water_tank_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Product Water Tank Service Unavailable!");
        return;
    }

    auto request = std::make_shared<demo_nova_sanctum::srv::Water::Request>();
    request->water = water;
    request->gas_bubbles = gas_bubbles;
    request->contaminants = contaminants;
    request->iodine_level = iodine_level;
    request->pressure = pressure;
    request->temperature = temperature;

    auto future_result = product_water_tank_client_->async_send_request(request);
    future_result.wait();

    auto response = future_result.get();
    RCLCPP_INFO(this->get_logger(), "Product Water Tank Response: %s", response->message.c_str());
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CatalyticReactorProcessor>());
    rclcpp::shutdown();
    return 0;
}
