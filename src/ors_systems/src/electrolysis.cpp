#include "demo_nova_sanctum/electrolysis.h"
#include <cmath>

ElectrolysisNode::ElectrolysisNode() : Node("electrolysis_node"), water_available_(false) {
   
    electrolysis_server_ = this->create_service<demo_nova_sanctum::srv::Water>(
        "/electrolysis",
        std::bind(&ElectrolysisNode::handle_electrolysis_request, this, std::placeholders::_1, std::placeholders::_2));

   
    gas_pub_ = this->create_publisher<demo_nova_sanctum::msg::Electrolysis>("/electrolysis_output", 10);

    
    electrolysis_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ElectrolysisNode::performElectrolysis, this)
    );

    
    this->declare_parameter("efficiency_factor", 0.95);  // 95% efficiency
    this->declare_parameter("required_pressure", 50.0);   // Minimum pressure for electrolysis
    this->declare_parameter("depletion_factor", 0.3);    // 30% depletion per cycle

    RCLCPP_INFO(this->get_logger(), "Electrolysis Node Initialized.");
}

void ElectrolysisNode::handle_electrolysis_request(
    const std::shared_ptr<demo_nova_sanctum::srv::Water::Request> request,
    std::shared_ptr<demo_nova_sanctum::srv::Water::Response> response) {
    
    if (!water_available_) {
        response->success = false;
        response->message = "Electrolysis is already processing a batch.";
        RCLCPP_WARN(this->get_logger(), "Electrolysis is already processing a batch.");
        return;
    }

    // Store received water parameters
    water_level_ = request->water;
    pressure_ = request->pressure;
    temperature_ = request->temperature;
    water_available_ = true;  

    RCLCPP_INFO(this->get_logger(), "Received Clean Water for Electrolysis - Water: %.2f L, Pressure: %.2f, Temp: %.2f",
                water_level_, pressure_, temperature_);

    response->success = true;
    response->message = "Electrolysis started successfully.";
}



void ElectrolysisNode::performElectrolysis() {
    if (!water_available_ || water_level_ <= 0) {
        RCLCPP_WARN(this->get_logger(), "No water available! Waiting for new supply...");
        water_available_ = false;  
        return;
    }

    // Get parameters
    efficiency_factor_ = this->get_parameter("efficiency_factor").as_double();
    required_pressure_ = this->get_parameter("required_pressure").as_double();
    depletion_factor_ = this->get_parameter("depletion_factor").as_double();

    if (pressure_ < required_pressure_) {
        RCLCPP_WARN(this->get_logger(), "Insufficient pressure (%.2f). Adjusting pressure...", pressure_);
        pressure_ = required_pressure_;
    }

    // **Enhanced Electrolysis Simulation**
    double energy_required_per_mol = 237.1;  // kJ/mol
    double water_moles = water_level_ * 55.5; // Convert liters to moles
    double total_energy = water_moles * energy_required_per_mol;  // kJ

    // Realistic loss factor
    double actual_energy_used = total_energy * efficiency_factor_;

    // Hydrogen and oxygen production
    double hydrogen_moles = (2.0 * water_moles * efficiency_factor_);
    double oxygen_moles = (1.0 * water_moles * efficiency_factor_);

    // Publish electrolysis output
    auto gas_msg = demo_nova_sanctum::msg::Electrolysis();
    gas_msg.header.stamp = this->get_clock()->now();
    gas_msg.h2 = hydrogen_moles;
    gas_msg.o2 = oxygen_moles;
    gas_msg.temperature.temperature = temperature_ + (actual_energy_used / 1000);  
    gas_msg.pressure.fluid_pressure = pressure_ + (hydrogen_moles * 0.1); 

    // gas_pub_->publish(gas_msg);

    RCLCPP_INFO(this->get_logger(), "===========================");
    RCLCPP_INFO(this->get_logger(), "Electrolysis: Produced H₂ = %.2f moles, O₂ = %.2f moles", hydrogen_moles, oxygen_moles);
    RCLCPP_INFO(this->get_logger(), "Total Energy Used: %.2f kJ", actual_energy_used);
    RCLCPP_INFO(this->get_logger(), "Temperature: %.2f C, Pressure: %.2f Pa", gas_msg.temperature.temperature, gas_msg.pressure.fluid_pressure);
    RCLCPP_INFO(this->get_logger(), "===========================");

    // Reduce water level realistically
    water_level_ *= (1.0 - depletion_factor_);
    
    if (water_level_ <= 0.0) {
        water_level_ = 0.0; 
          
        RCLCPP_WARN(this->get_logger(), "Water fully depleted! Ready to accept new batch.");
        water_available_ = false;
    } else {
        RCLCPP_INFO(this->get_logger(), "Remaining Water After Electrolysis: %.2f L", water_level_);
    }
}



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ElectrolysisNode>());
    rclcpp::shutdown();
    return 0;
}
