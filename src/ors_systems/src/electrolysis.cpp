#include "demo_nova_sanctum/electrolysis.h"

ElectrolysisNode::ElectrolysisNode() : Node("electrolysis_node") {
    // Subscriber for ionized water input
    water_sub_ = this->create_subscription<demo_nova_sanctum::msg::Water>(
        "/ionized_water", 10, std::bind(&ElectrolysisNode::waterCallback, this, std::placeholders::_1)
    );

    // Publishers for hydrogen and oxygen outputs
    hydrogen_pub_ = this->create_publisher<std_msgs::msg::Float64>("/hydrogen", 10);
    oxygen_pub_ = this->create_publisher<std_msgs::msg::Float64>("/oxygen_supply", 10);
    temp_pub_ = this->create_publisher<std_msgs::msg::Float64>("/thermal_control", 10);

    // Timer to run electrolysis process at 1 Hz
    electrolysis_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ElectrolysisNode::performElectrolysis, this)
    );

    // Declare and initialize ROS Parameters
    this->declare_parameter("efficiency_factor", 1.0);  // 100% efficiency
    this->declare_parameter("cooling_rate", 5.0);       // Cooling per second
    this->declare_parameter("required_pressure", 2.0); // Minimum pressure for electrolysis
    this->declare_parameter("depletion_factor", 0.3);  // 30% depletion per cycle

    water_available_ = false;  // Initially, no water available

    RCLCPP_INFO(this->get_logger(), "Electrolysis Node Initialized.");
}

void ElectrolysisNode::waterCallback(const demo_nova_sanctum::msg::Water::SharedPtr msg) {
    if (!water_available_ || water_level_ == 0) { // Only update when water is depleted
        water_level_ = msg->water_level;
        pressure_ = msg->pressure;
        temperature_ = msg->temperature;
        water_available_ = true;

        RCLCPP_INFO(this->get_logger(), "Received New Ionized Water - Water: %f L, Pressure: %f, Temp: %f",
                    water_level_, pressure_, temperature_);
    }
}

void ElectrolysisNode::performElectrolysis() {
    if (water_level_ <= 0) {
        RCLCPP_WARN(this->get_logger(), "Water depleted! Waiting for new water supply...");
        water_available_ = false;
        return;
    }

    // Get parameters
    efficiency_factor_ = this->get_parameter("efficiency_factor").as_double();
    required_pressure_ = this->get_parameter("required_pressure").as_double();
    depletion_factor_ = this->get_parameter("depletion_factor").as_double();

    if (pressure_ < required_pressure_) {
        RCLCPP_WARN(this->get_logger(), "Insufficient pressure (%f). Adjusting pressure...", pressure_);
        pressure_ = required_pressure_;
    }

    // Stoichiometry (H₂O → H₂ + O₂)
    double water_moles = water_level_; // Assuming 1 liter ~ 55.5 moles of H₂O
    double hydrogen_moles = 2.0 * water_moles * efficiency_factor_;
    double oxygen_moles = 1.0 * water_moles * efficiency_factor_;

    std_msgs::msg::Float64 hydrogen_msg;
    hydrogen_msg.data = hydrogen_moles;
    hydrogen_pub_->publish(hydrogen_msg);

    std_msgs::msg::Float64 oxygen_msg;
    oxygen_msg.data = oxygen_moles;
    oxygen_pub_->publish(oxygen_msg);

    RCLCPP_INFO(this->get_logger(), "Electrolysis: Produced H₂ = %f moles, O₂ = %f moles", hydrogen_moles, oxygen_moles);

    // Reduce water level by 30%
    double new_water_level = water_level_ * (1.0 - depletion_factor_);
    water_level_ = (new_water_level > 0) ? new_water_level : 0;  // Prevent negative water level

    RCLCPP_INFO(this->get_logger(), "Water Reduced: %f L remaining.", water_level_);

    // Send remaining water to recirculation loop
    recirculationLoop();
}

void ElectrolysisNode::recirculationLoop() {
    // Get cooling rate from parameters
    cooling_rate_ = this->get_parameter("cooling_rate").as_double();

    double cooling_tank_temp = temperature_;
    
    while (cooling_tank_temp > 25.0) {
        cooling_tank_temp -= cooling_rate_;

        RCLCPP_INFO(this->get_logger(), "Cooling Tank: Temp = %f C", cooling_tank_temp);

        std_msgs::msg::Float64 temp_msg;
        temp_msg.data = cooling_tank_temp;
        temp_pub_->publish(temp_msg);
    }

    // Adjust pressure for next electrolysis cycle
    if (pressure_ < required_pressure_) {
        pressure_ = required_pressure_;
    }

    // Loop control: if water is still available, continue electrolysis
    if (water_level_ > 0) {
        RCLCPP_INFO(this->get_logger(), "Water Recycled: %f L available for next electrolysis cycle.", water_level_);
    } else {
        RCLCPP_WARN(this->get_logger(), "Water fully depleted! Waiting for new ionized water input...");
        water_available_ = false;
    }
}

// Main function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ElectrolysisNode>());
    rclcpp::shutdown();
    return 0;
}
