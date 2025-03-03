#include "demo_nova_sanctum/ars_system/adsorbent_tank.hpp"
#include <mutex> 

using namespace std::chrono_literals;

AdsorbentBed::AdsorbentBed()
: Node("adsorbent_bed"), is_active_(false), previous_error_(0.0), retained_co2_cumulative_(0.0) {

  RCLCPP_INFO(this->get_logger(), "Initializing Adsorbent Bed...");

  // Declare dynamic parameters (these can be configured via a YAML file)
  this->declare_parameter("co2_removal_efficiency", 0.90);
  this->declare_parameter("co2_to_space_ratio", 0.50);
  this->declare_parameter("desired_temperature", 430.0);
  this->declare_parameter("temperature_tolerance", 50.0);
  this->declare_parameter("kp", 0.5);
  this->declare_parameter("kd", 0.1);

  // Create the service to receive air from the Desiccant Bed
  adsorbent_service_ = this->create_service<demo_nova_sanctum::srv::CrewQuarters>(
    "/adsorbent_server", 
    std::bind(&AdsorbentBed::handle_air_processing_request, this, std::placeholders::_1, std::placeholders::_2));

  // Publisher for CO₂ data
  co2_publisher_ = this->create_publisher<demo_nova_sanctum::msg::AirData>("/co2", 10);

  RCLCPP_INFO(this->get_logger(), "Adsorbent Bed successfully initialized.");
}

void AdsorbentBed::handle_air_processing_request(
    const std::shared_ptr<demo_nova_sanctum::srv::CrewQuarters::Request> request,
    std::shared_ptr<demo_nova_sanctum::srv::CrewQuarters::Response> response) {

    if (is_active_) {
        response->success = false;
        response->message = "Adsorbent Bed is already processing a batch.";
        RCLCPP_WARN(this->get_logger(), "New air request rejected. Still processing previous batch.");
        return;
    }

    // Retrieve parameters dynamically from the YAML file
    this->get_parameter("co2_removal_efficiency", co2_removal_efficiency_);
    this->get_parameter("co2_to_space_ratio", co2_to_space_ratio_);

    // Accept air for processing
    is_active_ = true;
    co2_ = request->co2_mass;
    moisture_content_ = request->moisture_content;
    contaminants_ = request->contaminants;

    RCLCPP_INFO(this->get_logger(), "Received air batch from Desiccant Bed: CO₂=%.2f g, Moisture=%.2f %%, Contaminants=%.2f %%",
                co2_, moisture_content_, contaminants_);

    // Start processing
    timer_ = this->create_wall_timer(1s, std::bind(&AdsorbentBed::process_co2, this));

    response->success = true;
    response->message = "Adsorbent Bed started processing the new batch.";
}

void AdsorbentBed::process_co2() {
  std::lock_guard<std::mutex> lock(data_mutex_);

  // Retrieve dynamic PID parameters
  double desired_temperature, temperature_tolerance, kp, kd;
  this->get_parameter("desired_temperature", desired_temperature);
  this->get_parameter("temperature_tolerance", temperature_tolerance);
  this->get_parameter("kp", kp);
  this->get_parameter("kd", kd);

  // PID-based temperature control
  double error = desired_temperature - temperature_;
  double derivative = error - previous_error_;
  double adjustment = (kp * error) + (kd * derivative);

  temperature_ += adjustment;
  previous_error_ = error;

  RCLCPP_INFO(this->get_logger(), "Adsorbent bed temperature adjusted to %.2f°C.", temperature_);

  // Process CO₂ only within the optimal temperature range
  if (temperature_ >= (desired_temperature - temperature_tolerance) && temperature_ <= (desired_temperature + temperature_tolerance)) {
    
    double co2_decrement = co2_ * co2_removal_efficiency_ * 0.1; 
    double co2_to_space = co2_decrement * co2_to_space_ratio_;  
    double co2_retained = co2_decrement - co2_to_space;         

    co2_ -= co2_decrement;
    if (co2_ < 1.0) co2_ = 0.0;

    retained_co2_cumulative_ += co2_retained;

    demo_nova_sanctum::msg::AirData co2_msg;
    co2_msg.header.stamp = this->get_clock()->now();
    co2_msg.header.frame_id = "Adsorbent Bed";
    co2_msg.co2_mass = retained_co2_cumulative_; 
    co2_msg.moisture_content = moisture_content_;       
    co2_msg.contaminants = contaminants_;           
    co2_msg.temperature = temperature_;            

    RCLCPP_INFO(this->get_logger(),
                "Processed CO2: %.2f g retained (cumulative: %.2f g), %.2f g sent to space. Remaining CO2: %.2f g",
                co2_retained, retained_co2_cumulative_, co2_to_space, co2_);

    // Publish processed CO₂ data
    co2_publisher_->publish(co2_msg);
  }
  else if (temperature_ >= (desired_temperature + temperature_tolerance)) {
    RCLCPP_WARN(this->get_logger(), "Adsorbent bed temperature exceeded safe limit. Shutting down system.");
    is_active_ = false;
    return;
  }

  // Stop processing if all CO2 has been removed
  if (co2_ <= 1.0) {
    RCLCPP_INFO(this->get_logger(), "All CO2 has been processed. Total retained CO2: %.2f g.", retained_co2_cumulative_);
    is_active_ = false; 
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AdsorbentBed>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
