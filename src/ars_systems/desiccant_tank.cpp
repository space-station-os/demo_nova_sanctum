#include "demo_nova_sanctum/ars_system/desiccant_tank.hpp"
#include <cmath>
#include <mutex>

using namespace std::chrono_literals;

DesiccantServer::DesiccantServer()
: Node("desiccant_server"), is_active_(false), service_unavailable_count_(0),
  target_temperature_(70.0), target_pressure_(150000.0) // Set target conditions for optimal moisture removal
{
  RCLCPP_INFO(this->get_logger(), "Initializing Desiccant Server...");

  // Declare dynamic parameters
  this->declare_parameter("moisture_removal_rate", 0.95);
  this->declare_parameter("contaminant_removal_rate", 0.90);
  this->declare_parameter<int>("emergency_threshold", 5);
  this->declare_parameter("target_temperature", 70.0);
  this->declare_parameter("target_pressure", 150000.0);

  // Initialize PID Controllers for temperature & pressure control
  temp_pid_ = {0.5, 0.02, 0.01};  
  press_pid_ = {0.3, 0.01, 0.005};

  // Create service to process air from the crew quarters
  crew_co2_service_ = this->create_service<demo_nova_sanctum::srv::CrewQuarters>(
    "/crew_co2_service",
    std::bind(&DesiccantServer::handle_air_processing_request, this, std::placeholders::_1, std::placeholders::_2));

  // Subscribe to temperature and pressure updates
  temperature_subscriber_ = this->create_subscription<sensor_msgs::msg::Temperature>(
    "/temperature", 10, std::bind(&DesiccantServer::temperature_callback, this, std::placeholders::_1));

  pressure_subscriber_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
    "/pipe_pressure", 10, std::bind(&DesiccantServer::pressure_callback, this, std::placeholders::_1));

  // Client for Adsorbent Server
  adsorbent_server_client_ = this->create_client<demo_nova_sanctum::srv::CrewQuarters>("/adsorbent_server");

  RCLCPP_INFO(this->get_logger(), "Desiccant Server successfully initialized.");
}

void DesiccantServer::handle_air_processing_request(
    const std::shared_ptr<demo_nova_sanctum::srv::CrewQuarters::Request> request,
    std::shared_ptr<demo_nova_sanctum::srv::CrewQuarters::Response> response) {

    if (is_active_) {
        response->success = false;
        response->message = "Desiccant Bed is still processing the previous batch.";
        RCLCPP_WARN(this->get_logger(), "New air request rejected. Still processing previous batch.");
        return;
    }

    // Accept air for processing
    is_active_ = true;
    co2_ = request->co2_mass;
    moisture_content_ = request->moisture_content;
    contaminants_ = request->contaminants;

    RCLCPP_INFO(this->get_logger(), "Accepted new air batch: CO₂=%.2f g, Moisture=%.2f %%, Contaminants=%.2f %%",
                co2_, moisture_content_, contaminants_);

    // Start processing
    timer_ = this->create_wall_timer(500ms, std::bind(&DesiccantServer::process_air_data, this));

    response->success = true;
    response->message = "Desiccant Bed started processing the new batch.";
}

void DesiccantServer::process_air_data() {
  std::lock_guard<std::mutex> lock(data_mutex_);

  // Retrieve dynamic parameters
  double moisture_removal_rate, contaminant_removal_rate;
  int emergency_threshold;
  this->get_parameter("moisture_removal_rate", moisture_removal_rate);
  this->get_parameter("contaminant_removal_rate", contaminant_removal_rate);
  this->get_parameter("emergency_threshold", emergency_threshold);
  this->get_parameter("target_temperature", target_temperature_);
  this->get_parameter("target_pressure", target_pressure_);

  // **Control Temperature and Pressure for Efficient Moisture Removal**
  double temp_output = temp_pid_.compute(target_temperature_, current_temperature_, 0.5);
  double press_output = press_pid_.compute(target_pressure_, current_pressure_, 0.5);

  current_temperature_ += temp_output;  
  current_pressure_ += press_output;  

  // **Adjust removal efficiency based on temperature & pressure**
  double removal_factor = 0.1 * (current_temperature_ / target_temperature_) * (current_pressure_ / target_pressure_);
  
  double moisture_decrement = moisture_content_ * moisture_removal_rate * removal_factor;
  double contaminants_decrement = contaminants_ * contaminant_removal_rate * removal_factor;

  moisture_content_ -= moisture_decrement;
  contaminants_ -= contaminants_decrement;

  // Ensure no negative values
  if (moisture_content_ < 0.0) moisture_content_ = 0.0;
  if (contaminants_ < 0.0) contaminants_ = 0.0;

  RCLCPP_INFO(this->get_logger(),
              "Processing air: Remaining Moisture: %.2f %%, Remaining Contaminants: %.2f %%",
              moisture_content_, contaminants_);

  // **Send air to Adsorbent Bed ONLY after confirming minimal moisture**
  if (moisture_content_ <= 0.3 && contaminants_ <= 0.3) {
    RCLCPP_INFO(this->get_logger(), "Minimal moisture & contaminants. Sending air to Adsorbent Bed...");
    send_to_adsorbent_bed();
  }
}

void DesiccantServer::send_to_adsorbent_bed() {
  if (!adsorbent_server_client_->wait_for_service(5s)) {
    service_unavailable_count_++;
    RCLCPP_WARN(this->get_logger(), "Adsorbent Bed Service unavailable. Holding air... (%d attempts)", service_unavailable_count_);

    int emergency_threshold;
    this->get_parameter("emergency_threshold", emergency_threshold);

    if (service_unavailable_count_ >= emergency_threshold) {
      RCLCPP_ERROR(this->get_logger(), "EMERGENCY: Adsorbent Bed Service unavailable for too long! Immediate action required!");
    }
    return;
  }

  service_unavailable_count_ = 0;

  auto request = std::make_shared<demo_nova_sanctum::srv::CrewQuarters::Request>();
  request->co2_mass = co2_;
  request->moisture_content = moisture_content_;
  request->contaminants = contaminants_;

  RCLCPP_INFO(this->get_logger(), "Sending processed air to Adsorbent Bed: Moisture=%.2f %%, Contaminants=%.2f %%",
              request->moisture_content, request->contaminants);

  auto future = adsorbent_server_client_->async_send_request(request,
      [this](rclcpp::Client<demo_nova_sanctum::srv::CrewQuarters>::SharedFuture future) {
          auto response = future.get();
          if (response->success) {
              RCLCPP_INFO(this->get_logger(), "Air successfully transferred to Adsorbent Bed.");
              is_active_ = false;  // Only reset after successful transfer
          } else {
              RCLCPP_ERROR(this->get_logger(), "Failed to transfer air to Adsorbent Bed.");
          }
      }
  );
}


void DesiccantServer::temperature_callback(const sensor_msgs::msg::Temperature::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_temperature_ = msg->temperature;
}

void DesiccantServer::pressure_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_pressure_ = msg->fluid_pressure;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DesiccantServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
