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
  this->declare_parameter("humidification_rate", 1.5);


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


  adsorbent_co2_service_ = this->create_service<demo_nova_sanctum::srv::CrewQuarters>(
    "/desiccant_bed2",
    std::bind(&DesiccantServer::handle_moisture_request, this, std::placeholders::_1, std::placeholders::_2));
  
  cdra_status_publisher_=this->create_publisher<demo_nova_sanctum::msg::CdraStatus>("/cdra_status", 10);
  cdra=demo_nova_sanctum::msg::CdraStatus();
  RCLCPP_INFO(this->get_logger(), "Desiccant Server successfully initialized.");
}

void DesiccantServer::handle_air_processing_request(
  const std::shared_ptr<demo_nova_sanctum::srv::CrewQuarters::Request> request,
  std::shared_ptr<demo_nova_sanctum::srv::CrewQuarters::Response> response) {

  if (is_active_ && request->co2_mass > 0.0) {  // Only reject new unprocessed air
      response->success = false;
      response->message = "Desiccant Bed is still processing the previous batch.";
      cdra.co2_processing_state = 2;
      cdra.system_health = 0;
      cdra.data="Batch is still being processed";
      cdra_status_publisher_->publish(cdra);
      RCLCPP_WARN(this->get_logger(), "New air request rejected. Still processing previous batch.");
      return;
  }

 

  // Normal air processing
  is_active_ = true;
  co2_ = request->co2_mass;
  moisture_content_ = request->moisture_content;
  contaminants_ = request->contaminants;

  RCLCPP_INFO(this->get_logger(), "Accepted new air batch: CO₂=%.2f g, Moisture=%.2f %%, Contaminants=%.2f %%",
              co2_, moisture_content_, contaminants_);
  
  cdra.co2_processing_state = 2;
  cdra.system_health = 1;
  cdra.data="Collected Air received for moisture removal";
  cdra_status_publisher_->publish(cdra);

  // Start processing
  timer_ = this->create_wall_timer(500ms, std::bind(&DesiccantServer::process_air_data, this));

  response->success = true;
  response->message = "Desiccant Bed started processing the new batch.";
}


void DesiccantServer::handle_adsorbed_air(
  const std::shared_ptr<demo_nova_sanctum::srv::CrewQuarters::Request> request,
  std::shared_ptr<demo_nova_sanctum::srv::CrewQuarters::Response> response) {
  
    std::lock_guard<std::mutex> lock(data_mutex_);
    double humidification_rate;
    this->get_parameter("humidification_rate", humidification_rate);

    // If already processing a batch, accelerate instead of rejecting
    
    max_processing_speed_ = 2.0;
    if (is_active_desiccant2) {
        processing_speed_ *= 1.5;
        if (processing_speed_ > max_processing_speed_) {
            processing_speed_ = max_processing_speed_;
        }
        RCLCPP_WARN(this->get_logger(), "New batch received! Increasing speed to %.2fx", processing_speed_);
        response->success = true;
        response->message = "Processing speed increased to handle new request.";
        return;
    }

    // Accept new air batch for humidification
    is_active_desiccant2 = true;
    processing_speed_ = 1.0; // Reset processing speed
    moisture_content_ = 0.0; // Start from dry air

    RCLCPP_INFO(this->get_logger(), "Starting humidification process...");

    cdra.co2_processing_state = 2;
    cdra.system_health = 1;
    cdra.data = "Humidification process started";
    cdra_status_publisher_->publish(cdra);
    double target_moisture = 80.0; 
    // Simulated humidification process
    while (moisture_content_ < target_moisture) {
        double increment = humidification_rate * processing_speed_;
        moisture_content_ += increment;
        if (moisture_content_ > target_moisture) {
            moisture_content_ = target_moisture;
        }
        RCLCPP_INFO(this->get_logger(), "Current Moisture Level: %.2f%%", moisture_content_);
        std::this_thread::sleep_for(500ms);
    }

    RCLCPP_INFO(this->get_logger(), "Humidification complete. Sending air to Crew Quarters.");

    // Publish final status
    cdra.co2_processing_state = 3;
    cdra.system_health = 1;
    cdra.data = "Humidification Complete";
    cdra_status_publisher_->publish(cdra);

    
    is_active_desiccant2 = false;
    response->success = true;
    response->message = "Air successfully humidified and sent to Crew Quarters.";
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

  // **Wait for correct temperature before removing moisture**
  if (current_temperature_ < target_temperature_ - 2.0) {
      RCLCPP_WARN(this->get_logger(), "Waiting for temperature %.2f°C before starting moisture removal. Current: %.2f°C", 
                   target_temperature_, current_temperature_);
      return;  // Do not proceed yet
  }

  cdra.co2_processing_state = 2;
  cdra.system_health = 1;
  cdra.data = "Moisture Removal In Progress";
  cdra_status_publisher_->publish(cdra);

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
              "Processing air: Remaining Moisture: %.2f%%, Remaining Contaminants: %.2f%%",
              moisture_content_, contaminants_);
  RCLCPP_INFO(this->get_logger(),"==============================================");

  // **Send air to Adsorbent Bed ONLY after confirming minimal moisture**
  if (moisture_content_ <= 0.3 && contaminants_ <= 0.3) {
    RCLCPP_INFO(this->get_logger(), "Minimal moisture & contaminants. Sending air to Adsorbent Bed...");
    send_to_adsorbent_bed();

    cdra.co2_processing_state = 3;
    cdra.system_health = 1;
    cdra.data = "Moisture Removal Complete";
    cdra_status_publisher_->publish(cdra);
  }
}


void DesiccantServer::send_to_adsorbent_bed() {
  if (!adsorbent_server_client_->wait_for_service(5s)) {
    service_unavailable_count_++;
    RCLCPP_WARN(this->get_logger(), "Adsorbent Bed Service unavailable. Holding air... (%d attempts)", service_unavailable_count_);

    int emergency_threshold;
    this->get_parameter("emergency_threshold", emergency_threshold);

    if (service_unavailable_count_ >= emergency_threshold) {

      cdra.co2_processing_state = 3;
      cdra.system_health = 2;
      cdra.data="Adsorbent bed service unavailable";
      cdra_status_publisher_->publish(cdra);
      RCLCPP_ERROR(this->get_logger(), "EMERGENCY: Adsorbent Bed Service unavailable for too long! Immediate action required!");
    }
    return;
  }

  service_unavailable_count_ = 0;

  cdra.co2_processing_state = 3;
  cdra.system_health = 1;
  cdra.data = "Sending air to Adsorbent Bed";
  cdra_status_publisher_->publish(cdra);


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

              cdra.co2_processing_state = 4;
              cdra.system_health = 1;
              cdra.data = "Adsorbent bed received air";
              cdra_status_publisher_->publish(cdra);

          } else {
              RCLCPP_ERROR(this->get_logger(), "Failed to transfer air to Adsorbent Bed.");

              cdra.co2_processing_state = 4;
              cdra.system_health = 2;
              cdra.data = "Adsorbent bed Failure";
              cdra_status_publisher_->publish(cdra);
          }
      }
  );
}


void DesiccantServer::handle_moisture_request(
  const std::shared_ptr<demo_nova_sanctum::srv::CrewQuarters::Request> request,
  std::shared_ptr<demo_nova_sanctum::srv::CrewQuarters::Response> response) {

  std::lock_guard<std::mutex> lock(data_mutex_);

  // **Check if this is the returned processed air from Adsorbent Bed**
  if (request->co2_mass <= 5.0 && request->moisture_content > 0.0) {
      RCLCPP_INFO(this->get_logger(), "Received processed air from Adsorbent Bed. Reintroducing moisture...");
      moisture_content_ = request->moisture_content;
      contaminants_ = request->contaminants;
      is_active_ = false;  // Ready for next cycle

      cdra.co2_processing_state = 3;
      cdra.system_health = 1;
      cdra.data = "Received for humidification process";
      cdra_status_publisher_->publish(cdra);

      response->success = true;
      response->message = "Desiccant Bed 2 received processed air and is ready.";
      return;
  }

  // Accept air for processing
  is_active_dessicant = true;
  co2_ = request->co2_mass;
  contaminants_ = request->contaminants;

  double humidification_rate;
  this->get_parameter("humidification_rate", humidification_rate);

  cdra.co2_processing_state = 2;
  cdra.system_health = 1;
  cdra.data = "Dehumidification process started";
  cdra_status_publisher_->publish(cdra);

  double target_moisture = 80.0; 
  double increment = humidification_rate * 1.5;  // Factor applied for faster moisture increase

  RCLCPP_INFO(this->get_logger(), "Starting humidification process... Target Moisture: %.2f%%", target_moisture);

  // Incrementally increase moisture in a loop
  while (moisture_content_ < target_moisture) {
      moisture_content_ += increment;
      if (moisture_content_ > target_moisture) {
          moisture_content_ = target_moisture;
      }
      RCLCPP_INFO(this->get_logger(), "Moisture level: %.2f%%", moisture_content_);
  }

  RCLCPP_INFO(this->get_logger(), "Target moisture level reached.");
  response->success = true;
  response->message = "Desiccant Bed started processing the new batch.";

  process_air_data();  // Process air after reaching moisture target
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
