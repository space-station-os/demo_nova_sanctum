#include "demo_nova_sanctum/ars_system/adsorbent_tank.hpp"
#include <mutex>

using namespace std::chrono_literals;

AdsorbentBed::AdsorbentBed()
: Node("adsorbent_bed"), is_active_(false), previous_error_(0.0), retained_co2_cumulative_(0.0), co2_buffer_(0.0) {

  RCLCPP_INFO(this->get_logger(), "Initializing Adsorbent Bed...");

  // Declare dynamic parameters (can be modified via YAML)
  this->declare_parameter("co2_removal_efficiency", 0.90);
  this->declare_parameter("co2_to_space_ratio", 0.50);
  this->declare_parameter("desired_temperature", 430.0);
  this->declare_parameter("temperature_tolerance", 50.0);
  this->declare_parameter("desorption_temperature", 400.0);
  this->declare_parameter("kp", 0.5);
  this->declare_parameter("kd", 0.1);

  // Create the service to receive air from the Desiccant Bed
  adsorbent_service_ = this->create_service<demo_nova_sanctum::srv::CrewQuarters>(
    "/adsorbent_server", 
    std::bind(&AdsorbentBed::handle_air_processing_request, this, std::placeholders::_1, std::placeholders::_2));

  // Create a client to send processed air back to the Desiccant Bed
  desiccant_client_ = this->create_client<demo_nova_sanctum::srv::CrewQuarters>("/desiccant_bed2");

  // Publisher for CO₂ desorption (venting)
  co2_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/co2_vent", 10);
  
  cdra_status_publisher_=this->create_publisher<demo_nova_sanctum::msg::CdraStatus>("/cdra_status", 10);
  cdra=demo_nova_sanctum::msg::CdraStatus();

  // Initialize the publisher in your constructor (AdsorbentBed::AdsorbentBed())
  air_quality_publisher_ = this->create_publisher<demo_nova_sanctum::msg::AirData>("/adsorbent_air_quality", 10);

  RCLCPP_INFO(this->get_logger(), "\033[1;36mAdsorbent Bed successfully initialized.\033[0m");
  RCLCPP_INFO(this->get_logger(),"==============================================");
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

    cdra.co2_processing_state = 3;
    cdra.system_health = 1;
    cdra.data = "Adsorbing process started";
    cdra_status_publisher_->publish(cdra);
    // Retrieve parameters dynamically from YAML
    this->get_parameter("co2_removal_efficiency", co2_removal_efficiency_);
    this->get_parameter("co2_to_space_ratio", co2_to_space_ratio_);

    // Accept air for processing
    is_active_ = true;
    co2_ = request->co2_mass;
    moisture_content_ = request->moisture_content;
    contaminants_ = request->contaminants;

    RCLCPP_INFO(this->get_logger(), "\033[1;36mReceived air batch from Desiccant Bed: CO₂=%.2f g, Moisture=%.2f %%, Contaminants=%.2f %%\033[0m",
                co2_, moisture_content_, contaminants_);
    RCLCPP_INFO(this->get_logger(),"==============================================");
    // Start processing cycle
    timer_ = this->create_wall_timer(1s, std::bind(&AdsorbentBed::process_co2, this));

    response->success = true;
    response->message = "Adsorbent Bed started processing the new batch.";
}

void AdsorbentBed::process_co2() {
  std::lock_guard<std::mutex> lock(data_mutex_);

  // Retrieve PID parameters dynamically
  double desired_temperature, temperature_tolerance, kp, kd;
  this->get_parameter("desired_temperature", desired_temperature);
  this->get_parameter("temperature_tolerance", temperature_tolerance);
  this->get_parameter("kp", kp);
  this->get_parameter("kd", kd);

  // **Temperature Regulation using PD Control**
  double error = desired_temperature - temperature_;
  double derivative = error - previous_error_;
  double adjustment = (kp * error) + (kd * derivative);

  temperature_ += adjustment;
  previous_error_ = error;

  RCLCPP_INFO(this->get_logger(), "\033[1;36mAdsorbent bed temperature adjusted to %.2f°C.\033[0m", temperature_);
  RCLCPP_INFO(this->get_logger(),"==============================================");

  // **Wait until the bed reaches the required temperature before starting CO₂ adsorption**
  if (temperature_ < (desired_temperature - temperature_tolerance)) {
      RCLCPP_WARN(this->get_logger(), "Temperature (%.2f°C) too low for adsorption. Waiting...", temperature_);
      return;  // Do not proceed with adsorption yet
  }

  // **Start CO₂ Adsorption Once Temperature is in Range**
  if (temperature_ >= (desired_temperature - temperature_tolerance) &&
      temperature_ <= (desired_temperature + temperature_tolerance)) {

      double co2_decrement = co2_ * co2_removal_efficiency_ * 0.1;
      double co2_retained = co2_decrement;

      co2_ -= co2_decrement;
      if (co2_ < 1.0) co2_ = 0.0;

      co2_buffer_ += co2_retained;  // Store CO₂ in the buffer
      auto msg = demo_nova_sanctum::msg::AirData();
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id="Adsorbent Beds";
      msg.co2_mass = co2_;
      msg.moisture_content = moisture_content_;
      msg.contaminants = contaminants_;
      air_quality_publisher_->publish(msg);
      
      RCLCPP_INFO(this->get_logger(),
      "\033[1;36m ADSORBING CO2 and publishing data\033[0m - Header: [%s]", 
      msg.header.frame_id.c_str());
  

      RCLCPP_INFO(this->get_logger(),
                  "\033[1;36mCO₂ stored in buffer: %.2f g. Remaining CO₂ in air: %.2f g\033[0m",
                  co2_buffer_, co2_);
      RCLCPP_INFO(this->get_logger(),"==============================================");
  }

  

  
  // **Once CO₂ is fully adsorbed, accept new air again**
  if (co2_ <= 3.0) {
      RCLCPP_INFO(this->get_logger(), "\033[1;36mAll CO₂ has been adsorbed. Ready to accept new air.");
      is_active_ = false;  // Mark AdsorbentBed as available for new air
      send_processed_air();  // Send processed air back to Desiccant Bed 2

      cdra.co2_processing_state = 5;
      cdra.system_health = 1;
      cdra.data = "Transferring for desorption";
      cdra_status_publisher_->publish(cdra);
  }
}



void AdsorbentBed::send_processed_air() {
  if (!desiccant_client_->wait_for_service(3s)) {
      RCLCPP_WARN(this->get_logger(), "Desiccant Bed 2 service unavailable. Holding air...");
      return;
  }

  auto request = std::make_shared<demo_nova_sanctum::srv::CrewQuarters::Request>();
  request->co2_mass = 0.0;  // CO₂ is adsorbed, air should be clean
  request->moisture_content = moisture_content_;  // Maintain moisture balance
  request->contaminants = contaminants_;  // Maintain contaminant level

  RCLCPP_INFO(this->get_logger(), "\033[1;36mSending processed air (CO₂-free) to Desiccant Bed 2...\033[0m");
  auto future = desiccant_client_->async_send_request(request,
      [this](rclcpp::Client<demo_nova_sanctum::srv::CrewQuarters>::SharedFuture future) {
          auto response = future.get();
          if (response->success) {
              RCLCPP_INFO(this->get_logger(), "\033[1;36mProcessed air successfully returned to Desiccant Bed 2");
              
              // **Reset System for Next Cycle**
              is_active_ = false; 
              retained_co2_cumulative_ = 0.0;
              co2_buffer_ = 0.0;
              
              cdra.co2_processing_state = 5;
              cdra.system_health = 1;
              cdra.data = "Sending for humidification";
              cdra_status_publisher_->publish(cdra);
              // **Start desorption cycle for CO₂ venting**
              start_desorption_cycle();
          } else {
              RCLCPP_ERROR(this->get_logger(), "Failed to send processed air to Desiccant Bed 2  Retrying...");
              rclcpp::sleep_for(500ms);
              send_processed_air();  // Retry sending air
          }
      }
  );
}


// **Begin CO₂ Desorption**
void AdsorbentBed::start_desorption_cycle() {


  cdra.co2_processing_state = 6;
  cdra.system_health = 1;
  cdra.data = "Starting Desorption";
  cdra_status_publisher_->publish(cdra);

  RCLCPP_INFO(this->get_logger(), "\033[1;36mStarting CO₂ desorption cycle. Heating to 400°F...\033[0m");
  RCLCPP_INFO(this->get_logger(),"==============================================");
  timer_ = this->create_wall_timer(1s, std::bind(&AdsorbentBed::desorb_co2, this));
}

void AdsorbentBed::desorb_co2() {
  std::lock_guard<std::mutex> lock(data_mutex_);

  double desorption_temperature;
  this->get_parameter("desorption_temperature", desorption_temperature);

  // Increase temperature gradually until it reaches desorption temp
  if (temperature_ < desorption_temperature) {
      temperature_ += 5.0;
      RCLCPP_INFO(this->get_logger(), "\033[1;36mHeating to Desorption Temp: %.2f°F", temperature_);
      RCLCPP_INFO(this->get_logger(),"==============================================");
      return;
  }

  if (co2_buffer_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "No CO₂ stored for venting. Skipping venting cycle.");
      cdra.co2_processing_state = 7;
      cdra.system_health = 2;
      cdra.data = "Desorption failed: No CO2 available";
      cdra_status_publisher_->publish(cdra);

      return;
  }

 
  std_msgs::msg::Float64 co2_msg;
  co2_msg.data = co2_buffer_; 
  co2_publisher_->publish(co2_msg);


  cdra.co2_processing_state = 7;
  cdra.system_health = 1;
  cdra.data = "CO2 vented";
  cdra_status_publisher_->publish(cdra);
  RCLCPP_INFO(this->get_logger(), "\033[1;36mCO₂ vented: %.2f g\033[0m", co2_buffer_);
  RCLCPP_INFO(this->get_logger(),"==============================================");


  co2_buffer_ = 0.0;
  is_active_ = false;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AdsorbentBed>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
