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
  desiccant_client_ = this->create_client<demo_nova_sanctum::srv::CrewQuarters>("/processed_air_service");

  // Publisher for CO₂ desorption (venting)
  co2_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/co2_vent", 10);

  RCLCPP_INFO(this->get_logger(), "Adsorbent Bed successfully initialized.");
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

    // Retrieve parameters dynamically from YAML
    this->get_parameter("co2_removal_efficiency", co2_removal_efficiency_);
    this->get_parameter("co2_to_space_ratio", co2_to_space_ratio_);

    // Accept air for processing
    is_active_ = true;
    co2_ = request->co2_mass;
    moisture_content_ = request->moisture_content;
    contaminants_ = request->contaminants;

    RCLCPP_INFO(this->get_logger(), "Received air batch from Desiccant Bed: CO₂=%.2f g, Moisture=%.2f %%, Contaminants=%.2f %%",
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

  // **Temperature Regulation using PID**
  double error = desired_temperature - temperature_;
  double derivative = error - previous_error_;
  double adjustment = (kp * error) + (kd * derivative);

  temperature_ += adjustment;
  previous_error_ = error;

  RCLCPP_INFO(this->get_logger(), "Adsorbent bed temperature adjusted to %.2f°C.", temperature_);
  RCLCPP_INFO(this->get_logger(),"==============================================");

  // **Process CO₂ within optimal temperature range**
  if (temperature_ >= (desired_temperature - temperature_tolerance) && temperature_ <= (desired_temperature + temperature_tolerance)) {
      double co2_decrement = co2_ * co2_removal_efficiency_ * 0.1; 
      double co2_retained = co2_decrement;

      co2_ -= co2_decrement;
      if (co2_ < 1.0) co2_ = 0.0;

      co2_buffer_ += co2_retained; // Store CO₂ in the buffer

      RCLCPP_INFO(this->get_logger(),
                  "CO₂ stored in buffer: %.2f g. Remaining CO₂ in air: %.2f g",
                  co2_buffer_, co2_);

      RCLCPP_INFO(this->get_logger(),"==============================================");
  }

  // **Once CO₂ is fully adsorbed, accept new air again**
  if (co2_ <= 1.0) {
      RCLCPP_INFO(this->get_logger(), "All CO₂ has been adsorbed. Ready to accept new air.");
      is_active_ = false;  // Mark AdsorbentBed as available for new air
      send_processed_air();  // Send processed air back to Desiccant Bed2
  }
}


void AdsorbentBed::send_processed_air() {
  if (!desiccant_client_->wait_for_service(3s)) {
      RCLCPP_WARN(this->get_logger(), "Desiccant Bed service unavailable. Holding air...");
      return;
  }

  auto request = std::make_shared<demo_nova_sanctum::srv::CrewQuarters::Request>();
  request->co2_mass = 0.0;  // CO₂ is adsorbed, air should be clean
  request->moisture_content = moisture_content_;  // Maintain moisture balance
  request->contaminants = contaminants_;  // Maintain contaminant level

  RCLCPP_INFO(this->get_logger(), "Sending processed air (CO₂-free) to Desiccant Bed...");
  auto future = desiccant_client_->async_send_request(request,
      [this](rclcpp::Client<demo_nova_sanctum::srv::CrewQuarters>::SharedFuture future) {
          auto response = future.get();
          if (response->success) {
              RCLCPP_INFO(this->get_logger(), "Processed air successfully returned to Desiccant Bed.");
              
              // **Reset System for Next Cycle**
              is_active_ = false; 
              retained_co2_cumulative_ = 0.0;
              co2_buffer_ = 0.0;
              
              // **Start desorption cycle for CO₂ venting**
              start_desorption_cycle();
          } else {
              RCLCPP_ERROR(this->get_logger(), "Failed to send processed air to Desiccant Bed. Retrying...");
              rclcpp::sleep_for(500ms);
              send_processed_air();  // Retry sending air
          }
      }
  );
}


// **Begin CO₂ Desorption**
void AdsorbentBed::start_desorption_cycle() {
  RCLCPP_INFO(this->get_logger(), "Starting CO₂ desorption cycle. Heating to 400°F...");
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
      RCLCPP_INFO(this->get_logger(), "Heating to Desorption Temp: %.2f°F", temperature_);
      RCLCPP_INFO(this->get_logger(),"==============================================");
      return;
  }

  if (co2_buffer_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "No CO₂ stored for venting. Skipping venting cycle.");
      return;
  }

 
  std_msgs::msg::Float64 co2_msg;
  co2_msg.data = co2_buffer_; 
  co2_publisher_->publish(co2_msg);

  RCLCPP_INFO(this->get_logger(), "CO₂ vented: %.2f g", co2_buffer_);
  RCLCPP_INFO(this->get_logger(),"==============================================");

  // ✅ **Reset CO₂ Buffer Only AFTER Publishing**
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
