#include "demo_nova_sanctum/ars_system/air_collector_tank.hpp"
#include <cmath>

using namespace std::chrono_literals;

AirCollector::AirCollector()
    : Node("air_collector"), previous_mode_("idle"), service_unavailable_count_(0)  // Initialize previous_mode_
{
  RCLCPP_INFO(this->get_logger(), "Initializing Air Collector System...");

  // Load parameters dynamically
  this->declare_parameter("crew_onboard", 4);
  this->declare_parameter("cabin_pressure", 14.7);
  this->declare_parameter("temperature_cutoff", 450.0);
  this->declare_parameter("max_crew_limit", 6);
  this->declare_parameter("power_consumption", 1.0);
  this->declare_parameter("tank_capacity", 1000.0);
  this->declare_parameter("system_name", "demo_nova_sanctum");
  this->declare_parameter("mode_of_operation", "idle");  

  this->declare_parameter("co2_threshold", 500.0);
  this->declare_parameter("moisture_threshold", 70.0);
  this->declare_parameter("contaminants_threshold", 30.0);

  this->declare_parameter("temp_kp", 0.1);
  this->declare_parameter("temp_ki", 0.01);
  this->declare_parameter("temp_kd", 0.005);
  this->declare_parameter("press_kp", 0.1);
  this->declare_parameter("press_ki", 0.01);
  this->declare_parameter("press_kd", 0.005);

  this->declare_parameter("C_activity", 1.04);  

  temperature_publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>("/temperature", 10);
  pressure_publisher_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("/pipe_pressure", 10);
  desiccant_bed_client_ = this->create_client<demo_nova_sanctum::srv::CrewQuarters>("/crew_co2_service");

  timer_ = this->create_wall_timer(1s, std::bind(&AirCollector::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Air Collector System successfully initialized.");
}

void AirCollector::timer_callback()
{
  this->get_parameter("crew_onboard", crew_onboard_);
  this->get_parameter("cabin_pressure", cabin_pressure_);
  this->get_parameter("temperature_cutoff", temperature_cutoff_);
  this->get_parameter("power_consumption", power_consumption_);
  this->get_parameter("tank_capacity", tank_capacity_);

  this->get_parameter("co2_threshold", co2_threshold_);
  this->get_parameter("moisture_threshold", moisture_threshold_);
  this->get_parameter("contaminants_threshold", contaminants_threshold_);
  this->get_parameter("mode_of_operation", mode_of_operation_);

  double delta_t = 1.0;  

  double C_activity;
  this->get_parameter("C_activity", C_activity);

  if (mode_of_operation_ == "idle") {
      C_activity = 0.8;
  } else if (mode_of_operation_ == "exercise") {
      C_activity = 2.5;
  } else if (mode_of_operation_ == "emergency") {
      C_activity = 3.0;
  } else if (mode_of_operation_ == "biological_research") {
      C_activity = 1.2;
  } else if (mode_of_operation_ == "eva_repair") {
      C_activity = 2.8;
  }

  if (mode_of_operation_ != previous_mode_) {
      RCLCPP_INFO(this->get_logger(), "[Mode Change] %s -> %s | Adjusted CO₂ Activity Rate: %.2f g/min per astronaut", 
                  previous_mode_.c_str(), mode_of_operation_.c_str(), C_activity);
      previous_mode_ = mode_of_operation_; 
  }

  double delta_co2 = (crew_onboard_ * C_activity * (cabin_pressure_ / 14.7) * (temperature_cutoff_ / 450.0)) * delta_t;
  co2_mass_ += delta_co2;

  double moisture_factor = (mode_of_operation_ == "exercise") ? 1.5 :
                           (mode_of_operation_ == "emergency") ? 2.0 :
                           (mode_of_operation_ == "biological_research") ? 0.7 :
                           (mode_of_operation_ == "eva_repair") ? 0.2 : 0.5;

  double delta_moisture = (crew_onboard_ * moisture_factor + (temperature_cutoff_ / 450.0) * 2.5) * delta_t;
  moisture_content_ += delta_moisture;

  double contaminants_factor = (mode_of_operation_ == "exercise") ? 0.5 :
                               (mode_of_operation_ == "emergency") ? 0.7 :
                               (mode_of_operation_ == "biological_research") ? 0.4 :
                               (mode_of_operation_ == "eva_repair") ? 0.8 : 0.3;

  double delta_contaminants = (crew_onboard_ * contaminants_factor - 0.1 * power_consumption_) * delta_t;
  contaminants_ += delta_contaminants;

  RCLCPP_INFO(this->get_logger(), "CO₂ Mass: %.2f g, Moisture: %.2f %%, Contaminants: %.2f %%",
              co2_mass_, moisture_content_, contaminants_);
  RCLCPP_INFO(this->get_logger(),"==============================================");

  if (co2_mass_ > co2_threshold_ || moisture_content_ > moisture_threshold_ || contaminants_ > contaminants_threshold_) 
  {
    if (!desiccant_bed_client_->wait_for_service(3s))  
    {
      service_unavailable_count_++;
      RCLCPP_WARN(this->get_logger(), "Desiccant Bed Service unavailable. Holding air... (%d attempts)", service_unavailable_count_);
      
      if (service_unavailable_count_ >= 5)  
      {
        RCLCPP_ERROR(this->get_logger(), "EMERGENCY: Desiccant Bed Service unavailable for too long! Immediate action required!");
      }
    } 
    else 
    {
      service_unavailable_count_ = 0;  
      send_air_to_desiccant_bed();
    }
  }
}

void AirCollector::send_air_to_desiccant_bed() {
  if (!desiccant_bed_client_->wait_for_service(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Desiccant Bed service is not available!");
      return;
  }

  auto request = std::make_shared<demo_nova_sanctum::srv::CrewQuarters::Request>();
  request->co2_mass = co2_mass_;
  request->moisture_content = moisture_content_;
  request->contaminants = contaminants_;

  RCLCPP_INFO(this->get_logger(), "Requesting Desiccant Bed to process air batch...");
  RCLCPP_INFO(this->get_logger(),"==============================================");
  // ✅ Fix: Bind function correctly (no parentheses after function name)
  auto future = desiccant_bed_client_->async_send_request(request,
    std::bind(&AirCollector::process_desiccant_response, this, std::placeholders::_1));
}

void AirCollector::process_desiccant_response(rclcpp::Client<demo_nova_sanctum::srv::CrewQuarters>::SharedFuture future) {
  try {
      auto response = future.get();
      if (response->success) {
          RCLCPP_INFO(this->get_logger(), "Desiccant Bed accepted air for processing. Opening valve...");
          
          co2_mass_ = 0.0;
          moisture_content_ = 0.0;
          contaminants_ = 0.0;

          RCLCPP_INFO(this->get_logger(), "Air Collector reset. Continuing collection...");
      } else {
          RCLCPP_WARN(this->get_logger(), "Desiccant Bed is still processing. Holding air...");
      }
  } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception while calling Desiccant Bed: %s", e.what());
  }
}



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AirCollector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
