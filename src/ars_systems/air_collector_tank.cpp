#include "demo_nova_sanctum/ars_system/air_collector_tank.hpp"
#include <cmath>

using namespace std::chrono_literals;

AirCollector::AirCollector()
    : Node("air_collector"), previous_mode_("idle"), service_unavailable_count_(0), state_(AirCollectorState::IDLE)  // Initialize state to IDLE
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

  this->declare_parameter("co2_threshold", 100.0);
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

  cdra_status_publisher_ = this->create_publisher<demo_nova_sanctum::msg::CdraStatus>("/cdra_status", 10);
  cdra=demo_nova_sanctum::msg::CdraStatus();  
  RCLCPP_INFO(this->get_logger(), "Air Collector System successfully initialized.");
}

void AirCollector::timer_callback()
{
  // Retrieve parameters dynamically
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

  // **Simulate Temperature & Pressure**
  simulate_temperature_pressure();

  // **Adjust C_activity based on Mode of Operation**
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

  // **Simulate Air Composition (CO₂, Moisture, Contaminants)**
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

  // **Log Current Air Composition**
  RCLCPP_INFO(this->get_logger(), "State: %d | CO₂: %.2f g, Moisture: %.2f %%, Contaminants: %.2f %%",
              static_cast<int>(state_), co2_mass_, moisture_content_, contaminants_);
  RCLCPP_INFO(this->get_logger(),"==============================================");

  // **Update Transition System State**
  update_state();
}

void AirCollector::update_state() {
    static std::vector<demo_nova_sanctum::msg::TransitionPair> valid_transitions; // Persist across calls
    demo_nova_sanctum::msg::TransitionPair transition;

    RCLCPP_INFO(this->get_logger(), "[TS] Checking state transitions. Current state: %d", static_cast<int>(state_));

    bool transition_occurred = false;

    switch (state_) {
        case AirCollectorState::IDLE:
            if (co2_mass_ < co2_threshold_) { // Condition to transition
                transition.current_state = static_cast<int32_t>(AirCollectorState::IDLE);
                transition.next_state = static_cast<int32_t>(AirCollectorState::COLLECTING);
                
                if (valid_transitions.empty() || 
                    valid_transitions.back().current_state != transition.current_state ||
                    valid_transitions.back().next_state != transition.next_state) 
                {
                    valid_transitions.clear(); // Clear only when a change occurs
                    valid_transitions.push_back(transition);
                    transition_occurred = true;
                }

                RCLCPP_INFO(this->get_logger(), "[TS] Transition: IDLE → COLLECTING");
                state_ = AirCollectorState::COLLECTING;
            }
            break;

        case AirCollectorState::COLLECTING:
            if (co2_mass_ >= co2_threshold_ || moisture_content_ > moisture_threshold_) {
                transition.current_state = static_cast<int32_t>(AirCollectorState::COLLECTING);
                transition.next_state = static_cast<int32_t>(AirCollectorState::THRESHOLD_REACHED);

                if (valid_transitions.empty() || 
                    valid_transitions.back().current_state != transition.current_state ||
                    valid_transitions.back().next_state != transition.next_state) 
                {
                    valid_transitions.clear();
                    valid_transitions.push_back(transition);
                    transition_occurred = true;
                }

                RCLCPP_INFO(this->get_logger(), "[TS] Transition: COLLECTING → THRESHOLD_REACHED");
                state_ = AirCollectorState::THRESHOLD_REACHED;
            }
            break;

        case AirCollectorState::THRESHOLD_REACHED:
            if (desiccant_bed_client_->wait_for_service(3s)) {
                transition.current_state = static_cast<int32_t>(AirCollectorState::THRESHOLD_REACHED);
                transition.next_state = static_cast<int32_t>(AirCollectorState::SENDING_TO_DESICCANT);

                if (valid_transitions.empty() || 
                    valid_transitions.back().current_state != transition.current_state ||
                    valid_transitions.back().next_state != transition.next_state) 
                {
                    valid_transitions.clear();
                    valid_transitions.push_back(transition);
                    transition_occurred = true;
                }

                RCLCPP_INFO(this->get_logger(), "[TS] Transition: THRESHOLD_REACHED → SENDING_TO_DESICCANT");
                state_ = AirCollectorState::SENDING_TO_DESICCANT;
                send_air_to_desiccant_bed();
            } else {
                RCLCPP_ERROR(this->get_logger(), "[ERROR] Desiccant Bed service is unavailable. Holding air!");
            }
            break;

        case AirCollectorState::SENDING_TO_DESICCANT:
            transition.current_state = static_cast<int32_t>(AirCollectorState::SENDING_TO_DESICCANT);
            transition.next_state = static_cast<int32_t>(AirCollectorState::IDLE);

            if (valid_transitions.empty() || 
                valid_transitions.back().current_state != transition.current_state ||
                valid_transitions.back().next_state != transition.next_state) 
            {
                valid_transitions.clear();
                valid_transitions.push_back(transition);
                transition_occurred = true;
            }

            RCLCPP_INFO(this->get_logger(), "[TS] Transition: SENDING_TO_DESICCANT → IDLE");
            state_ = AirCollectorState::IDLE;
            break;
    }

    // **Ensure valid transitions are added to the message**
    cdra.co2_processing_state = static_cast<int>(state_);
    cdra.system_health = (service_unavailable_count_ < 3) ? 1 : 2;

    if (transition_occurred) {
        cdra.valid_transitions = valid_transitions;  // Update only on transition
    }

    if (valid_transitions.empty()) {
        RCLCPP_WARN(this->get_logger(), "[TS] Warning: No valid transitions detected for state %d", state_);
    } else {
        for (const auto& trans : valid_transitions) {
            RCLCPP_INFO(this->get_logger(), "[TS] Valid Transition Persisted: (%d → %d)", trans.current_state, trans.next_state);
        }
    }

    cdra_status_publisher_->publish(cdra);
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

  RCLCPP_INFO(this->get_logger(), "[REQ] Requesting Desiccant Bed to process air batch...");
 
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





void AirCollector::simulate_temperature_pressure() {
  // Retrieve PID parameters dynamically
  double temp_kp, temp_ki, temp_kd, press_kp, press_ki, press_kd;
  this->get_parameter("temp_kp", temp_kp);
  this->get_parameter("temp_ki", temp_ki);
  this->get_parameter("temp_kd", temp_kd);
  this->get_parameter("press_kp", press_kp);
  this->get_parameter("press_ki", press_ki);
  this->get_parameter("press_kd", press_kd);
  
  double desired_temperature = 70.0;  // **Target temp set to 70°C**
  double desired_pressure = cabin_pressure_;  // **Maintain cabin pressure**

  // **PD control logic for temperature adjustment**
  double temp_error = desired_temperature - current_temperature_;
  double temp_derivative = temp_error - prev_temp_error_;
  double temp_adjustment = (temp_kp * temp_error) + (temp_kd * temp_derivative);
  prev_temp_error_ = temp_error;

  // **Limit the temperature increase to prevent excessive oscillation**
  if (std::abs(temp_error) > 1.0) {  // Only adjust if deviation is significant
    current_temperature_ += temp_adjustment;
  }

  // **Prevent overshoot**
  if (current_temperature_ > desired_temperature + 1.0) {
    current_temperature_ = desired_temperature + 1.0;
  } else if (current_temperature_ < desired_temperature - 1.0) {
    current_temperature_ = desired_temperature - 1.0;
  }

  // **PD control logic for pressure adjustment**
  double press_error = desired_pressure - current_pressure_;
  double press_derivative = press_error - prev_press_error_;
  double press_adjustment = (press_kp * press_error) + (press_kd * press_derivative);
  prev_press_error_ = press_error;

  current_pressure_ += press_adjustment;

  // **Publish Temperature**
  sensor_msgs::msg::Temperature temp_msg;
  temp_msg.temperature = current_temperature_;
  temp_msg.variance = 0.5;  // Simulated variance
  temperature_publisher_->publish(temp_msg);

  // **Publish Pressure**
  sensor_msgs::msg::FluidPressure press_msg;
  press_msg.fluid_pressure = current_pressure_;
  press_msg.variance = 0.1;
  pressure_publisher_->publish(press_msg);

  // **Log Temperature and Pressure**
  RCLCPP_INFO(this->get_logger(), "Simulated Temperature: %.2f°C (Target: 70°C), Pressure: %.2f psi", 
              current_temperature_, current_pressure_);
}



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AirCollector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
