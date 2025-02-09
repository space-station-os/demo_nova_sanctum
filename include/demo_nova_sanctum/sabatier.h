#ifndef DEMO_NOVA_SANCTUM_SABATIER_PUBLISHER_HPP_
#define DEMO_NOVA_SANCTUM_SABATIER_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include "demo_nova_sanctum/msg/sabatier.hpp"
#include "demo_nova_sanctum/msg/air_data.hpp"
#include "std_msgs/msg/float64.hpp"
#include <chrono>
#include <memory>
#include <string>
#include <vector>

class Sabatier : public rclcpp::Node {
public:
  Sabatier();

private:
  void process_air_data(const demo_nova_sanctum::msg::AirData &msg);
  void process_hydrogen_data(const std_msgs::msg::Float64 &msg);
  void compute_reaction();
  void run_reactor();
  void publish_sabatier_outputs();
  bool safety_check();
  double pid_temperature(double desired, double current);
  double pid_pressure(double desired, double current);
  double pd_ghsv(double desired, double current); 
  
  
  // Air mixture parameters
  double co2_mass_;              // Current CO2 mass in grams
  double h2_mass_;               // Current H2 mass in grams
  double moisture_content_;      // Moisture content in percentage
  double contaminants_;          // Contaminants in percentage
  double dew_point_;             // Dew Point in Celsius
  double total_air_mass_;        // Total air mass in grams

  // Sabatier reaction parameters
  double reactor_temp_;          // Current reactor temperature in Celsius
  double desired_temp_;          // Desired reactor temperature in Celsius
  double reactor_pressure_;      // Current reactor pressure in psi
  double desired_pressure_;      // Desired reactor pressure in psi
  double h2_co2_ratio_;          // H2/CO2 molar ratio
  double ghsv_;                  // Gas Hourly Space Velocity (hr^-1)
  double desired_ghsv_;          // Desired Gas Hourly Space Velocity (hr^-1)
  double total_inlet_flow_;      // Total inlet flow rate (slpm)
  double methane_yield_;         // Methane yield in grams
  double water_yield_;           // Water yield in grams
  double reaction_efficiency_;   // Reaction efficiency in percentage

  
  double temp_integral_;
  double temp_previous_error_;
  double pressure_integral_;
  double pressure_previous_error_;
  double ghsv_previous_error_;
  double integral_temp_error_;
  double integral_pressure_error_;
  double last_temp_error_;
  double last_pressure_error_;
  double last_ghsv_error_;

  double temp_kp_;
  double temp_ki_;
  double temp_kd_;
  double pressure_kp_;
  double pressure_ki_;
  double pressure_kd_;
  double ghsv_kp_;
  double ghsv_kd_;

  // System states
  bool reactor_state_;           // Whether the reactor is active
  bool valve_state_;             // Whether the valve is open or closed

  // ROS publishers, subscribers, and timers
  rclcpp::Publisher<demo_nova_sanctum::msg::Sabatier>::SharedPtr sabatier_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr hydrogen_subscriber_;
  rclcpp::Subscription<demo_nova_sanctum::msg::AirData>::SharedPtr ars_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif // DEMO_NOVA_SANCTUM_SABATIER_PUBLISHER_HPP_
