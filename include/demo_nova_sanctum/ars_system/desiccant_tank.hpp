#ifndef DEMO_NOVA_SANCTUM_DESICCANT_BEDS_HPP_
#define DEMO_NOVA_SANCTUM_DESICCANT_BEDS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <demo_nova_sanctum/srv/crew_quarters.hpp>
#include <demo_nova_sanctum/msg/air_data.hpp>
#include <demo_nova_sanctum/msg/cdra_status.hpp>
#include <mutex>
#include <memory>

class DesiccantBed1 : public rclcpp::Node
{
public:
  DesiccantBed1();

private:
  // --- Callback for service ---
  void handle_request(
    const std::shared_ptr<demo_nova_sanctum::srv::CrewQuarters::Request> request,
    std::shared_ptr<demo_nova_sanctum::srv::CrewQuarters::Response> response);

  // --- Internal processing ---
  void process_air();

  // --- Internal state ---
  bool is_active_;
  double co2_;
  double moisture_;
  double contaminants_;
  int retry_count_;
  const int max_retries_;
  
  // --- ROS Interfaces ---
  rclcpp::Service<demo_nova_sanctum::srv::CrewQuarters>::SharedPtr bed1_service_;
  rclcpp::Client<demo_nova_sanctum::srv::CrewQuarters>::SharedPtr adsorbent_client_;
  rclcpp::Publisher<demo_nova_sanctum::msg::AirData>::SharedPtr air_quality_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --- Mutex for safe data access ---
  std::mutex data_mutex_;
};


class DesiccantBed2 : public rclcpp::Node {
public:
  DesiccantBed2();

private:
  void handle_request(
    const std::shared_ptr<demo_nova_sanctum::srv::CrewQuarters::Request> request,
    std::shared_ptr<demo_nova_sanctum::srv::CrewQuarters::Response> response);

  void humidify_air();

  double co2_, contaminants_;
  double moisture_;
  bool is_active_;
  double humidification_rate_;
  std::mutex data_mutex_;

  rclcpp::Service<demo_nova_sanctum::srv::CrewQuarters>::SharedPtr bed2_service_;
  rclcpp::Publisher<demo_nova_sanctum::msg::CdraStatus>::SharedPtr cdra_status_pub_;
  rclcpp::Publisher<demo_nova_sanctum::msg::AirData>::SharedPtr air_quality_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  demo_nova_sanctum::msg::CdraStatus cdra_;
};

#endif  // DEMO_NOVA_SANCTUM_DESICCANT_BEDS_HPP_
