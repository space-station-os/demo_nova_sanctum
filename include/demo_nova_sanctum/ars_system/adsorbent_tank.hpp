#ifndef DEMO_NOVA_SANCTUM_ADSORBENT_TANK_HPP_
#define DEMO_NOVA_SANCTUM_ADSORBENT_TANK_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include "demo_nova_sanctum/srv/crew_quarters.hpp"
#include "demo_nova_sanctum/msg/cdra_status.hpp"
#include "demo_nova_sanctum/msg/air_data.hpp"
#include <mutex>

/**
 * @brief Adsorbent Bed 1: Handles CO₂ adsorption from incoming air
 */
class AdsorbentBed1 : public rclcpp::Node {
public:
  AdsorbentBed1();

private:
  void handle_request(
    const std::shared_ptr<demo_nova_sanctum::srv::CrewQuarters::Request> request,
    std::shared_ptr<demo_nova_sanctum::srv::CrewQuarters::Response> response);
  void process_adsorption();
  void send_to_desiccant_bed_2();

  rclcpp::Service<demo_nova_sanctum::srv::CrewQuarters>::SharedPtr bed1_service_;
  rclcpp::Client<demo_nova_sanctum::srv::CrewQuarters>::SharedPtr desiccant_client_;
  rclcpp::Publisher<demo_nova_sanctum::msg::AirData>::SharedPtr air_quality_publisher_;
  rclcpp::Publisher<demo_nova_sanctum::msg::CdraStatus>::SharedPtr cdra_status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  demo_nova_sanctum::msg::CdraStatus cdra;
  rclcpp::Client<demo_nova_sanctum::srv::CrewQuarters>::SharedPtr desorption_client_;

  std::mutex data_mutex_;
  double co2_, moisture_, contaminants_, co2_buffer_;
  double temperature_ = 300.0;
  double previous_error_ = 0.0;
  bool is_active_;
  double tank_capacity_;
};

/**
 * @brief Adsorbent Bed 2: Handles desorption and stores CO₂ into tank
 */
class AdsorbentBed2 : public rclcpp::Node {
public:
  AdsorbentBed2();

private:
  void handle_request(
    const std::shared_ptr<demo_nova_sanctum::srv::CrewQuarters::Request> request,
    std::shared_ptr<demo_nova_sanctum::srv::CrewQuarters::Response> response);
  void desorb_co2();

  rclcpp::Service<demo_nova_sanctum::srv::CrewQuarters>::SharedPtr bed2_service_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr co2_publisher_;
  rclcpp::Publisher<demo_nova_sanctum::msg::CdraStatus>::SharedPtr cdra_status_publisher_;
  rclcpp::Client<demo_nova_sanctum::srv::CrewQuarters>::SharedPtr desiccant2_client_;

  rclcpp::TimerBase::SharedPtr timer_;
  demo_nova_sanctum::msg::CdraStatus cdra;

  std::mutex data_mutex_;
  double co2_buffer_;
  double temperature_ = 300.0;
  bool is_active_;
  bool has_heated_;
  
};

#endif  // DEMO_NOVA_SANCTUM_ADSORBENT_TANK_HPP_
