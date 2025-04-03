#ifndef DEMO_NOVA_SANCTUM_ADSORBENT_BED_HPP_
#define DEMO_NOVA_SANCTUM_ADSORBENT_BED_HPP_

#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include "demo_nova_sanctum/msg/air_data.hpp"
#include "demo_nova_sanctum/srv/crew_quarters.hpp"
#include "std_msgs/msg/float64.hpp"
#include "demo_nova_sanctum/msg/cdra_status.hpp"
#include "demo_nova_sanctum/msg/air_data.hpp"

/**
 * @class AdsorbentBed
 * @brief ROS 2 node to manage CO₂ adsorption, storage, and desorption.
 * 
 * - **Receives air from the Desiccant Bed** via `/adsorbent_server` service.
 * - **Adsorbs and stores CO₂** in a buffer before desorption.
 * - **Returns CO₂-free air to the Desiccant Bed** via `/processed_air_service`.
 * - **Heats up to 400°F to desorb and vent CO₂** after processing.
 * - **Publishes vented CO₂ to `/co2_vent`** topic.
 */
class AdsorbentBed : public rclcpp::Node {
public:
  /**
   * @brief Constructor initializes the Adsorbent Bed service.
   */
  AdsorbentBed();

private:
  /**
   * @brief Handles incoming air processing requests from the Desiccant Bed.
   * - If already processing, it rejects new requests.
   * - If available, it starts the CO₂ adsorption process.
   */
  void handle_air_processing_request(const std::shared_ptr<demo_nova_sanctum::srv::CrewQuarters::Request> request,
                                     std::shared_ptr<demo_nova_sanctum::srv::CrewQuarters::Response> response);

  /**
   * @brief Adsorbs CO₂ while regulating temperature.
   * - **Uses PID control** to maintain optimal adsorption temperature.
   * - **Stores CO₂ in a buffer** instead of immediately venting.
   */
  void process_co2();

  /**
   * @brief Sends CO₂-free air back to the Desiccant Bed.
   */
  void send_processed_air();

  /**
   * @brief Begins CO₂ desorption cycle by heating up.
   */
  void start_desorption_cycle();

  /**
   * @brief Heats up to 400°F and vents CO₂.
   */
  void desorb_co2();

  /*** CO₂ PROCESSING PARAMETERS ***/
  double co2_removal_efficiency_; ///< Efficiency of CO₂ adsorption (default: 90%)
  double co2_to_space_ratio_;     ///< Percentage of CO₂ sent to space (default: 50%)
  double retained_co2_cumulative_; ///< Total CO₂ retained before desorption
  double previous_error_; ///< Previous error for PID control
  double co2_buffer_; ///< Buffer to store CO₂ before desorption

  /*** AIR DATA VARIABLES ***/
  double co2_;                ///< Current CO₂ mass
  double moisture_content_;   ///< Current moisture content
  double contaminants_;       ///< Current contaminant level
  double temperature_;        ///< Current temperature

  /*** ROS COMMUNICATION INTERFACES ***/
  rclcpp::Service<demo_nova_sanctum::srv::CrewQuarters>::SharedPtr adsorbent_service_; ///< Receives air from Desiccant Bed
  rclcpp::Client<demo_nova_sanctum::srv::CrewQuarters>::SharedPtr desiccant_client_; ///< Sends processed air back
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr co2_publisher_; ///< Publishes vented CO₂ to `/co2_vent`
  rclcpp::TimerBase::SharedPtr timer_; ///< Timer for periodic processing
  rclcpp::Publisher<demo_nova_sanctum::msg::CdraStatus>::SharedPtr cdra_status_publisher_;
  demo_nova_sanctum::msg::CdraStatus cdra;
  rclcpp::Publisher<demo_nova_sanctum::msg::AirData>::SharedPtr air_quality_publisher_;
  /*** SYSTEM STATES ***/
  bool is_active_; ///< Indicates if the adsorbent bed is currently processing air

  /*** THREAD SAFETY ***/
  std::mutex data_mutex_; ///< Ensures safe access to shared air data variables
};

#endif // DEMO_NOVA_SANCTUM_ADSORBENT_BED_HPP_
