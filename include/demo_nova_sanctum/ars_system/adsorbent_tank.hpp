#ifndef DEMO_NOVA_SANCTUM_ADSORBENT_BED_HPP_
#define DEMO_NOVA_SANCTUM_ADSORBENT_BED_HPP_

#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include "demo_nova_sanctum/msg/air_data.hpp"
#include "demo_nova_sanctum/srv/crew_quarters.hpp"

/**
 * @class AdsorbentBed
 * @brief ROS 2 node to manage CO₂ adsorption and processing.
 * 
 * - Acts as a **server for the `/adsorbent_server` service** to receive air from the Desiccant Bed.
 * - **Gradually removes CO₂** and **splits it into two streams** (sent to space & retained for Sabatier).
 * - **Publishes processed CO₂ to the `/co2` topic** for further chemical conversion.
 * - **Regulates temperature dynamically using PID control** for efficient CO₂ adsorption.
 * - **Ensures controlled activation** through service requests.
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
   * - If available, it starts processing the CO₂ batch.
   */
  void handle_air_processing_request(const std::shared_ptr<demo_nova_sanctum::srv::CrewQuarters::Request> request,
                                     std::shared_ptr<demo_nova_sanctum::srv::CrewQuarters::Response> response);

  /**
   * @brief Gradually removes CO₂, ensuring temperature control.
   * - Uses **PID regulation** to maintain optimal adsorption temperature.
   * - **Splits processed CO₂** into space-bound and Sabatier processing streams.
   * - Publishes retained CO₂ to `/co2` for Sabatier processing.
   */
  void process_co2();

  /*** CO₂ PROCESSING PARAMETERS ***/
  double co2_removal_efficiency_; ///< Efficiency of CO₂ adsorption (default: 90%)
  double co2_to_space_ratio_;     ///< Percentage of CO₂ sent to space (default: 50%)
  double retained_co2_cumulative_; ///< Total CO₂ retained for Sabatier
  double previous_error_; ///< Previous error for PID control

  /*** AIR DATA VARIABLES ***/
  double co2_;                ///< Current CO₂ mass
  double moisture_content_;   ///< Current moisture content
  double contaminants_;       ///< Current contaminant level
  double temperature_;        ///< Current temperature
  double dew_point_;          ///< Current dew point

  /*** ROS COMMUNICATION INTERFACES ***/
  rclcpp::Service<demo_nova_sanctum::srv::CrewQuarters>::SharedPtr adsorbent_service_; ///< Receives air from Desiccant Bed
  rclcpp::Publisher<demo_nova_sanctum::msg::AirData>::SharedPtr co2_publisher_; ///< Publishes processed CO₂ to `/co2`
  rclcpp::TimerBase::SharedPtr timer_; ///< Timer for periodic CO₂ processing

  /*** SYSTEM STATES ***/
  bool is_active_; ///< Indicates if the adsorbent bed is currently processing air

  /*** THREAD SAFETY ***/
  std::mutex data_mutex_; ///< Ensures safe access to shared air data variables
};

#endif // DEMO_NOVA_SANCTUM_ADSORBENT_BED_HPP_
