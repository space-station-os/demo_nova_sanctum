#ifndef DEMO_NOVA_SANCTUM_DESICCANT_TANK_HPP_
#define DEMO_NOVA_SANCTUM_DESICCANT_TANK_HPP_

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "demo_nova_sanctum/srv/crew_quarters.hpp"
#include <mutex>
#include "demo_nova_sanctum/msg/cdra_status.hpp"
#include "demo_nova_sanctum/msg/air_data.hpp"
/**
 * @brief A simple PID Controller for temperature and pressure regulation.
 */
struct PIDController
{
  double kp, ki, kd;
  double prev_error = 0.0;
  double integral = 0.0;

  /**
   * @brief Computes the PID output given the setpoint and measured value.
   * 
   * @param setpoint The desired target value.
   * @param measured The current measured value.
   * @param dt The time interval between control updates.
   * @return The computed control output.
   */
  double compute(double setpoint, double measured, double dt)
  {
    double error = setpoint - measured;
    integral += error * dt;
    double derivative = (error - prev_error) / dt;
    double output = (kp * error) + (ki * integral) + (kd * derivative);
    prev_error = error;
    return output;
  }
};

/**
 * @class DesiccantServer
 * @brief Handles moisture and contaminant removal in the desiccant tank.
 * 
 * - **Acts as the `/crew_co2_service` server** to process air from the air collection system.
 * - **Uses a differentiable model** for gradual moisture & contaminant removal.
 * - **Controls temperature & pressure dynamically using PID controllers.**
 * - **Sends air to the Adsorbent Bed when moisture & contaminants reach minimal levels.**
 * - **Handles service failures and triggers emergency alerts if needed.**
 */
class DesiccantServer : public rclcpp::Node
{
public:
  /**
   * @brief Constructor initializes parameters, timers, and service clients.
   */
  DesiccantServer();

private:
  /**
   * @brief Handles incoming air processing requests from the Air Collector system.
   * - If the Desiccant Bed is busy, it rejects new air requests.
   * - If available, it starts processing the new air batch.
   */
  void handle_air_processing_request(const std::shared_ptr<demo_nova_sanctum::srv::CrewQuarters::Request> request,
                                     std::shared_ptr<demo_nova_sanctum::srv::CrewQuarters::Response> response);
  void handle_adsorbed_air(const std::shared_ptr<demo_nova_sanctum::srv::CrewQuarters::Request> request,
                           std::shared_ptr<demo_nova_sanctum::srv::CrewQuarters::Response> response);
  /**
   * @brief Gradually increases moisture to the desired level.
   */ 

  void handle_moisture_request(const std::shared_ptr<demo_nova_sanctum::srv::CrewQuarters::Request> request,
                                      std::shared_ptr<demo_nova_sanctum::srv::CrewQuarters::Response> response);
  /**
   * @brief Gradually reduces moisture and contaminants based on temperature and pressure effects.
   */
  void process_air_data();

  /**
   * @brief Sends the processed air to the Adsorbent Bed once moisture & contaminants reach minimal levels.
   * - Ensures the Adsorbent Bed is available before sending air.
   * - Holds air if the service is unavailable.
   * - Triggers emergency alerts if service remains unavailable for too long.
   */
  void send_to_adsorbent_bed();

  /**
   * @brief Processes the response from the Adsorbent Bed.
   * - Handles service request failures.
   * - Ensures proper system reset after air transfer.
   */
  void process_adsorbent_response(rclcpp::Client<demo_nova_sanctum::srv::CrewQuarters>::SharedFuture future);

  /**
   * @brief Adjusts temperature dynamically for optimal moisture removal efficiency.
   */
  void temperature_callback(const sensor_msgs::msg::Temperature::SharedPtr msg);

  /**
   * @brief Adjusts pressure dynamically for optimal contaminant removal efficiency.
   */
  void pressure_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg);

  /*** AIR PROCESSING PARAMETERS ***/
  double co2_;                 ///< Current CO₂ mass in grams
  double moisture_content_;     ///< Current moisture content in percentage
  double contaminants_;         ///< Current contaminants in percentage

  /*** DYNAMIC PARAMETERS ***/
  double moisture_removal_rate_; ///< Moisture removal efficiency (default 95%)
  double contaminant_removal_rate_; ///< Contaminant removal efficiency (default 90%)
  int emergency_threshold_;  ///< Number of failures before triggering emergency
  double target_temperature_;   ///< Optimal temperature for moisture removal
  double target_pressure_;      ///< Optimal pressure for moisture removal

  /*** ENVIRONMENTAL VARIABLES ***/
  double current_temperature_;  ///< Current chamber temperature
  double current_pressure_;     ///< Current chamber pressure
  double processing_speed_;
  double max_processing_speed_;
  /*** TEMPERATURE & PRESSURE CONTROL USING PID ***/
  PIDController temp_pid_;      ///< PID Controller for temperature regulation
  PIDController press_pid_;     ///< PID Controller for pressure regulation

  /*** ROS INTERFACES ***/
  rclcpp::Service<demo_nova_sanctum::srv::CrewQuarters>::SharedPtr crew_co2_service_; ///< Processes incoming air
  rclcpp::Client<demo_nova_sanctum::srv::CrewQuarters>::SharedPtr adsorbent_server_client_; ///< Sends air to Adsorbent Bed
  rclcpp::Service<demo_nova_sanctum::srv::CrewQuarters>::SharedPtr adsorbent_co2_service_; ///< Sends processed air to Adsorbent Bed
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr temperature_subscriber_; ///< Listens to /temperature
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_subscriber_; ///< Listens to /pipe_pressure
  rclcpp::Publisher<demo_nova_sanctum::msg::CdraStatus>::SharedPtr cdra_status_publisher_;
  rclcpp::Service<demo_nova_sanctum::srv::CrewQuarters>::SharedPtr desiccant_bed_2;
  rclcpp::TimerBase::SharedPtr timer_; ///< Timer for gradual air processing
  rclcpp::Publisher<demo_nova_sanctum::msg::AirData>::SharedPtr air_quality_publisher_;


  demo_nova_sanctum::msg::CdraStatus cdra; ///< CDRA status message
  /*** THREAD SAFETY ***/
  std::mutex data_mutex_; ///< Ensures thread safety when modifying shared air properties

  /*** SERVICE HANDLING ***/
  int service_unavailable_count_; ///< Tracks service failures to prevent system errors

  /*** ACTIVE STATE ***/
  bool is_active_; //< Indicates if the desiccant bed 1 is currently processing air
  bool is_active_dessicant;
  bool is_active_desiccant2; ///< Indicates if the desiccant bed 2 is currently processing air
};

#endif // DEMO_NOVA_SANCTUM_DESICCANT_TANK_HPP_
