#include "demo_nova_sanctum/water_status.h"

using namespace std::chrono_literals;

Water::Water()
: Node("water_publisher"),
  water_level(this->declare_parameter<double>("water_level", 100.0)),
  contaminants_level(this->declare_parameter<double>("contaminants_level", 0.0)) {

  water_pub = this->create_publisher<demo_nova_sanctum::msg::Water>("/potable_water", 10);
  ogs_client = this->create_client<std_srvs::srv::Trigger>("/ionization_bed");
  timer_ = this->create_wall_timer(1s, std::bind(&Water::water_publisher, this));
}

void Water::water_publisher() {
  // Update water and contaminants levels
  double contaminants_increment = 0.1; // Simulate contaminants increase
  contaminants_level += contaminants_increment*3;

  // Create and publish the water message
  auto message = demo_nova_sanctum::msg::Water();
  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = "water_collector_tank";
  message.water_level = water_level;
  message.gas_bubbles = 0.0;  //TODO: Simulate gas bubbles
  message.contaminants_level = contaminants_level;
  message.iodine_level=20.0; //ppm
  message.iodine_level=20.0; //ppm
  message.pressure = 14.7; 
  message.temperature = 25.0; 

  RCLCPP_INFO(this->get_logger(), "Publishing water data: Level: %.2f L, Contaminants: %.2f %%", 
              water_level, contaminants_level);

  water_pub->publish(message);

  // Check for high contaminants level and trigger the OGS system if needed
  if (contaminants_level >= 15.0) {
    RCLCPP_WARN(this->get_logger(), "High contaminants level detected! Triggering OGS system...");
    ogs_sys_trigger(ogs_client);
    open_valve();
    contaminants_level = 0.0; // Reset contaminants level after processing
  }
}

void Water::ogs_sys_trigger(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client) {
  if (!client->wait_for_service(5s)) {
    RCLCPP_ERROR(this->get_logger(), "OGS service is not available!");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);

  try {
    auto response = future.get();
    if (response->success) {
      RCLCPP_INFO(this->get_logger(), "OGS system triggered successfully.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to trigger OGS system: %s", response->message.c_str());
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception while calling OGS service: %s", e.what());
  }
}

void Water::open_valve() {
  RCLCPP_INFO(this->get_logger(), "Opening the valve to release water...");
  // Simulate valve opening
  water_level -= 10.0; // Reduce water level after releasing
  if (water_level < 0.0) {
    water_level = 0.0;
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Water>());
  rclcpp::shutdown();
  return 0;
}
