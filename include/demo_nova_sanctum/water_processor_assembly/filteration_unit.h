#ifndef WPA_SERVICE_SERVER_HPP
#define WPA_SERVICE_SERVER_HPP

#include <rclcpp/rclcpp.hpp>
#include "demo_nova_sanctum/srv/distillation.hpp"
#include "demo_nova_sanctum/srv/filteration.hpp"

class WaterProcessorAssembly : public rclcpp::Node {
public:
    WaterProcessorAssembly();

private:
    rclcpp::Service<demo_nova_sanctum::srv::Distillation>::SharedPtr wpa_service_;
    rclcpp::Client<demo_nova_sanctum::srv::Filteration>::SharedPtr next_stage_client_;

    void handle_water_processing(
        const std::shared_ptr<demo_nova_sanctum::srv::Distillation::Request> request,
        std::shared_ptr<demo_nova_sanctum::srv::Distillation::Response> response);

    void send_to_next_stage(double filtered_water, double remaining_contaminants, double organics, double ammonia);
};

#endif // WPA_SERVICE_SERVER_HPP
