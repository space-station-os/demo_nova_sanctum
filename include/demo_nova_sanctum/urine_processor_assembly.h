#ifndef UPA_SERVICE_SERVER_HPP
#define UPA_SERVICE_SERVER_HPP

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp"
#include "demo_nova_sanctum/srv/distillation.hpp"
#include "demo_nova_sanctum/srv/upa.hpp"

class UrineProcessorAssembly : public rclcpp::Node {
public:
    UrineProcessorAssembly();

private:
    rclcpp::Service<demo_nova_sanctum::srv::Upa>::SharedPtr upa_service_;
    rclcpp::Client<demo_nova_sanctum::srv::Distillation>::SharedPtr wpa_client_;

    void handle_urine_request(
        const std::shared_ptr<demo_nova_sanctum::srv::Upa::Request> request,
        std::shared_ptr<demo_nova_sanctum::srv::Upa::Response> response);

    void simulate_distillation(double urine_volume, double &distilled_urine, double &contaminants);
    void simulate_purge_pump(double &distilled_urine, double &temperature);
    void send_to_wpa(double processed_water, double contaminants);
};

#endif // UPA_SERVICE_SERVER_HPP
