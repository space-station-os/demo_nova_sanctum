#ifndef URINE_PROCESSOR_ASSEMBLY_HPP
#define URINE_PROCESSOR_ASSEMBLY_HPP

#include <rclcpp/rclcpp.hpp>
#include "demo_nova_sanctum/srv/upa.hpp"
#include "demo_nova_sanctum/srv/distillation.hpp"

class UrineProcessorAssembly : public rclcpp::Node {
public:
    UrineProcessorAssembly();

private:
    rclcpp::Service<demo_nova_sanctum::srv::Upa>::SharedPtr upa_service_;
    rclcpp::Client<demo_nova_sanctum::srv::Distillation>::SharedPtr wpa_client_;

    bool is_processing_;       // Tracks if UPA is currently processing a batch
    double unprocessed_water_; // Stores processed water if WPA is unavailable

    void handle_urine_request(
        const std::shared_ptr<demo_nova_sanctum::srv::Upa::Request> request,
        std::shared_ptr<demo_nova_sanctum::srv::Upa::Response> response);

    void simulate_distillation(double urine_volume, double &distilled_urine, double &contaminants);
    void simulate_purge_pump(double &distilled_urine);
    void send_to_wpa(double processed_water, double contaminants);
    void handle_wpa_response(rclcpp::Client<demo_nova_sanctum::srv::Distillation>::SharedFuture future);
};

#endif // URINE_PROCESSOR_ASSEMBLY_HPP
