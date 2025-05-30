#ifndef MULTI_FILTRATION_PROCESSOR_HPP
#define MULTI_FILTRATION_PROCESSOR_HPP

#include <rclcpp/rclcpp.hpp>
#include "demo_nova_sanctum/srv/filteration.hpp"
#include "demo_nova_sanctum/srv/ion_bed.hpp"
#include "demo_nova_sanctum/msg/water_crew.hpp"

class MultiFiltrationProcessor : public rclcpp::Node {
public:
    MultiFiltrationProcessor();

private:
    rclcpp::Service<demo_nova_sanctum::srv::Filteration>::SharedPtr multi_filtration_service_;
    rclcpp::Client<demo_nova_sanctum::srv::IonBed>::SharedPtr catalytic_reactor_client_;
    rclcpp::Publisher<demo_nova_sanctum::msg::WaterCrew>::SharedPtr waste_status_publisher_;
   
    double unprocessed_water_;  // Stores water if Catalytic Reactor is unavailable

    void handle_multi_filtration(
        const std::shared_ptr<demo_nova_sanctum::srv::Filteration::Request> request,
        std::shared_ptr<demo_nova_sanctum::srv::Filteration::Response> response);
    void publish_total_water(double water, double gas_bubbles, double contaminants, double iodine_level, double pressure, double temperature);

    void remove_ammonia_and_organics(double &filtered_water, double &contaminants, double &organics, double &ammonia);
    void send_to_catalytic_reactor(double filtered_water, double remaining_contaminants, double organics);
    void handle_catalytic_reactor_response(rclcpp::Client<demo_nova_sanctum::srv::IonBed>::SharedFuture future);
};

#endif // MULTI_FILTRATION_PROCESSOR_HPP
