#ifndef WPA_CATALYTIC_REACTOR_HPP
#define WPA_CATALYTIC_REACTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include "demo_nova_sanctum/srv/ion_bed.hpp"
#include "demo_nova_sanctum/srv/water.hpp"

class CatalyticReactorProcessor : public rclcpp::Node {
public:
    CatalyticReactorProcessor();

private:
    rclcpp::Service<demo_nova_sanctum::srv::IonBed>::SharedPtr catalytic_reactor_service_;
    rclcpp::Client<demo_nova_sanctum::srv::Water>::SharedPtr product_water_tank_client_;

    void handle_catalytic_reactor(
        const std::shared_ptr<demo_nova_sanctum::srv::IonBed::Request> request,
        std::shared_ptr<demo_nova_sanctum::srv::IonBed::Response> response);

    void process_ion_exchange(double &filtered_water, double &contaminants, double &iodine_level, double &gas_bubbles);
    void send_to_product_water_tank(double water, double gas_bubbles, double contaminants, double iodine_level, double pressure, double temperature);
};

#endif // WPA_CATALYTIC_REACTOR_HPP
