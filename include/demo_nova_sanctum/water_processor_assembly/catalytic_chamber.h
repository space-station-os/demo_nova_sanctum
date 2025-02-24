#ifndef WPA_MULTI_FILTRATION_HPP
#define WPA_MULTI_FILTRATION_HPP

#include <rclcpp/rclcpp.hpp>
#include "demo_nova_sanctum/srv/filteration.hpp"
#include "demo_nova_sanctum/srv/ion_bed.hpp"

class MultiFiltrationProcessor : public rclcpp::Node {
public:
    MultiFiltrationProcessor();

private:
    rclcpp::Service<demo_nova_sanctum::srv::Filteration >::SharedPtr multi_filtration_service_;
    rclcpp::Client<demo_nova_sanctum::srv::IonBed>::SharedPtr catalytic_reactor_client_;

    void handle_multi_filtration(
        const std::shared_ptr<demo_nova_sanctum::srv::Filteration ::Request> request,
        std::shared_ptr<demo_nova_sanctum::srv::Filteration ::Response> response);

    void remove_ammonia_and_organics(double &filtered_water, double &contaminants, double &organics, double &ammonia);
    void simulate_catalytic_reactor(double &filtered_water, double &contaminants);
    void send_to_catalytic_reactor(double filtered_water, double remaining_contaminants);
};

#endif // WPA_MULTI_FILTRATION_HPP
