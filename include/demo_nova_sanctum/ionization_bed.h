#ifndef demo_nova_sanctum_IONIZATION_BED_HPP_
#define demo_nova_sanctum_IONIZATION_BED_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "demo_nova_sanctum/msg/water.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class IonizationBed : public rclcpp::Node {
    public:
        IonizationBed();
    private:
        void trigger_callback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                              const std_srvs::srv::Trigger::Response::SharedPtr response);
        void potable_water(const demo_nova_sanctum::msg::Water::SharedPtr msg);
        void contamination_removal();
        void deionization();
        void gas_sensor();
        void open_three_way_valve();

        void contamination_removal_pipeline();
        void publish_processed_water();
        bool is_active;
        double ionized_water_;
        bool removed_bubble_;
        double contaminated_water_; 
        double iodine_level_;


        double water_;
        double contaminants_;
        double gas_bubbles_;

        
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr server_;
        rclcpp::Subscription<demo_nova_sanctum::msg::Water>::SharedPtr water_subscriber_;

        rclcpp::Publisher<demo_nova_sanctum::msg::Water>::SharedPtr ionized_water_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
};

#endif // demo_nova_sanctum_IONIZATION_BED_HPP_
