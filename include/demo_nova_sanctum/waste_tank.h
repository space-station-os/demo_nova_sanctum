#ifndef WHC_WASTE_TANK_HPP
#define WHC_WASTE_TANK_HPP

#include <rclcpp/rclcpp.hpp>
#include "demo_nova_sanctum/srv/upa.hpp"

class WHCWasteTank : public rclcpp::Node {
public:
    WHCWasteTank();

private:
    rclcpp::Client<demo_nova_sanctum::srv::Upa>::SharedPtr upa_client_;
    rclcpp::TimerBase::SharedPtr retry_timer_;
    rclcpp::TimerBase::SharedPtr urine_collection_timer_;

    double urine_volume_;
    double pretreatment_volume_;
    double flush_volume_;
    double total_water_volume_;
    double tank_capacity_;
    double processing_threshold_;
    bool upa_available_;

    void simulate_urine_collection();
    void process_waste_transfer();
    void process_urine_response(rclcpp::Client<demo_nova_sanctum::srv::Upa>::SharedFuture future);
    void retry_process_waste_transfer();
};

#endif // WHC_WASTE_TANK_HPP
