#ifndef DEMO_NOVA_SANCTUM_POTABLE_WATER_PUBLISHER_HPP_
#define DEMO_NOVA_SANCTUM_POTABLE_WATER_PUBLISHER_HPP_

#include "demo_nova_sanctum/msg/water.hpp"
#include <chrono>
#include <memory> 
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <bits/stdc++.h>

using namespace std; 

class Water : public rclcpp :: Node {
    public : 
            Water();
    private : 
            void water_publisher();
            void ogs_sys_trigger(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client);
            void open_valve();

            double water_level;
            double gas_bubbles; 
            double contaminants_level;

            rclcpp::Publisher<demo_nova_sanctum::msg::Water>::SharedPtr water_pub;
            rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr ogs_client;


            rclcpp::TimerBase::SharedPtr timer_;


};

#endif