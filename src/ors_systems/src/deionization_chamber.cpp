#include "demo_nova_sanctum/ionization_bed.h"
#include <chrono>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

IonizationBed::IonizationBed()
    : Node("ionization_bed"),
      ionized_water_(0.0),
      removed_bubble_(false),
      contaminated_water_(0.0),
      is_active(false),
      water_(0.0),
      contaminants_(0.0),
      gas_bubbles_(0.0),
      iodine_level_(0.0) {

    // Service to activate the ionization bed
    server_ = this->create_service<std_srvs::srv::Trigger>(
        "/ionization_bed",
        std::bind(&IonizationBed::trigger_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Subscriber to potable water topic
    water_subscriber_ = this->create_subscription<demo_nova_sanctum::msg::Water>(
        "/potable_water", 10, std::bind(&IonizationBed::potable_water, this, std::placeholders::_1));

    // Publisher to ionized water topic
    ionized_water_pub_ = this->create_publisher<demo_nova_sanctum::msg::Water>("/ionized_water", 10);
}

void IonizationBed::potable_water(const demo_nova_sanctum::msg::Water::SharedPtr msg) {
    water_ = msg->water_level;
    contaminants_ = msg->contaminants_level;
    gas_bubbles_ = msg->gas_bubbles;
    iodine_level_ = msg->iodine_level;  // Assume iodine level is part of the input message
}

void IonizationBed::deionization() {
    if (water_ >= 10.0) {  // Maximum capacity of the chamber (adjust as needed)
        double iodine_removal_rate = 0.5;  // Rate of iodine removal per iteration
        iodine_level_ -= iodine_removal_rate;

    if (iodine_level_ < 0.01) iodine_level_ = 0.0;  // Ensure it reaches a minimum safe threshold

        RCLCPP_INFO(this->get_logger(), "Deionization complete. Remaining iodine: %.2f", iodine_level_);
    }
}

void IonizationBed::contamination_removal() {
    double contaminants_decrement = 2.0;  // Rate of contaminant removal
    contaminants_ -= contaminants_decrement;
    if (contaminants_ < 0.0) contaminants_ = 0.0;

    RCLCPP_INFO(this->get_logger(), "Contamination removal complete. Remaining contaminants: %.2f", contaminants_);
}

void IonizationBed::gas_sensor() {
    if (gas_bubbles_ > 0.0) {
        RCLCPP_WARN(this->get_logger(), "Gas bubbles detected: %.2f. Redirecting via three-way valve.", gas_bubbles_);
        open_three_way_valve();
    } else {
        RCLCPP_INFO(this->get_logger(), "No gas bubbles detected.");
    }
}

void IonizationBed::open_three_way_valve() {
    RCLCPP_INFO(this->get_logger(), "Three-way valve opened. Redirecting water with gas bubbles.");
    gas_bubbles_ = 0.0;  // Assume all bubbles are removed after redirection
}

void IonizationBed::contamination_removal_pipeline() {
    if (!is_active) {
        RCLCPP_WARN(this->get_logger(), "Ionization bed is inactive. No water is being processed.");
        return;
    }

    if (water_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "No water available to process.");
        return;
    }

    // Process through the stages
    deionization();
    if (iodine_level_ <= 0.01) {
        contamination_removal();
    }

    if (contaminants_ <= 0.0) {
        gas_sensor();
    }

    if (iodine_level_ == 0.0 && contaminants_ == 0.0 && gas_bubbles_ == 0.0) {
        publish_processed_water();
    }
}

void IonizationBed::publish_processed_water() {
    auto message = demo_nova_sanctum::msg::Water();
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "ionization_bed";

    message.water_level = water_;
    message.contaminants_level = contaminants_;
    message.gas_bubbles = gas_bubbles_;
    message.iodine_level = iodine_level_;
    message.pressure = 14.0;  // Simulated pressure
    message.temperature = 25.0;  // Simulated temperature

    ionized_water_pub_->publish(message);

    RCLCPP_INFO(this->get_logger(), "Processed water published with 0 contaminants and iodine.");
    RCLCPP_INFO(this->get_logger(), "Ionization bed deactivated.");
    RCLCPP_INFO(this->get_logger(), "Water level remaining: %.2f", water_);
}

// Service callback to activate the ionization bed
void IonizationBed::trigger_callback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                     std_srvs::srv::Trigger::Response::SharedPtr response) {
    if (!is_active) {
        is_active = true;
        response->success = true;
        response->message = "Ionization bed activated successfully.";
        RCLCPP_INFO(this->get_logger(), "Ionization bed activated.");

        timer_ = this->create_wall_timer(1s, std::bind(&IonizationBed::contamination_removal_pipeline, this));
    } else {
        response->success = false;
        response->message = "Ionization bed is already active.";
        RCLCPP_WARN(this->get_logger(), "Ionization bed is already active.");
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IonizationBed>());
    rclcpp::shutdown();
    return 0;
}
