#include "rclcpp/rclcpp.hpp"
#include "demo_nova_sanctum/msg/cdra_status.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <unordered_map>

enum class CO2State {
    IDLE,               // S₀
    AIR_COLLECTION,     // S₁
    MOISTURE_REMOVAL,   // S₂
    CO2_ADSORPTION_1,   // S₃
    HUMIDIFICATION,     // S₄
    CO2_TRANSFER,       // S₅
    CO2_DESORPTION,     // S₆
    CO2_TO_SPACE,       // S₇A
    CO2_TO_SABATIER     // S₇B
};

class CO2TransitionSystem : public rclcpp::Node {
public:
    CO2TransitionSystem() : Node("co2_transition_system"), current_state_(CO2State::IDLE) {
        subscription_ = this->create_subscription<demo_nova_sanctum::msg::CdraStatus>(
            "/cdra_status", 10, std::bind(&CO2TransitionSystem::status_callback, this, std::placeholders::_1));
        
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/transition_graph", 10);
    }

private:
    CO2State current_state_;
    rclcpp::Subscription<demo_nova_sanctum::msg::CdraStatus>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

    void status_callback(const demo_nova_sanctum::msg::CdraStatus::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received CO₂ Processing Status: State %d", msg->co2_processing_state);

        static std::unordered_map<int, CO2State> state_mapping = {
            {0, CO2State::IDLE}, {1, CO2State::AIR_COLLECTION}, {2, CO2State::MOISTURE_REMOVAL},
            {3, CO2State::CO2_ADSORPTION_1}, {4, CO2State::HUMIDIFICATION}, {5, CO2State::CO2_TRANSFER},
            {6, CO2State::CO2_DESORPTION}, {7, CO2State::CO2_TO_SPACE}, {8, CO2State::CO2_TO_SABATIER}
        };

        if (state_mapping.find(msg->co2_processing_state) != state_mapping.end()) {
            current_state_ = state_mapping[msg->co2_processing_state];
            publish_graph();
        }
    }

    void publish_graph() {
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker edge_marker;

        edge_marker.header.frame_id = "map";
        edge_marker.header.stamp = this->now();
        edge_marker.ns = "co2_graph";
        edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        edge_marker.action = visualization_msgs::msg::Marker::ADD;
        edge_marker.scale.x = 0.05; // Line width
        edge_marker.color.a = 1.0;
        edge_marker.color.r = 0.0;
        edge_marker.color.g = 1.0;
        edge_marker.color.b = 0.0;

        static std::unordered_map<CO2State, std::pair<double, double>> state_positions = {
            {CO2State::IDLE, {0.0, 0.0}}, {CO2State::AIR_COLLECTION, {1.0, 1.0}},
            {CO2State::MOISTURE_REMOVAL, {2.0, 1.0}}, {CO2State::CO2_ADSORPTION_1, {3.0, 1.0}},
            {CO2State::HUMIDIFICATION, {4.0, 1.0}}, {CO2State::CO2_TRANSFER, {5.0, 1.0}},
            {CO2State::CO2_DESORPTION, {6.0, 1.0}}, {CO2State::CO2_TO_SPACE, {7.0, 2.0}},
            {CO2State::CO2_TO_SABATIER, {7.0, 0.0}}
        };

        for (auto it = state_positions.begin(); it != state_positions.end(); ++it) {
            if (std::next(it) != state_positions.end()) {
                geometry_msgs::msg::Point p1, p2;
                p1.x = it->second.first;
                p1.y = it->second.second;
                p2.x = std::next(it)->second.first;
                p2.y = std::next(it)->second.second;
                edge_marker.points.push_back(p1);
                edge_marker.points.push_back(p2);
            }
        }

        marker_array.markers.push_back(edge_marker);
        marker_publisher_->publish(marker_array);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CO2TransitionSystem>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}