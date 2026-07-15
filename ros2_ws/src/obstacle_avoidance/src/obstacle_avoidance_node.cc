#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "volksface/msg/vel_gp.hpp"

#include "obstacle_avoidance/k-freespace.h"

class ObstacleAvoidanceNode : public rclcpp::Node {
public:
    ObstacleAvoidanceNode() : Node("obstacle_avoidance_node") {
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&ObstacleAvoidanceNode::laser_callback, this, std::placeholders::_1));

        // Publisher fuer Fahrbefehle
        cmd_pub_ = this->create_publisher<volksface::msg::VelGP>("vel_gp", 10);

        RCLCPP_INFO(this->get_logger(), "Obstacle Avoidance Node gestartet.");
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        int nr = scan->ranges.size();
        if (nr == 0) return;

        std::vector<int> distances(nr);
        std::vector<double> x(nr);
        std::vector<double> y(nr);

        double min_distance_front = scan->range_max;
        
        for (int i = 0; i < nr; ++i) {
            double r = scan->ranges[i];
            
            if (std::isnan(r) || r < scan->range_min || r > scan->range_max) {
                r = scan->range_max; 
            }

            double angle = scan->angle_min + i * scan->angle_increment;

            if (angle > -0.35 && angle < 0.35) {
                if (r < min_distance_front) {
                    min_distance_front = r;
                }
            }

            distances[i] = static_cast<int>(r * 1000.0);
            x[i] = r * std::cos(angle);
            y[i] = r * std::sin(angle);
        }

        // 1. Ausweichwinkel über k-freespace berechnen
        double steering_angle = calc_freespace(nr, distances.data(), x.data(), y.data());

        // 2. Bestimmung von linearer und angularer Wunschgeschwindigkeit
        double linear_vel = 0.0;
        double angular_vel = steering_angle * 0.6; // Drehung dämpfen

        double max_speed = 0.25;      // m/s
        double stop_distance = 0.4;   // Meter
        double slowdown_zone = 1.2;   // Meter

        if (min_distance_front <= stop_distance) {
            linear_vel = 0.0;
            RCLCPP_WARN(this->get_logger(), "NOTSTOPP! Hindernis extrem nah: %.2fm", min_distance_front);
        } 
        else if (min_distance_front < slowdown_zone) {
            double factor = (min_distance_front - stop_distance) / (slowdown_zone - stop_distance);
            linear_vel = max_speed * factor;
        } 
        else {
            linear_vel = max_speed;
        }

        // 3. Umrechnung in Radgeschwindigkeiten (Differentialantrieb)
        // Spurbreite des Volksbots (Abstand zwischen den Rädern in Metern)
        double track_width = 0.45; 

        auto msg = volksbot_msgs::msg::VelGP();
        msg.left  = linear_vel - (angular_vel * track_width / 2.0);
        msg.right = linear_vel + (angular_vel * track_width / 2.0);

        RCLCPP_INFO(this->get_logger(), "Motoren -> Links: %.2f | Rechts: %.2f (Distanz: %.2fm)", 
                    msg.left, msg.right, min_distance_front);

        // 4. Senden an den Volksbot
        cmd_pub_->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<volksface::msg::VelGP>::SharedPtr cmd_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleAvoidanceNode>());
    rclcpp::shutdown();
    return 0;
}