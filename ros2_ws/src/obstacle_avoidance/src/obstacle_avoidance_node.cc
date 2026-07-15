#include <memory>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "volksface/msg/vel_gp.hpp"
#include "volksface/msg/rover.hpp"
#include "volksface/volksbot.h"

#include "obstacle_avoidance/k-freespace.h"

class ObstacleAvoidanceNode : public rclcpp::Node {
public:
    ObstacleAvoidanceNode() : Node("obstacle_avoidance_node") {
        // Standard-Parameter deklarieren (Fallback, falls kein Rover-Topic empfangen wird)
        this->declare_parameter<double>("default_track_width", 0.50);
        track_width_ = this->get_parameter("default_track_width").as_double();
        rover_detected_ = false;
        // Subscriber für die Rover-Konfiguration (Topic-Name analog zur Odometrie)
        rover_sub_ = this->create_subscription<VB::msg::Rover>(VB::TOPIC_NAME_ROVER, 10, std::bind(&ObstacleAvoidanceNode::rover_callback, this, std::placeholders::_1));
        // Laser-Subscriber
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&ObstacleAvoidanceNode::laser_callback, this, std::placeholders::_1));
        // Publisher fuer Fahrbefehle
        cmd_pub_ = this->create_publisher<volksface::msg::VelGP>(VB::TOPIC_NAME_VEL_GP, 10);

        RCLCPP_INFO(this->get_logger(), "Obstacle Avoidance Node gestartet.");
    }

private:
    // Callback für die automatische Erkennung der Roboterdaten
    void rover_callback(const volksbot_msgs::msg::Rover::SharedPtr rover) {
        if (rover->is_valid) {
            // wheel_base kommt in cm aus der YAML, wir brauchen Meter für die Kinematik
            track_width_ = rover->wheel_base / 100.0; 
            
            if (!rover_detected_) {
                RCLCPP_INFO(this->get_logger(), 
                            "Rover erkannt: '%s' | Spurbreite auf %.2fm gesetzt.", 
                            rover->name.c_str(), track_width_);
                rover_detected_ = true;
            }
        }
    }

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        if (!rover_detected_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                "Noch keine Rover-Konfiguration empfangen. Node inaktiv.");
            return;
        }

        int nr = scan->ranges.size();
        if (nr == 0) return;

        std::vector<int> filtered_distances;
        std::vector<double> filtered_x;
        std::vector<double> filtered_y;

        double min_distance_front = scan->range_max;
        
        // 135 Grad in Bogenmaß (die Mitte des 270°-Scanners)
        const double angle_straight_ahead = 135.0 * M_PI / 180.0; 

        for (int i = 0; i < nr; ++i) {
            double r = scan->ranges[i];
            
            if (std::isnan(r) || r < scan->range_min || r > scan->range_max) {
                r = scan->range_max; 
            }

            // Der rohe Winkel aus dem ROS-Topic (0 bis 270 Grad)
            double raw_angle = scan->angle_min + i * scan->angle_increment;

            // --- TRANSFORMATION AUF ROBOTER-KOORDINATEN ---
            // Wir verschieben den Winkel so, dass 135° (geradeaus) zu 0° wird.
            // Wenn der Scanner im Uhrzeigersinn dreht, ziehen wir den Winkel ab.
            double robot_angle = angle_straight_ahead - raw_angle;

            // --- 180° FILTER (Alles hinter der Achse ignorieren) ---
            // Wir betrachten nur den Bereich von -90° (rechts) bis +90° (links)
            if (robot_angle < -M_PI / 2.0 || robot_angle > M_PI / 2.0) {
                continue; 
            }

            // Sicherheits-Check direkt vor dem Roboter (-20° bis +20° im Roboter-Frame)
            if (robot_angle > -0.35 && robot_angle < 0.35) {
                if (r < min_distance_front) {
                    min_distance_front = r;
                }
            }

            // Daten für den k-freespace Algorithmus speichern
            filtered_distances.push_back(static_cast<int>(r * 1000.0));
            
            // Kartesische Koordinaten berechnen (jetzt korrekt an der Fahrtrichtung ausgerichtet!)
            filtered_x.push_back(r * std::cos(robot_angle));
            filtered_y.push_back(r * std::sin(robot_angle));
        }

        int num_filtered_points = filtered_distances.size();
        if (num_filtered_points == 0) return;

        // 1. Ausweichwinkel mit den korrekt ausgerichteten Daten berechnen
        double steering_angle = calc_freespace(num_filtered_points, 
                                            filtered_distances.data(), 
                                            filtered_x.data(), 
                                            filtered_y.data());

        // 2. Wunschgeschwindigkeiten bestimmen
        double linear_vel = 0.0;
        double angular_vel = steering_angle * 0.6; 

        double max_speed = 0.25;      
        double stop_distance = 0.4;   
        double slowdown_zone = 1.2;   

        if (min_distance_front <= stop_distance) {
            linear_vel = 0.0;
            RCLCPP_WARN(this->get_logger(), "NOTSTOPP! Hindernis im Weg: %.2fm", min_distance_front);
        } 
        else if (min_distance_front < slowdown_zone) {
            double factor = (min_distance_front - stop_distance) / (slowdown_zone - stop_distance);
            linear_vel = max_speed * factor;
        } 
        else {
            linear_vel = max_speed;
        }

        // 3. Umrechnung in Radgeschwindigkeiten
        auto msg = volksface::msg::VelGP();
        msg.left  = linear_vel - (angular_vel * track_width_ / 2.0);
        msg.right = linear_vel + (angular_vel * track_width_ / 2.0);

        // 4. Senden an den Volksbot
        cmd_pub_->publish(msg);
}


    rclcpp::Subscription<VB::msg::Rover>::SharedPtr rover_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<volksface::msg::VelGP>::SharedPtr cmd_pub_;

    double track_width_;
    bool rover_detected_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleAvoidanceNode>());
    rclcpp::shutdown();
    return 0;
}