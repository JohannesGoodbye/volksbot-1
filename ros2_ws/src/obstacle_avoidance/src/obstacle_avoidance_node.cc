#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "obstacle_avoidance/k-freespace.h"

class ObstacleAvoidanceNode : public rclcpp::Node {
public:
    ObstacleAvoidanceNode() : Node("obstacle_avoidance_node") {
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&ObstacleAvoidanceNode::laser_callback, this, std::placeholders::_1));

        // Publisher fuer Fahrbefehle (z.B. "cmd_vel" fuer den Volksbot)
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Obstacle Avoidance Node gestartet.");
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        int nr = scan->ranges.size();
        if (nr == 0) return;

        std::vector<int> distances(nr);
        std::vector<double> x(nr);
        std::vector<double> y(nr);

        for (int i = 0; i < nr; ++i) {
            double r = scan->ranges[i];
            
            // Ungueltige Messwerte herausfiltern (z.B. unendlich weit weg oder zu nah)
            if (std::isnan(r) || r < scan->range_min || r > scan->range_max) {
                r = scan->range_max; 
            }

            // In Millimeter konvertieren (weil calc_freespace mit mm arbeitet)
            distances[i] = static_cast<int>(r * 1000.0);

            // Winkel des Laserstrahls berechnen
            double angle = scan->angle_min + i * scan->angle_increment;

            // In kartesische Koordinaten (relativ zum Roboter) umwandeln
            x[i] = r * std::cos(angle);
            y[i] = r * std::sin(angle);
        }

        // C-Algorithmus aufrufen
        double steering_angle = calc_freespace(nr, distances.data(), x.data(), y.data());

        /*
        RCLCPP_INFO(this->get_logger(), "Berechneter Freiraum-Ausweichwinkel: %.2f Grad", steering_angle * 57.29578);

        // Steuerbefehl erzeugen
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 0.2; // Fahre langsam vorwaerts
        msg.angular.z = steering_angle * 0.5; // Lenke sanft in Richtung des Freiraums
        cmd_pub_->publish(msg);
        */

        // Erstellen der Twist-Nachricht fuer die Robotersteuerung
        auto msg = geometry_msgs::msg::Twist();

        // Parameter fuer die Geschwindigkeitsregelung
        double max_speed = 0.25;      // Maximale Vorwaertsgeschwindigkeit in m/s
        double stop_distance = 0.4;   // Sicherheitsabstand: Halt bei 40 cm
        double slowdown_zone = 1.2;   // Bremsbereich: Ab 1,20 m wird der Roboter langsamer

        // Logik fuer Bremsen und Stoppen (Kollisionsschutz)
        if (min_distance_front <= stop_distance) {
            // Zu nah! Roboter stoppt die Vorwaertsbewegung, darf sich aber drehen, um wegzukommen
            msg.linear.x = 0.0;
            RCLCPP_WARN(this->get_logger(), "NOTSTOPP! Hindernis extrem nah: %.2fm", min_distance_front);
        } 
        else if (min_distance_front < slowdown_zone) {
            // Linearer Bremsfaktor zwischen 0.0 (bei stop_distance) und 1.0 (bei slowdown_zone)
            double factor = (min_distance_front - stop_distance) / (slowdown_zone - stop_distance);
            msg.linear.x = max_speed * factor;
            RCLCPP_INFO(this->get_logger(), "Bremse aktiv. Geschwindigkeit verlangsamt auf: %.2f m/s", msg.linear.x);
        } 
        else {
            // Bahn ist frei, fahre mit maximaler Testgeschwindigkeit
            msg.linear.x = max_speed;
        }

        // Lenkung anwenden (Multiplikation mit Daempfungsfaktor 0.6 fuer sanftere Kurven)
        msg.angular.z = steering_angle * 0.6; 

        RCLCPP_INFO(this->get_logger(), "Lenkwinkel: %.1f Grad | Distanz vor dem Roboter: %.2fm", 
                    steering_angle * 57.29578, min_distance_front);

        // Fahrbefehl auf das "cmd_vel" Topic publizieren
        cmd_pub_->publish(msg);   
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleAvoidanceNode>());
    rclcpp::shutdown();
    return 0;
}