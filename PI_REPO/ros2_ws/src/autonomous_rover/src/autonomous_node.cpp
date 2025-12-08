#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <iostream>

class Autopilot : public rclcpp::Node {
public:
    Autopilot() : Node("auto_publisher") {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel_auto", 10);

        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Autopilot::lidarCallback, this, std::placeholders::_1));
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&Autopilot::publish_cmd, this));
    }

private:
    float front = std::numeric_limits<float>::infinity();
    float front_left = std::numeric_limits<float>::infinity();
    float left = std::numeric_limits<float>::infinity();
    float back_left = std::numeric_limits<float>::infinity();
    float back = std::numeric_limits<float>::infinity();
    float back_right = std::numeric_limits<float>::infinity();
    float right = std::numeric_limits<float>::infinity();
    float front_right = std::numeric_limits<float>::infinity();
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (msg->ranges.empty())
            return;

        const size_t n = msg->ranges.size();
        const size_t sector = n / 8;

        auto min_range = [&](size_t start, size_t end) {
            auto begin = msg->ranges.begin() + start;
            auto finish = msg->ranges.begin() + end;
            auto it = std::min_element(begin, finish, [](float a, float b) {
                if (!std::isfinite(a)) return false;
                if (!std::isfinite(b)) return true;
                return a < b;
            });
            return (it != finish) ? *it : std::numeric_limits<float>::infinity();
        };

        front       = std::min(min_range(0, sector/2), min_range(7*sector + sector/2, n)); // wrap-around
        front_left  = min_range(sector, 2 * sector);
        left        = min_range(2 * sector, 3 * sector);
        back_left   = min_range(3 * sector, 4 * sector);
        back        = min_range(4 * sector, 5 * sector);
        back_right  = min_range(5 * sector, 6 * sector);
        right       = min_range(6 * sector, 7 * sector);
        front_right = min_range(7 * sector, n);
            
    }
    
void publish_cmd()
{
    geometry_msgs::msg::Twist msg;

    const float front_limit = 0.35;
    const float side_limit  = 0.22;
    const float diagonal_limit = 0.35;

    bool front_blocked = (front < front_limit);
    bool left_blocked  = (left  < side_limit);
    bool right_blocked = (right < side_limit);
    bool sides_blocked = (left < 0.40 && right < 0.40);

    
    if (!front_blocked && sides_blocked) 
    {
        std::cout << "FRI VEI RETT FREM MED BLOKKERTE SIDER" << std::endl;
        msg.linear.x = 1.0;   // fremover
        msg.linear.y = 0.0;   // ingen strafe
        msg.angular.z = 0.0;  // ingen rotasjon
    }
    
    // klar bane framover og på sidene
    else if (!front_blocked && !sides_blocked) 
    {
        msg.linear.x = 1.0;   // fremover
        msg.linear.y = 0.0;   // ingen strafe
        msg.angular.z = 0.0;  // ingen rotasjon
        if(!left_blocked && right_blocked) {
            // Begge sider blokkert, ingen strafe
            msg.linear.y = 1.0;
            msg.linear.x = 0.0;
        }
        else if (left_blocked && !right_blocked) {
            msg.linear.y = -1.0;   // strafe RIGHT (y positiv er avhengig av din motor logikk)
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;

        }

        
    }


    // hindring foran, roter mot mer åpen side
    else 
    {
        // Snu mot mer åpen side
        if (front_left > front_right)
            msg.angular.z = 1.0;    // venstre
        else
            msg.angular.z = -1.0;   // høyre

        msg.linear.x = 0.0;
        msg.linear.y = 0.0;
    }
    pub_->publish(msg);
}
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Autopilot>());
    rclcpp::shutdown();
    return 0;
}