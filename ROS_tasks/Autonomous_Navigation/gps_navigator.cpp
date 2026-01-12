#include <cmath>
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#define PI 3.1416
#define EARTH_RADIUS 6371000.0

class GPSNavigator : public rclcpp::Node
{
    public:
        GPSNavigator() : Node("gps_navigator")
        {
            // GPS Subsrciber
            gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                "/gps/fix", 10,
                std::bind(&GPSNavigator::gpsCallback, this, std::placeholders::_1));

            // IMU Subscriber
            imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                "/imu", 10,
                std::bind(&GPSNavigator::imuCallback, this, std::placeholders::_1));

            // Goal sodhwa maate
            goal_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
                "/set_gps_goal", 10,
                [this](const geometry_msgs::msg::Point::SharedPtr msg) {
                    tar_lat_ = msg->x;
                    tar_lon_ = msg->y;
                    mission_active_ = true;
                    phase_ = NavPhase::ALIGN_X;

                    RCLCPP_INFO(this->get_logger(),
                        "New Goal Received: Lat %.6f Lon %.6f",
                        tar_lat_, tar_lon_);
                });

            // Speed mokalwa
            cmd_vel_pub_ =
                this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

            // loop maate
            loop_runner_ = this->create_wall_timer(
                std::chrono::milliseconds(50),
                std::bind(&GPSNavigator::controlLoop, this));

            RCLCPP_INFO(this->get_logger(),
                "GPS Navigator Started (X then Y, BOTH SIGNS FIXED)");
        }

    private:

        
        enum class NavPhase {
            ALIGN_X,
            MOVE_X,
            ALIGN_Y,
            MOVE_Y,
            DONE
        };

        NavPhase phase_ = NavPhase::ALIGN_X;

        // GPS na signal nu update 
        void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
            cur_lat_ = msg->latitude;
            cur_lon_ = msg->longitude;
            gps_ready_ = true;
        }

        // IMU na signal nu update
        void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
            tf2::Quaternion q(
                msg->orientation.x,
                msg->orientation.y,
                msg->orientation.z,
                msg->orientation.w);

            tf2::Matrix3x3 m(q);
            double roll, pitch;
            m.getRPY(roll, pitch, cur_yaw_);

            imu_ready_ = true;
        }

        // Control Unit
        void controlLoop() {
            if (!mission_active_ || !gps_ready_ || !imu_ready_) return;

            double dLat = (tar_lat_ - cur_lat_) * PI / 180.0;
            double dLon = (tar_lon_ - cur_lon_) * PI / 180.0;
            double lat  = cur_lat_ * PI / 180.0;
            double x_dif = -EARTH_RADIUS * dLon * cos(lat);
            double y_dif = -EARTH_RADIUS * dLat;

            geometry_msgs::msg::Twist cmd;

            auto normalize = [](double a) {
                while (a > PI) a -= 2 * PI;
                while (a < -PI) a += 2 * PI;
                return a;
            };

            switch (phase_) {
            
            // Aligning X
            case NavPhase::ALIGN_X: {
                double target_yaw = (x_dif >= 0) ? 0: PI;
                double yaw_dif = normalize(target_yaw - cur_yaw_);

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "yaw difference = %.2f", yaw_dif);

                if (std::abs(yaw_dif) < 0.1) {
                    phase_ = NavPhase::MOVE_X;
                    RCLCPP_INFO(this->get_logger(), "Aligned to X-axis");
                } else {
                    cmd.angular.z = (yaw_dif > 0) ? 0.5 : -0.5;
                }
                break;
            }

            // Moving in X
            case NavPhase::MOVE_X: {
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "x difference = %.2f", x_dif);

                if (std::abs(x_dif) < 0.3) {
                    phase_ = NavPhase::ALIGN_Y;
                    RCLCPP_INFO(this->get_logger(), "X reached → Aligning Y");
                } else {
                    if (std::abs(x_dif) > 1.0){
                        cmd.linear.x = 0.5;
                    }
                    else{
                        cmd.linear.x = 0.2;
                    }
                }
                break;
            }

            // Aligning Y
            case NavPhase::ALIGN_Y: {
                double target_yaw = (y_dif >= 0) ? PI / 2 : -PI / 2;
                double yaw_dif = normalize(target_yaw - cur_yaw_);

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "yaw difference = %.2f",yaw_dif);

                if (std::abs(yaw_dif) < 0.1) {
                    phase_ = NavPhase::MOVE_Y;
                    RCLCPP_INFO(this->get_logger(), "Aligned to Y-axis");
                } else {
                    cmd.angular.z = (yaw_dif > 0) ? 0.5 : -0.5;
                }
                break;
            }

            // Moving in X
            case NavPhase::MOVE_Y: {
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "y difference = %.2f", y_dif);

                if (std::abs(y_dif) < 0.3) {
                    phase_ = NavPhase::DONE;
                } else {
                    if (std::abs(y_dif) > 1.0){
                        cmd.linear.x = 0.5;
                    }
                    else{
                        cmd.linear.x = 0.2;
                    }
                }
                break;
            }

            // Reached
            case NavPhase::DONE:
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                mission_active_ = false;
                RCLCPP_INFO(this->get_logger(),
                    "GOAL REACHED (X then Y)");
                break;
            }

            cmd_vel_pub_->publish(cmd);
        }

        // Required Subscribers and Publishers

        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::TimerBase::SharedPtr loop_runner_;

        double cur_lat_ = 0.0;
        double cur_lon_ = 0.0;
        double cur_yaw_ = 0.0;
        double tar_lat_ = 0.0;
        double tar_lon_ = 0.0;

        bool gps_ready_ = false;
        bool imu_ready_ = false;
        bool mission_active_ = false;
};

/* ===================== MAIN ===================== */

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSNavigator>());
    rclcpp::shutdown();
    return 0;
}
