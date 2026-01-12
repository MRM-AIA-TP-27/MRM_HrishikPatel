#include <cmath>
#include <memory>
#include <chrono>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#define PI 3.141592653589793
#define EARTH_RADIUS 6371000.0

class GPSNavigator : public rclcpp::Node
{
    public:
        GPSNavigator() : Node("gps_navigator")
        {
            // GPS Subscriber
            gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
                "/gps/fix", 10,
                std::bind(&GPSNavigator::gpsCallback, this, std::placeholders::_1));

            // IMU Subscriber
            imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
                "/imu", 10,
                std::bind(&GPSNavigator::imuCallback, this, std::placeholders::_1));

            //Goal sodhwa maate
            goal_sub_ = create_subscription<geometry_msgs::msg::Point>(
                "/set_gps_goal", 10,
                [this](geometry_msgs::msg::Point::SharedPtr msg) {
                    tar_lat_ = msg->x;
                    tar_lon_ = msg->y;
                    mission_active_ = true;
                    phase_ = NavPhase::ROTATE_START;
                    final_rotate_done_ = false;

                    RCLCPP_INFO(this->get_logger(),
                        "New Goal Received: Lat %.6f Lon %.6f",
                        tar_lat_, tar_lon_);
                });

            // speed mokalwa
            cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

            // loop maate
            loop_runner_ = create_wall_timer(
                std::chrono::milliseconds(50),
                std::bind(&GPSNavigator::controlLoop, this));

            RCLCPP_INFO(this->get_logger(), "GPS Navigator started");
        }

    private:

        enum class NavPhase {
            ROTATE_START,
            MOVE,
            ROTATE_NEAR,
            DONE
        };

        NavPhase phase_ = NavPhase::ROTATE_START;
        bool final_rotate_done_ = false;

        // GPS na signal nu update 
        void gpsCallback(sensor_msgs::msg::NavSatFix::SharedPtr msg) {
            cur_lat_ = msg->latitude;
            cur_lon_ = msg->longitude;
            gps_ready_ = true;
        }

        void imuCallback(sensor_msgs::msg::Imu::SharedPtr msg) {
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
        void controlLoop()
        {
            if (!mission_active_ || !gps_ready_ || !imu_ready_) return;

            double dLat = (tar_lat_ - cur_lat_) * PI / 180.0;
            double dLon = (tar_lon_ - cur_lon_) * PI / 180.0;
            double lat  = cur_lat_ * PI / 180.0;
            double x = -EARTH_RADIUS * dLon * cos(lat);
            double y = -EARTH_RADIUS * dLat;

            geometry_msgs::msg::Twist cmd;
            double dist = std::sqrt(x*x + y*y);
            double target_yaw = std::atan2(y, x);

            auto normalize = [](double a) {
                while (a > PI) a -= 2*PI;
                while (a < -PI) a += 2*PI;
                return a;
            };

            double yaw_err = normalize(target_yaw - cur_yaw_);

            const double YAW_TOL = 0.05;
            const double ROTATE_DIST = 2.0;
            const double STOP_DIST = 0.3;
            const double HEADING_KP = 0.8;
            const double MAX_YAW_RATE = 0.3;

            // Stop maate
            if (dist < STOP_DIST) {
                RCLCPP_INFO(this->get_logger(),
                    "[STATE: DONE] Goal reached (dist=%.2f m)", dist);
                mission_active_ = false;
                phase_ = NavPhase::DONE;
                cmd_vel_pub_->publish(cmd);
                return;
            }

            switch (phase_) {

                // Aligning
                case NavPhase::ROTATE_START:
                    RCLCPP_INFO_THROTTLE(
                        get_logger(), *get_clock(), 1000,
                        "[STATE: ROTATE_START] yaw_err=%.3f", yaw_err);

                    if (std::abs(yaw_err) < YAW_TOL) {
                        phase_ = NavPhase::MOVE;
                    } else {
                        cmd.angular.z = (yaw_err > 0) ? 0.4 : -0.4;
                    }
                    break;

                // Moving
                case NavPhase::MOVE: {
                    double v = std::min(0.5, std::max(0.15, 0.6 * dist));
                    double w = std::clamp(HEADING_KP * yaw_err,
                                        -MAX_YAW_RATE, MAX_YAW_RATE);

                    RCLCPP_INFO_THROTTLE(
                        get_logger(), *get_clock(), 1000,
                        "[STATE: MOVE] dist=%.2f yaw_err=%.3f v=%.2f w=%.2f",
                        dist, yaw_err, v, w);

                    if (dist < ROTATE_DIST && !final_rotate_done_) {
                        phase_ = NavPhase::ROTATE_NEAR;
                    } else {
                        cmd.linear.x = v;
                        cmd.angular.z = w;
                    }
                    break;
                }

                // Aligning when near
                case NavPhase::ROTATE_NEAR:
                    RCLCPP_INFO_THROTTLE(
                        get_logger(), *get_clock(), 1000,
                        "[STATE: ROTATE_NEAR] dist=%.2f yaw_err=%.3f",
                        dist, yaw_err);

                    if (std::abs(yaw_err) < YAW_TOL) {
                        final_rotate_done_ = true;
                        phase_ = NavPhase::MOVE;
                    } else {
                        cmd.angular.z = (yaw_err > 0) ? 0.3 : -0.3;
                    }
                    break;

                case NavPhase::DONE:
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

        double cur_lat_ = 0.0, cur_lon_ = 0.0, cur_yaw_ = 0.0;
        double tar_lat_ = 0.0, tar_lon_ = 0.0;

        bool gps_ready_ = false;
        bool imu_ready_ = false;
        bool mission_active_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSNavigator>());
    rclcpp::shutdown();
    return 0;
}
