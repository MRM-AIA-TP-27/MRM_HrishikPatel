#include <cmath>
#include <memory>
#include <chrono>
#include <vector>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#define PI 3.141592653589793
#define EARTH_RADIUS 6371000.0

class ReactiveGPSNavigator : public rclcpp::Node
{
public:
    ReactiveGPSNavigator() : Node("reactive_gps_nav")
    {
        // Subscribers
        gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", 10, std::bind(&ReactiveGPSNavigator::gpsCallback, this, std::placeholders::_1));

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&ReactiveGPSNavigator::imuCallback, this, std::placeholders::_1));

        pcl_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/obstacles", 10, std::bind(&ReactiveGPSNavigator::pclCallback, this, std::placeholders::_1));

        goal_sub_ = create_subscription<geometry_msgs::msg::Point>(
            "/set_gps_goal", 10, [this](geometry_msgs::msg::Point::SharedPtr msg) {
                tar_lat_ = msg->x;
                tar_lon_ = msg->y;
                mission_active_ = true;
                current_wp_idx_ = 0;
                RCLCPP_INFO(this->get_logger(), "New goal set via topic: Lat %.6f Lon %.6f", tar_lat_, tar_lon_);
            });

        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Load parameters
        declare_parameters();

        // Timer
        loop_runner_ = create_wall_timer(std::chrono::milliseconds(50),
                                         std::bind(&ReactiveGPSNavigator::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Reactive GPS Navigator started");
    }

private:
    enum class NavPhase { ROTATE_START, MOVE, ROTATE_NEAR, DONE };

    // --- Parameters ---
    std::vector<double> wp_latitudes_;
    std::vector<double> wp_longitudes_;
    double waypoint_tolerance_;
    double min_linear_speed_;
    double max_linear_speed_;
    double max_angular_speed_;
    double align_threshold_;
    size_t current_wp_idx_ = 0;

    // --- State ---
    NavPhase phase_ = NavPhase::ROTATE_START;
    bool final_rotate_done_ = false;
    double cur_lat_ = 0.0, cur_lon_ = 0.0, cur_yaw_ = 0.0;
    double tar_lat_ = 0.0, tar_lon_ = 0.0;
    bool gps_ready_ = false, imu_ready_ = false, mission_active_ = false;
    double start_lat_ = 0.0;
    double start_lon_ = 0.0;
    bool start_pos_recorded_ = false;
    bool initial_loop_completed_ = false;
    // --- Obstacle ---
    bool obstacle_detected_ = false;
    double obs_linear_x_ = 0.0;
    double obs_angular_z_ = 0.0;

    // --- Subscribers / Publishers ---
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr loop_runner_;

    // --- Parameter Loading ---
    void declare_parameters()
    {
        // Explicitly declare parameters with default values
        this->declare_parameter<std::vector<double>>("latitudes", std::vector<double>{});
        this->declare_parameter<std::vector<double>>("longitudes", std::vector<double>{});
        this->declare_parameter<double>("waypoint_tolerance", 0.00005);
        this->declare_parameter<double>("min_linear_speed", 0.15);
        this->declare_parameter<double>("max_linear_speed", 1.2);
        this->declare_parameter<double>("max_angular_speed", 1.5);
        this->declare_parameter<double>("align_threshold", 0.04);

        // Now retrieve them
        wp_latitudes_ = this->get_parameter("latitudes").as_double_array();
        wp_longitudes_ = this->get_parameter("longitudes").as_double_array();
        waypoint_tolerance_ = this->get_parameter("waypoint_tolerance").as_double();
        min_linear_speed_ = this->get_parameter("min_linear_speed").as_double();
        max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
        align_threshold_ = this->get_parameter("align_threshold").as_double();

        RCLCPP_INFO(this->get_logger(),
            "Loaded %zu waypoints, tolerance %.8f, min/max linear %.2f/%.2f, max angular %.2f, align threshold %.2f",
            wp_latitudes_.size(), waypoint_tolerance_, min_linear_speed_, max_linear_speed_, max_angular_speed_, align_threshold_);
        for (size_t i = 0; i < wp_latitudes_.size(); ++i) {
            RCLCPP_INFO(get_logger(), "Waypoint %zu: Lat %.6f Lon %.6f", i+1, wp_latitudes_[i], wp_longitudes_[i]);
        }
        // Use first waypoint as start if available
        if (!wp_latitudes_.empty() && !wp_longitudes_.empty()) {
            tar_lat_ = wp_latitudes_[0];
            tar_lon_ = wp_longitudes_[0];
            mission_active_ = true;
        }
    }

    // --- Callbacks ---
    void gpsCallback(sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        cur_lat_ = msg->latitude;
        cur_lon_ = msg->longitude;
        
        // Capture the starting point once
        if (!start_pos_recorded_) {
            start_lat_ = cur_lat_;
            start_lon_ = cur_lon_;
            start_pos_recorded_ = true;
            RCLCPP_INFO(this->get_logger(), "HOME POSITION RECORDED: Lat %.8f, Lon %.8f", start_lat_, start_lon_);
        }
        
        gps_ready_ = true;
    }

    void imuCallback(sensor_msgs::msg::Imu::SharedPtr msg)
    {
        RCLCPP_DEBUG(this->get_logger(),"IMU update: yaw=%.3f rad", cur_yaw_);
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, cur_yaw_);
        imu_ready_ = true;
    }

    void pclCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Basic obstacle detection: if any points exist, mark as obstacle
        if (obstacle_detected_) {
            RCLCPP_WARN(this->get_logger(), "Obstacle detected! Stopping and turning away.");
        } else {
            RCLCPP_DEBUG(this->get_logger(), "No obstacle detected.");
        }
        obstacle_detected_ = msg->width * msg->height > 0;
        if (obstacle_detected_) {
            obs_linear_x_ = 0.0;        // stop
            obs_angular_z_ = 0.6;       // turn away
        } else {
            obs_linear_x_ = 0.0;
            obs_angular_z_ = 0.0;
        }
    }

    // --- Utilities ---
    double normalizeAngle(double a)
    {
        while (a > PI) a -= 2 * PI;
        while (a < -PI) a += 2 * PI;
        return a;
    }

    // --- Main Control Loop ---
    void controlLoop()
    {
        if (!mission_active_ || !gps_ready_ || !imu_ready_) return;

        // Current waypoint
        double target_lat = wp_latitudes_[current_wp_idx_];
        double target_lon = wp_longitudes_[current_wp_idx_];

        // GPS distance & heading
        double dLat = (target_lat - cur_lat_) * PI / 180.0;
        double dLon = (target_lon - cur_lon_) * PI / 180.0;
        double lat = cur_lat_ * PI / 180.0;
        double x = -EARTH_RADIUS * dLon * cos(lat);
        double y = -EARTH_RADIUS * dLat;
        double dist = std::sqrt(x * x + y * y);
        double target_yaw = std::atan2(y, x);
        double yaw_err = normalizeAngle(target_yaw - cur_yaw_);

        geometry_msgs::msg::Twist cmd;
        RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *get_clock(), 1000, 
            "Nav Status: [WP " << current_wp_idx_ + 1 << "] Dist: " << dist << "m | "
            "Yaw Err: " << yaw_err << " rad | Phase: " << static_cast<int>(phase_));

        // --- Obstacle avoidance overrides ---
        if (obstacle_detected_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 500, "OBSTACLE ACTIVE: Overriding GPS navigation!");
            cmd.linear.x = obs_linear_x_;
            cmd.angular.z = obs_angular_z_;
            cmd_vel_pub_->publish(cmd);
            return;
        }

        // --- Waypoint reached ---
        if (dist < waypoint_tolerance_) {
            RCLCPP_INFO(this->get_logger(), "Reached Waypoint %zu", current_wp_idx_ + 1);

            if (current_wp_idx_ < wp_latitudes_.size() - 1) {
                // Normal progression through the list (WP 1, 2, 3)
                current_wp_idx_++;
            } 
            else if (current_wp_idx_ == wp_latitudes_.size() - 1 && !initial_loop_completed_) {
                // We just reached the LAST waypoint (WP 4)
                // Now, manually override the target to the Home coordinates
                wp_latitudes_.push_back(start_lat_);
                wp_longitudes_.push_back(start_lon_);
                
                current_wp_idx_++; // Move to the "Home" waypoint we just added
                initial_loop_completed_ = true;
                RCLCPP_INFO(this->get_logger(), "Waypoint 4 reached. Returning to RECORDED START POSITION.");
            } 
            else {
                // We reached the recorded start position
                phase_ = NavPhase::DONE;
                mission_active_ = false;
                RCLCPP_INFO(this->get_logger(), "MISSION COMPLETE: Returned to start. Shutting down motors.");
            }

            // Reset for next leg of journey
            phase_ = NavPhase::ROTATE_START;
            final_rotate_done_ = false;
            return;
        }

        const double YAW_TOL = align_threshold_;
        const double ROTATE_DIST = 2.0;
        const double STOP_DIST = 0.3;
        const double HEADING_KP = 0.8;
        // Log when targeting a new waypoint after the index increments
        static size_t last_logged_wp = -1;
        if (current_wp_idx_ != last_logged_wp) {
            RCLCPP_INFO(this->get_logger(), "Targeting Waypoint %zu: [%.6f, %.6f]. Distance: %.2fm", 
                        current_wp_idx_ + 1, target_lat, target_lon, dist);
            last_logged_wp = current_wp_idx_;
        }

        switch (phase_) {
        case NavPhase::ROTATE_START:
            if (std::abs(yaw_err) < YAW_TOL){
                RCLCPP_INFO(this->get_logger(), "Alignment complete (Err: %.3f rad). Starting forward motion.", yaw_err);
                phase_ = NavPhase::MOVE;
                RCLCPP_INFO(this->get_logger(), "Phase changed to %s",
                    (phase_ == NavPhase::ROTATE_START) ? "ROTATE_START" :
                    (phase_ == NavPhase::MOVE) ? "MOVE" :
                    (phase_ == NavPhase::ROTATE_NEAR) ? "ROTATE_NEAR" :
                    "DONE");
            }
            else
                cmd.angular.z = (yaw_err > 0) ? max_angular_speed_ : -max_angular_speed_;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Aligning to waypoint... Current Yaw: %.3f, Target: %.3f, Error: %.3f", cur_yaw_, target_yaw, yaw_err);
            break;

        case NavPhase::MOVE: {
            double v = std::min(max_linear_speed_, std::max(min_linear_speed_, 0.6 * dist));
            double w = std::clamp(HEADING_KP * yaw_err, -max_angular_speed_, max_angular_speed_);
            if (dist < ROTATE_DIST && !final_rotate_done_){
                phase_ = NavPhase::ROTATE_NEAR;
                RCLCPP_INFO(this->get_logger(), "Phase changed to %s",
                    (phase_ == NavPhase::ROTATE_START) ? "ROTATE_START" :
                    (phase_ == NavPhase::MOVE) ? "MOVE" :
                    (phase_ == NavPhase::ROTATE_NEAR) ? "ROTATE_NEAR" :
                    "DONE");                
            }
            else {
                cmd.linear.x = v;
                cmd.angular.z = w;
            }
            break;
        }

        case NavPhase::ROTATE_NEAR:
            if (std::abs(yaw_err) < YAW_TOL) {
                final_rotate_done_ = true;
                phase_ = NavPhase::MOVE;
                RCLCPP_INFO(this->get_logger(), "Phase changed to %s",
                    (phase_ == NavPhase::ROTATE_START) ? "ROTATE_START" :
                    (phase_ == NavPhase::MOVE) ? "MOVE" :
                    (phase_ == NavPhase::ROTATE_NEAR) ? "ROTATE_NEAR" :
                    "DONE");                
            } else {
                cmd.angular.z = (yaw_err > 0) ? max_angular_speed_ / 2.0 : -max_angular_speed_ / 2.0;
            }
            break;

        case NavPhase::DONE:
            if (phase_ == NavPhase::DONE) {
                RCLCPP_INFO(this->get_logger(), "Mission complete: stopped at final position lat=%.8f lon=%.8f", cur_lat_, cur_lon_);
            }        
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            break;
        }

        cmd_vel_pub_->publish(cmd);
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *get_clock(), 500,"Current WP %zu | dist=%.2f m | yaw_err=%.3f rad | cmd linear=%.2f angular=%.2f",current_wp_idx_, dist, yaw_err, cmd.linear.x, cmd.angular.z);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveGPSNavigator>());
    rclcpp::shutdown();
    return 0;
}