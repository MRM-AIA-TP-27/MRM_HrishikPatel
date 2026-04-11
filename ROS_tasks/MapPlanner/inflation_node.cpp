#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <vector>
#include <iostream>
#include <algorithm>

class InflationNode : public rclcpp::Node
{
public:
    InflationNode() : Node("inflation_node")
    {
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10,
            std::bind(&InflationNode::mapCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&InflationNode::odomCallback, this, std::placeholders::_1));

        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/inflated_map", 10);

        RCLCPP_INFO(this->get_logger(), "Inflation Node Started (ODOM based)");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    nav_msgs::msg::OccupancyGrid current_map_;
    bool map_received_ = false;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    bool odom_received_ = false;

    int inflation_radius_ = 5;

    // ---------------- ODOM ----------------
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;

        odom_received_ = true;
    }

    // ---------------- MAP ----------------
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        current_map_ = *msg;
        map_received_ = true;

        process();
    }

    // ---------------- PROCESS ----------------
    void process()
    {
        if (!map_received_ || !odom_received_)
            return;

        auto inflated_map = inflateMap(current_map_);
        map_pub_->publish(inflated_map);

        printMap(inflated_map);
    }

    // ---------------- INFLATION ----------------
    nav_msgs::msg::OccupancyGrid inflateMap(const nav_msgs::msg::OccupancyGrid &map)
    {
        auto new_map = map;

        int width = map.info.width;
        int height = map.info.height;

        std::vector<int8_t> data = map.data;

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int idx = y * width + x;

                if (map.data[idx] > 50)
                {
                    for (int dy = -inflation_radius_; dy <= inflation_radius_; dy++)
                    {
                        for (int dx = -inflation_radius_; dx <= inflation_radius_; dx++)
                        {
                            int nx = x + dx;
                            int ny = y + dy;

                            if (nx >= 0 && ny >= 0 && nx < width && ny < height)
                            {
                                data[ny * width + nx] = 100;
                            }
                        }
                    }
                }
            }
        }

        new_map.data = data;
        return new_map;
    }

    // ---------------- PRINT ----------------
    void printMap(const nav_msgs::msg::OccupancyGrid &map)
    {
        int width = map.info.width;
        int height = map.info.height;

        double res = map.info.resolution;
        double origin_x = map.info.origin.position.x;
        double origin_y = map.info.origin.position.y;

        system("clear");

        // 🔥 Convert world → map index
        int rx = static_cast<int>((robot_x_ - origin_x) / res);
        int ry = static_cast<int>((robot_y_ - origin_y) / res);

        // Clamp
        rx = std::max(0, std::min(width - 1, rx));
        ry = std::max(0, std::min(height - 1, ry));

        std::cout << "Robot world: (" << robot_x_ << ", " << robot_y_ << ")\n";
        std::cout << "Robot grid: (" << rx << ", " << ry << ")\n\n";

        for (int y = height - 1; y >= 0; y--)
        {
            for (int x = 0; x < width; x++)
            {
                if (x == rx && y == ry)
                {
                    std::cout << "\033[42mR \033[0m";
                    continue;
                }

                int val = map.data[y * width + x];

                if (val == -1)
                    std::cout << ". ";
                else if (val > 50)
                    std::cout << "\033[31m# \033[0m";
                else
                    std::cout << "  ";
            }
            std::cout << "\n";
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InflationNode>());
    rclcpp::shutdown();
    return 0;
}