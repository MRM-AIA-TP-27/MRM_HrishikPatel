#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <iostream>

class MapPrinter : public rclcpp::Node
{
public:
    MapPrinter() : Node("map_monitor")
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
        qos.reliable();
        qos.transient_local();

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map",
            qos,
            std::bind(&MapPrinter::mapCallback, this, std::placeholders::_1)
        );

        std::cout << "Map printer started..." << std::endl;
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        int width = msg->info.width;
        int height = msg->info.height;

        std::cout << "\nMap received: " << width << " x " << height << std::endl;

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int index = y * width + x;
                int val = msg->data[index];

                if (val == -1)
                    std::cout << " . ";   // Unknown
                else if (val > 50)
                    std::cout << " ❌ ";   // Occupied
                else
                    std::cout << " 0 ";   // Free
            }
            std::cout << std::endl;
        }

        std::cout << "--------------------------------\n";
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapPrinter>());
    rclcpp::shutdown();
    return 0;
}