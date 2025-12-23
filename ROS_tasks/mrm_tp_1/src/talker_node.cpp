#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

using namespace std::chrono_literals;

class TalkerNode : public rclcpp::Node {
public:
  TalkerNode() : Node("talker_node") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatroom",10);
    
    //callback is 1s because I thought it will be a sufficient gap to check both nodes output
    timer_ = this->create_wall_timer(1s, std::bind(&TalkerNode::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hrishik has published this and Count: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_ = 1;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TalkerNode>());
  rclcpp::shutdown();
  return 0;
}