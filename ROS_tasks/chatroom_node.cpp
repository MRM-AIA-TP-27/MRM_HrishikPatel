#include <iostream>
#include <string>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ChatRoomNode : public rclcpp::Node {
public:
    ChatRoomNode() : Node("chatroom_node") {

        std::cout << "Enter your username: ";
        std::getline(std::cin, username_);

        publisher_ = this->create_publisher<std_msgs::msg::String>("chatroom_topic", 10);
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "chatroom_topic", 10, std::bind(&ChatRoomNode::topic_callback, this, std::placeholders::_1));

        input_thread_ = std::thread(&ChatRoomNode::read_input, this);
        
        RCLCPP_INFO(this->get_logger(), "Chatroom active. Start typing!");
    }

    ~ChatRoomNode() {
        if (input_thread_.joinable()) input_thread_.join();
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {

        if (msg->data.find(username_ + ":") != 0) {
            std::cout << "\n" << msg->data << std::endl;
            std::cout << "> " << std::flush; 
        }
    }

    void read_input() {
        while (rclcpp::ok()) {
            std::string input;
            std::cout << "> ";
            std::getline(std::cin, input);

            if (!input.empty()) {
                auto message = std_msgs::msg::String();
                message.data = username_ + ": " + input;
                publisher_->publish(message);
            }
        }
    }

    std::string username_;
    std::thread input_thread_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChatRoomNode>());
  rclcpp::shutdown();
  return 0;
}