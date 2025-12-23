# mrm_tp_1 Chatroom

### Task and Approach
My approach was to kind of replicate the demo talker and listner nodes but in way that it starts using a single launch file
**Approach:** I used a single C++ class that inherits from `rclcpp::Node` and then create a publisher and a subscriber of the topic

### ROS Topics
- `/chatroom`: The topic where all text messages are published

### Messages/Services
- `std_msgs/msg/String`: Used for transmitting the chat text concatenated with the sender's node name and the counter
