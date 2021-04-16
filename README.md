# `piracer_msgs`

The `piracer_msgs` package provides some ROS 2 messages specific to the
`piracer` package.

## Getting started

1. Download package (either as a `.zip` or with `git clone`)
2. Build package using `colcon build`
3. Add package as a dependency in your `CMakeLists.txt` file (with 
   `find_package` and `ament_target_dependencies`)
4. Add package as a dependency in your `package.xml` file
5. Include the appropriate header file(s)

### C++ example

```c++
#include <rclcpp/rclcpp.hpp>
#include <piracer_msgs/msg/system_power.hpp>

class MySub : public rclcpp::Node {
public:
    MySub() : Node("my_sub") {
        subscription_ = this->create_subscription<piracer_msgs::msg::SystemPower>(
                "my_topic", 10, std::bind(&MySub::sub_cb, this, _1));
    }
    
private:
    void sub_cb(const piracer_msgs::msg::SystemPower::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "Voltage: %f", msg->voltage);
        RCLCPP_INFO(this->get_logger(), "Current: %f", msg->current);
        RCLCPP_INFO(this->get_logger(), "Power: %f", msg->power);
    }
    
    rclcpp::Subscription<priacer_msgs::msg::SystemPower>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MySub>());
    rclcpp::shutdown();
    
    return 0;
}
```

### Python example

```python
import rclpy
from rclpy.node import Node

from piracer_msgs.msg import SystemPower


class MySub(Node):
   def __init__(self):
      super(MySub, self).__init__('my_sub')
      self.sub = self.create_subscription(msg_type=SystemPower,
                                          topic='my_topic',
                                          callable=self._msg_cb,
                                          qos_profile=10)

   def _msg_cb(self, msg: SystemPower):
      self.get_logger().info(f'Voltage: {msg.voltage}')
      self.get_logger().info(f'Current: {msg.current}')
      self.get_logger().info(f'Power: {msg.power}')


def main():
    rclpy.init()
    node = MySub()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Messages (`.msg` files)

Message Type | Description
-------------|------------
`SystemPower` | The measured voltage, current, and power provided to the car
