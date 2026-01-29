# Work 3: Activity 01 - ROS2 Topics
## 1) Activity Goals
Understand the Publisher/Subscriber communication pattern.
2Develop a Subscriber node to receive and process data from a topic.
Create a ROS2 node that publishes messages to a specific topic.
Utilize ROS2 CLI tools such as ros2 topic list, echo, and info for debugging.
Configure the package.xml and setup.py (or CMakeLists.txt) files correctly.
## 2) Materials
No materials required 
## 3) Code
### This code of Publisher 
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class NumberPublisher(Node):
    def __init__(self):
        super().__init__('number_publisher')
        self.get_logger().info('Beep Boop R2D2 is publishing')
        self.counter =0
        self.publisher_ = self.create_publisher(String, 'number', 10)
        self.create_timer(1.0, self.r2d2_number)
    def r2d2_number(self):
        msg = String()
        msg.data = str(self.counter)
        self.get_logger().info(f'R2D2 says number: {msg.data}')
        self.publisher_.publish(msg)
        self.counter += 1
        
def main(args=None):
    rclpy.init(args=args)
    r2d2_node = NumberPublisher()
    rclpy.spin(r2d2_node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()
### Code of listener

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64


class NumberCounter(Node):

    def _init_(self):
        super()._init_('number_counter')
        self.counter = 0
        self.subscription = self.create_subscription(
            Int64,
            '/number',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(
            Int64,
            '/number_count',
            10
        )
        self.get_logger().info('Number counter working...')

    def listener_callback(self, msg):
        self.counter += msg.data
        out_msg = Int64()
        out_msg.data = self.counter
        self.publisher_.publish(out_msg)
        self.get_logger().info(
            f'Recibido: {msg.data} | Total: {self.counter}'
        )

def main(args=None):
    rclpy.init(args=args)
    nodito = NumberCounter()
    rclpy.spin(nodito)
    rclpy.shutdown()

if _name_ == '_main_':
    main()
with the two codes, we have our program:
![Diagrama del sistema](recursos/imgs/3.jpeg)