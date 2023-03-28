#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("first_node") # name of node
        self.counter_ = 0
        self.get_logger().info("ROS2") # info to print
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.counter_ += 1
        self.get_logger().info("Hello " + str(self.counter_))



def main(args=None):
    # init ros2 communication
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    # shutdown ros2 communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()

