#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from turtlesim.srv import Spawn
from turtlesim.action import RotateAbsolute
from turtlesim.msg import Pose
from rclpy.task import Future
from dobo

PI = 3.1415926


class TurtleSprawnerNode(Node):
    def __init__(self):
        super().__init__("turtles_spawner")
        self.position = Pose()
        self.goal = RotateAbsolute.Goal()
        self.subscription = self.create_subscription(
            Pose, 'turtle1/pose', self.position_callback, 1)
        self.publisher = self.create_publisher(Pose, 'turtle1/pose', 1)
        self.act_cli = ActionClient(
            self, RotateAbsolute, "turtle1/rotate_absolute")
        self.cli = self.create_client(Spawn, 'spawn')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Spawn.Request()
        self.state = 0
        self.all_done = Future()
        self.declare_parameter('second_name', 'turtle2')
        self.declare_parameter('third_name', 'turtle3')

    def position_callback(self, msg):
        self.position = msg

    def create_turtle(self, turtle_number):
        self.req.x = self.position.x
        self.req.y = self.position.y
        self.req.theta = self.position.theta
        if turtle_number == 2:
            self.req.name = self.get_parameter(
                'second_name').get_parameter_value()
        elif turtle_number == 3:
            self.req.name = self.get_parameter(
                'third_name').get_parameter_value().string_value
        self.cli.wait_for_service()
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.done_callback)

    def set_angle(self, goals):
        self.goal.theta = goals
        self.act_cli.wait_for_server()
        self.future = self.act_cli.send_goal_async(self.goal)
        self.future.add_done_callback(self.goal_was_send)

    def goal_was_send(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self.future = goal_handle.get_result_async()
        self.future.add_done_callback(self.done_callback)

    def run_simulation(self):
        if self.state == 0:
            self.set_angle(PI)
        elif self.state == 1:
            self.create_turtle(2)
        elif self.state == 2:
            self.set_angle(PI/4)
        elif self.state == 3:
            self.set_angle(-PI/2)
        elif self.state == 4:
            self.create_turtle(3)
            self.all_done._done = True
        return self.all_done

    def done_callback(self, arg):
        self.get_logger().info(str(self.future.result()))
        if self.state <= 3:
            self.state += 1
            self.run_simulation()


def main(arg=None):
    rclpy.init(args=arg)
    node = TurtleSprawnerNode()
    future = node.run_simulation()
    rclpy.spin_until_future_complete(node, future)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
