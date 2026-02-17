#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np
import time

class SixWheelRoverController(Node):
    def __init__(self):
        super().__init__('six_wheel_rover_controller')

        self.pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # === 1. Define Robot Geometry ===
        self.r = 0.1     # Wheel radius
        self.b = 0.275   # Half track width (distance from center to wheel)
        self.d = 0.4    # Distance from Center of Mass to Front/Rear Axle (Adjust this to match your URDF!)

        self.last_msg_time = 0.0
        self.current_cmd = np.array([0.0, 0.0, 0.0]) # [vx, vy, omega]
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Six Wheel Kinematic Controller Started.')

    def cmd_callback(self, msg):
        # === 2. Fix Input Mapping ===
        self.current_cmd = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        self.last_msg_time = time.time()

    def control_loop(self):
        if time.time() - self.last_msg_time > 0.5:
            q_dot = np.array([0.0, 0.0, 0.0])
        else:
            q_dot = self.current_cmd

        wL = (q_dot[0] - (q_dot[2]*self.b)/2)/self.r
        wR = (q_dot[0] + (q_dot[2]*self.b)/2)/self.r

        # Publish to all 6 wheels
        cmd = Float64MultiArray()
        cmd.data = [wL, wL, wL, wR, wR, wR]
        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    controller = SixWheelRoverController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()