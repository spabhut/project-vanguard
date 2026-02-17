#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Settings
msg = """
Reading from the keyboard!
---------------------------
   w
 a s d

w/s : linear velocity (forward/backward)
a/d : angular velocity (left/right)
space: force stop

CTRL-C to quit
"""
MAX_LIN_VEL = 2.0
MAX_ANG_VEL = 2.0

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info("Teleop Started")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        print(msg)
        try:
            while rclpy.ok():
                key = self.get_key()
                twist = Twist()
                
                if key == 'w':
                    twist.linear.x = MAX_LIN_VEL
                elif key == 's':
                    twist.linear.x = -MAX_LIN_VEL
                elif key == 'a':
                    twist.angular.z = MAX_ANG_VEL
                elif key == 'd':
                    twist.angular.z = -MAX_ANG_VEL
                elif key == ' ':
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                elif key == '\x03': # CTRL+C
                    break
                
                if key in ['w', 's', 'a', 'd', ' ']:
                    self.publisher_.publish(twist)

        except Exception as e:
            print(e)

        finally:
            twist = Twist()
            twist.linear.x = 0.0; twist.angular.z = 0.0
            self.publisher_.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = Teleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
