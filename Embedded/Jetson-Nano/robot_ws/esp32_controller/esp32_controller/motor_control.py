#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty
import select


class ASMRControlPublisher(Node):


    def __init__(self):
        super().__init__('motor_control_publisher')
        self.publisher_ = self.create_publisher(String, '/Motor_Control', 10)  # Queue size 10
        self.timer = self.create_timer(0.1, self.timer_callback)


        # Mapping the key presses to certain actions.
        self.key_mapping = {
            'w': "Forward",
            'a': "Left",
            's': "Reverse",
            'd': "Right",
            'l': "Stop"
        }


    def timer_callback(self): # TODO: FOR SLAM UPDATE THIS FUNCTION TO SEND COMMANDS WHEN SLAMS WANTS
        key = self.get_key()
        if key in self.key_mapping:
            command = self.key_mapping[key]
            
            msg = String()
            msg.data = command
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published: {command}')
   
    def get_key(self):
        # Check if there's input on stdin (non-blocking)
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            return sys.stdin.read(1)
        return ''


def main(args=None):
    # Set terminal to raw mode to capture keypresses
    old_attrs = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())


    rclpy.init(args=args)
    node = ASMRControlPublisher()


    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attrs)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()