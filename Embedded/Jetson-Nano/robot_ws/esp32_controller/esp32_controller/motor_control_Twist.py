#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import sys
import tty
import termios
import select

class ASMRControlPublisher(Node):
    def __init__(self):
        super().__init__('motor_control_publisher_twist')

        self.publisher_ = self.create_publisher(Twist, '/Motor_Control', 5)
        self.speed_publisher = self.create_publisher(Float32, '/Speed_Control', 5)
        self.servo_publisher = self.create_publisher(Twist, '/Servo_Control', 5)

        self.speed = 1.0
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.key_mapping = {
            's': (1.0, 0.0, 0.0),
            'a': (0.0, 1.0, 0.0),
            'w': (-1.0, 0.0, 0.0),
            'd': (0.0, -1.0, 0.0),
            'l': (0.0, 0.0, 0.0),
            'q': 'servo_left',
            'e': 'servo_right',
            'r': 'servo_reset',
            'z': 'decrease_speed',
            'x': 'increase_speed'
        }

    def timer_callback(self):
        key = self.get_key()
        if key in self.key_mapping:
            command = self.key_mapping[key]

            if command == 'increase_speed':
                self.speed = min(self.speed + 0.5, 4.0)
                msg = Float32()
                msg.data = self.speed
                self.speed_publisher.publish(msg)
                self.get_logger().info(f'Speed increased to {self.speed}')

            elif command == 'decrease_speed':
                self.speed = max(self.speed - 0.5, 0.1)
                msg = Float32()
                msg.data = self.speed
                self.speed_publisher.publish(msg)
                self.get_logger().info(f'Speed decreased to {self.speed}')

            elif command == 'servo_left':
                msg = Twist()
                msg.angular.y = 1.0
                self.servo_publisher.publish(msg)
                self.get_logger().info('Published: Servo LEFT')

            elif command == 'servo_right':
                msg = Twist()
                msg.angular.y = -1.0
                self.servo_publisher.publish(msg)
                self.get_logger().info('Published: Servo RIGHT')

            elif command == 'servo_reset':
                msg = Twist()
                msg.angular.y = 5.0
                self.servo_publisher.publish(msg)
                self.get_logger().info('Published: Servo RESET')

            else:
                linear_x, angular_z, _ = command
                msg = Twist()
                msg.linear.x = linear_x
                msg.angular.z = angular_z
                msg.angular.x = 0.0
                self.publisher_.publish(msg)
                self.get_logger().info(
                    f'Published: linear.x={linear_x}, angular.z={angular_z}')

    def get_key(self):
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            return sys.stdin.read(1)
        return ''

def main(args=None):
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
