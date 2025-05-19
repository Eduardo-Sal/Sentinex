#!/usr/bin/env python3

import rclpy
import json
from rclpy.node import Node
from std_msgs.msg import String

## SHOULD NOT EXISTS 

class ControlSubscriber(Node):

    def __init__(self):
        super().__init__('control_subscriber')
        self.subscription = self.create_subscription(
            String,
            'robot/control',
            self.control_callback,
            10
        )
        self.get_logger().info("✅ ControlSubscriber is running and listening to 'robot/control'...")

    def control_callback(self, msg):
        try:
            # Try to parse as JSON (if sent like { "message": "left" })
            data = json.loads(msg.data)
            command = data.get("message", "").lower().strip()
        except json.JSONDecodeError:
            # Fallback to raw string
            command = msg.data.lower().strip()

        if command in ["up", "down", "left", "right"]:
            self.get_logger().info(f"🎮 Command received: {command}")
            # TODO: Add GPIO, motor command, or publish to cmd_vel
        else:
            self.get_logger().warn(f" Unknown command: {command}")

def main(args=None):
    rclpy.init(args=args)
    node = ControlSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()