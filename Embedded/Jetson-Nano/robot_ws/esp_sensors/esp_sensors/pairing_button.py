#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import requests

API_URL = "" # Input URL here

class PairingButtonSubscriber(Node):
    def __init__(self):
        super().__init__('pairing_button_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            '/Pairing_Mode',
            self.listener_callback,
            10)
        self.subscription  

    def listener_callback(self, msg):
        value = msg.data
        self.get_logger().info(f"Received /Pairing_Mode value: {value}")

        if value == 1.0:
            self.enable_pairing()

    def enable_pairing(self):
        payload = {"pairing_enabled": True}
        try:
            response = requests.patch(API_URL, json=payload)
            if response.status_code == 200:
                self.get_logger().info("Pairing enabled successfully.")
            else:
                self.get_logger().warn(f"API returned status code {response.status_code}")
        except requests.RequestException as e:
            self.get_logger().error(f"HTTP request failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PairingButtonSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
