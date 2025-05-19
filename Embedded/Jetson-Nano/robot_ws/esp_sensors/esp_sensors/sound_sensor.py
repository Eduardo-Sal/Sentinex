#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import requests

THRESHOLD_DB = 75.0  # dB threshold for upload
API_URL = "" # Input URL here

class MicrophoneSubscriber(Node):
    def __init__(self):
        super().__init__('mic_db_uploader')
        self.subscription = self.create_subscription(
            Float32,
            '/Microphone_dB',
            self.listener_callback,
            10)
        self.subscription 

    def listener_callback(self, msg):
        db_level = msg.data
        self.get_logger().info(f"Received dB: {db_level:.2f}")

        if db_level >= THRESHOLD_DB:
            self.upload_db_level(db_level)
        else:
            self.get_logger().info("Below threshold; not uploading.")

    def upload_db_level(self, db_level):
        payload = {"db_level": db_level}
        try:
            response = requests.post(API_URL, json=payload)
            if response.status_code == 200:
                self.get_logger().info("Successfully uploaded dB level.")
            else:
                self.get_logger().warn(f"Upload failed with status {response.status_code}")
        except requests.RequestException as e:
            self.get_logger().error(f"Request failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MicrophoneSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
