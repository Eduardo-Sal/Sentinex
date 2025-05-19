#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from awscrt import mqtt 
from connection_helper import ConnectionHelper

RETRY_WAIT_TIME_SECONDS = 5


class MqttPublisher(Node):
    def __init__(self):
        super().__init__('mqtt_publisher')
        self.declare_parameter("path_for_config", "")
        self.declare_parameter("discover_endpoints", False)

        path_for_config = self.get_parameter("path_for_config").get_parameter_value().string_value
        discover_endpoints = self.get_parameter("discover_endpoints").get_parameter_value().bool_value
        self.connection_helper = ConnectionHelper(self.get_logger(), path_for_config, discover_endpoints)

        self.init_subs()

    def init_subs(self):
        """Subscribe to mock ros2 telemetry topic"""
        self.subscription = self.create_subscription(
            String,
            '/robot_control',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        """Callback for the mock ros2 telemetry topic"""
        message_json = msg.data
        self.get_logger().info("Received data on ROS2 {}\nPublishing to AWS IoT".format(msg.data))
        self.connection_helper.mqtt_conn.publish(
            topic="ros2_mock_telemetry_topic",
            payload=message_json,
            qos=mqtt.QoS.AT_LEAST_ONCE
        )

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MqttPublisher()

    rclpy.spin(minimal_subscriber)

    # Destroy the node
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

