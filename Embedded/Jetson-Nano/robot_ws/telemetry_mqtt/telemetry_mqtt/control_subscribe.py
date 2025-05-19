#!/usr/bin/env python3

import json
import rclpy
from rclpy.node import Node
from awscrt import mqtt
from awsiot import mqtt_connection_builder
import os

DEFAULT_CONFIG_PATH = "/home/asmr/robot_ws/iot_certs_and_config/iot_config.json"

class IoTListener(Node):
    def __init__(self):
        super().__init__('iot_control_subscriber_min')

        self.declare_parameter("path_for_config", DEFAULT_CONFIG_PATH)
        config_path = self.get_parameter("path_for_config").get_parameter_value().string_value

        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Config file not found: {config_path}")

        with open(config_path, 'r') as f:
            cert_data = json.load(f)

        self.get_logger().info(f"Loaded config:\n{json.dumps(cert_data, indent=2)}")

        self.mqtt_conn = self.connect_to_aws(cert_data)
        self.subscribe_to_command()

    def connect_to_aws(self, config):
        mqtt_conn = mqtt_connection_builder.mtls_from_path(
            endpoint=config["endpoint"],
            port=config["port"],
            cert_filepath=config["certificatePath"],
            pri_key_filepath=config["privateKeyPath"],
            ca_filepath=config["rootCAPath"],
            client_id=config["clientID"],
            clean_session=False,
            keep_alive_secs=60,
        )
        self.get_logger().info("Connecting to AWS IoT Core...")
        mqtt_conn.connect().result()
        self.get_logger().info("Connected to AWS IoT Core")
        return mqtt_conn

    def subscribe_to_command(self):
        topic = "ros2_mock_telemetry_topic"
        try:
            subscribe_future, packet_id = self.mqtt_conn.subscribe(
                topic=topic,
                qos=mqtt.QoS.AT_LEAST_ONCE,
                callback=self.on_command_received
            )
            suback = subscribe_future.result()
            self.get_logger().info(f" Subscribed to topic '{topic}' (QoS {suback.get('qos')})")
        except Exception as e:
            self.get_logger().error(f" Subscription failed: {e}")

    def on_command_received(self, topic, payload, **kwargs):
        try:
            payload_str = payload.decode()
            self.get_logger().info(f"Message received on '{topic}': {payload_str}")
        except Exception as e:
            self.get_logger().error(f"Failed to decode payload: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = IoTListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()