#!/usr/bin/env python3

import rclpy
import json
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32
from awscrt import mqtt
from telemetry_mqtt.connection_helper import ConnectionHelper

class IotListener(Node):
    def __init__(self):
        super().__init__('iot_listener')

        # Load parameters
        self.declare_parameter("path_for_config", "")
        self.declare_parameter("discover_endpoints", False)

        config_path = self.get_parameter("path_for_config").get_parameter_value().string_value
        discover = self.get_parameter("discover_endpoints").get_parameter_value().bool_value

        # Create MQTT connection
        self.connection_helper = ConnectionHelper(self.get_logger(), config_path, discover)
        self.mqtt_conn = self.connection_helper.mqtt_conn

        # ROS publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/Motor_Control', 5)
        self.speed_pub = self.create_publisher(Float32, '/Speed_Control', 5)
        self.servo_pub = self.create_publisher(Twist, '/Servo_Control', 5)
        self.camera_cmd_pub = self.create_publisher(String, '/Camera', 5)

        self.current_speed = 1.0

        self.subscribe_to_command_topics()

    def subscribe_to_command_topics(self):
        topics = [
            "robot-4/data",
            "robot-4/camera",
            "robot-4/direction",
            "robot-4/mode",
            "robot-4/speed"
        ]
        for topic in topics:
            self.get_logger().info(f"Attempting to subscribe to AWS IoT topic: {topic}")
            try:
                subscribe_future, _ = self.mqtt_conn.subscribe(
                    topic=topic,
                    qos=mqtt.QoS.AT_LEAST_ONCE,
                    callback=self.handle_command
                )
                subscribe_future.result()
                self.get_logger().info(f"Successfully subscribed to topic: {topic}")
            except Exception as e:
                self.get_logger().error(f"Failed to subscribe to topic {topic}: {e}")

    def handle_command(self, topic, payload, **kwargs):
        try:
            raw = payload.decode('utf-8')
            data = json.loads(raw)
            self.get_logger().info(f"Received on '{topic}': {data}")
        except Exception as e:
            self.get_logger().error(f"Error decoding message on '{topic}': {e}")
            return

        if topic == "robot-4/camera":
            mode = data.get("mode")
            twist = Twist()

            if mode == "left":
                twist.angular.y = 1.0
            elif mode == "right":
                twist.angular.y = -1.0
            elif mode == "reset":
                twist.angular.y = 5.0
            elif mode == "captureFace":
                name = data.get("name", "unknown")
                msg = String()
                msg.data = json.dumps({"name": name})
                self.camera_cmd_pub.publish(msg)
                self.get_logger().info(f"Published face capture request to /Camera: {msg.data}")
                return
            else:
                self.get_logger().warn(f"Unknown camera mode: {mode}")
                return

            self.servo_pub.publish(twist)
            self.get_logger().info(f"Published camera/servo command: {mode}")

        elif topic == "robot-4/direction":
            command = data.get("command")
            twist = Twist()
            if command == "up":
                twist.linear.x = -1.0
            elif command == "down":
                twist.linear.x = 1.0
            elif command == "left":
                twist.angular.z = 1.0
            elif command == "right":
                twist.angular.z = -1.0
            else:
                self.get_logger().warn(f"Unknown direction command: {command}")
                return
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(f"Published movement command: {command}")

        elif topic == "robot-4/mode":
            mode = data.get("number")
            # Mode switch logic placeholder
            self.get_logger().info(f"Mode switch requested: {mode}, enabled={data.get('enabled')}")

        elif topic == "robot-4/speed":
            command = data.get("command")
            msg = Float32()
            if command == "increase":
                self.current_speed = min(self.current_speed + 0.5, 4.0)
            elif command == "decrease":
                self.current_speed = max(self.current_speed - 0.5, 0.1)
            else:
                self.get_logger().warn(f"Unknown speed command: {command}")
                return
            msg.data = self.current_speed
            self.speed_pub.publish(msg)
            self.get_logger().info(f"Updated speed to: {self.current_speed}")

        elif topic == "robot-4/data":
            # Future: publish telemetry or status
            self.get_logger().info(f"Received general data: {data}")

        else:
            self.get_logger().warn(f"Unhandled topic: {topic}")

def main(args=None):
    rclpy.init(args=args)
    node = IotListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
