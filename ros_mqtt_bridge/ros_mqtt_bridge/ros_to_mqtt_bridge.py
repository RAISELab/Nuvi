import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int64, Int32
import paho.mqtt.client as mqtt
import json

MQTT_BROKER = "test.mosquitto.org"
MQTT_PORT = 1883

class RosToMqttBridge(Node):
    def __init__(self):
        super().__init__('ros_to_mqtt_bridge')

        # MQTT 연결
        self.client = mqtt.Client()
        self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
        self.client.loop_start()

        # 혼잡도 수신 (Vector3)
        self.create_subscription(
            Vector3,
            '/congestion_info',
            self.congestion_callback,
            10
        )

        # 방향 수신 (Int64)
        self.create_subscription(
            Int64,
            '/direction',
            self.direction_callback,
            10
        )

        # 도착 여부 수신 (Int32)
        self.create_subscription(
            Int32,
            '/goal_succeed',
            self.goal_callback,
            10
        )

        self.get_logger().info('✅ ROS → MQTT 브리지 시작됨')

    def congestion_callback(self, msg):
        self.get_logger().info(f"📡 혼잡도: x={msg.x}, y={msg.y}, z={msg.z}")
        payload = json.dumps({"x": msg.x, "y": msg.y, "z": msg.z})
        self.client.publish('congestion_info', payload)

    def direction_callback(self, msg):
        self.get_logger().info(f"➡️ 방향: {msg.data}")
        self.client.publish('direction', str(msg.data))

    def goal_callback(self, msg):
        self.get_logger().info(f"🏁 도착 여부: {msg.data}")
        self.client.publish('goal_succeed', str(msg.data))

def main(args=None):
    rclpy.init(args=args)
    node = RosToMqttBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()