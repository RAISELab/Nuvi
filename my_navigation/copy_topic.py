import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class PointCloudRelay(Node):
    def __init__(self):
        super().__init__('pointcloud_relay')
        self.sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.callback,
            10)
        self.pub = self.create_publisher(
            PointCloud2,
            '/camera/camera/depth/color/points_global',
            10)
        self.get_logger().info('PointCloud2 Relay Node Started')

    def callback(self, msg):
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()