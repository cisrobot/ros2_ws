import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid

class CostmapToMap(Node):
    def __init__(self):
        super().__init__('costmap_to_map')

        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL  # 🟢 QoS 수정
        )

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            10)
        self.publisher = self.create_publisher(
            OccupancyGrid,
            '/map',
            qos_profile)  # 🟢 QoS 설정 추가

    def costmap_callback(self, msg):
        self.get_logger().info("Converting costmap to map...")
        map_msg = msg
        map_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(map_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CostmapToMap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
