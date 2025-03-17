import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('pointcloud_to_laserscan')

        # ZED PointCloud2 데이터 구독
        self.subscription = self.create_subscription(
            PointCloud2,
            '/zed/zed_node/point_cloud/cloud_registered',  # ✅ 구독할 토픽
            self.pointcloud_callback,
            10
        )

    def pointcloud_callback(self, msg):
        """PointCloud2 메시지를 받아 단순 출력"""
        self.get_logger().info(f"Received PointCloud2 data")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
