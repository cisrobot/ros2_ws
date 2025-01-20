import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraViewerNode(Node):
    def __init__(self):
        super().__init__('camera_viewer_node')

        self.bridge = CvBridge()

        # 구독자 설정
        self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',  # ZED 2i rgb
            self.image_callback,
            10
        )

        # 카메라 화면을 한 번만 띄우기 위해 초기화
        self.window_name = 'ZED Camera Stream'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)  # 윈도우 크기 조정 가능하도록 설정

    def image_callback(self, msg):
        # ROS 이미지 메시지를 OpenCV 형식으로 변환
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 결과 이미지 표시
        cv2.imshow(self.window_name, cv_image)
        cv2.waitKey(1)  # 화면 갱신을 위한 간격 설정

def main(args=None):
    rclpy.init(args=args)
    node = CameraViewerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
