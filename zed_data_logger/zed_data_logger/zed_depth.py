import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import numpy as np

class DepthSubscriber(Node):
    def __init__(self):
        super().__init__('depth_subscriber')
        qos_profile = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)

        self.depth_sub = self.create_subscription(
            Image, '/zed/zed_node/depth/depth_registered', self.depth_callback, qos_profile)

        self.camera_sub = self.create_subscription(
            Image, '/zed/zed_node/rgb/image_rect_color', self.image_callback, qos_profile)

        self.bridge = CvBridge()
        self.window_name = 'ZED Camera Stream'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.center_distance = 0.0
        self.v_curr = 0.0

        self.curr_vel_subscriber = self.create_subscription(
            Twist, '/current_vel', self.current_vel_callback, 10)

    def current_vel_callback(self, msg):
        self.v_curr = msg.linear.x

    def depth_callback(self, msg):
        if msg.encoding != '32FC1':
            self.get_logger().error(f'Unsupported encoding: {msg.encoding}')
            return 
        try:
            depth_image = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
        except ValueError as e:
            self.get_logger().error(f'Error reshaping data: {e}')
            return 
        
        center_x = msg.width // 2
        center_y = msg.height // 2
        self.center_distance = depth_image[center_y, center_x] 
        self.get_logger().info(f'Center distance: {self.center_distance:.2f} m')
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # GPU 메모리에 이미지 업로드
            gpu_image = cv2.cuda_GpuMat()
            gpu_image.upload(cv_image)
            
            # GPU에서 리사이징 (640x480)
            gpu_resized = cv2.cuda.resize(gpu_image, (640, 480))
            
            # GPU에서 BGR -> GRAY 변환 (옵션, 필요 시 사용)
            # gpu_gray = cv2.cuda.cvtColor(gpu_resized, cv2.COLOR_BGR2GRAY)
            
            # 다시 CPU로 다운로드
            processed_image = gpu_resized.download()
            
            # OpenCV의 putText는 GPU 지원이 없으므로 CPU에서 처리
            text = f"Depth: {self.center_distance:.2f} m"
            text1 = f"linear_x: {self.v_curr:.2f} m/s"
            cv2.putText(processed_image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
            cv2.putText(processed_image, text1, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
            
            cv2.imshow(self.window_name, processed_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
