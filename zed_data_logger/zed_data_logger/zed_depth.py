import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import numpy as np

class DepthSubscriber(Node):
    def __init__(self):
        super().__init__('depth_subscriber') #노드명

        #qos_profile: QoS(서비스 품질) 설정을 통해 구독자의 메시지 전송 신뢰성 및 저장 정책을 설정합니다. 여기서는 'Best Effort' 정책을 사용하여 가능한 한 많은 메시지를 수신하도록 합니다.
        qos_profile = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)

        #구독자 설정
        self.depth_sub = self.create_subscription(
            Image,
            '/zed/zed_node/depth/depth_registered',
            self.depth_callback,
            qos_profile
        )

        self.camera_sub = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.image_callback,
            qos_profile
        )

        # OpenCV 브리지 초기화
        self.bridge = CvBridge()

        # OpenCV 윈도우 설정
        self.window_name = 'ZED Camera Stream'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)  # 윈도우 크기 조정 가능하도록 설정
        self.center_distance = 0.0
        self.v_curr = 0.0

        self.curr_vel_subscriber = self.create_subscription(
            Twist,
            '/current_vel',  # 'zed_odom' 토픽 구독
            self.current_vel_callback,
            10
        )
    def current_vel_callback(self, msg): 
    # 구독한 linear.x 값을 로그로 출력
        self.v_curr = msg.linear.x

    #콜백 함수
    def depth_callback(self,msg):
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
            # ROS 이미지 메시지를 OpenCV 형식으로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            #resize_image = cv2.resize(cv_image, (640, 480))



            # 결과 이미지 표시
            text = f"Depth: {self.center_distance:.2f} m"
            text1 = f"linear_x:{self.v_curr:.2f} m/s"
            cv2.putText(cv_image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
            cv2.putText(cv_image, text1, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
            cv2.imshow(self.window_name, cv_image)
            cv2.waitKey(1)  # 화면 갱신을 위한 간격 설정
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()  # OpenCV 윈도우 종료
    rclpy.shutdown()

if __name__ == '__main__':
    main() 