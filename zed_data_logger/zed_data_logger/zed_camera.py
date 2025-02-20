import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

class CameraViewerNode(Node):
    def __init__(self):
        super().__init__('camera_viewer_node')

        self.bridge = CvBridge()

        # 구독자 설정
        self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',  # ZED 2i RGB
            self.image_callback,
            10
        )

        # TensorRT 설정
        self.trt_logger = trt.Logger(trt.Logger.WARNING)
        self.trt_engine = self.create_tensorrt_engine()
        self.context = self.trt_engine.create_execution_context()

        # GPU 메모리 할당
        self.input_size = (3, 720, 1280)  # (C, H, W)
        self.output_size = self.input_size
        self.input_mem = cuda.mem_alloc(np.prod(self.input_size) * np.float32().nbytes)
        self.output_mem = cuda.mem_alloc(np.prod(self.output_size) * np.float32().nbytes)
        self.stream = cuda.Stream()

        # 카메라 화면을 한 번만 띄우기 위해 초기화
        self.window_name = 'ZED Camera Stream'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)  # 윈도우 크기 조정 가능하도록 설정

    def create_tensorrt_engine(self):
        """TensorRT 엔진을 생성하는 함수"""
        builder = trt.Builder(self.trt_logger)
        network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
        config = builder.create_builder_config()
        config.max_workspace_size = 1 << 20  # 1MB
        
        input_tensor = network.add_input("input", trt.DataType.FLOAT, (3, 720, 1280))
        scale_layer = network.add_constant((3, 1, 1), np.array([1.2, 1.2, 1.2], dtype=np.float32))
        output_tensor = network.add_elementwise(input_tensor, scale_layer.get_output(0), trt.ElementWiseOperation.PROD).get_output(0)
        network.mark_output(output_tensor)

        return builder.build_engine(network, config)

    def image_callback(self, msg):
        # ROS 이미지 메시지를 OpenCV 형식으로 변환
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # OpenCV 이미지를 TensorRT 호환 텐서로 변환 (HWC → CHW)
        img_np = np.transpose(cv_image, (2, 0, 1)).astype(np.float32)

        # GPU로 데이터 전송
        cuda.memcpy_htod_async(self.input_mem, img_np, self.stream)

        # TensorRT 실행
        self.context.execute_async_v2(
            bindings=[int(self.input_mem), int(self.output_mem)],
            stream_handle=self.stream.handle
        )

        # GPU에서 결과 가져오기
        processed_img = np.empty(self.input_size, dtype=np.float32)
        cuda.memcpy_dtoh_async(processed_img, self.output_mem, self.stream)
        self.stream.synchronize()

        # TensorRT 출력 (CHW → HWC 변환)
        processed_img = np.transpose(processed_img, (1, 2, 0)).astype(np.uint8)

        # 결과 이미지 표시
        cv2.imshow(self.window_name, processed_img)
        cv2.waitKey(1)  # 화면 갱신을 위한 간격 설정

def main(args=None):
    rclpy.init(args=args)
    node = CameraViewerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
