import struct
import serial
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist  # Twist 메시지 추가
import csv
from datetime import datetime  # 날짜 및 시간 관리를 위한 모듈 추가
from scipy.ndimage import median_filter 

class ArdSerial():
    def __init__(self):
        self.ard_received = False
        self.ard_port = '/dev/ttyACM0'
        self.baud = 57600
        self.ser = serial.Serial(self.ard_port, self.baud, timeout=1)  # 시리얼 통신 인스턴스 생성
        self.rcv_struct_format = '3s2i'  # 수신할 데이터의 패킷 형식 header='ARD', steer, esc
        self.rcv_struct_size = struct.calcsize(self.rcv_struct_format)  # 수신할 데이터의 패킷 크기
        self.ard_steer_pwm = None
        self.ard_throttle_pwm = None

    def rcv_ard(self):
        if self.ser.readable():
            buffer = self.ser.read_all()
            len_buffer = len(buffer)

            if len_buffer >= self.rcv_struct_size:
                strt_idx = -1
                for idx in range(len_buffer - self.rcv_struct_size + 1):
                    if buffer[idx] == ord('A') and buffer[idx+1] == ord('R') and buffer[idx+2] == ord('D'):
                        strt_idx = idx
                        break

                if strt_idx != -1:
                    rcv_struct = buffer[strt_idx:strt_idx + self.rcv_struct_size]
                    try:
                        rcv_data = struct.unpack(self.rcv_struct_format, rcv_struct)
                        self.ard_steer_pwm = rcv_data[1]
                        self.ard_throttle_pwm = rcv_data[2]
                        self.ard_received = True
                    except:
                        pass


class ZedDataLogger(Node):
    def __init__(self):
        super().__init__('zed_odom')  # 노드명

        # ArdSerial 객체 생성
        self.ard_serial = ArdSerial()

        # Subscriber 설정
        self.subscriber = self.create_subscription(
            Odometry,
            '/zed/zed_node/odom',  # 구독할 토픽
            self.subscriber_callback,
            10
        )

        # Timer 설정 (1초 간격)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # linear.x, angular_z만 발행하는 Publisher 추가
        self.current_publisher = self.create_publisher(Twist, '/current_vel', 10)

        # 데이터를 저장할 변수들 초기화
        self.position = None
        self.orientation = None
        self.twist_linear = None
        self.twist_angular = None

        # 타임스탬프를 기반으로 파일 이름 생성
        current_time = datetime.now().strftime("%Y%m%d_%H%M%S")  # 파일 이름에 추가할 타임스탬프 생성
        file_name = f"src/zed_data_logger/zed_data_{current_time}.csv"# 새로운 파일 이름 생성


        # 로그 파일 생성 및 CSV 헤더 작성
        self.data_file = open(file_name, "w", newline='')
        self.data_writer = csv.writer(self.data_file)


        # 헤더 작성
        self.data_writer.writerow([
            'Position_X', 'Position_Y', 'Position_Z',
            'Orientation_X', 'Orientation_Y', 'Orientation_Z', 'Orientation_W',
            'Twist_Linear_X', 'Twist_Linear_Y', 'Twist_Linear_Z',
            'Twist_Angular_X', 'Twist_Angular_Y', 'Twist_Angular_Z',
            'Steer_PWM', 'Throttle_PWM'
        ])

    def subscriber_callback(self, msg):
        # Odometry 메시지로부터 위치, 방향, 속도 데이터를 저장
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        self.twist_linear = msg.twist.twist.linear
        self.twist_angular = msg.twist.twist.angular

        # linear.x 값을 포함하는 Twist 메시지 발행
        twist_msg = Twist()
        twist_msg.linear.x = self.twist_linear.x
        twist_msg.angular.z = self.twist_angular.z
        self.current_publisher.publish(twist_msg)


    def timer_callback(self):
        # ArdSerial에서 데이터를 수신
        self.ard_serial.rcv_ard()

        # ZED 및 Arduino 데이터를 모두 기록
        if (self.position and self.orientation and self.twist_linear and self.twist_angular and
                self.ard_serial.ard_received):

            # 모든 데이터를 한 행에 기록
            self.data_writer.writerow([
                self.position.x, self.position.y, self.position.z,
                self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w,
                self.twist_linear.x, self.twist_linear.y, self.twist_linear.z,
                self.twist_angular.x, self.twist_angular.y, self.twist_angular.z,
                self.ard_serial.ard_steer_pwm, self.ard_serial.ard_throttle_pwm
            ])

            self.get_logger().info('Data written to file')  # 터미널에 로그가 되고 있음을 출력
            # 기록을 즉시 파일에 반영
            self.data_file.flush()
        else:
            self.get_logger().info('로그되지 않음')

    def destroy(self):
        # 노드 종료 시 파일 닫기
        self.data_file.close() 
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    # 노드 실행
    node = ZedDataLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
