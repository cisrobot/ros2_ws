import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class CmdVelLim(Node):

    def __init__(self):
        super().__init__('cmd_vel_lim')

        # 파라미터 선언
        self.declare_parameter('sub_topic', '/ps5_joy/cmd_vel')
        self.declare_parameter('pub_topic', '/cmd_vel')
        self.declare_parameter('v_fwd_max', 2.5)
        self.declare_parameter('v_bwd_max', -0.8)
        self.declare_parameter('differential_drive', False)
        self.declare_parameter('wheelbase', 0.38)
        self.declare_parameter('steer_rad_lim', 0.3631)
        self.declare_parameter('w_fwd_max', 1.0)
        self.declare_parameter('w_bwd_max', 1.0)

        # 파라미터 값 읽기
        self.sub_topic = self.get_parameter('sub_topic').get_parameter_value().string_value
        self.pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
        self.v_fwd_max = self.get_parameter('v_fwd_max').get_parameter_value().double_value
        self.v_bwd_max = self.get_parameter('v_bwd_max').get_parameter_value().double_value
        self.differential_drive = self.get_parameter('differential_drive').get_parameter_value().bool_value
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.steer_rad_lim = self.get_parameter('steer_rad_lim').get_parameter_value().double_value
        self.w_fwd_max = self.get_parameter('w_fwd_max').get_parameter_value().double_value
        self.w_bwd_max = self.get_parameter('w_bwd_max').get_parameter_value().double_value

        # Differential drive 경우 w_fwd_max, w_bwd_max 계산
        if not self.differential_drive:
            self.w_fwd_max = self.v_fwd_max * math.tan(self.steer_rad_lim) / self.wheelbase
            self.w_bwd_max = abs(self.v_bwd_max * math.tan(self.steer_rad_lim) / self.wheelbase)

        # 퍼블리셔와 구독자 설정
        self.publisher = self.create_publisher(Twist, self.pub_topic, 10)
        self.subscription = self.create_subscription(
            Twist,
            self.sub_topic,
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info(f'CmdVelLim node started. Publishing to: {self.pub_topic}')

    def cmd_vel_callback(self, msg):
        scaled_msg = Twist()

        # 선속도, 각속도 스케일링
        if msg.linear.x >= 0:
            scaled_msg.linear.x = msg.linear.x * self.v_fwd_max
            scaled_msg.angular.z = msg.angular.z * self.w_fwd_max
        else:
            scaled_msg.linear.x = abs(msg.linear.x) * self.v_bwd_max
            scaled_msg.angular.z = msg.angular.z * self.w_bwd_max

        # 퍼블리시 결과
        self.publisher.publish(scaled_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelLim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
