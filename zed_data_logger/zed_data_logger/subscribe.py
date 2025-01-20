import rclpy
import numpy as np
import math
import serial
import struct
import signal
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist
import datetime
import csv


class ArduinoSerialControl(Node):

    def __init__(self):
        super().__init__('arduino_serial_control')
        self.declare_parameter("log_enable", True)
        self.declare_parameter("ard_enable", True)
        self.declare_parameter("ard_port", '/dev/ttyACM0')
        self.declare_parameter("log_period_sec", 0.1)
        self.declare_parameter("rcv_period_sec", 0.03)
        self.declare_parameter("control_period_sec", 0.03)
        self.declare_parameter('steer_rad_lim', 0.4)
        self.declare_parameter("steer_pwm_center", 1477)
        self.declare_parameter("steer_pwm_gap", 300)
        self.declare_parameter("v_pwm_stop", 1530)
        self.declare_parameter("v_pwm_min", 1561)
        self.declare_parameter("v_pwm_max", 1589)
        self.declare_parameter("v_min", 0.5)    # m/s
        self.declare_parameter("v_max", 2.5)    # m/s
        self.declare_parameter("v_step", 0.5)    # m/s
        self.declare_parameter("p_gain", 1.0)
        self.declare_parameter("d_gain", 0.15)

        self.declare_parameter("gain_tuning_test", False)

        # self.declare_parameter("d_gain", 0.0)
        self.log_enable = self.get_parameter("log_enable").get_parameter_value().bool_value
        self.ard_enable = self.get_parameter("ard_enable").get_parameter_value().bool_value
        self.ard_port = self.get_parameter("ard_port").get_parameter_value().string_value
        self.log_period_sec = self.get_parameter("log_period_sec").get_parameter_value().double_value
        self.rcv_period_sec = self.get_parameter("rcv_period_sec").get_parameter_value().double_value
        self.control_period_sec = self.get_parameter("control_period_sec").get_parameter_value().double_value
        self.steer_rad_lim = self.get_parameter("steer_rad_lim").get_parameter_value().double_value
        self.steer_pwm_center = self.get_parameter("steer_pwm_center").get_parameter_value().integer_value
        self.steer_pwm_gap = self.get_parameter("steer_pwm_gap").get_parameter_value().integer_value
        self.steer_pwm_min = self.steer_pwm_center - self.steer_pwm_gap #1186
        self.steer_pwm_max = self.steer_pwm_center + self.steer_pwm_gap #1786
        self.v_pwm_stop = self.get_parameter("v_pwm_stop").get_parameter_value().integer_value
        self.v_pwm_min = self.get_parameter("v_pwm_min").get_parameter_value().integer_value
        self.v_pwm_max = self.get_parameter("v_pwm_max").get_parameter_value().integer_value
        
        self.v_min = self.get_parameter("v_min").get_parameter_value().double_value
        self.v_max = self.get_parameter("v_max").get_parameter_value().double_value
        self.v_step = self.get_parameter("v_step").get_parameter_value().double_value
        self.p_gain = self.get_parameter("p_gain").get_parameter_value().double_value
        self.d_gain = self.get_parameter("d_gain").get_parameter_value().double_value

        now = datetime.datetime.now()
        self.log_filename = "/home/twins/ros2_ws/src/data/arduino_log" + now.strftime("_%m_%d_%H_%M_%S")
        if self.log_enable:
            self.get_logger().info("Enable data recording")
            with open(self.log_filename + '.csv', 'w', newline='') as csv_file:
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow([
                                'ard_steer_pwm',
                                'ard_throttle_pwm',
                                'v_ref',
                                'v_call',
                                'v_curr',
                                'error',
                                'v_control',
                                'w_curr',
                                'w_call',
                                'w_ref',
                                'steering_angle',
                                'u',
                                'btn',
                            ])  # CSV 파일 헤더 작성

            # self.csv_file = open(self.log_filename + '.csv', 'w', newline='')
            # self.csv_writer = csv.writer(self.csv_file)
            # self.csv_writer.writerow([
            #                         'ard_steer_pwm',
            #                         'ard_throttle_pwm',
            #                         'v_curr',
            #                         'btn',
            #                         ])  # CSV 파일 헤더 작성

        self.baud = 57600  # 통신 속도 (보드레이트)

        self.robot_run = False
        self.v_ref = 0.0
        self.v_curr = 0.0
        self.v_err = 0.0
        self.v_ctl = 0.0
        self.v_call = 0.0
        self.w_curr = 0.0
        self.w_ref = 0.0
        self.w_call = 0.0
        self.u=0.0
        self.a =0.8
        self.length = 0.38
        self.radius = 1.0
        self.ang =0.0
        self.steer_ang = 0.0
        self.prev_v_err = 0.0

        if self.ard_enable:
            self.ser = serial.Serial(self.ard_port, self.baud, timeout=1)  # 시리얼 통신 인스턴스 생성

        self.terminate = False
        self.rcv_struct_format = '3s3i'  # 수신할 데이터의 패킷 형식
        self.rcv_struct_size = struct.calcsize(self.rcv_struct_format)  # 수신할 데이터의 패킷 크기
        self.btn = 0  # 버튼 상태

        self.snd_struct_format = '3s3i'  # 송신할 데이터의 패킷 형식
        self.snd_struct_size = struct.calcsize(self.snd_struct_format)  # 송신할 데이터의 패킷 크기
        self.snd_header = b'JOY'  # 송신 데이터 헤더
        self.pwm_data = [self.steer_pwm_center, self.v_pwm_stop, 0] # steer, velocity, etc
        self.v_to_pwm_scale = self.v_pwm_max - self.v_pwm_min
        self.ang_to_pwm_scale = self.steer_pwm_gap

        self.wait_time = 5 # wait_time 초 동안 v_pwm_stop을 줘서 ESC를 준비시킴
        self.wait_thres = int(self.wait_time / self.control_period_sec)

        self.vel_sub = self.create_subscription(
          Twist, 
         '/cmd_vel', 
         self.vel_sub_callback, 
         10)

        self.ard_received = False
        self.log_timer = self.create_timer(self.log_period_sec, self.log_callback)
        self.rcv_timer = self.create_timer(self.rcv_period_sec, self.rcv_ard)
        self.snd_timer = self.create_timer(self.control_period_sec, self.control_loop)


        signal.signal(signal.SIGINT, self.shutdown_handler)

        # 카메라 값을 구독하는 추가 구독자 생성 zed 카메라로 측정한 속도 
        self.curr_vel_subscriber = self.create_subscription(
            Twist,
            '/current_vel',  # 'zed_odom' 토픽 구독
            self.current_vel_callback,
            10
        )


    def current_vel_callback(self, msg): 
    # 구독한 linear.x 값을 로그로 출력
        self.v_curr = msg.linear.x
        self.w_curr = msg.angular.z
        #self.get_logger().info(f'Subscribed linear.x: {round(self.v_curr,3)}, angular_z = {round(self.w_curr,3)}')
    

    def shutdown_handler(self, signum, frame):
        self.get_logger().info("Ctrl + c pressed, stop the robot")
        self.robot_run = False
        self.terminate = True
        self.pwm_data[1] = self.v_pwm_stop


    def vel_sub_callback(self, msg): #v_ref, angular.z 키보드로 할당
        self.v_ref = msg.linear.x
        self.w_ref= msg.angular.z
        #self.get_logger().info(f'V: {round(self.v_ref,3)}, W: {round(self.w_ref,3)}')


    # 서보모터 및 DC모터 데이터를 아두이노로 송신하는 함수
    def control_loop(self):
        # self.get_logger().info("%d" % (self.pwm_data[1]))        
        #self.pwm_data[0] = self.convert_ang_to_pwm(self.steer_ang)
        #self.pwm_data[1] = self.convert_v_to_pwm(self.v_ref)

        if self.v_ref < self.v_min:
            self.pwm_data[1] = self.v_pwm_stop
        else:
            self.v_curr = max(self.v_curr, 0)            
            self.v_call = self.a*self.v_call +(1-self.a)*self.v_curr #lowpass 필터 '''            
            self.v_err = self.v_ref - self.v_call
            self.dv_err = (self.v_err - self.prev_v_err) / self.control_period_sec
            self.u =self.p_gain*self.v_err+self.d_gain*self.dv_err

            #self.u = min(self.u_clamp, max(self.u, -self.u_clamp))
            #v_pd = self.p_gain * v_err + self.d_gain * dv_err
            # v_pd = min(max(-self.v_step, v_pd), self.v_step)    #갑작스런 큰 가감속을 억제하기 위함
            # v_ctl = self.v_ref + v_pd

            # self.get_logger().info("v_ref, v_curr, v_ctl, err, derr = %f, %f, %f, %f" % (self.v_ref, self.v_curr, v_ctl, dv_err))
            #self.v_ctl = self.v_ref + min(max(-self.ua,self.u),self.ua) 오버슈트 방지
            self.v_ctl = self.v_ref+self.u
            self.pwm_data[1] = self.convert_v_to_pwm(self.v_ctl)
            self.prev_v_err = self.v_err
            # self.get_logger().info("v_ref(%.2f), v_curr(%.2f), v_ctl(%.2f), PWM(%d)" % (self.v_ref, self.v_curr, v_ctl, self.pwm_data[1]))
        
        if self.v_ref == 0:
            self.pwm_data[0] = self.steer_pwm_center
        else:
            self.steer_ang = np.arctan2(self.w_ref*self.length,self.v_call)
            if self.steer_ang > self.steer_rad_lim :
                self.steer_ang = self.steer_rad_lim
            elif self.steer_ang <-self.steer_rad_lim :
                self.steer_ang= -self.steer_rad_lim
            self.w_call = self.a*self.w_call+(1-self.a)*self.w_curr
            self.pwm_data[0] =self.convert_ang_to_pwm(self.steer_ang)
        
        self.get_logger().info(f'linear.x: {round(self.v_call,2)}, angular_z = {round(self.w_call,2)}')

        

        if self.btn != 1 and self.ard_enable:
            send_data = struct.pack(self.snd_struct_format, self.snd_header, *self.pwm_data)
            self.ser.write(send_data)
        if self.terminate:
            self.destroy_node()
            rclpy.shutdown()

    def convert_v_to_pwm(self, vel):
        f_v = -0.68 + 1.76 * vel -0.98 * (vel**2) + 0.22 * (vel**3)
        ctl_v_pwm = self.v_to_pwm_scale * f_v + self.v_pwm_min
        clamp_ctl_v_pwm = min(max(self.v_pwm_min, ctl_v_pwm), self.v_pwm_max)
        # self.get_logger().info("control vel = %f, (%f)" % (clamp_ctl_v_pwm, ctl_v_pwm))
        return int(clamp_ctl_v_pwm)


    def convert_ang_to_pwm(self, ang):
        f_ang = 2.49 * ang
        ctl_steer_pwm = self.ang_to_pwm_scale * f_ang + self.steer_pwm_center
        clamp_ctl_steer_pwm = min(max(self.steer_pwm_min, ctl_steer_pwm), self.steer_pwm_max)
        # self.get_logger().info("control steer = %f, %f" % (ctl_steer_pwm, clamp_ctl_steer_pwm))
        return int(clamp_ctl_steer_pwm)


    # 아두이노에서 데이터를 수신하는 함수
    def rcv_ard(self):
        if self.ser.readable():
            buffer = self.ser.read_all()
            len_buffer = len(buffer)

            # self.get_logger().info("buffer is = %d, must be %d" % (len_buffer, self.rcv_struct_size))
            if len_buffer >= self.rcv_struct_size:
                strt_idx = -1
                for idx in range(len_buffer - self.rcv_struct_size + 1):
                    if buffer[idx] == ord('A') and buffer[idx+1] == ord('R') and buffer[idx+2] == ord('D'):
                        strt_idx = idx
                        break
                if strt_idx == -1:
                    self.get_logger().info('Fail detecting Arduino msg {}'.format(buffer[idx]))
                else:
                    rcv_struct = buffer[strt_idx:strt_idx + self.rcv_struct_size]
                    rcv_data = struct.unpack(self.rcv_struct_format, rcv_struct)
                    self.ard_received = False
                    try:
                        rcv_data = struct.unpack(self.rcv_struct_format, rcv_struct)
                        self.btn = rcv_data[-1]
                        self.ard_throttle_pwm = rcv_data[-2]
                        self.ard_steer_pwm = rcv_data[-3]
                        self.ard_received = True
                        # self.get_logger().info("Rcv from Arduino steer(%d), throttle(%d), btn(%d)" % (self.ard_steer_pwm, self.ard_vel_pwm, self.btn))
                    except:
                        # self.get_logger().info('Corrupted!')
                        pass

    def log_callback(self):
        if self.ard_received and self.log_enable:
            self.ard_received = False
            #self.get_logger().info("Steer(%d), throttle(%d), btn(%d)" % (self.ard_steer_pwm, self.ard_throttle_pwm, self.btn))
            with open(self.log_filename + '.csv', 'a', newline='') as csv_file:
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow([
                                        self.ard_steer_pwm,
                                        self.ard_throttle_pwm,
                                        round(self.v_ref,3),
                                        round(self.v_call, 3),
                                        round(self.v_curr, 3),
                                        round(self.v_err, 3),
                                        round((self.v_ctl), 1),
                                        round((self.w_curr),3),
                                        round((self.w_call),3),
                                        round((self.w_ref),3),
                                        round((self.steer_ang),3),
                                        round((self.u),3),
                                        self.btn
                                    ])  # CSV 파일 헤더 작성

def main(args=None):
    rclpy.init(args=args)
    arduino_serial_control = ArduinoSerialControl()
    rclpy.spin(arduino_serial_control)
    arduino_serial_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 