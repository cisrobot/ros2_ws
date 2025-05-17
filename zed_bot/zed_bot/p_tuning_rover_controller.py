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
from nav_msgs.msg import Odometry
import datetime
import time
import csv


class PTuningRoverController(Node):

    def __init__(self):
        super().__init__('p_tuning_rover_controller')
        self.declare_parameter("vel_group", 0)
        self.declare_parameter("log_enable", False)
        # self.declare_parameter("ard_enable", True)
        self.declare_parameter("ard_enable", False)
        self.declare_parameter("xbee_teleop_mode", False)
        self.declare_parameter("ard_port", '/dev/arduino')
        self.declare_parameter("odom_topic", '/odom')
        self.declare_parameter("log_period_sec", 0.1)
        self.declare_parameter("rcv_period_sec", 0.03)
        self.declare_parameter("control_period_sec", 0.05)
        self.declare_parameter('wheelbase', 0.38)
        self.declare_parameter('steer_rad_lim', 0.3631)
        self.declare_parameter("steer_pwm_center", 1477)
        self.declare_parameter("steer_pwm_gap", 300)
        self.declare_parameter("control_cap_ratio", 20)
        self.declare_parameter("v_pwm_stop", 1530)
        self.declare_parameter("v_pwm_min", 1561)
        self.declare_parameter("v_pwm_max", 1588)
        self.declare_parameter("v_pwm_bwd_min", 1500)
        self.declare_parameter("v_pwm_bwd_max", 1460)
        self.declare_parameter("v_min", 0.5)    # m/s
        self.declare_parameter("v_max", 2.5)    # m/s
        self.declare_parameter("v_bwd_min", -0.2)    # m/s
        self.declare_parameter("v_bwd_max", -0.8)    # m/s
        self.declare_parameter("w_min", 0.05)    # rad/s
        self.declare_parameter("p_gain", 1.2)
        self.declare_parameter("d_gain", 0.15)

        self.declare_parameter("gain_tuning_test", False)

        # self.declare_parameter("d_gain", 0.0)
        vel_group = self.get_parameter("vel_group").get_parameter_value().integer_value
        self.vel_list = []
        if vel_group == 0:
            self.vel_list = [0.5, 1.0, 1.5]
        elif vel_group == 1:
            self.vel_list = [2.0, 2.5, 2.5]
        elif vel_group == 2:
            self.vel_list = [0.5, 1.5, 2.5]
        
        self.log_enable = self.get_parameter("log_enable").get_parameter_value().bool_value
        self.ard_enable = self.get_parameter("ard_enable").get_parameter_value().bool_value
        self.xbee_teleop_mode = self.get_parameter("xbee_teleop_mode").get_parameter_value().bool_value
        self.ard_port = self.get_parameter("ard_port").get_parameter_value().string_value
        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.log_period_sec = self.get_parameter("log_period_sec").get_parameter_value().double_value
        self.rcv_period_sec = self.get_parameter("rcv_period_sec").get_parameter_value().double_value
        self.control_period_sec = self.get_parameter("control_period_sec").get_parameter_value().double_value
        self.wheelbase = self.get_parameter("wheelbase").get_parameter_value().double_value
        self.steer_rad_lim = self.get_parameter("steer_rad_lim").get_parameter_value().double_value
        self.steer_pwm_center = self.get_parameter("steer_pwm_center").get_parameter_value().integer_value
        self.steer_pwm_gap = self.get_parameter("steer_pwm_gap").get_parameter_value().integer_value
        self.control_cap_ratio = self.get_parameter("control_cap_ratio").get_parameter_value().integer_value
        self.steer_pwm_min = self.steer_pwm_center - self.steer_pwm_gap #1177
        self.steer_pwm_max = self.steer_pwm_center + self.steer_pwm_gap #1777
        self.v_pwm_stop = self.get_parameter("v_pwm_stop").get_parameter_value().integer_value
        self.v_pwm_min = self.get_parameter("v_pwm_min").get_parameter_value().integer_value
        self.v_pwm_max = self.get_parameter("v_pwm_max").get_parameter_value().integer_value
        self.v_pwm_bwd_min = self.get_parameter("v_pwm_bwd_min").get_parameter_value().integer_value        
        self.v_pwm_bwd_max = self.get_parameter("v_pwm_bwd_max").get_parameter_value().integer_value        
        
        self.v_min = self.get_parameter("v_min").get_parameter_value().double_value
        self.v_max = self.get_parameter("v_max").get_parameter_value().double_value
        self.v_bwd_min = self.get_parameter("v_bwd_min").get_parameter_value().double_value
        self.v_bwd_max = self.get_parameter("v_bwd_max").get_parameter_value().double_value
        self.w_min = self.get_parameter("w_min").get_parameter_value().double_value
        self.p_gain = self.get_parameter("p_gain").get_parameter_value().double_value
        self.d_gain = self.get_parameter("d_gain").get_parameter_value().double_value

        self.ard_throttle_pwm = self.v_pwm_stop
        self.ard_steer_pwm = self.steer_pwm_center
        self.steer_norm_scale = self.steer_rad_lim / self.steer_pwm_gap
        self.vel_fwd_norm_scale = (self.v_max - self.v_min) / (self.v_pwm_max - self.v_pwm_min)
        self.vel_bwd_norm_scale = (self.v_bwd_min - self.v_bwd_max) / (self.v_pwm_bwd_min - self.v_pwm_bwd_max)


        self.w_max = self.v_max * np.tan(self.steer_rad_lim) / self.wheelbase
        self.w_bwd_max = self.v_bwd_max * np.tan(self.steer_rad_lim) / self.wheelbase   # negative!!!


        self.control_pwm_cap = abs(int((self.v_pwm_max - self.v_pwm_min) / self.control_cap_ratio))
        self.control_pwm_cap = 0
        self.control_pwm_bwd_cap = abs(int((self.v_pwm_bwd_min - self.v_pwm_bwd_max) / self.control_cap_ratio))
        # self.get_logger().info(f"Fwd cap: {self.control_pwm_cap}, bwd cap: {self.control_pwm_bwd_cap}, w_max: {self.w_max}")

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
                                'v_curr',
                                'error',
                                'v_control',
                                'w_curr',
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
        self.w_curr = 0.0
        self.w_ref = 0.0
        self.u = 0.0
        self.a = 0.8
        self.ang = 0.0
        self.steer_rad = 0.0
        self.prev_v_err = 0.0

        if self.ard_enable:
            try:
                self.ser = serial.Serial(self.ard_port, self.baud, timeout=1)
                self.get_logger().info(f"Serial port {self.ard_port} opened.")

                # flush input buffer and wait for Arduino to boot up
                self.ser.flushInput()
                time.sleep(2.0)  # ESP32가 부팅될 시간 필요

                # Arduino에서 데이터가 도착할 때까지 기다림
                max_wait_time = 5  # 최대 5초 기다림
                waited = 0
                while self.ser.in_waiting == 0 and waited < max_wait_time:
                    self.get_logger().info("Waiting for Arduino data...")
                    time.sleep(0.1)
                    waited += 0.1

                if self.ser.in_waiting == 0:
                    self.get_logger().warn("Arduino did not send any data. Check connection.")
                else:
                    self.get_logger().info("Arduino data detected. Communication synced.")

            except serial.SerialException as e:
                self.get_logger().error(f"Failed to open serial port: {e}")
                self.ard_enable = False


        self.terminate = False
        self.rcv_struct_format = '3s3i'  # 수신할 데이터의 패킷 형식
        self.rcv_struct_size = struct.calcsize(self.rcv_struct_format)  # 수신할 데이터의 패킷 크기
        self.btn = 0  # 버튼 상태
        self.prev_btn = 0

        self.snd_struct_format = '3s3i'  # 송신할 데이터의 패킷 형식
        self.snd_struct_size = struct.calcsize(self.snd_struct_format)  # 송신할 데이터의 패킷 크기
        self.snd_header = b'JOY'  # 송신 데이터 헤더
        self.pwm_data = [self.steer_pwm_center, self.v_pwm_stop, 0] # steer, velocity, etc
        self.v_to_pwm_scale = self.v_pwm_max - self.v_pwm_min
        self.ang_to_pwm_scale = self.steer_pwm_gap

        self.wait_time = 5 # wait_time 초 동안 v_pwm_stop을 줘서 ESC를 준비시킴
        self.wait_thres = int(self.wait_time / self.control_period_sec)

        self.rover_vel = Twist()
        self.rover_vel.linear.x = 0.0
        self.rover_vel.linear.y = 0.0
        self.rover_vel.linear.z = 0.0
        self.rover_vel.angular.x = 0.0
        self.rover_vel.angular.y = 0.0
        self.rover_vel.angular.z = 0.0
        self.rover_cmd_vel_pub = self.create_publisher(Twist, '/rover_cmd_vel', 10) 
        self.v_show_flag = False


        if self.xbee_teleop_mode:   # Get cmd vel ref from xbee
            timer_period = 0.1  # 0.1초 주기 = 10Hz
            self.xbee_vel_timer = self.create_timer(timer_period, self.xbee_vel_callback)
        else:
            self.vel_sub = self.create_subscription(
                Twist, 
                '/cmd_vel', 
                self.vel_sub_callback, 
                10)
            

        self.ard_received = False
        self.log_timer = self.create_timer(self.log_period_sec, self.log_callback)
        self.rcv_timer = self.create_timer(self.rcv_period_sec, self.rcv_ard)
        self.snd_timer = self.create_timer(self.control_period_sec, self.control_loop)


        # signal.signal(signal.SIGINT, self.shutdown_handler)
        rclpy.get_default_context().on_shutdown(self.shutdown_handler)

        # 오도메트리로부터 현재속도를 구함 
        self.curr_vel_subscriber = self.create_subscription(
            Odometry,
            self.odom_topic,  # 오도메트리 토픽 이름 (필요시 수정 가능)
            self.odom_vel_callback,
            10
        )


    def odom_vel_callback(self, msg): 
    # 구독한 linear.x 값을 로그로 출력
        self.v_curr = msg.twist.twist.linear.x
        self.w_curr = msg.twist.twist.angular.z        
        # self.get_logger().info(f'Subscribed linear.x: {round(self.v_curr,3)}, angular_z = {round(self.w_curr,3)}')
        # self.rover_vel.linear.x = self.v_ref
        # self.rover_vel.angular.z = self.w_ref
        # self.rover_cmd_vel_pub.publish(self.rover_vel)

    
    def xbee_vel_callback(self):
        # self.get_logger().info(f"(steer_ang = {steer_ang:.2f})")
        self.v_ref = 0.0
        self.w_ref = 0.0
        if self.btn == 3:
            self.v_ref = self.vel_list[0]            
        elif self.btn == 2:
            self.v_ref = self.vel_list[1]
        elif self.btn == 4:
            self.v_ref = self.vel_list[2]
            
        if self.btn != self.prev_btn:
            self.prev_btn = self.btn
            self.v_show_flag = False
            
        if self.v_show_flag is False:
            self.get_logger().info(f'Set linear.x: {self.v_ref}')
            self.v_show_flag = True
            
        self.rover_vel.linear.x = self.v_ref
        self.rover_vel.angular.z = self.w_ref
        self.rover_cmd_vel_pub.publish(self.rover_vel)


    def shutdown_handler(self):
        self.get_logger().info("ROS shutdown signal received. Stopping robot and closing serial...")
        self.robot_run = False
        self.pwm_data[1] = self.v_pwm_stop
        # 시리얼 포트 닫기
        if self.ard_enable and hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")


    def vel_sub_callback(self, msg): #v_ref, angular.z 키보드로 할당        
        self.v_ref = np.clip(msg.linear.x, self.v_bwd_max, self.v_max)
        if self.v_ref > 0:
            self.w_ref = np.clip(msg.angular.z, -self.w_max, self.w_max)
        else:
            self.w_ref = np.clip(msg.angular.z, self.w_bwd_max, -self.w_bwd_max)
        # self.get_logger().info(f'Ref V: {round(self.v_ref,3)}, W: {round(self.w_ref,3)}')
        # 수신한 속도에 제한을 걸어서 다시 퍼블리쉬        
        self.rover_vel.linear.x = self.v_ref
        self.rover_vel.angular.z = self.w_ref
        self.rover_cmd_vel_pub.publish(self.rover_vel)


    # 서보모터 및 DC모터 데이터를 아두이노로 송신하는 함수
    def control_loop(self):
        # self.get_logger().info("%d" % (self.pwm_data[1]))        
        # self.get_logger().info(f"(v = {self.v_ref:.2f}, w = {self.w_ref:.2f}, v_ctl =  {self.v_ctl:.2f}, steer = {self.steer_rad:.2f}")
        if self.btn == 1 or (self.v_bwd_min < self.v_ref < self.v_min) or \
                (self.v_ref <= self.v_bwd_min and self.v_curr > self.v_min) or \
                (self.v_ref >= self.v_min and self.v_curr < self.v_bwd_min):
            self.pwm_data[0] = self.steer_pwm_center
            self.pwm_data[1] = self.v_pwm_stop
            self.v_ref = 0.0
            self.w_ref = 0.0
            self.v_ctl = 0.0
            self.prev_v_err = 0.0
            self.steer_rad = 0.0
        else:
            if abs(self.w_ref) < self.w_min:
                self.w_ref = 0.0
            self.v_err = self.v_ref - self.v_curr
            self.dv_err = (self.v_err - self.prev_v_err) / self.control_period_sec
            self.u = self.p_gain * self.v_err + self.d_gain * self.dv_err
            if self.v_ref > 0:
                self.v_ctl = self.v_ref + self.u
                self.pwm_ref = self.convert_v_to_fwd_pwm(self.v_ref)
                self.pwm_data[1] = self.convert_v_to_fwd_pwm(self.v_ctl)
                # self.pwm_data[1] = np.clip(self.pwm_data[1], self.pwm_ref - self.control_pwm_cap, self.pwm_ref + self.control_pwm_cap)
                # self.pwm_data[1] = np.clip(self.pwm_data[1], self.v_pwm_min, self.v_pwm_max)
            else:
                self.v_ctl = self.v_ref + self.u
                self.pwm_ref = self.convert_v_to_bwd_pwm(self.v_ref)
                self.pwm_data[1] = self.convert_v_to_bwd_pwm(self.v_ctl)
                self.pwm_data[1] = np.clip(self.pwm_data[1], self.pwm_ref - self.control_pwm_bwd_cap, self.pwm_ref + self.control_pwm_bwd_cap)
                self.pwm_data[1] = np.clip(self.pwm_data[1], self.v_pwm_bwd_max, self.v_pwm_bwd_min)
            self.prev_v_err = self.v_err

            self.steer_rad = np.arctan(self.w_ref * self.wheelbase / self.v_ref)
            self.steer_rad = np.clip(self.steer_rad, -self.steer_rad_lim, self.steer_rad_lim)            
            self.pwm_data[0] = self.convert_ang_to_pwm(self.steer_rad)
            self.get_logger().info(f"(v_ref = {self.v_ref:.2f}, w_ref = {self.w_ref:.2f}, v_ctl =  {self.v_ctl:.2f}, steer = {self.steer_rad:.2f}")

        if self.ard_enable:            
            # self.get_logger().info(f'Send to ARD: v = {self.v_ctl}, ang = {self.steer_rad}')        
            send_data = struct.pack(self.snd_struct_format, self.snd_header, *self.pwm_data)
            self.ser.write(send_data)        

        # self.get_logger().info(f"(v = {self.v_ref:.2f}, w = {self.w_ref:.2f}, v_ctl = {self.v_ctl:.2f}, steer = {self.steer_rad:.2f}, v_pwm = {self.pwm_data[1]}, steer_pwm = {self.pwm_data[0]}")
        


    def convert_v_to_fwd_pwm(self, vel):
        f_v = 0.1481 * vel**3 - 0.7407 * vel**2 + 1.4444 * vel - 0.5556
        ctl_v_pwm = self.v_to_pwm_scale * f_v + self.v_pwm_min        
        # self.get_logger().info("control vel = %f, (%f)" % (clamp_ctl_v_pwm, ctl_v_pwm))
        return int(ctl_v_pwm)
    
    def convert_v_to_bwd_pwm(self, vel):
        ctl_v_pwm = 290.5 * vel**3 + 315.7 * vel**2 + 138.2 * vel + 1519        
        # self.get_logger().info("control vel = %f, (%f)" % (clamp_ctl_v_pwm, ctl_v_pwm))
        return int(ctl_v_pwm)


    def convert_ang_to_pwm(self, ang):
        f_ang = 0.9792*ang**3 +2.6250*ang
        ctl_steer_pwm = self.ang_to_pwm_scale * f_ang + self.steer_pwm_center
        ctl_steer_pwm = np.clip(ctl_steer_pwm, self.steer_pwm_min, self.steer_pwm_max)
        # self.get_logger().info("control steer = %f, %f" % (ctl_steer_pwm, clamp_ctl_steer_pwm))
        return int(ctl_steer_pwm)


    def rcv_ard(self):
        if not self.ard_enable:
            return

        # 수신 가능한 바이트 수 확인
        if self.ser.in_waiting >= self.rcv_struct_size:
            buffer = self.ser.read(self.ser.in_waiting)
            len_buffer = len(buffer)

            if len_buffer >= self.rcv_struct_size:
                strt_idx = -1
                for idx in range(len_buffer - self.rcv_struct_size + 1):
                    if buffer[idx] == ord('A') and buffer[idx+1] == ord('R') and buffer[idx+2] == ord('D'):
                        strt_idx = idx
                        break
                if strt_idx == -1:
                    self.get_logger().info('Fail detecting Arduino msg {}'.format(buffer))
                else:
                    rcv_struct = buffer[strt_idx:strt_idx + self.rcv_struct_size]
                    self.ard_received = False
                    try:
                        rcv_data = struct.unpack(self.rcv_struct_format, rcv_struct)
                        self.btn = rcv_data[-1]
                        self.ard_throttle_pwm = rcv_data[-2]
                        self.ard_steer_pwm = rcv_data[-3]
                        self.ard_received = True
                        # self.get_logger().info(f"Rcv from Arduino steer({self.ard_steer_pwm}), throttle({self.ard_throttle_pwm}), btn({self.btn})")
                    except struct.error as e:
                        self.get_logger().warn(f"Corrupted packet or size mismatch: {e}")
                        self.get_logger().debug(f"rcv_struct: {rcv_struct.hex()}")
            else:
                self.get_logger().info(f"Not enough bytes after read: {len_buffer}")
        else:
            self.get_logger().info(f"Too small data = {self.ser.in_waiting}")


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
                                        round(self.v_curr, 3),
                                        round(self.v_err, 3),
                                        round((self.v_ctl), 1),
                                        round((self.w_curr),3),
                                        round((self.w_ref),3),
                                        round((self.steer_rad),3),
                                        round((self.u),3),
                                        self.btn
                                    ])  # CSV 파일 헤더 작성

def main(args=None):
    rclpy.init(args=args)
    arduino_serial_control = PTuningRoverController()
    rclpy.spin(arduino_serial_control)
    arduino_serial_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 