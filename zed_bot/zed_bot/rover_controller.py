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


class RoverController(Node):

    def __init__(self):
        super().__init__('rover_controller')
        # self.declare_parameter("ard_enable", True)
        self.declare_parameter("ard_enable", False)
        self.declare_parameter("ard_port", '/dev/arduino')
        self.declare_parameter("odom_topic", '/odom')
        self.declare_parameter("vel_alpha", 0.7)
        self.declare_parameter("rcv_period_sec", 0.02)
        self.declare_parameter("control_period_sec", 0.05)
        self.declare_parameter('wheelbase', 0.38)
        self.declare_parameter('steer_rad_lim', 0.3631)
        self.declare_parameter("steer_pwm_center", 1477)
        self.declare_parameter("steer_pwm_gap", 300)
        self.declare_parameter("control_cap_ratio", 20)
        self.declare_parameter("v_pwm_stop", 1530)
        self.declare_parameter("v_pwm_min", 1561)
        self.declare_parameter("v_pwm_max", 1585)
        self.declare_parameter("v_pwm_bwd_min", 1503)
        self.declare_parameter("v_pwm_bwd_max", 1460)
        self.declare_parameter("v_min", 0.4)    # m/s
        self.declare_parameter("v_max", 2.8)    # m/s
        self.declare_parameter("v_bwd_min", -0.2)    # m/s
        self.declare_parameter("v_bwd_max", -0.8)    # m/s
        self.declare_parameter("w_min", 0.05)    # rad/s
        self.declare_parameter("p_min", 0.1)
        self.declare_parameter("p_max", 3.5)
        self.declare_parameter("p_scaler", 5.0)
        self.declare_parameter("d_min", 0.05)
        self.declare_parameter("d_max", 0.35)
        self.declare_parameter("d_scaler", 5.0)
        

        self.declare_parameter("gain_tuning_test", False)

        # self.declare_parameter("d_gain", 0.0)
        self.ard_enable = self.get_parameter("ard_enable").get_parameter_value().bool_value
        self.ard_port = self.get_parameter("ard_port").get_parameter_value().string_value
        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.vel_alpha = self.get_parameter("vel_alpha").get_parameter_value().double_value
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
        self.p_min = self.get_parameter("p_min").get_parameter_value().double_value
        self.p_max = self.get_parameter("p_max").get_parameter_value().double_value
        self.p_scaler = self.get_parameter("p_scaler").get_parameter_value().double_value
        self.d_min = self.get_parameter("d_min").get_parameter_value().double_value
        self.d_max = self.get_parameter("d_max").get_parameter_value().double_value
        self.d_scaler = self.get_parameter("d_scaler").get_parameter_value().double_value
        self.p_gap = self.p_max - self.p_min
        self.d_gap = self.d_max - self.d_min

        self.v_gap = self.v_max - self.v_min
        
        self.steer_norm_scale = self.steer_rad_lim / self.steer_pwm_gap
        self.vel_fwd_norm_scale = (self.v_max - self.v_min) / (self.v_pwm_max - self.v_pwm_min)
        self.vel_bwd_norm_scale = (self.v_bwd_min - self.v_bwd_max) / (self.v_pwm_bwd_min - self.v_pwm_bwd_max)

        self.vel_beta = 1 - self.vel_alpha


        self.w_max = self.v_max * np.tan(self.steer_rad_lim) / self.wheelbase
        self.w_bwd_max = self.v_bwd_max * np.tan(self.steer_rad_lim) / self.wheelbase   # negative!!!


        self.control_pwm_cap = abs(int((self.v_pwm_max - self.v_pwm_min) / self.control_cap_ratio))
        self.control_pwm_bwd_cap = abs(int((self.v_pwm_bwd_min - self.v_pwm_bwd_max) / self.control_cap_ratio))
        # self.get_logger().info(f"Fwd cap: {self.control_pwm_cap}, bwd cap: {self.control_pwm_bwd_cap}, w_max: {self.w_max}")

        now = datetime.datetime.now()
        
        self.baud = 115200  # 통신 속도 (보드레이트)

        self.robot_run = False
        self.v_ref = 0.0
        self.v_curr = 0.0
        self.v_err = 0.0
        self.v_ctl = 0.0
        self.w_curr = 0.0
        self.w_ref = 0.0
        self.u_ctl = 0.0
        self.a = 0.8
        self.ang = 0.0
        self.steer_rad = 0.0
        self.prev_v_err = 0.0

        self.no_data = 0

        if self.ard_enable:
            try:
                self.ser = serial.Serial(self.ard_port, self.baud, timeout=1)
                self.get_logger().info(f"Serial port {self.ard_port} opened.")

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
        self.rcv_struct_format = '3s1i'  # 수신할 데이터의 패킷 형식
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

        self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.vel_sub_callback, 10)
            
        self.ard_received = False
        self.ard_last_recv_time = time.time()
        self.no_data_warn_interval = 2.0  # 2초 이상 수신 없으면 경고
        self.last_warn_time = 0.0

        self.rcv_timer = self.create_timer(self.rcv_period_sec, self.rcv_ard)
        self.snd_timer = self.create_timer(self.control_period_sec, self.control_loop)

        self.rcv_success = False
        
        # 반올림한 4차 다항 계수
        rounded_coeffs_vel_to_pwm = [0.91, -0.39, -1.71, 2.19, -0.01]
        self.poly_vel_to_pwm_rounded = np.poly1d(rounded_coeffs_vel_to_pwm)

        # signal.signal(signal.SIGINT, self.shutdown_handler)
        rclpy.get_default_context().on_shutdown(self.shutdown_handler)

        # 오도메트리로부터 현재속도를 구함 
        self.curr_vel_subscriber = self.create_subscription(
            Odometry,
            self.odom_topic,  # 오도메트리 토픽 이름 (필요시 수정 가능)
            self.odom_vel_callback,
            10
        )


    def rcv_ard(self):
        if not self.ard_enable:
            self.get_logger().warn("[rcv_ard] Arduino not enabled. Skipping.")
            return

        waiting = self.ser.in_waiting
        # self.get_logger().info(f"[rcv_ard] in_waiting = {waiting}")
        try:
            waiting = self.ser.in_waiting
        except OSError as e:
            self.get_logger().warn(f"Serial read error: {e}")
            return


        # 수신된 데이터가 없을 경우 주기적으로 경고 출력
        now = time.time()
        if waiting == 0:
            if now - self.ard_last_recv_time > self.no_data_warn_interval and now - self.last_warn_time > self.no_data_warn_interval:                
                if self.no_data < 5:
                    self.get_logger().warn("No data from Arduino for {:.1f} sec".format(now - self.ard_last_recv_time))
                    self.no_data += 1
                self.last_warn_time = now
            return

        self.no_data = 0
        # 슬라이딩 수신 방식
        buffer = self.ser.read(self.ser.in_waiting)
        len_buffer = len(buffer)
        # self.get_logger().info(f"[rcv_ard] Received raw buffer: {buffer.hex()}")

        if len_buffer >= self.rcv_struct_size:
            strt_idx = -1
            for idx in range(len_buffer - self.rcv_struct_size + 1):
                if buffer[idx:idx+3] == b'ARD':
                    strt_idx = idx
                    break

            if strt_idx != -1:
                rcv_struct = buffer[strt_idx:strt_idx + self.rcv_struct_size]
                try:
                    rcv_data = struct.unpack(self.rcv_struct_format, rcv_struct)
                    # self.btn = rcv_data[-1]                                        
                    self.btn = rcv_data[1]                                        
                    # self.ard_last_recv_time = time.time()                    
                    if 0 <= self.btn <= 4:
                        self.ard_received = True
                        # self.get_logger().info(f"[rcv_ard] Received button: {self.btn}")
                    else:
                        self.get_logger().info(f"[rcv_ard] Corrupted button: {self.btn}. Set to 0")
                        self.btn = 0
                    self.rcv_success = True

                except struct.error as e:
                    self.get_logger().warn(f"Corrupted Arduino packet: {e}")
            else:
                decoded = buffer.decode(errors='ignore').strip()
                self.get_logger().warn(f"[rcv_ard] No valid ARD header found in buffer {decoded}")                                

            self.ard_last_recv_time = time.time()
        elif len_buffer > 0:
            # self.get_logger().warn(f"[rcv_ard] Not enough data. Buffer size: {len_buffer}")
            self.get_logger().info(f"[rcv_ard2] {buffer.decode(errors='ignore')}")
            self.ard_last_recv_time = time.time()


    def odom_vel_callback(self, msg): 
    # 구독한 linear.x 값을 로그로 출력
        self.v_curr = msg.twist.twist.linear.x
        self.w_curr = msg.twist.twist.angular.z        
        # self.get_logger().info(f'Subscribed linear.x: {round(self.v_curr,3)}, angular_z = {round(self.w_curr,3)}')


    def odom_vel_callback(self, msg): 
        raw_v_curr = msg.twist.twist.linear.x
        raw_w_curr = msg.twist.twist.angular.z

        # 간단한 1차 LPF 적용 (Exponential Moving Average)
        self.v_curr = self.vel_alpha * self.v_curr + self.vel_beta * raw_v_curr
        self.w_curr = self.vel_alpha * self.w_curr + self.vel_beta * raw_w_curr


    
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


    # 서보모터 및 DC모터 데이터를 아두이노로 송신하는 함수
    def control_loop(self):
        # self.get_logger().info("%d" % (self.pwm_data[1]))                
        # if self.btn == 3:
        #     if self.v_ref != 0.5:
        #         self.v_ref = 0.5
        #         self.get_logger().info(f"Set v_ref = {self.v_ref:.2f}")            
        # elif self.btn == 2:
        #     if self.v_ref != 1.0:
        #         self.v_ref = 1.0
        #         self.get_logger().info(f"Set v_ref = {self.v_ref:.2f}")            
        # elif self.btn == 4:
        #     if self.v_ref != 1.5:
        #         self.v_ref = 1.5
        #         self.get_logger().info(f"Set v_ref = {self.v_ref:.2f}")   


        # if self.btn == 3:
        #     if self.v_ref != 2.0:
        #         self.v_ref = 2.0
        #         self.get_logger().info(f"Set v_ref = {self.v_ref:.2f}")       
        # elif self.btn == 2:
        #     if self.v_ref != 2.5:
        #         self.v_ref = 2.5
        #         self.get_logger().info(f"Set v_ref = {self.v_ref:.2f}")          


        # if self.btn == 3:
        #     self.v_ref = 1.0
        #     self.get_logger().info("Turn left with 1.0m/s")       
        # elif self.btn == 2:
        #     self.v_ref = 1.0
        #     self.get_logger().info("Turn right with 1.0m/s")               
        

        if 0 <= self.btn <= 1 or (self.v_bwd_min < self.v_ref < self.v_min) or \
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

            p_gain, d_gain = self.pd_gain_func(self.v_ref)
            
            self.u_ctl = p_gain * self.v_err + d_gain * self.dv_err
            self.v_ctl = self.v_ref + self.u_ctl            

            if self.v_ref > 0:            
                self.pwm_data[1] = self.convert_v_to_fwd_pwm(self.v_ctl)                
                self.pwm_data[1] = np.clip(self.pwm_data[1], self.v_pwm_min, self.v_pwm_max)
            else:
                self.pwm_data[1] = self.convert_v_to_bwd_pwm(self.v_ctl)
                self.pwm_data[1] = np.clip(self.pwm_data[1], self.v_pwm_bwd_max, self.v_pwm_bwd_min)
            self.prev_v_err = self.v_err

            self.steer_rad = np.arctan(self.w_ref * self.wheelbase / self.v_ref)
            self.steer_rad = np.clip(self.steer_rad, -self.steer_rad_lim, self.steer_rad_lim)            
            self.pwm_data[0] = self.convert_ang_to_pwm(self.steer_rad)
            # self.get_logger().info(f"(v_ref = {self.v_ref:.2f}, pwm = {self.pwm_data[1]}")
            # self.get_logger().info(f"(v_ref = {self.v_ref:.2f}, w_ref = {self.w_ref:.2f}, self.v_ctl =  {self.v_ctl:.2f}, steer = {self.steer_rad:.2f}")
            # self.get_logger().info(f"(ref:{self.v_ref:.2f}, ctl:{self.v_ctl:.2f}, ctl_raw:{v_ctl_raw:.2f}, pwm = {self.pwm_data[1]}, pwm_raw = {pwm_raw}")

            # if self.btn == 3:           
            #     if self.pwm_data[0] != self.steer_pwm_min:
            #         self.pwm_data[0] = self.steer_pwm_min
            #         self.get_logger().info("Turn right with 1.0m/s")    
            # elif self.btn == 2:
            #     if self.pwm_data[0] != self.steer_pwm_max:
            #         self.pwm_data[0] = self.steer_pwm_max
            #         self.get_logger().info("Turn left with 1.0m/s")    


        if self.ard_enable and self.ard_received:            
            self.ard_received = False
            # self.get_logger().info(f'Send to ARD: v = {self.v_ctl}, ang = {self.steer_rad}')        
            send_data = struct.pack(self.snd_struct_format, self.snd_header, *self.pwm_data)
            self.ser.write(send_data)        

        # self.get_logger().info(f"(v_ref = {self.v_ref:.2f}, w_ref = {self.w_ref:.2f}, self.v_ctl = {self.v_ctl:.2f}, steer = {self.steer_rad:.2f}, v_pwm = {self.pwm_data[1]}, steer_pwm = {self.pwm_data[0]}")
        
    # === vel_to_pwm 함수 정의 (소수 둘째자리 반올림 계수 사용) ===
    def normalize_vel(self, vel):
        vel = np.clip(vel, self.v_min, self.v_max)
        return (vel - self.v_min) / (self.v_max - self.v_min)

    def denormalize_pwm(self, norm_pwm):
        return norm_pwm * (self.v_pwm_max - self.v_pwm_min) + self.v_pwm_min


    def convert_v_to_fwd_pwm(self, vel_input):
        norm_vel_input = self.normalize_vel(vel_input)
        norm_vel_input = np.clip(norm_vel_input, 0.0, 1.0)
        norm_pwm_output = self.poly_vel_to_pwm_rounded(norm_vel_input)
        norm_pwm_output = np.clip(norm_pwm_output, 0.0, 1.0)
        pwm_output = self.denormalize_pwm(norm_pwm_output)
        return int(pwm_output)

    
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
    

    def pd_gain_func(self, v_ref):
        v_norm = (v_ref - self.v_min) / self.v_gap
        p_gain = self.p_gap * (1.0 - np.exp(-self.p_scaler * v_norm)) + self.p_min
        d_gain = self.d_gap * (1.0 - np.exp(-self.d_scaler * v_norm)) + self.d_min
        return p_gain, d_gain


def main(args=None):
    rclpy.init(args=args)
    arduino_serial_control = RoverController()
    rclpy.spin(arduino_serial_control)
    arduino_serial_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 