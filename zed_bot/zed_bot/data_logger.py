#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import csv
from tf_transformations import euler_from_quaternion
from datetime import datetime

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')

        self.declare_parameter("log_fwd_odom", False)
        self.declare_parameter("log_bwd_odom", False)
        self.declare_parameter("log_ekf_odom", True)
        self.log_fwd_odom = self.get_parameter("log_fwd_odom").get_parameter_value().bool_value
        self.log_bwd_odom = self.get_parameter("log_bwd_odom").get_parameter_value().bool_value
        self.log_ekf_odom = self.get_parameter("log_ekf_odom").get_parameter_value().bool_value

        self.csv_header = ['timestamp', 'x', 'y', 'yaw', 'linear_x', 'linear_y', 'angular_z', 'v_ref', 'w_ref']
     
        # timestamp for filenames
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        if self.log_fwd_odom:
            self.odom_fwd_csv = f'odom_fwd_data_{timestamp}.csv'
            self.init_timestep_fwd = None
            self.initialize_csv(self.odom_fwd_csv)
            self.create_subscription(Odometry, '/odom_fwd', self.odom_fwd_callback, 10)
        if self.log_bwd_odom:
            self.odom_bwd_csv = f'odom_bwd_data_{timestamp}.csv'
            self.init_timestep_bwd = None
            self.initialize_csv(self.odom_bwd_csv)
            self.create_subscription(Odometry, '/odom_bwd', self.odom_bwd_callback, 10)
        if self.log_ekf_odom:
            self.odom_ekf_csv = f'odom_ekf_data_{timestamp}.csv'
            self.init_timestep_ekf = None
            self.initialize_csv(self.odom_ekf_csv)
            self.create_subscription(Odometry, '/odom_ekf', self.odom_ekf_callback, 10)
                    
        
        # Velocity reference
        self.v_ref = 0.0
        self.w_ref = 0.0

        self.create_subscription(Twist, '/rover_cmd_vel', self.cmd_callback, 10)

        self.get_logger().info('DataLogger node started, subscribing to /odom_fwd, /odom_bwd, /odom_ekf, /rover_cmd_vel')

    def initialize_csv(self, file_path):
        with open(file_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(self.csv_header)

    def cmd_callback(self, msg):
        self.v_ref = msg.linear.x
        self.w_ref = msg.angular.z
        # self.get_logger().info(f'Updated cmd_vel: v_ref={self.v_ref:.2f}, w_ref={self.w_ref:.2f}')

    def odom_fwd_callback(self, msg):
        self.log_odometry(msg, self.odom_fwd_csv, "Odom FWD", 'init_timestep_fwd')

    def odom_bwd_callback(self, msg):
        self.log_odometry(msg, self.odom_bwd_csv, "Odom BWD", 'init_timestep_bwd')

    def odom_ekf_callback(self, msg):
        self.log_odometry(msg, self.odom_ekf_csv, "Odom EKF", 'init_timestep_ekf')

    def log_odometry(self, msg, file_path, label, time_attr):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        linear_x = msg.twist.twist.linear.x
        linear_y = msg.twist.twist.linear.y
        angular_z = msg.twist.twist.angular.z

        timestamp = self.get_clock().now().seconds_nanoseconds()
        time_sec = timestamp[0] + timestamp[1] * 1e-9

        if getattr(self, time_attr) is None:
            setattr(self, time_attr, time_sec)
        exp_time = time_sec - getattr(self, time_attr)

        data = [time_sec, x, y, yaw, linear_x, linear_y, angular_z, self.v_ref, self.w_ref]
        with open(file_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(data)

        # self.get_logger().info(f'{label} logged: t={exp_time:.2f}, x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}, v_ref={self.v_ref:.2f}, w_ref={self.w_ref:.2f}')

def main(args=None):
    rclpy.init(args=args)
    data_logger = DataLogger()

    try:
        rclpy.spin(data_logger)
    except KeyboardInterrupt:
        data_logger.get_logger().info('Shutting down DataLogger node')

    data_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
