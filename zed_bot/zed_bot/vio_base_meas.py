import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, TransformStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf_transformations import quaternion_matrix, translation_matrix, translation_from_matrix, quaternion_from_matrix, euler_from_quaternion, quaternion_from_euler
import numpy as np
import math

class VioBaseMeas(Node):
    def __init__(self):
        super().__init__('vio_base_meas')

        # 파라미터 선언
        self.declare_parameter('sub_topic', '/sensor_odom')
        self.declare_parameter('pub_topic', '/odom')        
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('lookup_child_frame_id', 'base_link')
        self.declare_parameter('broadcast_child_frame_id', 'base_link')
        self.declare_parameter('cam_frame_id', 'camera_link')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('smoothing_factor', 0.3)
        self.declare_parameter('v_fwd_max', 3.0)
        self.declare_parameter('v_bwd_max', -1.0)
        self.declare_parameter('vy_thres', 0.5)
        self.declare_parameter('w_max', 3.0)
        self.declare_parameter('v_magnitude_min', 0.1)



        # 파라미터 값 읽기
        self.sub_topic = self.get_parameter('sub_topic').get_parameter_value().string_value
        self.pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.lookup_child_frame_id = self.get_parameter('lookup_child_frame_id').get_parameter_value().string_value
        self.broadcast_child_frame_id = self.get_parameter('broadcast_child_frame_id').get_parameter_value().string_value
        self.cam_frame_id = self.get_parameter('cam_frame_id').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.smoothing_factor = self.get_parameter('smoothing_factor').get_parameter_value().double_value # low-pass filter weight
        self.v_fwd_max = self.get_parameter('v_fwd_max').get_parameter_value().double_value
        self.v_bwd_max = self.get_parameter('v_bwd_max').get_parameter_value().double_value
        self.vy_thres = self.get_parameter('vy_thres').get_parameter_value().double_value
        self.w_max = self.get_parameter('w_max').get_parameter_value().double_value
        self.v_magnitude_min = self.get_parameter('v_magnitude_min').get_parameter_value().double_value

        self.min_dt = 0.05

        # Initialize reusable odometry message
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.frame_id
        self.odom_msg.child_frame_id = self.lookup_child_frame_id

        self.tf_msg = TransformStamped()
        self.tf_msg.header.frame_id = self.frame_id
        self.tf_msg.child_frame_id = self.broadcast_child_frame_id

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            Odometry,
            self.sub_topic,
            self.vio_callback,
            10
        )
        self.pub = self.create_publisher(Odometry, self.pub_topic, 10)

        self.static_transform_ready = False
        self.T_base_cam = None
        self.T_cam_base = None
        
        # Set simple PSD-safe covariance (diagonal with small positive values)
        diag_variance = 1e-3
        self.odom_msg.pose.covariance = [0.0] * 36
        self.odom_msg.twist.covariance = [0.0] * 36
        self.odom_msg.twist.twist.linear.x = 0.0
        self.odom_msg.twist.twist.linear.y = 0.0
        self.odom_msg.twist.twist.linear.z = 0.0
        self.odom_msg.twist.twist.angular.x = 0.0
        self.odom_msg.twist.twist.angular.y = 0.0
        self.odom_msg.twist.twist.angular.z = 0.0
        for i in range(6):
            self.odom_msg.pose.covariance[i * 6 + i] = diag_variance
            self.odom_msg.twist.covariance[i * 6 + i] = diag_variance

        # Filtering
        self.prev_pose = [0.0, 0.0, 0.0] # Global x, y, yaw
        self.est_vw = [0.0, 0.0] # Local vx, w        
        self.prev_time = None

        # thres_ang = 30.0 # deg
        thres_ang = 20.0 # deg
        thres_ang_rad = np.radians(thres_ang)
        self.lateral_sin_thres = np.sin(thres_ang_rad)
        

    def lookup_static_transform(self):
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.lookup_child_frame_id, self.cam_frame_id, rclpy.time.Time()
            )
            self.T_base_cam = self.transform_to_matrix(tf_msg.transform)
            self.T_cam_base = np.linalg.inv(self.T_base_cam)
            self.static_transform_ready = True
            self.get_logger().info('Static transform base_link -> cam_link acquired.')
        except Exception as e:
            self.get_logger().warn(f'Waiting for static transform: {e}')


    def vio_callback(self, msg: Odometry):        
        if not self.static_transform_ready:
            self.lookup_static_transform()
            if not self.static_transform_ready:
                return
        
        # 시간 계산
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9    
        if self.prev_time is None:
            self.prev_time = curr_time

        dt = curr_time - self.prev_time        
        if dt < self.min_dt:
            # self.get_logger().warn(f"dt is smaller than {self.min_dt:.2f}.")
            return
        self.prev_time = curr_time

        # Use only yaw from orientation, set roll and pitch to 0
        q = msg.pose.pose.orientation
        _, _, mes_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        mes_yaw = math.atan2(math.sin(mes_yaw), math.cos(mes_yaw))  # wrap to [-pi, pi]

        new_q = quaternion_from_euler(0.0, 0.0, mes_yaw)
        msg.pose.pose.orientation.x = new_q[0]
        msg.pose.pose.orientation.y = new_q[1]
        msg.pose.pose.orientation.z = new_q[2]
        msg.pose.pose.orientation.w = new_q[3]

        # T_fix_cam -> cam_link from odom_vio
        T_fix_cam_cam = self.pose_to_matrix(msg.pose.pose)

        # Final transform:
        T_odom_base = np.matmul(np.matmul(self.T_base_cam, T_fix_cam_cam), self.T_cam_base)

        # Convert to Pose
        mes_pose = self.matrix_to_pose(T_odom_base)
        p_global = np.array([mes_pose.position.x - self.prev_pose[0], mes_pose.position.y - self.prev_pose[1]])
        v_global = p_global / dt
        v_magnitude = np.linalg.norm(v_global)

        # yaw 변화율 계산 with wrapping and filtering   
        delta_yaw = math.atan2(math.sin(mes_yaw - self.prev_pose[2]), math.cos(mes_yaw - self.prev_pose[2]))
        mes_w = delta_yaw / dt

        self.prev_pose = [mes_pose.position.x, mes_pose.position.y, mes_yaw]

        if v_magnitude > self.v_fwd_max:
            self.get_logger().warn(f"Too big vel! (vx, vy)=({v_global[0]:.2f}, {v_global[1]:.2f}, v_mag={v_magnitude:.2f}")
            return        
                        
        if abs(mes_w) <= self.w_max:
            # self.est_vw[1] = self.smoothing_factor * mes_w + (1 - self.smoothing_factor) * self.est_vw[1]
            self.est_vw[1] = mes_w
        else:            
            self.get_logger().warn(f"W out of range detected: curr_w = {mes_w:.2f}, prev_w = {self.est_vw[1]:.2f}")
            return
                            
        cos_yaw = np.cos(-mes_yaw)
        sin_yaw = np.sin(-mes_yaw)
        Mat_G2L = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
        v_local = Mat_G2L @ v_global
        
        lateral_sin = 0.0
        if v_magnitude > self.v_magnitude_min:
            lateral_sin = abs(v_local[1] / v_magnitude)
                
        # self.get_logger().warn(f"lateral_sin = {lateral_sin:.2f}") 
        if lateral_sin > self.lateral_sin_thres:
            self.get_logger().warn(f"large vy angle detected: (vx, vy) = ({v_local[0]:.2f}, {v_local[1]:.2f}), lateral_sin = {lateral_sin:.2f}")        
            return
        elif abs(v_local[1]) > self.vy_thres:
            self.get_logger().warn(f"large vy magnitude detected: (vx, vy) = ({v_local[0]:.2f}, {v_local[1]:.2f}), lateral_sin = {lateral_sin:.2f}")        
            return
        else:
            if self.v_bwd_max <= v_local[0] <= self.v_fwd_max:
                # self.est_vw[0] = self.smoothing_factor * v_local[0] + (1 - self.smoothing_factor) * self.est_vw[0]
                self.est_vw[0] = v_local[0]
            else:                
                self.get_logger().warn(f"VX out of range detected: curr_vx = {v_local[0]:.2f}, prev_vx = {self.est_vw[0]:.2f} ")
                return
        
        # Update and publish odometry with velocity
        self.odom_msg.header.stamp = msg.header.stamp
        # self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.pose.pose = mes_pose
        self.odom_msg.twist.twist.linear.x = self.est_vw[0]            
        self.odom_msg.twist.twist.linear.y = v_local[1]     
        self.odom_msg.twist.twist.angular.z = self.est_vw[1]
        self.pub.publish(self.odom_msg)
        
        # Broadcast TF
        if self.publish_tf:                        
            self.tf_msg.header.stamp = self.odom_msg.header.stamp                        
            self.tf_msg.transform.translation.x = mes_pose.position.x
            self.tf_msg.transform.translation.y = mes_pose.position.y
            self.tf_msg.transform.translation.z = 0.0  # z 고정
            self.tf_msg.transform.rotation = mes_pose.orientation
            self.tf_broadcaster.sendTransform(self.tf_msg)
        
     

    def pose_to_matrix(self, pose: Pose):
        trans = translation_matrix([pose.position.x, pose.position.y, 0.0])  # z 고정
        rot = quaternion_matrix([
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w
        ])
        return np.matmul(trans, rot)

    def transform_to_matrix(self, transform):
        trans = translation_matrix([
            transform.translation.x,
            transform.translation.y,
            transform.translation.z
        ])
        rot = quaternion_matrix([
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w
        ])
        return np.matmul(trans, rot)

    def matrix_to_pose(self, mat):
        trans = translation_from_matrix(mat)
        _, _, yaw = euler_from_quaternion(quaternion_from_matrix(mat))
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))  # wrap
        quat = quaternion_from_euler(0.0, 0.0, yaw)  # roll, pitch 0으로 고정
        pose = Pose()
        pose.position.x = trans[0]
        pose.position.y = trans[1]
        pose.position.z = 0.0  # z 고정
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = VioBaseMeas()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
