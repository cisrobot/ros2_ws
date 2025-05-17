import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import QuaternionStamped
from tf2_ros.transform_listener import TransformListener
import tf_transformations
import numpy as np

class ImuBaseMeas(Node):
    def __init__(self):
        super().__init__('imu_base_meas')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.publisher = self.create_publisher(Imu, '/imu_meas', 10)

        self.static_transform_received = False
        self.static_transform = None
        self.rotation_matrix = None

        self.transformed_imu = Imu()
        self.transformed_imu.header.frame_id = 'base_link'

    def rotate_vector(self, vector, R):
        vec_np = np.array([vector.x, vector.y, vector.z])
        rotated = R @ vec_np
        return rotated

    def rotate_orientation(self, quat_msg):
        q_imu = [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w]
        R_imu = tf_transformations.quaternion_matrix(q_imu)[:3, :3]
        R_result = self.rotation_matrix @ R_imu
        R_full = np.identity(4)
        R_full[:3, :3] = R_result
        q_result = tf_transformations.quaternion_from_matrix(R_full)
        return q_result

    def imu_callback(self, msg: Imu):
        if not self.static_transform_received:
            try:
                self.static_transform = self.tf_buffer.lookup_transform(
                    'base_link', 'imu_link', rclpy.time.Time()
                )
                quat = self.static_transform.transform.rotation
                self.rotation_matrix = tf_transformations.quaternion_matrix([
                    quat.x, quat.y, quat.z, quat.w
                ])[:3, :3]  # 3x3 회전 행렬만 사용
                self.get_logger().info('Static transform base_link -> imu_link acquired.')
                self.static_transform_received = True
            except Exception as e:
                self.get_logger().warn(f'Waiting for transform base_link -> imu_link: {e}')
                return

        # # Update timestamp with current time
        self.transformed_imu.header.stamp = self.get_clock().now().to_msg()
        # Update timestamp with current time
        # self.transformed_imu.header.stamp = msg.header.stamp

        # Orientation 변환
        q_rotated = self.rotate_orientation(msg.orientation)
        self.transformed_imu.orientation.x = q_rotated[0]
        self.transformed_imu.orientation.y = q_rotated[1]
        self.transformed_imu.orientation.z = q_rotated[2]
        self.transformed_imu.orientation.w = q_rotated[3]
        self.transformed_imu.orientation_covariance = msg.orientation_covariance

        # Angular velocity
        ang_vel = self.rotate_vector(msg.angular_velocity, self.rotation_matrix)
        self.transformed_imu.angular_velocity.x = ang_vel[0]
        self.transformed_imu.angular_velocity.y = ang_vel[1]
        self.transformed_imu.angular_velocity.z = ang_vel[2]
        self.transformed_imu.angular_velocity_covariance = msg.angular_velocity_covariance

        # Linear acceleration
        lin_acc = self.rotate_vector(msg.linear_acceleration, self.rotation_matrix)
        self.transformed_imu.linear_acceleration.x = lin_acc[0]
        self.transformed_imu.linear_acceleration.y = lin_acc[1]
        self.transformed_imu.linear_acceleration.z = lin_acc[2]
        self.transformed_imu.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.publisher.publish(self.transformed_imu)


def main(args=None):
    rclpy.init(args=args)
    node = ImuBaseMeas()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
