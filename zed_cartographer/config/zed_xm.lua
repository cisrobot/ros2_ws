include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,  -- ✅ 프레임 발행 비활성화
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,  -- ✅ LaserScan 사용 (PointCloud 비활성화)
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,  -- ✅ PointCloud 입력 제거
  publish_to_tf = false,  -- ✅ TF 발행 비활성화
  lookup_transform_timeout_sec = 0.1,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 100e-3,
  trajectory_publish_period_sec = 100e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 0.8,  -- 0.2에서 0.8로 수정
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 0.5,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 10.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 11.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- ✅ PointCloud 관련 설정 제거
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 6.
--TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.01

TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 9.81
POSE_GRAPH.optimization_problem.log_solver_summary = true

POSE_GRAPH.constraint_builder.min_score = 0.7
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.75

-- ✅ 루프 클로저 최적화
POSE_GRAPH.optimize_every_n_nodes = 5
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

return options
