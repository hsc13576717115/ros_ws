-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- Cartographer 2D SLAM configuration for N10P LiDAR with IMU fusion
-- This configuration enables pure laser odometry with IMU assistance
-- to fix map drift during turns

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "gyro_link",      -- Use IMU frame for better orientation
  published_frame = "base_link",     -- Publish robot pose in base_link
  odom_frame = "odom",               -- For pure laser SLAM, odom is virtual
  provide_odom_frame = true,         -- Cartographer will publish odom->base_link
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = false,              -- Pure laser SLAM, no wheel odometry
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,               -- Use single echo laser scan
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,          -- Use all IMU data
  landmarks_sampling_ratio = 1.0,
}

-- ============================================================================
-- Use 2D trajectory builder
-- ============================================================================
MAP_BUILDER.use_trajectory_builder_2d = true

-- ============================================================================
-- TRAJECTORY BUILDER 2D - Laser configuration for N10P
-- ============================================================================

-- N10P LiDAR specifications
TRAJECTORY_BUILDER_2D.min_range = 0.2          -- 20cm minimum range
TRAJECTORY_BUILDER_2D.max_range = 200.0         -- 200m maximum range
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10  -- Accumulate 10 scans
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05  -- 5cm voxel size

-- Adaptive voxel filter for scan matching
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 200.0

-- Loop closure voxel filter
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.9
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 50
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 200.0

-- Real-time correlative scan matcher
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- Ceres scan matcher (IMU-aided)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.only_optimize_yaw = false

-- ============================================================================
-- IMU FUSION - KEY CONFIGURATION TO FIX TURN DRIFT
-- ============================================================================

-- Enable IMU data for orientation estimation
TRAJECTORY_BUILDER_2D.use_imu_data = true

-- Gravity time constant: higher = more trust in accelerometer for gravity
-- Lower = more responsive to changes. 10.0 is good for indoor robots
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0

-- ============================================================================
-- SUBMAPS configuration
-- ============================================================================
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 180
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05  -- 5cm resolution

-- ============================================================================
-- POSE GRAPH optimization
-- ============================================================================

-- Matching tolerances
POSE_GRAPH.matching_translation_weight = 1.0
POSE_GRAPH.matching_rotation_weight = 1.0
POSE_GRAPH.constraint_builder.max_num_distance_constraints = 200
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- Fast correlative scan matcher for loop closure
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = 30.0 * (math.pi / 180.0)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

-- Ceres solver configuration
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimization_problem.acceleration_weight = 1e3
POSE_GRAPH.optimization_problem.rotation_weight = 1.6e4
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

-- Optimizer settings
POSE_GRAPH.optimize_every_n_scans = 30
POSE_GRAPH.max_num_final_iterations = 200
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.log_residual_histograms = true
POSE_GRAPH.global_constraint_search_after_n_seconds = 30.0

return options
