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

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

MAX_2D_RANGE = 8
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.min_range = 0.5
TRAJECTORY_BUILDER_2D.max_range = MAX_2D_RANGE
TRAJECTORY_BUILDER_2D.min_z = -3
TRAJECTORY_BUILDER_2D.max_z = 3

TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 3
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.8
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.2

POSE_GRAPH.optimize_every_n_nodes = 50
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 20
POSE_GRAPH.constraint_builder.min_score = 0.48
POSE_GRAPH.constraint_builder.log_matches = false
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 1.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(5.)

POSE_GRAPH.optimization_problem.huber_scale = 1e3
POSE_GRAPH.optimization_problem.acceleration_weight = 0.0001e1
POSE_GRAPH.optimization_problem.rotation_weight = 1e1
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 1
POSE_GRAPH.optimization_problem.log_solver_summary = false
POSE_GRAPH.global_constraint_search_after_n_seconds = 50.

return options
