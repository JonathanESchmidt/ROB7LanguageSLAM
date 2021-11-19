include "map_builder.lua"
include "trajectory_builder.lua"
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "/robot7/base_footprint",
  published_frame = "/robot7/odom",
  odom_frame = "/robot7/odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,


}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 200
POSE_GRAPH.optimize_every_n_nodes = 0
return options
