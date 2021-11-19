include "map_builder_server.lua"

MAP_BUILDER_SERVER.map_builder.use_trajectory_builder_2d = true
MAP_BUILDER_SERVER.server_address = "192.168.0.110:50100"
MAP_BUILDER_SERVER.uplink_server_address = ""


POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.optimization_problem.huber_scale = 3
POSE_GRAPH.optimize_every_n_nodes = 1
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e3
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e2
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e3
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e2

return MAP_BUILDER_SERVER
