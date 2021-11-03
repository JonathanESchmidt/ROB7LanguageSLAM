include "map_builder_server.lua"
MAP_BUILDER_SERVER.map_builder.use_trajectory_builder_2d = true
MAP_BUILDER_SERVER.map_builder.pose_graph.optimize_every_n_nodes = 9999999999999
MAP_BUILDER_SERVER.server_address = os.getenv("robot_ip")
MAP_BUILDER_SERVER.uplink_server_address = "192.168.1.110:50100"

return MAP_BUILDER_SERVER