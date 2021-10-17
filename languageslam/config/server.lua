include "map_builder.lua"

MAP_BUILDER_SERVER = {
  map_builder = MAP_BUILDER,
  num_event_threads = 8,
  num_grpc_threads = 8,
  server_address = "0.0.0.0:50051",
  uplink_server_address = "",
  upload_batch_size = 100,
  enable_ssl_encryption = false,
  enable_google_auth = false,
}

MAP_BUILDER_SERVER.map_builder.use_trajectory_builder_2d = true

return MAP_BUILDER_SERVER
