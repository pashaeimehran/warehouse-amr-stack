# One track of the world model (tracking_core, constant-velocity Kalman filter).
# Mirrors robot_core::TrackedObject.
uint8 SOURCE_CAMERA=0
uint8 SOURCE_LIDAR=1
uint8 SOURCE_FUSED=2

uint8 STATUS_TENTATIVE=0                 # born, not yet confirmed (fewer than N hits)
uint8 STATUS_CONFIRMED=1
uint8 STATUS_COASTING=2                  # predicted only, missed the last update(s)

std_msgs/Header header                   # stamp of the last measurement; frame map
uint64 track_id                          # stable for the track's lifetime
ObjectClass object_class
uint8 source
uint8 status

geometry_msgs/Point position             # m, z unused (0)
geometry_msgs/Vector3 velocity           # m/s, z unused (0)
float64[4] position_covariance           # row-major 2x2 [xx, xy, yx, yy], m^2
