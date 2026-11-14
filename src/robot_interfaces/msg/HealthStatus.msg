# Health of one monitored component (topic, node or subsystem), published by
# health_monitor on /health. Levels mirror diagnostic_msgs/DiagnosticStatus.
uint8 OK=0
uint8 WARN=1
uint8 ERROR=2
uint8 STALE=3                            # no data within the component's deadline

std_msgs/Header header
string component                         # e.g. "camera", "perception", "lidar"
uint8 level
string message
builtin_interfaces/Duration age          # time since the last valid input
