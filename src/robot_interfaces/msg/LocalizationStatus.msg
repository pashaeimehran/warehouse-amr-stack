# Trust assessment of the robot pose estimate, published by the localization
# adapter. The mission and safety layers never navigate on an untrusted pose.
uint8 TRUSTED=0
uint8 DEGRADED=1                         # covariance above the warning threshold
uint8 UNTRUSTED=2                        # covariance above the stop threshold, or no estimate

std_msgs/Header header
uint8 status
string estimator                         # e.g. "ekf", "isam2", "amcl"
float64 position_stddev                  # m, sqrt of the largest position eigenvalue
float64 yaw_stddev                       # rad
builtin_interfaces/Duration since_last_landmark
