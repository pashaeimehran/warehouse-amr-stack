# Output of the deterministic safety_supervisor FSM on /safety_state.
# Application-level software safety logic in simulation, not a certified
# safety function (see docs/12-limitations.md).
uint8 NORMAL=0                           # full speed
uint8 SLOW=1                             # protective field 2 occupied: speed capped
uint8 STOP=2                             # protective field 1 occupied: protective stop
uint8 DEGRADED=3                         # a sensor or estimator is degraded: reduced speed
uint8 SAFE_STOP=4                        # hard fault: controlled stop and hold

std_msgs/Header header
uint8 state
float64 speed_limit                      # m/s currently enforced on /cmd_vel
bool arm_motion_allowed                  # interlock: base stopped and no person within 1.5 m
string reason                            # human-readable cause of the current state
float64 nearest_person_distance          # m, +inf if none
