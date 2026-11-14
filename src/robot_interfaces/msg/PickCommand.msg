# Request to the manipulation layer to pick or place one item.
uint8 PICK=0
uint8 PLACE=1

std_msgs/Header header
uint8 action
string item_id
geometry_msgs/PoseStamped target_pose    # grasp TCP pose (PICK) or release TCP pose (PLACE)
float64 approach_distance                # m, along the approach axis
float64 gripper_opening                  # m, finger opening before closing
