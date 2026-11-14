# One leg of a mission: drive to `station`, then perform `action` there.
uint8 ACTION_NONE=0
uint8 ACTION_PICK=1
uint8 ACTION_PLACE=2

Waypoint station
uint8 action
string item_id                           # required for PICK and PLACE
geometry_msgs/PoseStamped target_pose    # grasp or place TCP pose for PICK and PLACE
