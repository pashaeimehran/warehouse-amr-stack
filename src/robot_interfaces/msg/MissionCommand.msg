# A command to the mission manager, usually mapped from a VDA 5050 order by
# fleet_interface. START carries the mission; the other commands only need
# mission_id.
uint8 START=0
uint8 CANCEL=1
uint8 PAUSE=2
uint8 RESUME=3

std_msgs/Header header
uint8 command
string mission_id
MissionLeg[] legs
Waypoint home                            # where the robot returns after the last leg
