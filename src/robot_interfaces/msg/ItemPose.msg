# 3D pose of a shelf item for grasping, from perception_3d. header.stamp is
# the source image capture time.
std_msgs/Header header
string item_id
ObjectClass object_class
geometry_msgs/PoseWithCovariance pose
geometry_msgs/Vector3 size               # m, bounding box extent
float32 confidence
