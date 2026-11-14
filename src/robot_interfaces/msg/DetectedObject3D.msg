# One detection localized in 3D by perception_3d (camera geometry plus depth).
# header.stamp is the capture time of the source image, never the processing
# time; header.frame_id is the frame of `pose` (usually map).
std_msgs/Header header

ObjectClass object_class
float32 confidence                       # detector confidence, [0, 1]

# Position with covariance (m, m^2). Orientation is unused for people and set
# to identity; its covariance rows are -1 when unknown (REP 145 convention).
geometry_msgs/PoseWithCovariance pose

# 2D box in the source image (pixels), kept for debugging and association.
float32 box_x_min
float32 box_y_min
float32 box_x_max
float32 box_y_max
