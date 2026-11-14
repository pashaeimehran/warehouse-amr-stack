# All 3D detections from one source image. header.stamp is that image's
# capture time, so the tracker can order and gate measurements correctly.
std_msgs/Header header
DetectedObject3D[] objects
