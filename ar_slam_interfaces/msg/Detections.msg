std_msgs/Header header

# unique ID for capture used to produce detections
string capture_uid

# width and height of capture image in pixels
uint32 image_height
uint32 image_width

# filepath to source image (for debugging purposes)
string image_path

# names of different detectors that generated the detections
string[] detectors

ar_slam_interfaces/Detection[] detections
