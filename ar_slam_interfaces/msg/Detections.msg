std_msgs/Header header

# unique ID for capture used to produce detections
string capture_uid

# width and height of capture image in pixels
uint32 image_height
uint32 image_width

# filepath to source image (for debugging purposes)
string image_path

# image used to create detection
sensor_msgs/Image image

# names of different detectors that generated the detections
string[] detector_types

ar_slam_interfaces/Detection[] detections
