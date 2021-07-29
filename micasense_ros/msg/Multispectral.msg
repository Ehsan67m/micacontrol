string capture_id
string file_name
string time_string
string GPS_source
# Latitude [degrees]. Positive is north of equator; negative is south.
float64 latitude

# Longitude [degrees]. Positive is east of prime meridian; negative is west.
float64 longitude

# Altitude [m]. Positive is above the WGS 84 ellipsoid
# (quiet NaN if no altitude is available).
float64 altitude

# 5 different spectral raw image. Use CvBridge to convert between OpenCv and ROS message
sensor_msgs/Image[] raw_bands

