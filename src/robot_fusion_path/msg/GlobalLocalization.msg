# Fused vehicle pose expressed as WGS84 geodetic position plus an ENU attitude.
# latitude/longitude are degrees and altitude is meters above the same ellipsoid
# used by the anchoring sensor_msgs/NavSatFix.
std_msgs/Header header

# Frame of the body pose and twist (normally base_footprint).
string child_frame_id

float64 latitude
float64 longitude
float64 altitude

# Body orientation in the local East-North-Up tangent frame at the anchor.
geometry_msgs/Quaternion orientation

# Twist keeps the nav_msgs/Odometry convention and is expressed in child_frame_id.
geometry_msgs/Twist twist

# Covariance order is x/east, y/north, z/up, roll, pitch, yaw.
float64[36] pose_covariance
float64[36] twist_covariance

# The GNSS status/service used when the global anchor was established.
int8 anchor_fix_status
uint16 anchor_fix_service

# NavSatFix has no satellite-count field. Keep this extensible field at
# SATELLITES_UNKNOWN until a receiver-specific status topic is connected.
uint16 SATELLITES_UNKNOWN=65535
uint16 satellites_used

builtin_interfaces/Time anchor_stamp
