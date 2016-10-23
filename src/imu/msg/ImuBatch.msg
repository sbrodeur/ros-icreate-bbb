# This is a message to hold batch data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec

Header header

time[] stamps

geometry_msgs/Quaternion[] orientations

geometry_msgs/Vector3[] angular_velocities

geometry_msgs/Vector3[] linear_accelerations