std_msgs/Header header
Spline spline
std_msgs/Bool if_full_window
std_msgs/Float64 runtime
# 6x6 IEKF posterior covariance for the last knot, row-major [x,y,z,rx,ry,rz]
float64[36] pose_covariance
