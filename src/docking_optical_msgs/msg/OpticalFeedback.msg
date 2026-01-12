# NavServer-> 视觉端
# 等价于旧TCP回传：depth_error nav_heading nextX nextY dIMUX dIMUY

std_msgs/Header header

# 深度误差（比如：light_depth - camera_depth）
float64 depth_error

# 当前航向（deg）
float64 nav_heading_deg

# NavServer计算出的下一步相对目标点（米）
float64 next_x
float64 next_y

# IMU增量（按NavServer逻辑：当前 - 上一次）
float64 delta_imu_x
float64 delta_imu_y
