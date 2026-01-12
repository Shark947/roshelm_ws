# 视觉端 -> NavServer
# 等价于旧TCP输入：dHeading thetaX thetaY flag navx navy

std_msgs/Header header

# 目标相对航向误差（单位：deg）
# 目前视觉只负责测量(thetaX/thetaY/valid)，那么这个字段可以先置0，由后续算法或NavServer计算
float64 d_heading_deg

# 相机光轴坐标系下的角度误差（单位：deg）
# 推荐约定：
#   theta_x_deg：图像水平方向角度（u轴方向）
#   theta_y_deg：图像竖直方向角度（v轴方向）
float64 theta_x_deg
float64 theta_y_deg

# 视觉结果是否有效（对应旧vflag）
bool valid

# 当valid=false时的退化备用（对应旧navx/navy）
# 例如：用上一帧或其它传感器估计的平面偏移
float64 fallback_x
float64 fallback_y
