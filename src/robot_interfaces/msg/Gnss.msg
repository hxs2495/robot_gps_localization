# 状态信息
string state        # 状态

# 时间信息
float64 time        # 时间

# 位置相关信息
uint8 pos_lon       # 经度的精度
float64 lon         # 经度
uint8 pos_lat       # 纬度的精度
float64 lat         # 纬度
float32 alt         # 高度
float32 vel         # 速度

# 姿态信息
float32 yaw         # 偏航角
float32 pitch       # 俯仰角
float32 roll        # 滚转角

# 加速度与陀螺仪数据
float32 acc_x       # X轴加速度
float32 acc_y       # Y轴加速度
float32 acc_z       # Z轴加速度
float32 gyro_x      # X轴陀螺仪数据
float32 gyro_y      # Y轴陀螺仪数据
float32 gyro_z      # Z轴陀螺仪数据