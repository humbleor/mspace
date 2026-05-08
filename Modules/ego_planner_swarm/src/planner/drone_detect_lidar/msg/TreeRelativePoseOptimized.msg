# 因子图优化后的相对位姿
Header header
uint32 src_drone_id          # 源无人机 ID
uint32 dst_drone_id          # 目标无人机 ID
float64 dx                   # 优化后相对 X (m)
float64 dy                   # 优化后相对 Y (m)
float64 yaw                  # 优化后相对偏航角 (rad)
float64 rms_residual         # 优化前匹配残差 (m)
int32 num_matched_trees      # 匹配树干数量
