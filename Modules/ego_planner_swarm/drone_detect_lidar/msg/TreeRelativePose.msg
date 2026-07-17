# 两机间相对位姿测量结果
Header header
uint32 src_drone_id          # 源无人机 ID
uint32 dst_drone_id          # 目标无人机 ID
float64 dx                   # 相对 X (m)
float64 dy                   # 相对 Y (m)
float64 yaw                  # 相对偏航角 (rad)
float64[9] covariance        # 3x3 协方差矩阵 (行优先)
float64 rms_residual         # 匹配残差 RMS (m)
int32 num_matched_trees      # 匹配树干数量
