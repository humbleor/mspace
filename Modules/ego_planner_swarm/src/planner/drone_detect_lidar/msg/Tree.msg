# 单棵树干检测结果
uint32 id              # 树的唯一 ID
float64 x              # 树干中心 X (world frame)
float64 y              # 树干中心 Y (world frame)
float64 z_base         # 树干底部 Z (地面高度)
float64 height         # 树高 (m)
float64 diameter       # 树干直径 (m)
float64 linearity      # PCA 线性度 [0,1]，越高越像树干
float64 confidence     # 检测置信度 [0,1]
