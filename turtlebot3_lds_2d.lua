-- 新增闭环检测配置
POSE_GRAPH.constraint_builder.min_score = 0.7  -- 提高闭环匹配阈值，减少误匹配
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.8  -- 全局定位阈值
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3  -- 降低采样率，提升实时性
POSE_GRAPH.optimize_every_n_nodes = 20  -- 每20个节点优化一次，减少漂移
POSE_GRAPH.max_num_final_iterations = 10  -- 增加优化迭代次数

-- 新增子图配置
SPARSE_POSE_GRAPH.submaps.num_range_data = 90  -- 子图包含的激光数据量
SPARSE_POSE_GRAPH.matcher.translation_weight = 10.0  -- 平移权重
SPARSE_POSE_GRAPH.matcher.rotation_weight = 1.0  -- 旋转权重
