def clip_action(action):
    # 线速度、角速度限幅（实机必备）
    v = max(min(action[0], 0.3), -0.3)
    w = max(min(action[1], 1.0), -1.0)
    return v, w
