from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from rl_env import RobotRLAvoidEnv
import os

# 创建训练环境
env = make_vec_env(RobotRLAvoidEnv, n_envs=1)

# 初始化 PPO 模型
model = PPO(
    "MlpPolicy",
    env,
    learning_rate=3e-4,
    n_steps=2048,
    batch_size=64,
    gamma=0.99,
    verbose=1
)

# 训练模型
model.learn(total_timesteps=1000000)

# 保存模型
model_dir = "./models"
os.makedirs(model_dir, exist_ok=True)
model.save(os.path.join(model_dir, "ppo_robot_avoid"))
print("模型已保存至 ./models/ppo_robot_avoid")

# 测试模型
obs = env.reset()
while True:
    action, _states = model.predict(obs, deterministic=True)
    obs, rewards, dones, info = env.step(action)
    if dones:
        obs = env.reset()
