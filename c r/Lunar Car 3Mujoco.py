# 月球机器人 MuJoCo + DQN 强化学习 完整移动训练代码
# 运行环境：Python3 + MuJoCo + mujoco-py + torch + gym
# 依赖安装：pip install mujoco-py gym torch numpy matplotlib
import mujoco_py
import gym
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from collections import deque
import random
import matplotlib.pyplot as plt

# ---------------------- DQN 神经网络（决策核心）----------------------
class DQNNet(nn.Module):
    def __init__(self, obs_dim, action_dim, hidden_dim=64):
        super(DQNNet, self).__init__()
        # 输入：月球车状态（位置、速度、姿态），输出：动作（线性速度/角速度）
        self.fc1 = nn.Linear(obs_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, action_dim)
        self.relu = nn.ReLU()

    def forward(self, x):
        x = self.relu(self.fc1(x))
        x = self.relu(self.fc2(x))
        return self.fc3(x)

# ---------------------- 月球机器人强化学习智能体 ----------------------
class LunarRoverDQN:
    def __init__(self, model_path="lunar_rover.xml", gamma=0.95, lr=1e-4, batch_size=32):
        # MuJoCo 月球车物理仿真初始化
        self.model = mujoco_py.load_model_from_path(model_path)
        self.sim = mujoco_py.MjSim(self.model)
        self.viewer = mujoco_py.MjViewer(self.sim)
        self.obs_dim = self.sim.data.qpos.flat.shape[0]  # 状态维度
        self.action_dim = 2  # 动作维度：[线性速度, 角速度]
        self.action_bound = 0.5  # 动作边界（月球低重力，速度不宜过大）

        # DQN 核心参数
        self.gamma = gamma  # 折扣因子
        self.lr = lr        # 学习率
        self.batch_size = batch_size
        self.memory = deque(maxlen=10000)  # 经验回放池

        # 构建DQN网络（评估网络 + 目标网络）
        self.eval_net = DQNNet(self.obs_dim, self.action_dim)
        self.target_net = DQNNet(self.obs_dim, self.action_dim)
        self.optimizer = optim.Adam(self.eval_net.parameters(), lr=self.lr)
        self.loss_func = nn.MSELoss()

        # 同步目标网络参数
        self.target_net.load_state_dict(self.eval_net.state_dict())

    # 动作选择（ε-贪婪策略：探索+利用）
    def choose_action(self, obs, epsilon=0.1):
        obs = torch.FloatTensor(obs).unsqueeze(0)
        if random.random() > epsilon:
            # 利用：选择Q值最大的动作
            action = self.eval_net(obs).detach().numpy()[0]
        else:
            # 探索：随机选择动作
            action = np.random.uniform(-self.action_bound, self.action_bound, self.action_dim)
        return action

    # 存储经验（s, a, r, s'）
    def store_experience(self, s, a, r, s_, done):
        self.memory.append((s, a, r, s_, done))

    # 经验回放训练
    def learn(self):
        if len(self.memory) < self.batch_size:
            return  # 经验池不足，不训练

        # 抽取批量经验
        batch = random.sample(self.memory, self.batch_size)
        s_batch = torch.FloatTensor([x[0] for x in batch])
        a_batch = torch.FloatTensor([x[1] for x in batch])
        r_batch = torch.FloatTensor([x[2] for x in batch]).unsqueeze(1)
        s_next_batch = torch.FloatTensor([x[3] for x in batch])
        done_batch = torch.FloatTensor([x[4] for x in batch]).unsqueeze(1)

        # 计算评估网络Q值
        q_eval = self.eval_net(s_batch)
        # 计算目标网络Q值（Bellman方程）
        q_target = r_batch + self.gamma * self.target_net(s_next_batch).detach() * (1 - done_batch)

        # 计算损失，反向传播
        loss = self.loss_func(q_eval, q_target)
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        return loss.item()

    # 重置机器人状态（月球车回到初始位置）
    def reset(self):
        self.sim.reset()
        return self.sim.data.qpos.flat[:]

    # 执行一步动作，计算奖励（核心：适配月球移动）
    def step(self, action):
        self.sim.data.ctrl[:] = action  # 执行动作
        self.sim.step()
        s_ = self.sim.data.qpos.flat[:]  # 下一个状态

        # ---------------------- 月球专属奖励函数 ----------------------
        # 1. 距离奖励：靠近目标点（假设目标点在(0,0)）
        distance = np.linalg.norm(s_[:2])
        dist_reward = -0.1 * distance
        # 2. 速度奖励：保持稳定速度（月球低重力，避免速度过大/过小）
        speed = np.linalg.norm(action[0])
        speed_reward = 0.2 if 0.1 <= speed <= 0.3 else -0.1
        # 3. 姿态奖励：避免月球车倾倒（姿态角接近0则奖励）
        pose_reward = -0.05 * np.abs(s_[2])
        # 总奖励
        reward = dist_reward + speed_reward + pose_reward

        # 终止条件：到达目标点 或 倾倒 或 超出边界
        done = False
        if distance < 0.1:  # 到达目标点
            reward += 10  # 额外奖励
            done = True
        if np.abs(s_[2]) > np.pi/4:  # 倾倒
            reward -= 5   # 惩罚
            done = True
        if distance > 5:  # 超出边界
            done = True

        return s_, reward, done

    # 完整训练+可视化运行
    def train(self, episodes=50, epsilon_decay=0.995):
        loss_list = []
        reward_list = []
        epsilon = 0.3  # 初始探索率

        for ep in range(episodes):
            s = self.reset()
            done = False
            total_reward = 0
            episode_loss = 0
            step_cnt = 0

            while not done:
                self.viewer.render()  # 可视化月球车移动
                step_cnt += 1

                # 选择动作，执行动作
                action = self.choose_action(s, epsilon)
                s_, reward, done = self.step(action)

                # 存储经验，学习
                self.store_experience(s, action, reward, s_, done)
                loss = self.learn()
                if loss:
                    episode_loss += loss

                # 更新状态和总奖励
                s = s_
                total_reward += reward

            # 探索率衰减（越训练，越倾向于利用）
            epsilon *= epsilon_decay
            # 每10个episode同步一次目标网络
            if (ep + 1) % 10 == 0:
                self.target_net.load_state_dict(self.eval_net.state_dict())

            # 记录数据
            loss_list.append(episode_loss / step_cnt if step_cnt !=0 else 0)
            reward_list.append(total_reward)
            print(f"📌 Episode {ep+1:2d} | Total Reward: {total_reward:.2f} | Avg Loss: {loss_list[-1]:.4f} | Epsilon: {epsilon:.3f}")

        # 绘制训练曲线
        self.plot_train_curve(reward_list, loss_list)
        # 保存训练好的模型
        torch.save(self.eval_net.state_dict(), "lunar_rover_dqn.pth")
        print("\n✅ 训练完成！模型已保存为：lunar_rover_dqn.pth")

    # 绘制训练奖励/损失曲线
    def plot_train_curve(self, rewards, losses):
        plt.figure(figsize=(12, 4))
        plt.subplot(1,2,1)
        plt.plot(rewards, label="Total Reward")
        plt.xlabel("Episode")
        plt.ylabel("Reward")
        plt.legend()
        plt.title("Lunar Rover DQN Training Reward")

        plt.subplot(1,2,2)
        plt.plot(losses, label="Average Loss", color="orange")
        plt.xlabel("Episode")
        plt.ylabel("Loss")
        plt.legend()
        plt.title("Lunar Rover DQN Training Loss")
        plt.show()

# ---------------------- 测试代码（直接运行）----------------------
if __name__ == "__main__":
    # 注意：替换为你的月球车MuJoCo模型路径（xml文件）
    rover_agent = LunarRoverDQN(model_path="lunar_rover.xml")
    # 启动训练（50个episode，可根据电脑性能调整）
    rover_agent.train(episodes=50)
