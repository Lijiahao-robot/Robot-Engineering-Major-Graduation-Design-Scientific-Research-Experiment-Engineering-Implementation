import torch
import torch.nn as nn

class RobotPolicy(nn.Module):
    def __init__(self, obs_dim=10, act_dim=2):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Linear(128, act_dim)
        )

    def forward(self, obs):
        return self.net(obs)
