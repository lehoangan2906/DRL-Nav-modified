import time
import rclpy
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from velodyne_env import GazeboEnv

class Actor(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Actor, self).__init__()
        self.layer_1 = nn.Linear(state_dim, 800)
        self.layer_2 = nn.Linear(800, 600)
        self.layer_3 = nn.Linear(600, action_dim)
        self.tanh = nn.Tanh()

    def forward(self, s):
        s = F.relu(self.layer_1(s))
        s = F.relu(self.layer_2(s))
        a = self.tanh(self.layer_3(s))
        return a

class TD3(object):
    def __init__(self, state_dim, action_dim):
        self.actor = Actor(state_dim, action_dim).to(device)

    def get_action(self, state):
        state = torch.Tensor(state.reshape(1, -1)).to(device)
        return self.actor(state).cpu().data.numpy().flatten()

    def load(self, filename, directory):
        self.actor.load_state_dict(
            torch.load("%s/%s_actor.pth" % (directory, filename))
        )

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
seed = 0
max_ep = 500
file_name = "TD3_velodyne"

environment_dim = 20
robot_dim = 4
rclpy.init()
env = GazeboEnv("td3", "multi_robot_scenario.launch.py", environment_dim)
time.sleep(5)
torch.manual_seed(seed)
np.random.seed(seed)
state_dim = environment_dim + robot_dim
action_dim = 2

network = TD3(state_dim, action_dim)
try:
    network.load(file_name, "./pytorch_models")
except:
    raise ValueError("Could not load the stored model parameters")

done = False
episode_timesteps = 0
state = env.reset()

while rclpy.ok():
    action = network.get_action(np.array(state))
    a_in = [(action[0] + 1) / 2, action[1]]
    next_state, reward, done, target = env.step(a_in)
    done = 1 if episode_timesteps + 1 == max_ep else int(done)

    if done:
        state = env.reset()
        done = False
        episode_timesteps = 0
    else:
        state = next_state
        episode_timesteps += 1

rclpy.shutdown()