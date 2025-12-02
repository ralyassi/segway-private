import torch
from torch import nn
import numpy as np
"""
SocialNavHumanPose

- Lidar 1D CNN Based on the implementation of: https://journals.sagepub.com/doi/pdf/10.1177/0278364920916531
- Human pose LSTM based on the implementation of: https://github.com/mit-acl/cadrl_ros/blob/master/scripts/network.py


Input: 
    - Vector of lidar (goes to 1D Conv)
    - Current action (v, w)
    - Goal (dist, theta) or (dist_x, dist_y)
    - Human pose: a list of (px,py, vx, vy)
    
Architecture ():
1. [Lidar] -> 1DConv -> 1DConv -> Lidar_Linear

2. human_pose -> lstm -> last hidden

3

3. cat_linear -> FC1 -> FC2 -> Action


"""

def compute_1dconv_size(L_in=50, kernel=3, stride=2, padding=0):
    """Computes the output size of 1d Conv"""
    L_out = int((L_in + 2* padding -(kernel-1) -1)/stride + 1)  # int acts as floor function
    return L_out


class SocialNavHumanPose(nn.Module):
    def __init__(self, params, **kwargs):
        super().__init__()
        actions_num = kwargs.pop('actions_num')
        input_shape = kwargs.pop('input_shape') # observation_space
        self.pref_dim = kwargs.get('value_size', 1)
        self.morl = self.pref_dim > 1



        # hard-coded, must be same as task. Alternativly must define a dict
        self.lidar_hist_steps = 1  # lidar time-steps TODO: fill
        self.lidar_num_rays = 40  # TODO: fill
        self.state_shape = 4  # for agent (vx,vy,gx,gy)
        self.human_obs_limit = 10  # max num of observabnle humnas  # TODO: fill
        self.human_state_shape = 4  # humans (px,py, vx,vy)
        self.add_lidar = True
        self.fixed_sigma = False

        assert input_shape[0] == self.lidar_hist_steps * self.lidar_num_rays + self.state_shape + self.human_obs_limit * self.human_state_shape, f" input shape:{input_shape} should match:{self.lidar_hist_steps * self.lidar_num_rays + self.state_shape + self.human_obs_limit * self.human_state_shape}"

        ### Config ###
        # 1DConv
        kernel1 = 5
        kernel2 = 3
        stride = 2
        channel1 = 32
        channel2 = 32
        lidar_out = 128 if self.add_lidar else 0
        # FC
        pref_emb = 32 if self.morl else 0
        FC1 = 256
        FC2 = 256
        # lstm
        self.lstm_hidden_size = 64
        ##############
        # human pose LSTM
        self.lstm = nn.LSTMCell(input_size=self.human_state_shape, hidden_size=self.lstm_hidden_size)

        # lidar
        if self.add_lidar:
            self.conv1 = nn.Conv1d(self.lidar_hist_steps, channel1, kernel1, stride=stride)  # 32, 5
            self.conv2 = nn.Conv1d(channel1, channel2, kernel2, stride=stride)  # 32, 3

            # Flatten (compute size of output)
            L_out = compute_1dconv_size(L_in=self.lidar_num_rays, kernel=kernel1, stride=stride)
            L_out = compute_1dconv_size(L_in=L_out, kernel=kernel2, stride=stride)
            self.lidar_linear = nn.Linear(channel2 * L_out, lidar_out)

        # linear
        if self.morl:
            self.pref_linear = nn.Linear(self.pref_dim, pref_emb)
        self.linear1 = nn.Linear(self.state_shape + lidar_out + self.lstm_hidden_size + pref_emb, FC1)
        self.linear2 = nn.Linear(FC1, FC2)

        self.mean_linear = nn.Linear(FC2, actions_num)
        self.value_linear = nn.Linear(FC2, self.pref_dim)

        if self.fixed_sigma:
            self.sigma_linear = nn.Parameter(torch.zeros(actions_num, requires_grad=True, dtype=torch.float32), requires_grad=True)
        else:
            self.sigma_linear = torch.nn.Linear(FC2, actions_num)

    def is_separate_critic(self):
        return False

    def is_rnn(self):
        return False

    def get_aux_loss(self):
        return None


    def forward(self, obs_dict):
        obs = obs_dict['obs']  # list of [robot_state, Lidar (w/history), human states]

        batch_size = obs.size(0)
        state = obs[:,0:4]  # Robot [vx,vy,gx,gy]
        human_obs = obs[:, 4 + self.lidar_hist_steps * self.lidar_num_rays:]
        human_obs = human_obs.reshape(batch_size, self.human_obs_limit, self.human_state_shape)

        if self.add_lidar:
            lidar = obs[:, 4: 4 + self.lidar_hist_steps * self.lidar_num_rays]
            lidar = lidar.reshape(batch_size, self.lidar_hist_steps, self.lidar_num_rays)
            lidar = torch.relu(self.conv1(lidar))  # relu
            lidar = torch.relu(self.conv2(lidar))  # relu
            lidar = torch.flatten(lidar, start_dim=1)
            lidar = self.lidar_linear(lidar)

        # lstm state init zero
        hx = torch.zeros(batch_size, self.lstm_hidden_size, device=obs.device)
        cx = torch.zeros(batch_size, self.lstm_hidden_size, device=obs.device)
        # Human [px,py,vx,vy] -> LSTM
        for i in range(self.human_obs_limit):
            # find batches where human state is non-zero because (0,0,0,0) state means no-human
            batch_active = torch.where(~torch.all(human_obs[:, i] == 0, dim=1))[0]
            hx[batch_active], cx[batch_active] = self.lstm(human_obs[batch_active, i], (hx[batch_active], cx[batch_active]))

        # cat + FC
        if self.add_lidar:
            x = torch.cat([state, lidar, hx], axis=-1)
        else:
            x = torch.cat([state, hx], axis=-1)

        if self.morl:
            preference = obs_dict['pref_vec']
            pref_emb = self.pref_linear(preference)
            x = torch.cat([x, pref_emb], axis=-1)

        x = torch.relu(self.linear1(x))
        x = torch.relu(self.linear2(x))

        action = self.mean_linear(x)
        action = torch.tanh(action)  # so vel [-1, 1]

        value = self.value_linear(x)

        if self.fixed_sigma:
            sigma = self.sigma_linear
        else:
            #sigma = torch.nn.functional.softplus(self.sigma_linear(x)) # so that sigma is always positive
            sigma = self.sigma_linear(x) # Code expects logstd, and network builder doesn't use activation function (None)

        # debug
        #if torch.isnan(action).any():
        #    pdb.set_trace()

        return action, sigma, value, None  # None is for rnn_states



"""
from rl_games.algos_torch.network_builder import NetworkBuilder

class SocialNavHumanPoseBuilder(NetworkBuilder):
    def __init__(self, **kwargs):
        NetworkBuilder.__init__(self)

    def load(self, params):
        self.params = params


    def build(self, name, **kwargs):
        return SocialNavHumanPose(self.params, **kwargs)

    def __call__(self, name, **kwargs):
        return self.build(name, **kwargs)

"""













