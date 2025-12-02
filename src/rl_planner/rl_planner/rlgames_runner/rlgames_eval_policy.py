import torch
import numpy as np
from rl_planner.rlgames_runner.rlnetwork.socialnav_humanpose import SocialNavHumanPose
from rl_planner.rlgames_runner.running_mean_std import RunningMeanStd
import os

# define network
class RLModel(torch.nn.Module):
    def __init__(self, a2c_network=None, actions_num=2, checkpoint_path='checkpoint/last_SocialNavHumanPoseMORL_ep_450_rew_446.22223.pth'):
        super().__init__()
        checkpoint_path = os.path.join("/home/segway/ros2_ws/src/rl_planner/rl_planner/rlgames_runner", checkpoint_path)
        checkpoint = torch.load(checkpoint_path, weights_only=False)
        obs_shape = (checkpoint['model']['running_mean_std.running_mean'].shape[0],)
        value_size = checkpoint['model']['value_mean_std.running_mean'].shape[0]
        if a2c_network is None:
            self.a2c_network = SocialNavHumanPose(params={}, input_shape=obs_shape, actions_num=actions_num, value_size=value_size)
        else:
            self.a2c_network = a2c_network(params={}, input_shape=obs_shape, actions_num=actions_num, value_size=value_size)
        self.running_mean_std = RunningMeanStd(obs_shape)
        self.value_mean_std = RunningMeanStd((value_size,))
        self.rnn_states = self.a2c_network.get_default_rnn_state() if self.a2c_network.is_rnn() else None
        self.a2c_network.eval()
        self.running_mean_std.eval()
        self.value_mean_std.eval()
        self.load_checkpoint(checkpoint_path)

    def reset_rnn_state(self):
        self.rnn_states = self.a2c_network.get_default_rnn_state() if self.a2c_network.is_rnn() else None

    @torch.no_grad()  # disable grad (for eval only)
    def forward(self, input_dict):
        input_dict['obs'] = self.running_mean_std(input_dict['obs'])
        input_dict['rnn_states'] = self.rnn_states
        mu, logstd, value, self.rnn_states = self.a2c_network(input_dict)
        sigma = torch.exp(logstd)
        distr = torch.distributions.Normal(mu, sigma, validate_args=False)

        selected_action = distr.sample()
        selected_action = selected_action.clamp(-1, 1)  # ensure action is in valid range
        neglogp = self.neglogp(selected_action, mu, sigma, logstd)
        result = {
            'neglogpacs': torch.squeeze(neglogp),
            'values': self.value_mean_std(value, denorm=True),
            'actions': selected_action,
            'rnn_states': self.rnn_states,
            'mus': mu,
            'sigmas': sigma
        }
        return result

    def load_checkpoint(self, checkpoint_path):
        checkpoint = torch.load(checkpoint_path, weights_only=False)
        self.load_state_dict(checkpoint['model'])


    def get_action(self, obs_dict, deterministic=True):
        """obs_dict: obs, pref_vec
        output action in normalized scale [-1, 1]"""
        output = self.forward(obs_dict)

        if deterministic:
            return output['mus']
        else:
            return output['actions']



    @torch.compile()
    def neglogp(self, x, mean, std, logstd):
        """Computes negative log probability"""
        return 0.5 * (((x - mean) / std) ** 2).sum(dim=-1) \
            + 0.5 * np.log(2.0 * np.pi) * x.size(-1) \
            + logstd.sum(dim=-1)
