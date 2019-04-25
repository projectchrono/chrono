"""
PPO: Proximal Policy Optimization
serial version
"""
import sys
#import chtrain_ant_fancygraphics as gym
sys.path.append('../envs')
import chtrain as gym
import numpy as np
#from gym import wrappers
from policy import Policy
from value_function import NNValueFunction
#import scipy.signal
from utils import Logger, Scaler
from datetime import datetime
import os
import argparse
import signal


class GracefulKiller:
    """ Gracefully exit program on CTRL-C """
    def __init__(self):
        self.kill_now = False
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

    def exit_gracefully(self, signum, frame):
        self.kill_now = True


def init_gym(env_name, render):
    """
    Initialize gym environment, return dimension of observation
    and action spaces.

    Args:
        render: True to toggle on visualization

    Returns: 3-tuple
        environment (object)
        number of observation dimensions (int)
        number of action dimensions (int)
    """
    env = gym.Init(env_name, render)
    obs_dim = env.observation_space.shape[0]
    act_dim = env.action_space.shape[0]

    return env, obs_dim, act_dim


def run_episode(env, policy, scaler, animate=True):
    """ Run single episode 

    Args:
        env: environment (object)
        policy: policy object with sample() method
        scaler: scaler object, scales/offsets each observation

    Returns: 4-tuple of NumPy arrays
        observes: shape = (episode len, obs_dim)
        actions: shape = (episode len, act_dim)
        rewards: shape = (episode len,)
        unscaled_obs: dataset for training scaler, shape = (episode len, obs_dim)
    """
    obs = env.reset()  #resets whenever an episode begins
    observes, actions, rewards, unscaled_obs = [], [], [], []
    done = False
    step = 0.0
    scale, offset = scaler.get()
    scale[-1] = 1.0  # don't scale time step feature
    offset[-1] = 0.0  # don't offset time step feature

    while not done:

        obs = obs.astype(np.float64).reshape((1, -1))  
        obs = np.append(obs, [[step]], axis=1)  # add time step feature TODO: check if this extra state is useful
        unscaled_obs.append(obs)
        obs = (obs - offset) * scale  # center and scale observations TODO: check ifscaler is useful (it should be according to literature)
        observes.append(obs)
        action = policy.sample(obs).reshape((1, -1)).astype(np.float64)
        actions.append(action)
        obs, reward, done, _ = env.step(action)  #state, reward, done, info = env.step(action)
        if not isinstance(reward, float):
            reward = np.asscalar(reward)
        rewards.append(reward)
        step += 1e-3  # increment time step feature

    return (np.concatenate(observes), np.concatenate(actions),
            np.array(rewards, dtype=np.float64), np.concatenate(unscaled_obs))


def run_policy(env, policy, scaler, logger, episodes):
    """ Run policy and collect data 

    Args:
        env: environment (object)
        policy: policy object with sample() method
        scaler: scaler object, scales/offsets each observation
        logger: logger object, used to save stats from episodes
        episodes: total episodes to run

    Returns: list of trajectory dictionaries, list length = number of episodes
        'observes' : NumPy array of states from episode
        'actions' : NumPy array of actions from episode
        'rewards' : NumPy array of (un-discounted) rewards from episode
        'unscaled_obs' : NumPy array of (un-scaled) states from episode
    """
    total_steps = 0
    trajectories = []
    for e in range(episodes):
        run_episode(env, policy, scaler)


def main(env_name, num_episodes, render, VideoSave, gamma, lam, kl_targ, batch_size):
    killer = GracefulKiller()
    env, obs_dim, act_dim = init_gym(env_name, render)
    obs_dim += 1  # add 1 to obs dimension for time step feature (see run_episode())
    now = datetime.utcnow().strftime("%b-%d_%H-%M-%S")  # create unique directories
    logger = Logger(logname=env_name, now=now)
    #aigym_path = os.path.join('/tmp', env_name, now)
    #env = wrappers.Monitor(env, aigym_path, force=True) 
    scaler = Scaler(obs_dim, env_name)
    scaler.resume()
    val_func = NNValueFunction(obs_dim, env_name)
    policy = Policy(obs_dim, act_dim, kl_targ, env_name)
    episode = 0
    capture = False
    while episode < num_episodes:
        if VideoSave and not capture:
            env.ScreenCapture(5)
            capture = True
        trajectories = run_policy(env, policy, scaler, logger, episodes=batch_size)
        episode += len(trajectories)
        

        if killer.kill_now:
            if input('Terminate training (y/[n])? ') == 'y':
                break
            killer.kill_now = False
    logger.close()
    policy.close_sess()
    val_func.close_sess()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=('Train policy on OpenAI Gym environment '
                                                  'using Proximal Policy Optimizer'))
    parser.add_argument('env_name', type=str, help='OpenAI Gym environment name')
    parser.add_argument('-n', '--num_episodes', type=int, help='Number of episodes to run',
                        default=1000)
    parser.add_argument('--renderON',action='store_true', default=False, dest='render', help='Toggle ON video')
    parser.add_argument('--renderOFF',action='store_false', default=False, dest='render', help='Toggle OFF video')
    parser.add_argument('--VideoSave',action='store_true', default=False, dest='VideoSave', help='Toggle ON video save')
    parser.add_argument('-g', '--gamma', type=float, help='Discount factor', default=0.995)
    parser.add_argument('-l', '--lam', type=float, help='Lambda for Generalized Advantage Estimation',
                        default=0.98)
    parser.add_argument('-k', '--kl_targ', type=float, help='D_KL target value',
                        default=0.003)
    parser.add_argument('-b', '--batch_size', type=int,
                        help='Number of episodes per training batch',
                        default=20)

    args = parser.parse_args()
    main(**vars(args))
