#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jan 26 11:36:45 2019

@author: simonebenatti
"""
import sys
sys.path.append('../envs')
import chtrain as gym
import numpy as np
from policy import Policy

from utils import Scaler

def run_parallel_episodes(arg):
        
        total_steps = 0
        env_c = gym.Init(arg[4], False)
        policy = Policy(arg[0], arg[1], arg[2], arg[4], True)
        scaler = Scaler(arg[0], arg[4])
        scaler.resume()
        observes, actions, rewards, unscaled_obs = run_episode(env_c, policy, scaler, arg[3])
        total_steps += observes.shape[0]
        trajectory = {'observes': observes,
                      'actions': actions,
                      'rewards': rewards,
                      'unscaled_obs': unscaled_obs}
        policy.close_sess()
        return trajectory
    
    
    
def run_episode(env, policy, scaler, time_state):
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
    if time_state:
        scale[-1] = 1.0  # don't scale time step feature
        offset[-1] = 0.0  # don't offset time step feature
   
    while not done:

        obs = obs.astype(np.float64).reshape((1, -1))
        if time_state:
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
        step += 1e-3  # increments time step feature

    return (np.concatenate(observes), np.concatenate(actions),
            np.array(rewards, dtype=np.float64), np.concatenate(unscaled_obs))


