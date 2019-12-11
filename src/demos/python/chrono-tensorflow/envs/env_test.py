
"""
Created on Fri Apr  6 15:12:11 2018

Python Script to test DRL envs without Tensorflow

@author: SB
"""

import chtrain_pendulum
import chtrain_ant
import matplotlib.pyplot as plt
import numpy as np

for i in range (2):
    if (i%2)==0:       
        env = chtrain_pendulum.Model(True)
        ac_dim = 1
    else:
        env = chtrain_ant.Model(True)
        ac_dim = 8
    n_episodes = 3
    T=2
    trajectories = []
    
    for episode in range(n_episodes):
              done = False
              env.reset()  
              while not done:
                  ac = 2*(0.5-np.random.rand(ac_dim,))
                  state, reward, done, _ = env.step(ac)
