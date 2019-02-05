Deep Reinforcement Learning with PyChrono and Tensorflow {#tutorial_pychrono_demo_tensorflow}
==========================
We will train a Neural Network to solve robotic tasks using virtual training environment created with [PyChrono](@ref pychrono_introduction).

## Environments
We provide two sample environments for robotic control. 

**ChronoPendulum** 
![](http://projectchrono.org/assets/manual/Tutorial_tensorflow_pendulum.jpg)

Reverse pendulum, the goal is to balance a pole on a cart.  1 action (force along the z axis) and 4 observations (position and speed of cart and pole).

**ChronoAnt** 
![](http://projectchrono.org/assets/manual/Tutorial_tensorflow_ant.jpg)
A 4-legged walker, the goal is learning to walk straight as fast as possible. 8 actions (motor torques) and 30 observations.

## DL algorithm

To train the Neural Networks to solve the tasks we used a reinforcement learning algorithm known as [Proximal Policy Optimization](https://arxiv.org/abs/1707.06347) (PPO). PPO is an on-policy actor-critic algorithm, thus you will find 2 NNs: the first one given the state prescribes an action (Policy), the second given the state evaluates the value function (VF).

The policy and value function codes are in the **Policy.py** and **VF.py** respectively.


## How to run examples
Make sure that you are using the right Python interpreter (the one with Pychrono an Tensorflow installed). 
Then simply run the script with the needed keyboard arguments

**EXAMPLES**: 

Train to solve the inverted pendulum over 1000 episodes:

~~~~~~~~~~~~~{.sh}

python ./train.py ChronoPendulum -n 1000

~~~~~~~~~~~~~

Learn to make the 4-legged ant walk over 20000 episodes:

~~~~~~~~~~~~~{.sh}

python ./train.py ChronoAnt -n 20000

~~~~~~~~~~~~~

Alternatively, launch the demo from your favourite IDE, but remember to add the required arguments.

### List of command line arguments
Besides the environment name and the number of episodes, there are some other arguments, mainly to hand-tune learning parameters.
** --renderON/--renderOFF ** toggles on and off the simulation render. Consider that visualizing the render will slow down the simulation and consequentially the learning process.

 - **Environment name** : 'env_name'
 - **Number of episodes**: '-n', '--num_episodes', default=1000
 - ** --renderON   /    --renderOFF **
 - **Discount factor**: '-g', '--gamma', default=0.995
 - **Lambda for GAE**: -l, --lam, default=0.98
 - **Kullback Leibler divergence target value**: -k, --kl_targ, default=0.003
 - **Batch size**: -b, --batch_size, default=20

## Saving and restoring

NN parameters and the other TF variables are stored inside the Policy and VF directories, while the scaler means and variances are stored in the scaler.dat saved numpy array. These files and folders are stored and used to restore a previous checkpoint.

## Tester

To test the Policy without further improving it execute **tester.py**. Set ** --VideoSave ** to save screenshots from the render.

