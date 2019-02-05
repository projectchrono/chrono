# chtrain

Reinforcement Learning examples created using the Python exrension of the open-source [Chrono Project](http://www.projectchrono.org/) physics engine (Pychrono) together with Tensorflow.
### Environments
I implemented 2 environments for robotic control (so both observations and action are continuous). 

***ChronoPendulum*** Classical reverse pendulum, has to balance a pole on a cart.  1 action (force along the z axis) and 4 observations.

***ChronoAnt*** a 4-legged walker that has to walk straight. 8 actions (joint torques) and 30 observations.
### DL algorithm
To train the Neural Networks to control solve the tasks I used a  reinforcement learning algorithm  known as [Proximal Policy Optimization](https://arxiv.org/abs/1707.06347) (PPO). PPO is an on-policy actor-critic algorithm, thus you will find 2 NNs: the first one given the state prescribes an action (Policy), the second given the state evaluates the value function (VF).

## Getting Started

### Prerequisites


```
PyChrono
Tensorflow
NumPy
SciPy
```
Of course Tensorflow is not needed if the environments are used to test learning model created with other libraries.

### Installing Pychrono:
#### 1-Installing Conda package
If you have Anaconda you can simply install Pychrono in any of your Python3 environments from my personal [Anaconda repository](https://anaconda.org/SimoneBenatti/pychrono) by typing:

For Linux:
```
conda install -c simonebenatti pychrono 
```
For Windows:
```
conda install -c simonebenatti/label/develop pychrono
```

#### 2-Compiling from source
You will need:
```
Any C++ compiler
Irrlich libraries
Git (or a Git client)
CMake
SWIG (for the Python wrapper)
```
Install Chrono following [these](http://api.projectchrono.org/development/tutorial_install_chrono.html) instructions.
Don't forget to:
- Allow recursive cloning of submodules
- Enable the Python Module from CMake options
- Follow the additional [instructions](http://api.projectchrono.org/development/module_python_installation.html) to build the Python module

## Running
**IMPORTANT**: before changing the environment, delete old saved model files (not the folders), otherwise it will fail trying to restore a model with different sizes. 
### How to run examples
Make sure that you are using the right Python interpreter. Than simply run the script with the needed keyboard arguments
***EXAMPLES***: 
For the inverted pendulum:

```python ./train_serial.py ChronoPendulum -n 1000```

For the 4-legged ant:

```python ./train_serial.py ChronoAnt -n 20000```

### List of command line arguments

 - Environment name : 'env_name'
 - Number of episodes: '-n', '--num_episodes', default=1000
 -  --renderON / --renderOFF
 - Discount factor: '-g', '--gamma', default=0.995
 - Lambda for GAE: -l, --lam, default=0.98
 - Kullback Leibler divergence target value: -k, --kl_targ, default=0.003
 - Batch size: -b, --batch_size, default=20

## Saving and restoring

NN parameters and the other TF variables are stored inside the Policy and VF directories, while the scaler means and variances are stored in the scaler.dat saved numpy array. These files and folders can be stored and used to restore a previous checkpoint.

## Parallel training
Parallel training collects datas from 6 simulations simultaneously, speeding up the process. So far this is available only for the Ant environment and is tested only for Tensorflow-GPU.
## Acknowledgments

* [Patrick Coady](https://github.com/pat-coady)
 
## TODO:
- Add clipped objective function
- Add new environemnts 