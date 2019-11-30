#!/usr/bin/python

from env.env_sim import rozum_sim
env=rozum_sim()

import gym
import numpy as np

import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'

import tensorflow as tf
tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR)

from tensorflow.python.util import deprecation
deprecation._PRINT_DEPRECATION_WARNINGS = False


from stable_baselines.sac.policies import CnnPolicy
from stable_baselines import SAC

print("\n Before training \n")
obs = env.reset()
while True:
    action = env.sample_action()
    obs, reward, dones, info = env.step(action)
    print(reward)

print("\n Start training \n")    
# env = gym.make('Pendulum-v0')

model = SAC(CnnPolicy, env, verbose=1,tensorboard_log="/sac_rozum")
model.learn(total_timesteps=50000, log_interval=100,tb_log_name="test")
model.save("sac_rozum")

del model # remove to demonstrate saving and loading

model = SAC.load("sac_rozum")
print("\n After training \n")
obs = env.reset()
while True:
    action, _states = model.predict(obs)
    obs, reward, dones, info = env.step(action)
    print(reward)
#     env.render()
