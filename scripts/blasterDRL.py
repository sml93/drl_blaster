#!/usr/bin/env python

import sys
import math
import rospy
import numpy
import random

from drl_blaster.msg import Num
from drl_blaster.msg import IP
from drl_blaster.msg import OP

## get UAV status:
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from drl_blaster.msg import AttitudeTarget
from drl_blaster.msg import Restart_Finished
from drl_blaster.msg import AttControlRunning

from keras.layers import *
from keras.optimizers import *
from keras.models import Sequential

class Intel:
  def __init__(self, num_state, num_action, RL_GAMMA = 0.99):
    self.num_state = num_state
    self.num_action = num_action
    self.model = self._createModel()
    self.GAMMA = RL_GAMMA

  def _createModel(self):
    model = Sequential()
    model.add(Dense(64, activation='relu', input_dim=self.num_state))
    model.add(Dense(64, activation='relu'))
    model.add(Dense(self.num_action,activation='linear'))
    opt = RMSprop(lr=0.00025)
    model.compile(loss='mse', optimizer=opt)
    return model

  def train(self, x, y, batch_size=64, epochs=1, verbose=0):
    self.model.fit(x, y, batch_size=batch_size, epochs=epochs, verbose=verbose)

  def predict(self, batch_states, verbose=0):
    return self.model.predict(batch_states, verbose=verbose)

  def predictOne(self, state_test):
    return self.predict(state_test.reshape(1, self.num_state)).flatten()


class Memory:
  def __init__(self, memory_capacity):
    self.memory_capacity = memory_capacity
    self.samples = []
  
  def add(self, experience):    # Experience: [state, action, reward, state_next]
    self.samples.append(experience)
    if len(self.samples) > self.memory_capacity:
      self.samples.pop(0)       # if full, First In First Out
  
  def sample(self, n):
    n = min(n,len(self.samples))
    return random.sample(self.samples, n)
  
  def num_experience(self):     # return the number of experience
    return len(self.samples)


class Agent:
  steps = 0
  def __init__(self, num_state, num_action):
    self.num_state = num_state
    self.num_action = num_action

    # Hyperparameters of Internal DRL algorithm
    # Memory
    self.MEMORY_CAPACITY = 100000
    
    # RL algorithm
    self.GAMMA = 0.99

    # Deep network
    self.MEMORY_BATCH_SIZE = 64   # number of data for one training 

    # Random selection proportion
    self.MAX_EPSILON = 1
    self.MIN_EPSILON = 0.01
    self.LAMBDA = 0.0015

    self.epsilon = self.MAX_EPSILON
    self.intel = Intel(num_state, num_action, RL_GAMMA=self.GAMMA)
    self.memory = Memory(self.MEMORY_CAPACITY)

  def act(self, state):   # action:[0,1,2,...,num_action-1]

    # Limit: 3) Forced input in Emergency: Vz is out of [-3, 3]
    global UAV_Vel
    if (UAV_Vel.twist.linear.z > 3.0):
      return 0 
    if (UAV_Vel.twist.linear.z < -3.0):
      return 1

    if random.random() < self.epsilon:
      return random.randint(0, self.num_action-1)
    else:
      return np.argmax(self.intel.predictOne(state_test=state))   # Get the index of the largest number, that is the action we should take.

    
  def observe(self, experience):
    self.memory.add(experience)
    self.steps += 1
    self.epsilon = self.MIN_EPSILON + (self.MAX_EPSILON - self.MIN_EPSILON) * math.exp(-self.LAMBDA*self.steps)


  def replay(self):   # get knowledge from experience!
    batch = self.memory.sample(self.MEMORY_BATCH_SIZE)
    len_batch = len(batch)

    no_state = np.zeros(self.num_state)

    batch_states = np.array([o[0] for o in batch])
    batch_states_ = np.array([(no_state if o[3] is None else o[3]) for o in batch])

    v = self.intel.predict(batch_states)
    v_ = self.intel.predict(batch_states_)


    # Inputs and outputs of the DRL network:
    x = np.zeros((len_batch, self.num_state))
    y = np.zeros((len_batch, self.num_action))

    for i in range(len_batch):
      o = batch[i]
      s = o[0]; a = int(o[1]); r = o[2]; s_ = o[3]

      v_t = v[i]
      if s_ is None:
        v_t[a] = r
      
      else:
        v_t[a] = r + self.GAMMA * np.amax(v_[i])    # we will get max reward if we select the best option.

      x[i] = s
      y[i] = v_t

    self.intel.train(x, y, batch_size=len_batch)


# Get UAV status:
UAV_Vel = TwistStamped()
UAV_Pos = PoseStamped()
att_running = AttControlRunning()
UAV_Att_Setpoint = AttitudeTarget()

def UAV_pos_cb(data):
  global UAV_Pos
  UAV_Pos = data

def UAV_vel_cb(data):
  global UAV_Vel
  UAV_Vel = data

def restart_finished_cb(data):
  pass

def att_running_cb(data):
  global att_running
  att_running = data

def local_attitude_setpoint_cb(data):
  global UAV_Att_Setpoint
  UAV_Att_Setpoint = data



# Publisher 
pub = rospy.Publisher('input', IP, queue_size=10)
env_ip = IP()
current_status = OP()
env_ip.action = 1

def status_update(data):
  global current_status
  current_status = data

def interact():
  global pub, env_ip
  global UAV_Vel, UAV_Pos

  pub.publish(env_ip)

  normalized_pos_z = (UAV_Pos.pose.position.z - 20.0)/10.0
  normalized_vel_z = (UAV_Vel.twist.linear.z / 3.0)
  normalized_thrust = (UAV_Att_Setpoint.thrust - 0.59) / 0.19
  state_ = np.array((normalized_pos_z,normalized_vel_z, normalized_thrust))

  done = False
  reward = 0.0


  if ((UAV_Pos.pose.position.z > 30.0) or (UAV_Pos.pose.position.z < 10.0)):
    done = True
    rospy.loginfo("Restarting!")

  if (math.fabs(UAV_Pos.pose.position.z - 20.0) < 0.3):
    reward = 1.0

  return state_, reward, done, True


def env_restore():
  rospy.sleep(1)


def main_loop():
  global current_status, pub, env_ip
  global UAV_Vel, UAV_Pos, att_running, UAV_Att_Setpoint

  rospy.init_node('blasterDRL', anonymous=True)
  
  rospy.Subscriber('')
