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

