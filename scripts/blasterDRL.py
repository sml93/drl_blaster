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

