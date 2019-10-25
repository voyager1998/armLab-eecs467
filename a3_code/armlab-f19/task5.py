import numpy as np
import time
from util.our_utils import *

D2R = 3.141592/180.0
R2D = 180.0/3.141592

# sample structure for a complex task
class task5():
    def __init__(self, StateMachine):
        self.fsm = StateMachine
        self.current_step = 0

    def operate_task(self):
        """TODO"""
        pass

    def begin_task(self, endpoint):
        pick_1x1_block(self.fsm.rexarm, endpoint)