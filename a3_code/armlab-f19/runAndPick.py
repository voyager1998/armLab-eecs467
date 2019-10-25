import numpy as np
from util.our_utils import *

# sample structure for a complex task
class runAndPick():
    def __init__(self, fsm):
        self.fsm = fsm
        self.current_step = 0

    def operate_task(self):
        """TODO"""
        pass

    def begin_task(self, block_pose):
        self.fsm.moving_mbot(block_pose)
        pass

