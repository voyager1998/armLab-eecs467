import numpy as np
from util.our_utils import *

# sample structure for a complex task
class task3():
    def __init__(self, fsm):
        self.fsm = fsm
        self.current_step = 0

    def operate_task(self):
        """TODO"""
        pass

    def begin_task(self, gripper_angle):
        put_block_in_trash(self.fsm.rexarm, gripper_angle)


