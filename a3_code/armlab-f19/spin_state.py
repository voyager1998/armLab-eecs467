import numpy as np
from util.our_utils import *

# sample structure for a complex task
class spin_state():

    PI = 3.141592

    def __init__(self, fsm):
        self.fsm = fsm
        self.current_step = 0

    def operate_task(self):
        """TODO"""
        pass

    def begin_task(self, tags, extrinsic_mtx):
         #publish_mbot_command(self, state, goal_pose, obstacles)
        print(find_closest_block(tags, extrinsic_mtx))
        ''' 
        angles = [0, PI/2, PI, -PI/2] 
        original_theta = self.fsm.slam_pose[2]
        april_tags = []
        closestTag = []
        for angle in angles:
            self.fsm.publish_mbot_command(mbot_command_t.STATE_MOVING, (self.fsm.slam_pose[0], self.fsm.slam_pose[1], angle), [])
            tag_positions = get_tag_positions(tags, extrinsic_mtx)

        self.fsm.slam_pose
        '''

