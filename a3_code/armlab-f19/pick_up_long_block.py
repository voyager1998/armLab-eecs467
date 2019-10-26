import numpy as np
import time
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

    def normalizeAngle(self, angle):
        newAngle = angle
        while newAngle <= -180: newAngle += 360
        while newAngle > 180: newAngle -= 360
        return newAngle

    def begin_task(self, tags, extrinsic_mtx):
         #publish_mbot_command(self, state, goal_pose, obstacles)4
        '''
        print(find_closest_block(tags, extrinsic_mtx))
        ''' 
        time.sleep(1)
        angles = [0, self.PI/2, self.PI, -self.PI/2] 
        closest_tag = []
        closest_distance = 9999
        closest_angle = 0
        i = 0
        while i <= self.PI*2:
            normalizedAngle = self.normalizeAngle(i)
            self.fsm.publish_mbot_command(mbot_command_t.STATE_MOVING, (self.fsm.slam_pose[0], self.fsm.slam_pose[1], normalizedAngle), [])
            closest_at_angle = find_closest_block(tags, extrinsic_mtx)
            if(closest_distance > closest_at_angle[1]):
                closest_tag = closest_at_angle[2]
                closest_angle = closest_at_angle[0]
            i += self.PI/4
            time.sleep(1)

        self.fsm.publish_mbot_command(mbot_command_t.STATE_MOVING, (self.fsm.slam_pose[0], self.fsm.slam_pose[1], closest_angle), [])
        

