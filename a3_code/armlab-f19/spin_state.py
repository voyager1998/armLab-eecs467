import numpy as np
import time
from util.our_utils import *

# sample structure for a complex task
class spin_state():

    PI = 3.141592
    num_of_turns = 0
    closest_tag = []
    closest_distance = 9999
    closest_angle = 0
    closest_tag_number = -1

    def __init__(self, fsm):
        self.fsm = fsm
        self.current_step = 0

    def operate_task(self, tags, extrinsic_mtx):
        """TODO"""
        if self.fsm.state == "spin_state_turning":
            new_theta = self.fsm.slam_pose[2] + PI/4
            new_theta_normalized = self.normalize_angle(new_theta)
            self.fsm.publish_mbot_command(mbot_command_t.STATE_MOVING, (self.fsm.slam_pose[0], self.fsm.slam_pose[1], new_theta_normalized), [])
            self.fsm.state = "spin_state_scan"
            self.num_of_turns += 1
        elif self.fsm.state == "spin_state_scan":
            if(num_of_turns == 8):
                self.fsm.state = "identify_block"
            closest_at_angle = find_closest_block(tags, extrinsic_mtx)
            if(self.closest_distance > closest_at_angle[1]):
                self.closest_tag = closest_at_angle[2]
                self.closest_angle = closest_at_angle[0]
                self.closest_tag_number = closest_at_angle[3]
            self.fsm.state = "spin_state_turning"
            time.sleep(0.5)
        elif self.fsm.state == "identify_block":
            if(closest_tag_number == 1):
                print("long block, ", self.closest_angle)
            

    def normalize_angle(self, angle):
        newAngle = angle
        while newAngle <= -180: newAngle += 360
        while newAngle > 180: newAngle -= 360
        return newAngle

    def begin_task(self):
         #publish_mbot_command(self, state, goal_pose, obstacles)4
        '''
        print(find_closest_block(tags, extrinsic_mtx))
        ''' 
        self.fsm.publish_mbot_command(mbot_command_t.STATE_MOVING, (self.fsm.slam_pose[0], self.fsm.slam_pose[1], 0), [])
        fsm.state = "spin_state_turning"
        closest_tag = []
        closest_distance = 9999
        closest_angle = 0
        closest_tag_number = -1

        '''
        time.sleep(1)
        angles = [0, self.PI/2, self.PI, -self.PI/2] 
        closest_tag = []
        closest_distance = 9999
        closest_angle = 0
        closest_tag_number = -1
        i = 0
        while i <= self.PI*2:
            normalizedAngle = self.normalizeAngle(i)
            self.fsm.publish_mbot_command(mbot_command_t.STATE_MOVING, (self.fsm.slam_pose[0], self.fsm.slam_pose[1], normalizedAngle), [])
            closest_at_angle = find_closest_block(tags, extrinsic_mtx)
            if(closest_distance > closest_at_angle[1]):
                closest_tag = closest_at_angle[2]
                closest_angle = closest_at_angle[0]
                closest_tag_number = closest_at_angle[3]
            i += self.PI/4
            time.sleep(1)

        self.fsm.publish_mbot_command(mbot_command_t.STATE_MOVING, (self.fsm.slam_pose[0], self.fsm.slam_pose[1], closest_angle), [])

        if(closest_tag_number == 1):
            print("long block")
        #else if ()
        '''
        

