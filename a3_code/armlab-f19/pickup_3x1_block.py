import numpy as np
from util.our_utils import *
from lcmtypes import mbot_status_t


# sample structure for a complex task
class pickup_3x1_block():
    def __init__(self, fsm):
        self.fsm = fsm
        self.state = 'idle'

    def operate_task(self):
        if self.state == 'idle':
            self.state = 'idle'
        if self.state == 'moving_to_closest' and self.fsm.mbot_status == mbot_status_t.STATUS_COMPLETE:
            print("reached the desired position for picking")
            self.state = 'grabbing'
        if self.state == 'grabbing':
            print("Start relocating the 3x1 block and picking")
            time.sleep(5)

            #if(sweep_for_duplicate()):
            location_of_knock_back = find_closest_block(self.fsm.tags, self.fsm.extrinsic_mtx)[2]
            location_of_knock_back[0] = location_of_knock_back[0] - 0.8*I2M
            location_of_knock_back[1] = location_of_knock_back[1] - 1.2*I2M
            location_of_knock_back[2] = location_of_knock_back[2] + 0.5*I2M
            print(location_of_knock_back)
            knock_back(location_of_knock_back, self.fsm.rexarm)
            self.state = 'push'
        if self.state == 'push':
            self.fsm.publish_mbot_command(mbot_command_t.STATE_MOVING, (self.fsm.slam_pose[0] + 6*I2M, self.fsm.slam_pose[1], 0), []) #may need to recalc in different frame
            self.state = 'reverse'
        if self.state == 'reverse' and self.fsm.mbot_status == mbot_status_t.STATUS_COMPLETE:
            set_snake(self.fsm.rexarm)
            self.fsm.publish_mbot_command(mbot_command_t.STATE_MOVING, (self.fsm.slam_pose[0] - 3*I2M, self.fsm.slam_pose[1], 0), [])
            '''
            print("block location received")
            time.sleep(1)
            print("start picking")
            pick_1x1_block(self.fsm.rexarm, new_location)
            self.state = 'idle'
            '''
            self.fsm.set_current_state('go_to_garbage')

    def begin_task(self):
        print("begin task pick up 1x1 block")
        target_tag = self.fsm.tags[0]  # TODO: use closest_tag fxn to find this tag
        target_pose = from_AprilTag_to_pose(target_tag, self.fsm.extrinsic_mtx)
        self.fsm.mbot_status = mbot_status_t.STATUS_IN_PROGRESS
        #self.fsm.moving_mbot(target_pose)
        self.state = "grabbing" #'moving_to_closest'
