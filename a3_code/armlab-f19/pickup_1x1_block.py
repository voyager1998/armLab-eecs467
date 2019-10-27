import numpy as np
from util.our_utils import *
from lcmtypes import mbot_status_t


# sample structure for a complex task
class pickup_1x1_block():
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
            print("Start relocating the block and picking")
            time.sleep(8)
            new_location = locate_1x1_block(self.fsm.tags, self.fsm.extrinsic_mtx)
            print("block location received")
            time.sleep(1)
            print("start picking")
            pick_1x1_block(self.fsm.rexarm, new_location)
            self.state = 'idle'
            self.fsm.set_current_state('go_to_garbage')

    def begin_task(self):
        print("begin task pick up 1x1 block")
        target_tag = self.fsm.tags[0]  # TODO: use closest_tag fxn to find this tag
        target_pose = from_AprilTag_to_pose(target_tag, self.fsm.extrinsic_mtx)
        self.fsm.mbot_status = mbot_status_t.STATUS_IN_PROGRESS
        self.fsm.moving_mbot_to_block(target_pose)
        self.state = 'moving_to_closest'
