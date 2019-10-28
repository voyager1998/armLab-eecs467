import numpy as np
from util.our_utils import *
from lcmtypes import mbot_status_t


# sample structure for a complex task
class pickup_corner_block():
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
            print("Start relocating the block in corner and picking")
            time.sleep(2)
            smack_dat_corner_block(self.fsm.rexarm)
            set_snake(self.fsm.rexarm)
            time.sleep(2)
            self.fsm.mbot_status = mbot_status_t.STATUS_IN_PROGRESS
            self.fsm.moving_mbot_to_block([0, 0], dist_to_block=0.12)
            self.state = 'pull_out'
        if self.state == 'pull_out' and self.fsm.mbot_status == mbot_status_t.STATUS_COMPLETE:
            self.fsm.set_current_state('spin_state')
            self.state = 'idle'

    def begin_task(self):
        print("begin task pick up corner block")
        prepare_for_probe(self.fsm.rexarm)
        self.fsm.mbot_status = mbot_status_t.STATUS_IN_PROGRESS
        endpoint = find_closest_block(self.fsm.tags, self.fsm.extrinsic_mtx)[2]
        self.fsm.moving_mbot_to_block(endpoint, dist_to_block=0.095)
        self.state = "moving_to_closest"
