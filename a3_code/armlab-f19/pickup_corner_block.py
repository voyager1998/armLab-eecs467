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
            time.sleep(10)
            endpoint = find_closest_block(self.fsm.tags, self.fsm.extrinsic_mtx)[2]
            endpoint[2] = 3.5*I2M
            endpoint[0] = endpoint[0] + 0.01
            prepare_for_probe(self.fsm.rexarm, endpoint)
            print('done first prepare for probe')
            initial_joints = smack_dat_corner_block(self.fsm.rexarm, endpoint)
            print('smack done')
            time.sleep(3)
            prepare_for_probe(self.fsm.rexarm, endpoint)
            print('done second prepare for probe')
            time.sleep(10)
            find_closest_block(self.fsm.tags, self.fsm.extrinsic_mtx)[2]
            pick_1x1_block_for_corner(self.fsm.rexarm, endpoint, initial_joints)
            unfuck_snake(self.fsm.rexarm)
            set_snake(self.fsm.rexarm)
            self.fsm.mbot_status = mbot_status_t.STATUS_IN_PROGRESS
            self.fsm.moving_mbot_to_block([0, 0], dist_to_block=0.12)
            self.state = 'pull_out'
        if self.state == 'pull_out' and self.fsm.mbot_status == mbot_status_t.STATUS_COMPLETE:
            self.fsm.set_current_state('spin_state')
            self.state = 'idle'

    def begin_task(self):
        print("begin task pick up corner block")
        # prepare_for_probe(self.fsm.rexarm)
        endpoint = find_closest_block(self.fsm.tags, self.fsm.extrinsic_mtx)[2]
        prepare_for_probe(self.fsm.rexarm, [0, 0.18, 3.5*I2M, -45])
        self.fsm.mbot_status = mbot_status_t.STATUS_IN_PROGRESS
        self.fsm.moving_mbot_to_block(endpoint, dist_to_block=0.14)
        self.state = "moving_to_closest"
