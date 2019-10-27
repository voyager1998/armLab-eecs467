import numpy as np
from util.our_utils import *
from lcmtypes import mbot_status_t


# sample structure for a complex task
class travel_square():

    square_queue = [(0, 0.5), (0, 1), (-0.5, 1), (-1, 1), (-1, 0.5), (-1, 0), (-0.5, 0), (0, 0)]

    def __init__(self, fsm):
        self.fsm = fsm
        self.state = 'idle'

    def operate_task(self):
        if self.state == 'moving_in_square' and self.fsm.mbot_status == mbot_status_t.STATUS_COMPLETE:
            self.fsm.set_current_state('spin_state')
        if self.state == 'move_to_waypoint':
            print("Move to waypoint in square")
            self.fsm.publish_mbot_command(mbot_command_t.STATE_MOVING, (self.square_queue[0][0], self.square_queue[0][1], 0), [])
            front = self.square_queue.pop(0)
            self.square_queue.append(front)
            self.state = 'moving_in_square'

    def begin_task(self):
        #print("begin task move to square")
        #target_tag = self.fsm.tags[0]  # TODO: use closest_tag fxn to find this tag
        #target_pose = from_AprilTag_to_pose(target_tag, self.fsm.extrinsic_mtx)
        self.fsm.mbot_status = mbot_status_t.STATUS_IN_PROGRESS
        #self.fsm.moving_mbot(target_pose)
        self.state = 'move_to_waypoint'
