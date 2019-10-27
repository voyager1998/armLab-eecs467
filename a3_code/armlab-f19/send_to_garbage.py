import numpy as np
from util.our_utils import *
from lcmtypes import mbot_status_t


# sample structure for a complex task
class send_to_garbage():

    square_queue = []
    original_position = 0
    reached_destination = False
    mode = "TO_GARBAGE"
    index_to_garbage = 0
    index_to_return = 0


    PI = 3.141592

    def __init__(self, fsm, travel_square):
        self.fsm = fsm
        self.travel_square = travel_square
        self.state = "idle"

    def operate_task(self):
        if(mode == "TO_GARBAGE"):
            if self.state == "moving_in_square" and self.fsm.mbot_status == mbot_status_t.STATUS_COMPLETE:
                if self.square_queue[self.index_to_garbage] == (0, 0):
                    self.state == "reached_garbage"
                else:
                    self.state == "move_to_waypoint"
            if self.state == "move_to_waypoint":
                print("Move to waypoint_forward in square")
                self.fsm.publish_mbot_command(mbot_command_t.STATE_MOVING, (self.square_queue[index_to_garbage][0], self.square_queue[index_to_garbage][1], 0), [])
                if(self.index_to_garbage == 0):
                    self.index_to_garbage == len(square_queue) - 1
                else:
                    self.index_to_garbage -= 1
                self.state = "moving_in_square"
            if self.state == "reached_garbage":
                print("Move to garbage can")
                self.fsm.publish_mbot_command(mbot_command_t.STATE_MOVING, (-0.25, 0, self.PI), [])
                set_erect(self.fsm.rexarm)
                put_block_in_trash(self.fsm.rexarm)
                self.mode = "TO_ORIGIN"
                self.state = "move_to_waypoint"
        elif (self.mode == "TO ORIGIN"):
            if self.state == "moving_in_square" and self.fsm.mbot_status == mbot_status_t.STATUS_COMPLETE:
                if self.index_to_return == self.original_position:
                    self.fsm.set_current_state("spin_state")
                else:
                    self.state = "move_to_waypoint"
            if self.state == "move_to_waypoint":
                print("Move to waypoint_back in square")
                self.fsm.publish_mbot_command(mbot_command_t.STATE_MOVING, (self.square_queue[self.index_to_return][0], self.square_queue[self.index_to_return][1], 0), [])
                self.index_to_return += 1
                self.state = "moving_in_square"

    def begin_task(self):
        self.fsm.mbot_status = mbot_status_t.STATUS_IN_PROGRESS
        self.square_queue = self.travel_square.square_queue
        self.original_position = self.travel_square.index
        self.index_to_garbage = self.travel_square.index
        self.mode = "TO_GARBAGE"
        self.state = "move_to_waypoint"
        
