import numpy as np
import math
from util.our_utils import *
from lcmtypes import mbot_status_t

PICK_RANGE = 0.15
PI = 3.141592
STEP = 18
RANGE = 2

# sample structure for a complex task
class pickup_1x1_block():
    def __init__(self, fsm):
        self.fsm = fsm
        self.state = 'idle'
        self.current_step = 0
        self.closest_tag = []
        self.closest_distance = 9999
        self.closest_angle = 0
        self.closest_tag_number = -1
        self.start_theta = 0

    def operate_task(self):
        if self.state == 'idle':
            self.state = 'idle'

        if self.state == 'moving_to_closest' and self.fsm.mbot_status == mbot_status_t.STATUS_COMPLETE:
            self.fsm.mbot_status = mbot_status_t.STATUS_IN_PROGRESS
            print("reached the desired position for picking")
            self.state = 'grabbing'
            self.current_step = 0
            self.start_theta = self.fsm.slam_pose[2]

        if self.state == 'grabbing':
            print("Start relocating the block and picking")
            time.sleep(8)
            if not self.fsm.tags:
                self.state = 'scan'
                self.start_theta = normalize_angle(self.fsm.slam_pose[2] - PI / STEP * RANGE)
                self.current_step = 0
                return

            new_location = locate_1x1_block(self.fsm.tags[0], self.fsm.extrinsic_mtx)
            print("block location received")
            time.sleep(1)
            print("start picking")
            if pick_1x1_block(self.fsm.rexarm, new_location) == 0:
                self.state = 'scan'
                self.start_theta = normalize_angle(self.fsm.slam_pose[2] - PI / STEP * RANGE)
                self.current_step = 0
            else:
                self.state = 'idle'
                self.fsm.set_current_state('go_to_garbage')

        if self.state == 'scan':
            print("in scan state")
            normalizedAngle = normalize_angle(self.start_theta + PI/STEP*self.current_step)
            print("target angle:", normalizedAngle)
            self.fsm.moving_mbot((self.fsm.slam_pose[0], self.fsm.slam_pose[1], normalizedAngle), 1)
            if self.fsm.mbot_status == mbot_status_t.STATUS_COMPLETE:
                print("scanned to desired pose")
                self.fsm.mbot_status = mbot_status_t.STATUS_IN_PROGRESS
                self.state = 'watch'

        if self.state == 'watch':
            # print("in watch state")
            time.sleep(3)
            if self.current_step >= RANGE * 2 + 1:
                self.state = 'face_to_closest'
                print("---------------finish one round---------------")
                if self.closest_tag_number == -1:
                    print("No block in sight")
                    return

                print("closest angle:", self.closest_angle)
                print("closest block id:", self.closest_tag_number)
                print("closest block distance:", self.closest_distance)
                if(self.closest_tag_number == 1):
                    print("long block")

                self.state = 'face_to_closest'
            else:
                tags = self.fsm.tags
                for tag in tags:
                    print("see tag", tag.tag_id)
                closest_at_angle = find_closest_block(tags, self.fsm.extrinsic_mtx)
                if(self.closest_distance > closest_at_angle[1]):
                    self.closest_tag = closest_at_angle[2]
                    self.closest_angle = closest_at_angle[0]+ PI/STEP*self.current_step #self.fsm.slam_pose[2]
                    self.closest_tag_number = closest_at_angle[3]
                    self.closest_distance = closest_at_angle[1]
                self.state = 'scan'
                print("current step", self.current_step)
                self.current_step += 1

        if self.state == 'face_to_closest':
            self.face_closest()
            print("faced towards the closest block")
            if self.fsm.mbot_status == mbot_status_t.STATUS_COMPLETE:
                print("Turned to facing block")
                self.fsm.mbot_status = mbot_status_t.STATUS_IN_PROGRESS
                self.state = 'grabbing'
                time.sleep(5)


    def begin_task(self):
        print("begin task pick up 1x1 block")
        if not self.fsm.tags:
            self.state = 'scan'
            self.start_theta = normalize_angle(self.fsm.slam_pose[2] - PI / STEP * RANGE)
            self.current_step = 0
        else:
            target_tag = self.fsm.tags[0]  # TODO: use closest_tag fxn to find this tag
            target_pose = from_AprilTag_to_pose(target_tag, self.fsm.extrinsic_mtx)
            block_pose = locate_1x1_block(target_tag, self.fsm.extrinsic_mtx)
            self.fsm.mbot_status = mbot_status_t.STATUS_IN_PROGRESS
            if pick_1x1_block(self.fsm.rexarm, block_pose) == 0:
                print("block is far away")
                self.fsm.moving_mbot_to_block(target_pose)
                self.state = 'moving_to_closest'
                    
    def face_closest(self):
        self.fsm.moving_mbot((self.fsm.slam_pose[0], self.fsm.slam_pose[1], self.closest_angle), 1)
