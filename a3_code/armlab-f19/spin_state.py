import numpy as np
import time
from util.our_utils import *

PI = 3.141592
STEP = 2

# sample structure for a complex task
class spin_state():

    def __init__(self, fsm):
        self.fsm = fsm
        self.current_step = 0
        self.state = 'idle'
        self.closest_tag = []
        self.closest_distance = 9999
        self.closest_angle = 0
        self.closest_tag_number = -1
        self.start_theta = 0
        self.tag_1_count = 0
        self.found_tag = False

    def operate_task(self):
        if self.state == 'idle':
            self.state = 'idle'

        if self.state == 'spin':
            # print("in spin state")
            self.fsm.get_mbot_feedback()
            normalizedAngle = normalize_angle(self.start_theta + PI/STEP*self.current_step)
            # print("target angle:", normalizedAngle)
            self.fsm.moving_mbot((self.fsm.slam_pose[0], self.fsm.slam_pose[1], normalizedAngle), 1)
            self.state = 'watch'

        if self.state == 'watch' and self.fsm.mbot_status == mbot_status_t.STATUS_COMPLETE:
            print("spinned to desired pose")
            self.fsm.mbot_status = mbot_status_t.STATUS_IN_PROGRESS
            print("in watch state")
            time.sleep(3)
            self.fsm.get_mbot_feedback()
            if self.current_step >= STEP * 2:
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
                self.face_closest()
            else:
                tags = self.fsm.tags
                for tag in tags:
                    print("see tag", tag.tag_id)
                    self.found_tag = True
                    if(tag.tag_id == 1):
                        tag_1_count += 1
                closest_at_angle = find_closest_block(tags, self.fsm.extrinsic_mtx)
                if self.closest_distance > closest_at_angle[1] and closest_at_angle[3] != 7:
                    self.closest_tag = closest_at_angle[2]
                    self.closest_angle = closest_at_angle[0]+ PI/STEP*self.current_step #self.fsm.slam_pose[2]
                    self.closest_tag_number = closest_at_angle[3]
                    self.closest_distance = closest_at_angle[1]
                self.state = 'spin'
                print("current step", self.current_step)
                self.current_step += 1

        if self.state == 'face_to_closest' and self.fsm.mbot_status == mbot_status_t.STATUS_COMPLETE:
            print("faced towards the closest block")
            self.fsm.get_mbot_feedback()
            self.fsm.mbot_status = mbot_status_t.STATUS_IN_PROGRESS
            self.state = 'idle'
            time.sleep(2)

    def begin_task(self):
        print("begin spinning")
        self.fsm.mbot_status = mbot_status_t.STATUS_COMPLETE
        self.state = 'watch'
        self.current_step = 0
        self.start_theta = self.fsm.slam_pose[2]
        self.tag_7_count = 0
        self.found_tag = False

    def face_closest(self):
        self.fsm.moving_mbot((self.fsm.slam_pose[0], self.fsm.slam_pose[1], self.closest_angle), 1)
