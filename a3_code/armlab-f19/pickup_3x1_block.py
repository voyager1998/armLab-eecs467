import numpy as np
from util.our_utils import *
from lcmtypes import mbot_status_t


# sample structure for a complex task
class pickup_3x1_block():

    recent_knock_back = []

    def __init__(self, fsm, travel_square):
        self.fsm = fsm
        self.state = 'idle'
        self.travel_square = travel_square

    def dotproduct(self, v1, v2):
        return sum((a*b) for a, b in zip(v1, v2))

    def length(self, v):
        return math.sqrt(self.dotproduct(v, v))

    def angle(self, v1, v2):
        if v1 == 0 or v2 == 0:
            return 0
        return math.acos(self.dotproduct(v1, v2) / (self.length(v1) * self.length(v2)))

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
            location_of_knock_back[0] = location_of_knock_back[0] - 0.3*I2M
            location_of_knock_back[1] = location_of_knock_back[1] - 0.5*I2M
            location_of_knock_back[2] = location_of_knock_back[2] + 0.5*I2M
            self.recent_knock_back = location_of_knock_back
            print(location_of_knock_back)
            knock_back(location_of_knock_back, self.fsm.rexarm)
            self.state = 'push'
        if self.state == 'push':
            print("wtf")
            #hyp = euclidian_distance(self.recent_knock_back[0] + self.fsm.slam_pose[0], self.recent_knock_back[1] + self.fsm.slam_pose[1], self.fsm.slam_pose[0], self.fsm.slam_pose[1])
            #print("hyp", hyp)
            #v1 = [self.recent_knock_back[0] + self.fsm.slam_pose[0], self.recent_knock_back[1] + self.fsm.slam_pose[1]]
           #print("v1", v1)
            #v2 = [self.fsm.slam_pose[0], self.fsm.slam_pose[1]]
            #angle_tri = self.angle(v1, v2)
            #print("angle: ", angle_tri) ok lets try this
            self.fsm.publish_mbot_command(mbot_command_t.STATE_MOVING, (self.fsm.slam_pose[0] + 0.20*math.cos(self.fsm.slam_pose[0]), self.fsm.slam_pose[1] + 0.20*math.sin(self.fsm.slam_pose[1]), 0), [], False) #may need to recalc in different frame
            print("thought it was a bug")
            self.state = 'reverse'
        if self.state == 'reverse' and self.fsm.mbot_status == mbot_status_t.STATUS_COMPLETE:
            set_snake(self.fsm.rexarm)
            self.fsm.publish_mbot_command(mbot_command_t.STATE_MOVING, (self.travel_square.square_queue[self.travel_square.index][0], self.travel_square.square_queue[self.travel_square.index][1], 0), [], False)
            self.state = 'finished_reversing'
            self.fsm.mbot_status = 0
            print('reverse pls')
            '''
            print("block location received")
            time.sleep(1)
            print("start picking")
            pick_1x1_block(self.fsm.rexarm, new_location)
            self.state = 'idle'
            '''
        if self.state == 'finished_reversing' and self.fsm.mbot_status == mbot_status_t.STATUS_COMPLETE:
            self.fsm.set_current_state('spin_state')

    def begin_task(self):
        print("begin task pick up 1x1 block")
        target_tag = self.fsm.tags[0]  # TODO: use closest_tag fxn to find this tag
        target_pose = from_AprilTag_to_pose(target_tag, self.fsm.extrinsic_mtx)
        self.fsm.mbot_status = mbot_status_t.STATUS_IN_PROGRESS
        #self.fsm.moving_mbot(target_pose)
        self.state = "grabbing" #'moving_to_closest'
        self.return_point = []
