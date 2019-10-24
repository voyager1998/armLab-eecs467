import numpy as np
import time

# sample structure for a complex task
class task5():
    def __init__(self, StateMachine):
        self.fsm = StateMachine
        self.current_step = 0

    def operate_task(self):
        """TODO"""
        pass

    def begin_task(self, endpoint, placepoint, D2R):
        rexarm = self.fsm.rexarm
        print("begin task 5!")
        current_angles = []

        #Get to block
        for phi in range(-20, -91, -10):
            endpoint[3] = phi
            joint_positions_endpoint = rexarm.rexarm_IK(endpoint)
            if joint_positions_endpoint != None:
                break

        set_positions = [0] * 5
        set_positions[4] = -20 # open gripper

        set_positions[0] = joint_positions_endpoint[0]
        rexarm.set_positions(set_positions, update_now = True)
        current_angles = set_positions
        time.sleep(1)
        for i in range(len(joint_positions_endpoint) - 1, 0, -1):
            set_positions[i] = joint_positions_endpoint[i]
            rexarm.set_positions(set_positions, update_now = True)
            current_angles = set_positions
            time.sleep(1)
        
        # close gripper
        set_positions[4] = 5
        rexarm.set_positions(set_positions, update_now = True)
        time.sleep(1)

        #Pick it up
        # for i in range(len(joint_positions_endpoint) - 1, 0, -1):
        #     set_positions[i] = joint_positions_endpoint[i]
        #     rexarm.set_positions(set_positions, update_now = True)
        #     current_angles = set_positions
        #     time.sleep(1)
        # set_positions[4] = 20 # close gripper
        # rexarm.set_positions(set_positions, update_now = True)

        # joint_positions_placepoint = rexarm.rexarm_IK(placepoint)
        # #Put it down in correct location
        # for i in range(len(joint_positions_placepoint) - 1, -1, -1):
        # 	set_positions = [0] * 5
        # 	for x in range(i, len(joint_positions_placepoint)):
        # 		set_positions[x] = joint_positions_placepoint[x]
        # 	if i == 0:
        # 		set_positions[4] = 20 #Find a constant that works here
        # 	rexarm.set_positions(set_positions, update_now = True)
        # 	time.sleep(0.05)
            

