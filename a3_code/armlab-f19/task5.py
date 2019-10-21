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
        joint_positions_endpoint = rexarm.rexarm_IK(endpoint)
        for i in range(len(joint_positions_endpoint) - 1, -1, -1):
            set_positions = [0] * 5
            for x in range(i, len(joint_positions_endpoint)):
                set_positions[x] = joint_positions_endpoint[x]
            if i == 0:
                set_positions[4] = 20 #Find a constant that works here
            rexarm.set_positions(set_positions, update_now = True)
            current_angles = set_positions
            time.sleep(0.05)
        
        # #Pick it up
        # for i in range(0, len(current_angles) - 1):
        # 	current_angles[i] = 0
        # 	rexarm.set_positions(current_angles, update_now = True)
        # 	time.sleep(0.05)

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
            

