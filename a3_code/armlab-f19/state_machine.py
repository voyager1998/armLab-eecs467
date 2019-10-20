import time
import numpy as np
import cv2

import lcm
import os
os.sys.path.append('lcmtypes/')
# Messages used for communication with Mbot programs.
# TODO: Add your customized lcm messages as needed.
from lcmtypes import pose_xyt_t
from lcmtypes import occupancy_grid_t
from lcmtypes import mbot_status_t
from lcmtypes import mbot_command_t

D2R = 3.141592/180.0
R2D = 180.0/3.141592

"""
TODO: Add states and state functions to this class
        to implement all of the required logics
"""
class StateMachine():
    def __init__(self, rexarm, planner):
        self.rexarm = rexarm
        self.tp = planner
        self.tags = []
        self.status_message = "State: Idle"
        self.current_state = "idle"

        self.lc = lcm.LCM()
        lcmSLAMPoseSub = self.lc.subscribe("SLAM_POSE", self.slampose_feedback_handler)
        lcmSLAMPoseSub.set_queue_capacity(1)

        lcmMbotStatusSub = self.lc.subscribe("MBOT_STATUS", self.mbotstatus_feedback_handler)

        # TODO: Add more variables here, such as RBG/HSV image here.
        self.current_step = 0
        self.path = []
        self.start_time = 0
        self.duration = 0
        self.slam_pose = None
        rotation_matrix = [[-0.0299101 , -0.9994962 , -0.01061728],
        [-0.36609689,  0.02083821, -0.9303434 ],
        [ 0.93009594, -0.02393971, -0.36653572]]
        tvec = [[ 0.00230733],[ 0.09105763],[-0.00400389]]
        # extrinsic_mtx translates from points in camera frame to world frame
        self.extrinsic_mtx = np.linalg.inv(self.rot_tran_to_homo(rotation_matrix, tvec))

    def rot_tran_to_homo(self, rotation_matrix, tvec):
        extrinsic = np.append(rotation_matrix, tvec, axis=1)
        bottom_row = [[0,0,0,1]]
        extrinsic = np.append(extrinsic, bottom_row, axis=0)
        return extrinsic

    def set_current_state(self, state):
        self.current_state = state

    """ This function is run continuously in a thread"""

    def run(self):
        if(self.current_state == "manual"):
            self.manual()

        if(self.current_state == "idle"):
            self.idle()
                
        if(self.current_state == "estop"):
            self.estop()  

        if(self.current_state == "calibrate"):
            self.calibrate()

        if(self.current_state == "moving_arm"):
            self.moving_arm()

        if(self.current_state == "moving_mbot"):
            self.moving_mbot()

        self.get_mbot_feedback()
        self.rexarm.get_feedback()
               

    """Functions run for each state"""
    def manual(self):
        self.status_message = "State: Manual - Use sliders to control arm"
        self.rexarm.send_commands()

    def idle(self):
        self.status_message = "State: Idle - Waiting for input"

    def estop(self):
        self.status_message = "EMERGENCY STOP - Check Rexarm and restart program"
        self.rexarm.disable_torque()
        
    def calibrate(self):
        """Perform camera calibration here
        TODO: Use appropriate cameraMatrix (intrinsics) parameters
        Change the arm frame 3d coordinate based on the point that you choose.
        Store the extrinsic matrix and load it on start.
        """
        # cameraMatrix = np.array([[610, 0, 320], [0, 610, 240], [0, 0, 1]])
        cameraMatrix = np.array([[639.86127538, 0, 320.40954469], [0, 637.62613535, 220.36223075], [0, 0, 1]])
        # 3D coordinates of the center of AprilTags in the arm frame in meters.
        I2M = 0.0254
        # To calibrate, form a square of cubes 8 inches from the front roller that looks like this
        # oo
        # oo
        # just
        objectPoints = np.array([[-1*I2M/2, 6*I2M, -0.05],
                                [1*I2M/2, 6*I2M, -0.05],
                                [-1*I2M/2, 6*I2M/2, -0.05+I2M],
                                [1*I2M/2, 6*I2M/2, -0.05+I2M]])

        # Use the center of the tags as image points. Make sure they correspond to the 3D points.
        imagePoints = np.array([tag.center for tag in self.tags])
        print(self.tags)
        print('imagePoints:', imagePoints)
        success, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, None)
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        print('rotation_matrix:',repr(rotation_matrix))
        print('tvec:', repr(tvec))

        # TODO: implement the function that transform pose_t of the tag to the arm's
        # frame of reference.
        # Update extrinsic_mtx but DONT persist it
        self.extrinsic_mtx = np.linalg.inv(self.rot_tran_to_homo(rotation_matrix, tvec))
        for tag in self.tags:
            # pose_t is the x,y,z of the center of the tag in camera frame
            pose_t = np.append(tag.pose_t,[1]).reshape((4,1))
            pose_t_rex = np.dot(self.extrinsic_mtx, pose_t)
            print('image pt, rex_pt: ',pose_t, pose_t_rex)
        
        self.status_message = "Calibration - Completed Calibration"
        time.sleep(1)

        # Set the next state to be idle
        self.set_current_state("idle")

    # def image_to_rex_pt(self, im_pt):


    def moving_arm(self):
        """TODO: Implement this function"""
        self.rexarm.send_commands()

    def moving_mbot(self):
        """TODO: Implement this function"""
        self.publish_mbot_command(mbot_command_t.STATE_MOVING, (1, 1, 0), [(2,2,0), (1,3,0)])


    def slampose_feedback_handler(self, channel, data):
        """
        Feedback Handler for slam pose
        this is run when a feedback message is recieved
        """
        msg = pose_xyt_t.decode(data)
        self.slam_pose = (msg.x, msg.y, msg.theta)

    def mbotstatus_feedback_handler(self, channel, data):
        """
        Feedback Handler for mbot status
        this is run when a feedback message is recieved
        """
        msg = mbot_status_t.decode(data)

    def get_mbot_feedback(self):
        """
        LCM Handler function
        Must be called continuously in the loop to get feedback.
        """
        self.lc.handle_timeout(10)

    def publish_mbot_command(self, state, goal_pose, obstacles):
        """
        Publishes mbot command.
        """
        msg = mbot_command_t()
        msg.utime = int(time.time() * 1e6)
        msg.state = state

        if state == mbot_command_t.STATE_STOPPED:
            pass
        elif state == mbot_command_t.STATE_MOVING:
            msg.goal_pose.x, msg.goal_pose.y, msg.goal_pose.theta = 1, 1, 1
            msg.num_obstacles = len(obstacles)
            for i in range(msg.num_obstacles):
                obs_pose = pose_xyt_t()
                obs_pose.utime = int(time.time() * 1e6)
                obs_pose.x, obs_pose.y, obs_pose.theta = obstacles[i]
                msg.obstacle_poses.append(obs_pose)
        else:
            raise NameError('Unknown mbot commanded state')

        self.lc.publish("MBOT_COMMAND", msg.encode())

    """TODO: Add more functions and states in the state machine as needed"""

