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
from util.our_utils import *
from pickup_1x1_block import pickup_1x1_block
from pickup_3x1_block import pickup_3x1_block
from pickup_corner_block import pickup_corner_block
from travel_square import travel_square
from spin_state import spin_state
from go_to_garbage import go_to_garbage

D2R = 3.141592/180.0
R2D = 180.0/3.141592
PICK_WALL_OFFSET = 0.03
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
        self.image = None

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
        rotation_matrix = [[ 0.99995118,  0.00709628, -0.00687564],
        [-0.00444373, -0.29853831, -0.95438731],
        [-0.00882524,  0.95437127, -0.2984922 ]]
        tvec = [[-0.00106227],[ 0.08470473],[-0.02910629]]
        # extrinsic_mtx translates from points in camera frame to world frame
        self.extrinsic_mtx = np.linalg.inv(rot_tran_to_homo(rotation_matrix, tvec))
        self.intrinsic_mtx = cameraMatrix = np.array([[596.13380911,   0,         322.69869837],
                                [0,         598.59497209, 232.09155051],
                                [0,           0,           1        ]])
        self.recent_color = None

        self.mbot_status = mbot_status_t.STATUS_COMPLETE
        self.pickup_1x1_block = pickup_1x1_block(self)
        self.pickup_corner_block = pickup_corner_block(self)
        self.travel_square = travel_square(self)
        self.spin_state = spin_state(self)
        self.pickup_3x1_block = pickup_3x1_block(self, travel_square)
        self.go_to_garbage = go_to_garbage(self, travel_square)

    def set_current_state(self, state):
        self.current_state = state
        if state not in ['manual', 'idle', 'estop', 'calibrate', 'moving_arm', 'moving_mbot']:
            state_obj = getattr(self, state)
            state_obj.begin_task()

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
            # self.moving_mbot()
            pass

        # this calls operate_task on the pickup_1x1_block object, pickup_corner_block object, etc
        if (self.current_state in ['pickup_1x1_block', 'spin_state', 'pickup_3x1_block', 'pickup_corner_block', 'travel_square', 'go_to_garbage']):
            # print("current state is ", self.current_state)
            # print("bot status:", self.mbot_status)
            time.sleep(1)
            state_obj = getattr(self, self.current_state)
            state_obj.operate_task()

            # if self.current_state in ['spin_state'] and state_obj.state == 'idle':
            #     time.sleep(5)
            #     # temp_pose = self.slam_pose
            #     # self.moving_mbot((temp_pose[0], temp_pose[1], temp_pose[2] - 0.1), 1)
            #     # if (self.mbot_status == mbot_status_t.STATUS_COMPLETE):
            #     #     self.moving_mbot((temp_pose[0], temp_pose[1], temp_pose[2]), 1)
            #     if state_obj.tag_1_count <= 1:
            #         self.set_current_state('pickup_1x1_block')
            #     else:
            #         self.set_current_state('pickup_3x1_block')
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
        # cameraMatrix = np.array([[639.86127538, 0,         320.40954469],
        #                         [ 0,         637.62613535, 220.36223075],
        #                         [0, 0, 1]])
        cameraMatrix = np.array([[596.13380911,   0,         322.69869837],
                                [0,         598.59497209, 232.09155051],
                                [0,           0,           1        ]])

        distortionCoeff = np.array([ 0.33714855, -1.52702923,  0.01138424,  0.00398338,  2.62973717])
                                
        # 3D coordinates of the center of AprilTags in the arm frame in meters.
        # To calibrate, form a square of cubes 8 inches from the front roller that looks like this
        # 34
        # 56
        # (order matters!!!! from bottom to top)
        objectPoints = np.array([[-1*I2M/2, 6*I2M, I2M/2*3],
                                [1*I2M / 2, 6*I2M, I2M/2*3],
                                [-1*I2M/2, 6*I2M, I2M/2],
                                [1*I2M/2, 6*I2M, I2M/2]])

        # Use the center of the tags as image points. Make sure they correspond to the 3D points.
        imagePoints = np.array([tag.center for tag in self.tags])
        print(self.tags)
        print('imagePoints:', imagePoints)
        success, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, distortionCoeff)
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        print('rotation_matrix:',repr(rotation_matrix))
        print('tvec:', repr(tvec))

        # TODO: implement the function that transform pose_t of the tag to the arm's
        # frame of reference.
        # Update extrinsic_mtx but DONT persist it
        self.extrinsic_mtx = np.linalg.inv(rot_tran_to_homo(rotation_matrix, tvec))
        for tag in self.tags:
            # pose_t is the x,y,z of the center of the tag in camera frame
            pose_t = np.append(tag.pose_t,[1]).reshape((4,1))
            pose_t_rex = np.dot(self.extrinsic_mtx, pose_t)
            print('tag: ', tag.tag_id ,'rex_pt:\n', pose_t_rex)
        
        self.status_message = "Calibration - Completed Calibration"
        time.sleep(1)

        # Set the next state to be idle
        self.set_current_state("idle")

    def moving_arm(self):
        """TODO: Implement this function"""
        self.rexarm.send_commands()

    def moving_mbot(self, target_pose_world, is_mode_spin):
        self.publish_mbot_command(mbot_command_t.STATE_MOVING,
                (target_pose_world[0], target_pose_world[1], target_pose_world[2]), [], is_mode_spin)

    DIST_TO_BLOCK = 0.12
    def moving_mbot_to_block(self, block_pose, dist_to_block=DIST_TO_BLOCK): # block_pose = [x, y, z] is the block pose in arm frame
        """TODO: Implement this function"""
        if self.slam_pose == None:
            self.slam_pose = [0, 0, 0]
        # mbot_to_world = np.array([[np.cos(self.slam_pose[2]), -np.sin(self.slam_pose[2]), 0, self.slam_pose[0]],
        #                             [np.sin(self.slam_pose[2]), np.cos(self.slam_pose[2]), 0, self.slam_pose[1]],
        #                             [0,0,1,0],
        #                             [0, 0, 0, 1]])
        # rex_to_mbot = np.array([[0, 1,0,-dist_to_block],
        #                         [-1,0,0,0],
        #                         [0, 0,1,0],
        #                         [0, 0,0,1]])
        # block_pose = block_pose.flatten()[0:3].reshape((3,1))
        # block_pose = np.append(block_pose,[1])
        # tar_pose = mbot_to_world @ rex_to_mbot @ block_pose
        # tar_x, tar_y = tar_pose[0], tar_pose[1]
        # print("Nico's block pose in world:", tar_x, tar_y)

        temp_theta = self.slam_pose[2]
        block_pose_world_x = self.slam_pose[0] + block_pose[0] * np.sin(temp_theta) + block_pose[1] * np.cos(temp_theta)
        block_pose_world_y = self.slam_pose[1] + block_pose[1] * np.sin(temp_theta) - block_pose[0] * np.cos(temp_theta)
        
        if block_pose_world_y > 0.8 and block_pose_world_y < 1.2:
            if block_pose_world_x > 0:
                print("this is a block near the wall")
                return (block_pose_world_x, block_pose_world_y-dist_to_block-PICK_WALL_OFFSET)
            else:
                print("this is a block in the corner")
                return (block_pose_world_x+dist_to_block+PICK_WALL_OFFSET, block_pose_world_y-dist_to_block-PICK_WALL_OFFSET)

        block_dist = np.sqrt(block_pose[0]** 2 + block_pose[1]** 2)
        if block_dist > dist_to_block:
            partial = (block_dist - dist_to_block) / block_dist
            print("partial = ", partial)
        else:
            partial = 1
        print("current theta:", temp_theta)
        print("current SLAM pose:", self.slam_pose[0], self.slam_pose[1])
        print("current block pose in arm:", block_pose[0], block_pose[1])
        tar_x = self.slam_pose[0] + partial*block_pose[0] * np.sin(temp_theta) + partial*block_pose[1] * np.cos(temp_theta)
        tar_y = self.slam_pose[1] + partial*block_pose[1] * np.sin(temp_theta) - partial*block_pose[0] * np.cos(temp_theta)
        print("Kun's block pose in world:", tar_x, tar_y)
        self.publish_mbot_command(mbot_command_t.STATE_MOVING, (tar_x, tar_y, 0), [], 0)
        return (tar_x, tar_y)


    def slampose_feedback_handler(self, channel, data):
        """
        Feedback Handler for slam pose
        this is run when a feedback message is recieved
        """
        msg = pose_xyt_t.decode(data)
        # print("new slam pose received:", msg.x, msg.y, msg.theta)
        self.slam_pose = (msg.x, msg.y, msg.theta)

    def mbotstatus_feedback_handler(self, channel, data):
        """
        Feedback Handler for mbot status
        this is run when a feedback message is recieved
        """
        # print("hahaha")
        msg = mbot_status_t.decode(data)
        # print("new status received:", msg.status)
        self.mbot_status = msg.status

    def get_mbot_feedback(self):
        """
        LCM Handler function
        Must be called continuously in the loop to get feedback.
        """
        self.lc.handle_timeout(10)

    def publish_mbot_command(self, state, goal_pose, obstacles, is_mode_spin):
        """
        Publishes mbot command.
        """
        msg = mbot_command_t()
        msg.utime = int(time.time() * 1e6)
        msg.state = state
        msg.is_mode_spin = is_mode_spin

        if state == mbot_command_t.STATE_STOPPED:
            pass
        elif state == mbot_command_t.STATE_MOVING:
            msg.goal_pose.x, msg.goal_pose.y, msg.goal_pose.theta = goal_pose[0], goal_pose[1], goal_pose[2]
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

