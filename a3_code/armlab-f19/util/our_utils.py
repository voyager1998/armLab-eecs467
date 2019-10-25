import numpy as np
import time
import lcm
from lcmtypes import pose_xyt_t
from lcmtypes import occupancy_grid_t
from lcmtypes import mbot_status_t
from lcmtypes import mbot_command_t


PICK_RANGE = 0.2

def rot_tran_to_homo(rotation_matrix, tvec):
    extrinsic = np.append(rotation_matrix, tvec, axis=1)
    bottom_row = [[0,0,0,1]]
    extrinsic = np.append(extrinsic, bottom_row, axis=0)
    return extrinsic

def return_home(rexarm, gripper_angle):
    print("begin returning home!")
    home_pose = [0] * 5
    home_pose[4] = gripper_angle
    rexarm.set_positions(home_pose, update_now = True)    

def pick_1x1_block(rexarm, endpoint, D2R):
    print("begin picking up 1x1 block!")
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
    time.sleep(1)
    for i in range(len(joint_positions_endpoint) - 1, 0, -1):
        set_positions[i] = joint_positions_endpoint[i]
        rexarm.set_positions(set_positions, update_now = True)
        time.sleep(1)

    print("got here")
    #rexarm.close_gripper()
    
    # close gripper
    set_positions[4] = 4 * D2R
    rexarm.set_positions(set_positions, update_now = True)
    time.sleep(1)

    # return home
    set_positions[0] = 0
    set_positions[1] = 0
    set_positions[2] = -90 * D2R
    set_positions[3] = 90 * D2R
    print("got here 2")
    rexarm.set_positions(set_positions, update_now = True)
    time.sleep(1)

def determine_shape():
    pass

def from_AprilTag_to_pose(tag, extrinsic_mtx):
    I2M = 0.0254
    # pose_t is the x,y,z of the center of the tag in camera frame
    pose_homo = rot_tran_to_homo(tag.pose_R, tag.pose_t)
    block_to_rex = np.dot(extrinsic_mtx, pose_homo)
    tag_pose_world = np.dot(block_to_rex, np.array([0, 0, I2M / 2, 1])).reshape(4,)
    return tag_pose_world


def locate_1x1_block(tags, extrinsic_mtx):
    I2M = 0.0254
    pick_block_position = [0, 3*I2M, I2M/2, -90]
    for tag in tags:
        pick_block_position = from_AprilTag_to_pose(tag, extrinsic_mtx)
    pick_block_position[3] = -45
    pick_block_position[0] = pick_block_position[0] - 0.01 # shift target 1 cm to the left
    print("Block pose: ", pick_block_position)
    return pick_block_position

def run_and_pick_util(block_pose): # block_pose = [x, y, z]
    if np.sqrt(block_pose[0]** 2 + block_pose[1]** 2) > PICK_RANGE:

        pass
    else:

        pass

    pass