import numpy as np
import time
import lcm
import math
from lcmtypes import pose_xyt_t
from lcmtypes import occupancy_grid_t
from lcmtypes import mbot_status_t
from lcmtypes import mbot_command_t


PICK_RANGE = 0.2

D2R = 3.141592/180.0
GRIPPER_OPEN = -60 * D2R
I2M = 0.0254

PI = 3.141592

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

def pick_1x1_block(rexarm, endpoint):
    print("begin picking up 1x1 block!")
    #Get to block
    GRASP_OFFSET = 0.02
    endpoint[0] += endpoint[0] / np.sqrt(endpoint[0]** 2 + endpoint[1]** 2) * GRASP_OFFSET
    endpoint[1] += endpoint[1] / np.sqrt(endpoint[0]** 2 + endpoint[1]** 2) * GRASP_OFFSET
    
    for phi in range(-20, -91, -10):
        endpoint[3] = phi
        joint_positions_endpoint = rexarm.rexarm_IK(endpoint)
        if joint_positions_endpoint != None:
            break

    set_positions = [0] * 5
    set_positions[4] = GRIPPER_OPEN # open gripper

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
    set_positions[4] = 10 * D2R
    rexarm.set_positions(set_positions, update_now = True)
    time.sleep(1)

    set_erect(rexarm)
    time.sleep(2)
    
    set_snake(rexarm)
    time.sleep(1)

def set_erect(rexarm, gripper=None):
    # return home
    set_positions = [0] * 5
    set_positions[0] = 0 * D2R
    set_positions[1] = 0 * D2R
    set_positions[2] = -90 * D2R
    set_positions[3] = 90 * D2R
    if gripper is None:
        set_positions[4] = rexarm.get_positions()[4]
    else:
        set_positions[4] = gripper

    rexarm.set_positions(set_positions, update_now = True)

def set_snake(rexarm, gripper=None):
    set_positions = [0] * 5
    set_positions[0] = 90 * D2R
    set_positions[1] = 90 * D2R
    set_positions[2] = -90 * D2R
    set_positions[3] = -90 * D2R
    if gripper is None:
        set_positions[4] = rexarm.get_positions()[4]
    else:
        set_positions[4] = gripper

    rexarm.set_positions(set_positions, update_now = True)

def determine_shape():
    pass

def from_AprilTag_to_pose(tag, extrinsic_mtx):
    # pose_t is the x,y,z of the center of the tag in camera frame
    pose_homo = rot_tran_to_homo(tag.pose_R, tag.pose_t)
    block_to_rex = np.dot(extrinsic_mtx, pose_homo)
    tag_pose_world = np.dot(block_to_rex, np.array([0, 0, I2M / 2, 1])).reshape(4,)
    return tag_pose_world


def locate_1x1_block(tags, extrinsic_mtx):
    pick_block_position = [0, 3*I2M, I2M/2, -90]
    for tag in tags:
        pick_block_position = from_AprilTag_to_pose(tag, extrinsic_mtx)
    pick_block_position[3] = -45
    pick_block_position[0] = pick_block_position[0] - 0.01 # shift target 1 cm to the left
    print("Block pose: ", pick_block_position)
    return pick_block_position

def put_block_in_trash(rexarm, gripper_angle):
    set_positions = [0]*5
    set_positions[0] = 0 * D2R
    set_positions[1] = 0 * D2R
    set_positions[2] = 0 * D2R
    set_positions[3] = -90 * D2R
    set_positions[4] = gripper_angle
    rexarm.set_positions(set_positions, update_now = True)
    time.sleep(1)

    set_positions[4] = GRIPPER_OPEN
    rexarm.set_positions(set_positions, update_now = True)
    time.sleep(1)


def runToBlock(block_pose):
    pass

def run_and_pick_util(block_pose): # block_pose = [x, y, z]
    if np.sqrt(block_pose[0]** 2 + block_pose[1]** 2) > PICK_RANGE:
        pass
    else:
        pass

def euclidian_distance(pose1_x, pose1_y, pose2_x, pose2_y):
    # print(pose1_x, pose2_x, pose1_y, pose2_y)
    return math.sqrt((pose1_x - pose2_x)**2 + (pose1_y - pose2_y)**2)

def get_tag_positions(tags, extrinsic_mtx):
    tag_poses = []
    for tag in tags:
        tag_poses.append(from_AprilTag_to_pose(tag, extrinsic_mtx))
    return tag_poses

def find_closest_block(tags, extrinsic_mtx):
    closest_pose = [9999, 9999]
    closest_tag_id = -1
    for tag in tags:
        current_pose = from_AprilTag_to_pose(tag, extrinsic_mtx)
        if(euclidian_distance(closest_pose[0], closest_pose[1], 0, 0) > euclidian_distance(current_pose[0], current_pose[1], 0, 0)):
            closest_pose = current_pose
            closest_tag_id = tag.tag_id
    # angle positive in counter-clockwise
    angle = np.arctan2(-closest_pose[0], closest_pose[1])
    return (angle, euclidian_distance(closest_pose[0], closest_pose[1], 0, 0), closest_pose, closest_tag_id)

def normalize_angle(angle):
    newAngle = angle
    while newAngle <= -PI: newAngle += 2*PI
    while newAngle > PI: newAngle -= 2*PI
    return newAngle
