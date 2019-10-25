import numpy as np
import math
import time

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

def pick_1x1_block(rexarm, endpoint, placepoint, D2R):
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
    print("target pose: ", pick_block_position)
    return pick_block_position

def euclidian_distance(pose1X, pose1Y, pose1Z, pose2X, pose2Y, pose2Z):
    return math.sqrt((pose1X - pose2X)**2 + (pose1Y - pose2Y)**2 + (pose1Z - pose2Z)**2)

def has_double_tags(tags, extrinsic_mtx):

    I2M = 0.0254
    
    for tag in tags:
        for tag2 in tags:
            tag1Pose = from_AprilTag_to_pose(tag, extrinsic_mtx)
            tag2Pose = from_AprilTag_to_pose(tag2, extrinsic_mtx)
            if euclidian_distance(tag1Pose[0], tag1Pose[1], tag1Pose[2], tag2Pose[0], tag2Pose[1], tag2Pose[2]) < 1.5 * I2M):
                return True
    return False

    