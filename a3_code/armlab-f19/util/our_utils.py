import numpy as np
import time
import lcm
import math
from lcmtypes import pose_xyt_t
from lcmtypes import occupancy_grid_t
from lcmtypes import mbot_status_t
from lcmtypes import mbot_command_t
import cv2

PICK_RANGE = 0.2

D2R = 3.141592/180.0
GRIPPER_OPEN = -30 * D2R
GRIPPER_CLOSE = 8 * D2R
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
    endpoint[2] += 0.003
    
    for phi in range(-20, -91, -10):
        endpoint[3] = phi
        joint_positions_endpoint = rexarm.rexarm_IK(endpoint)
        if joint_positions_endpoint != None:
            break
    if joint_positions_endpoint is None:
        return 0

    set_positions = [0] * 5
    set_positions[4] = GRIPPER_OPEN # open gripper

    set_positions[0] = joint_positions_endpoint[0]
    rexarm.set_positions(set_positions, update_now = True)
    time.sleep(1)
    for i in range(len(joint_positions_endpoint) - 1, 0, -1):
        set_positions[i] = joint_positions_endpoint[i]
        rexarm.set_positions(set_positions, update_now = True)
        time.sleep(1)

def pick_1x1_block_for_corner(rexarm, endpoint, initial_joints):
    print("begin picking up 1x1 block!")
    #Get to block
    GRASP_OFFSET = 0.02
    endpoint[0] += endpoint[0] / np.sqrt(endpoint[0]** 2 + endpoint[1]** 2) * GRASP_OFFSET
    endpoint[1] += endpoint[1] / np.sqrt(endpoint[0]** 2 + endpoint[1]** 2) * GRASP_OFFSET
    endpoint[2] += 0.003
    
    for phi in range(-90, -19, 10):
        endpoint[3] = phi
        joint_positions_endpoint = rexarm.rexarm_IK(endpoint)
        if joint_positions_endpoint != None:
            break
    if joint_positions_endpoint is None:
        return 0

    if initial_joints is None:
        set_positions = [0] * 5
    else:
        set_positions = initial_joints
    
    set_positions[4] = -15*D2R # open gripper

    set_positions[0] = joint_positions_endpoint[0]
    rexarm.set_positions(set_positions, update_now = True)
    time.sleep(1)
    for i in range(len(joint_positions_endpoint) - 1, 0, -1):
        set_positions[i] = joint_positions_endpoint[i]
        rexarm.set_positions(set_positions, update_now = True)
        time.sleep(1.5)

    print("got here")
    #rexarm.close_gripper()
    
    # close gripper
    set_positions[4] = GRIPPER_CLOSE
    rexarm.set_positions(set_positions, update_now = True)
    time.sleep(1)

# INITIAL_POSITION_FOR_SMACK = [0, 0.15, 2.5*I2M, -45]
def prepare_for_probe(rexarm, initial_position):
    initial_position = initial_position.copy()
    for phi in range(-20, -91, -10):
        initial_position[3] = phi
        joint_positions_endpoint = rexarm.rexarm_IK(initial_position)
        if joint_positions_endpoint != None:
            break
    print('IK returned: ',joint_positions_endpoint)
    joint_positions_endpoint += [20]
    rexarm.set_positions(joint_positions_endpoint, update_now = True)
    time.sleep(5)

def smack_dat_corner_block(rexarm, initial_position):
    initial_position = initial_position.copy()
    initial_position[2] = 1.85*I2M
    print("begin picking up corner block!")
    joint_positions_endpoint = None
    for phi in range(-20, -91, -10):
        initial_position[3] = phi
        joint_positions_endpoint = rexarm.rexarm_IK(initial_position)
        # print(joint_positions_endpoint, initial_position)
        if joint_positions_endpoint != None:
            break
    joint_positions_endpoint += [20]
    rexarm.set_positions(joint_positions_endpoint, update_now = True)
    time.sleep(5)

    # scape arm back
    SCRAPE_STEPS = 10
    for i in range(1, SCRAPE_STEPS + 1):
        initial_position[1] = initial_position[1] - 2.5 * I2M / SCRAPE_STEPS
        print("SCRAPE STEP: ", i, initial_position)
        for phi in range(-20, -91, -10):
            initial_position[3] = phi
            joint_positions_endpoint = rexarm.rexarm_IK(initial_position)
            if joint_positions_endpoint != None:
                break
        joint_positions_endpoint += [20]
        rexarm.set_positions(joint_positions_endpoint, update_now = True)
        time.sleep(1)
    print('done scraping')    
    return joint_positions_endpoint

def unfuck_snake(rexarm):
    set_positions = [0] * 5
    set_positions[0] = 0 * D2R
    set_positions[1] = 0 * D2R
    set_positions[2] = -90 * D2R
    set_positions[3] = -90 * D2R
    set_positions[4] = GRIPPER_CLOSE

    rexarm.set_positions(set_positions, update_now = True)
    time.sleep(2)

def detectColor(tag, fsm, rgb_image):
    # print("calculating the color")
    pose_homo = rot_tran_to_homo(tag.pose_R, tag.pose_t)
    intr = np.append(fsm.intrinsic_mtx, np.zeros((3,1)), axis=1)
    tag_to_pixel = np.dot(intr, pose_homo)
    pixel_top = np.dot(tag_to_pixel, np.array([I2M/2, 0, 0, 1])).reshape(3,)
    pixel_bottom = np.dot(tag_to_pixel, np.array([-I2M/2, 0, 0, 1])).reshape(3,)

    pixel_top = pixel_top / pixel_top[2]
    pixel_bottom = pixel_bottom / pixel_bottom[2]

    if pixel_top[1] < pixel_bottom[1]:
        color_pixel = pixel_top.flatten()
    else:
        color_pixel = pixel_bottom.flatten()
    color_pixel[1] = color_pixel[1] - 15

    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)
    hsv = hsv_image[int(color_pixel[1]), int(color_pixel[0])]
    color = hue_to_classification(hsv[0])
    # print('PIXELS: ', pixel_top, pixel_bottom)
    return color

def set_erect(rexarm, gripper=GRIPPER_CLOSE):
    # return home
    i = 0
    #while i < 20:
        #try:
    set_positions = [0] * 5
    set_positions[0] = 0 * D2R
    set_positions[1] = 0 * D2R
    set_positions[2] = -90 * D2R
    set_positions[3] = 90 * D2R
    set_positions[4] = gripper
        #except:
          #  i += 1
    '''
    if gripper is None:
        set_positions[4] = rexarm.get_positions()[4]
    else:
        set_positions[4] = gripper
    '''

    rexarm.set_positions(set_positions, update_now = True)

def set_snake(rexarm, gripper=10*D2R):
    set_positions = [0] * 5
    set_positions[0] = 90 * D2R
    set_positions[1] = 90 * D2R
    set_positions[2] = -90 * D2R
    set_positions[3] = -90 * D2R
    set_positions[4] = gripper
    '''
    if gripper is None:
        set_positions[4] = rexarm.get_positions()[4]
    else:
        set_positions[4] = gripper
    '''

    rexarm.set_positions(set_positions, update_now = True)

def determine_shape():
    pass

def from_AprilTag_to_pose(tag, extrinsic_mtx):
    # pose_t is the x,y,z of the center of the tag in camera frame
    pose_homo = rot_tran_to_homo(tag.pose_R, tag.pose_t)
    block_to_rex = np.dot(extrinsic_mtx, pose_homo)
    tag_pose_world = np.dot(block_to_rex, np.array([0, 0, I2M / 2, 1])).reshape(4,)
    return tag_pose_world

def knock_back(initial_position, rexarm):
    print("begin picking up 1x1 block!")
    #Get to block
    for phi in range(-20, -91, -10):
        initial_position[3] = phi
        joint_positions_endpoint = rexarm.rexarm_IK(initial_position)
        if joint_positions_endpoint != None:
            break

    set_positions = [0] * 5
    set_positions[4] = 20 # closed gripper

    set_positions[0] = joint_positions_endpoint[0]
    rexarm.set_positions(set_positions, update_now = True)
    time.sleep(1)
    for i in range(len(joint_positions_endpoint) - 1, 0, -1):
        set_positions[i] = joint_positions_endpoint[i]
        rexarm.set_positions(set_positions, update_now = True)
        time.sleep(1)
        
    '''
    while set_positions[3] > 0:
        set_positions[3] = set_positions - 10
        rexarm.set_positions(set_positions, update_now = True)
        time.sleep(1)
    '''


def locate_1x1_block(tag, extrinsic_mtx):
    pick_block_position = [0, 3*I2M, I2M/2, -90]
    # for tag in tags:
    pick_block_position = from_AprilTag_to_pose(tag, extrinsic_mtx)
    pick_block_position[3] = -45
    pick_block_position[0] = pick_block_position[0] - 0.01 # shift target 1 cm to the left
    print("Block pose: ", pick_block_position)
    return pick_block_position

def put_block_in_trash(rexarm, gripper_angle=10*D2R):
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
    # if(closest_tag_id == -1):
    #     return None
    angle = np.arctan2(-closest_pose[0], closest_pose[1])
    return (angle, euclidian_distance(closest_pose[0], closest_pose[1], 0, 0), closest_pose, closest_tag_id)

def find_closest_tag(tags, extrinsic_mtx):
    closest_pose = [9999, 9999]
    result_tag = None
    for tag in tags:
        current_pose = from_AprilTag_to_pose(tag, extrinsic_mtx)
        if(euclidian_distance(closest_pose[0], closest_pose[1], 0, 0) > euclidian_distance(current_pose[0], current_pose[1], 0, 0)):
            closest_pose = current_pose
            result_tag = tag
    return result_tag
    
def normalize_angle(angle):
    newAngle = angle
    while newAngle <= -PI: newAngle += 2*PI
    while newAngle > PI: newAngle -= 2*PI
    return newAngle

# might do kmeans later so figured we might as well do the distance thing now
def hue_to_classification(hue):
    hue = 2*hue*D2R
    # R G B P O Y
    colors = ['red', 'green', 'blue', 'purple', 'orange', 'yellow']
    # means = np.array([177.3, 75.67, 105.0, 132.833, 8.833, 28.83])
    means = np.array([2.0, 69.0, 99.0, 170.0, 10.0, 28.5])
    means = 2*means*D2R
    min_dist = float('inf')
    min_i = 0
    for i,mean in enumerate(means):
        dist = (np.cos(hue) - np.cos(mean))**2 + (np.sin(hue) - np.sin(mean))**2
        if dist < min_dist:
            min_dist = dist
            min_i = i
    return colors[min_i]