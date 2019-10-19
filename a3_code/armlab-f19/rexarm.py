#rexarm.py
import numpy as np
import time
import math

""" 
TODO:

Implement the missing functions
add anything you see fit

"""
def clamp_radians_joints(theta, angle_min, angle_max):
    return np.clip(theta, angle_min, angle_max)


""" Radians to/from  Degrees conversions """
D2R = 3.141592/180.0
R2D = 180.0/3.141592

def clamp_radians(theta):
    return np.clip(theta, -np.pi, np.pi)

class Rexarm():
    def __init__(self, joints):
        self.joints = joints
        self.gripper = joints[-1]
        self.gripper_open_pos = np.deg2rad(-60.0)
        self.gripper_closed_pos = np.deg2rad(0.0)
        self.gripper_state = True
        self.estop = False
        """Find the physical angle limits of the Rexarm. Remember to keep track of this if you include more motors"""
        # in radians
        self.angle_limits = np.array([
                            [-120.00, -100.00, -100.00, -100.00, -70.00],
                            [ 120.00,  20.00,  40.00,  70.00,  22.00]], dtype=np.float)*D2R

        """ Commanded Values """
        self.num_joints = len(joints)
        self.position = [0.0] * self.num_joints     # degrees
        self.speed = [1.0] * self.num_joints        # 0 to 1
        self.max_torque = [1.0] * self.num_joints   # 0 to 1

        """ Feedback Values """
        self.joint_angles_fb = [0.0] * self.num_joints # degrees
        self.speed_fb = [0.0] * self.num_joints        # 0 to 1   
        self.load_fb = [0.0] * self.num_joints         # -1 to 1  
        self.temp_fb = [0.0] * self.num_joints         # Celsius
        self.move_fb = [0] *  self.num_joints

        """ Arm Lengths """
        # Fill in the measured dimensions.
        self.base_len     = 0.023
        self.shoulder_len = 0.055
        self.elbow_len    = 0.055
        self.wrist_len    = 0.12

        """ DH Table """
        # TODO: Fill in the variables.
        self.dh_table = [{"d" : self.base_len, "a" : 0, "alpha": 0}, \
                         {"d" : self.shoulder_len, "a" : 0, "alpha": 0}, \
                         {"d" : self.elbow_len, "a" : 0, "alpha": 0}, \
                         {"d" : self.wrist_len, "a" : 0, "alpha": 0}]

    def initialize(self):
        for joint in self.joints:
            joint.enable_torque()
            joint.set_position(0.0)
            joint.set_torque_limit(0.25)
            joint.set_speed(0.25)

    def open_gripper(self):
        """ TODO """
        self.gripper_state = False
        pass

    def close_gripper(self):
        """ TODO """
        self.gripper_state = True
        pass

    def set_positions(self, joint_angles, update_now = True):
        joint_angles = self.clamp(joint_angles)
        for i,joint in enumerate(self.joints):
            self.position[i] = joint_angles[i]
            if(update_now):
                joint.set_position(joint_angles[i])
    
    def set_speeds_normalized_global(self, speed, update_now = True):
        for i,joint in enumerate(self.joints):
            self.speed[i] = speed
            if(update_now):
                joint.set_speed(speed)

    def set_speeds_normalized(self, speeds, update_now = True):
        for i,joint in enumerate(self.joints):
            self.speed[i] = speeds[i]
            if(update_now):
                joint.set_speed(speeds[i])

    def set_speeds(self, speeds, update_now = True):
        for i,joint in enumerate(self.joints):
            self.speed[i] = speeds[i]
            speed_msg = abs(speeds[i]/joint.max_speed)
            if (speed_msg < 3.0/1023.0):
                speed_msg = 3.0/1023.0
            if(update_now):
                joint.set_speed(speed_msg)
    
    def set_torque_limits(self, torques, update_now = True):
        for i,joint in enumerate(self.joints):
            self.max_torque[i] = torques[i]
            if(update_now):
                joint.set_torque_limit(torques[i])

    def send_commands(self):
        self.set_positions(self.position)
        self.set_speeds_normalized(self.speed)
        self.set_torque_limits(self.max_torque)

    def enable_torque(self):
        for joint in self.joints:
            joint.enable_torque()

    def disable_torque(self):
        for joint in self.joints:
            joint.disable_torque()

    def get_positions(self):
        for i,joint in enumerate(self.joints):
            self.joint_angles_fb[i] = joint.get_position()
        return self.joint_angles_fb

    def get_speeds(self):
        for i,joint in enumerate(self.joints):
            self.speed_fb[i] = joint.get_speed()
        return self.speed_fb

    def get_loads(self):
        for i,joint in enumerate(self.joints):
            self.load_fb[i] = joint.get_load()
        return self.load_fb

    def get_temps(self):
        for i,joint in enumerate(self.joints):
            self.temp_fb[i] = joint.get_temp()
        return self.temp_fb

    def get_moving_status(self):
        for i,joint in enumerate(self.joints):
            self.move_fb[i] = joint.is_moving()
        return self.move_fb

    def get_feedback(self):
        self.get_positions()
        self.get_speeds()
        self.get_loads()
        self.get_temps()
        self.get_moving_status()

    def pause(self, secs):
        time_start = time.time()
        while((time.time()-time_start) < secs):
            self.get_feedback()
            time.sleep(0.05)
            if(self.estop == True):
                break

    def clamp(self, joint_angles):
        """Implement this function to clamp the joint angles"""
        for i in range(0,5):
            joint_angles[i] = np.clip(joint_angles[i], self.angle_limits[0][i], self.angle_limits[1][i])
        return joint_angles

    def calc_A_FK(self, theta, link):
        """
        TODO: Implement this function
        theta is radians of the link number
        link is the index of the joint, and it is 0 indexed (0 is base, 1 is shoulder ...)
        returns a matrix A(2D array)
        """
        A = [[1, 0, 0, 0], \
            [0, 1, 0, 0], \
            [0, 0, 1, 0], \
            [0, 0, 0, 1]]
        if link == 0: #base
            A = [[math.cos(theta), 0, -math.sin(theta), 0], \
                [math.sin(theta), 0, math.cos(theta), 0], \
                [0, -1, 1, self.dh_table[0]["d"]], \
                [0, 0, 0, 1]]            
        if link == 1:
            A = [[0, 0, 1, 0], \
                [-1, 0, 0, 0], \
                [0, -1, 0, self.dh_table[1]["d"]], \
                [0, 0, 0, 1]]    
        if link == 2:
            A = [[math.cos(theta), -math.sin(theta), 0, self.dh_table[2]["d"] * math.cos(theta)], \
                [math.sin(theta), math.cos(theta), 0, self.dh_table[2]["d"] * math.sin(theta)], \
                [0, 0, 1, 0], \
                [0, 0, 0, 1]]
        return A

    def rexarm_FK(self, joint_num=4):
        """
        TODO: implement this function
        Calculates forward kinematics for rexarm
        takes a DH table filled with DH parameters of the arm
        and the link to return the position for
        returns a 4-tuple (x, y, z, phi) representing the pose of the
        desired link
        """
        current_pose = np.array([0.0, 0.0, 0.0, 1.0])
        for i in range(0, joint_num):
            A = self.calc_A_FK(self.position[i], i)
            current_pose = np.matmul(np.array(A), current_pose)
        phi = math.pi/2 - self.position[0] - self.position[1] - self.position[2]
        return (current_pose[0], current_pose[1], current_pose[2], phi)

    def rexarm_IK(self, pose):
        """
        TODO: implement this function

        Calculates inverse kinematics for the rexarm
        pose is a tuple (x, y, z, phi) which describes the desired
        end effector position and orientation.
        If the gripper is perpendicular to the floor and facing down,
        then phi is -90 degree.
        If the gripper is parallel to the floor,
        then phi is 0 degree.
        returns a 4-tuple of joint angles or None if configuration is impossible
        """

# calculated in Radian
        pose.phi = pose.phi * D2R

        base_angle = np.arctan2(pose.y, pose.x)
        if base_angle < self.angle_limits[0][0] or base_angle > self.angle_limits[1][0]:
            print("Out of theta0 limit!")
            return None

        z3 = pose.z - self.wrist_len * math.sin(pose.phi)
        l3 = math.sqrt(pose.x ** 2 + pose.y ** 2) - self.wrist_len * math.cos(pose.phi)

        longedge2 = l3**2 + (z3-self.dh_table[0]["d"])**2
        cosalpha = (self.dh_table[1]["d"]** 2 + longedge2 - self.dh_table[2]["d"]** 2) / (2 * self.dh_table[1]["d"] * math.sqrt(longedge2))
        if cosalpha < -1 or cosalpha > 1:
            return None
        alpha = math.acos(cosalpha)
        beta = np.arctan2(z3 - self.dh_table[0]["d"], l3)
        theta1 = math.pi / 2 - alpha - beta
        if theta1 < self.angle_limits[0][1] or theta1 > self.angle_limits[1][1]:
            print("Out of theta1 limit!")
            return None
        
        cosgamma = (self.dh_table[1]["d"]** 2 + self.dh_table[2]["d"]** 2 - longedge2) / (2 * self.dh_table[1]["d"] * self.dh_table[2]["d"])
        if cosgamma < -1 or cosgamma > 1:
            return None
        gamma = math.acos(cosgamma)
        theta2 = math.pi - gamma
        if theta2 < self.angle_limits[0][2] or theta1 > self.angle_limits[1][2]:
            print("Out of theta2 limit!")
            return None

        theta3 = math.pi/2 - theta1 - theta2 - pose.phi
        if theta3 < self.angle_limits[0][3] or theta1 > self.angle_limits[1][3]:
            print("Out of theta3 limit!")
            return None

# joint angles are in radians
        joint_angles = [base_angle, theta1, theta2, theta3]
        
        return joint_angles
