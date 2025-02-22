#!/usr/bin/env python
# //==============================================================================
# /*
#     Software License Agreement (BSD License)
#     Copyright (c) 2020-2021 Johns Hopkins University (JHU), Worcester Polytechnic Institute (WPI) All Rights Reserved.


#     All rights reserved.

#     Redistribution and use in source and binary forms, with or without
#     modification, are permitted provided that the following conditions
#     are met:

#     * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.

#     * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.

#     * Neither the name of authors nor the names of its contributors may
#     be used to endorse or promote products derived from this software
#     without specific prior written permission.

#     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#     COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#     ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#     POSSIBILITY OF SUCH DAMAGE.


#     \author    <amunawar@jhu.edu>
#     \author    Adnan Munawar
#     \version   1.0
# */
# //==============================================================================

import argparse
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, Vector3, Quaternion
from std_msgs.msg import String, Float32
import rospy
import math
from PyKDL import Rotation
import numpy as np
from scipy.spatial.transform import Rotation as sc_Rotation
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_from_matrix
# from scipy.spatial.transform import Rotation as R

class RobotData:
    def __init__(self):
        self.measured_js = JointState()
        self.measured_cp = TransformStamped()

class ControllerData:
    def __init__(self):
        self.left_pos = [Vector3(0.0, 0.0, 0.0)]
        self.right_pos = [Vector3(0.0, 0.0, 0.0)]
        self.left_rot = Quaternion(0.0, 0.0, 0.0, 0.0)
        self.right_rot = Quaternion(0.0, 0.0, 0.0, 0.0)
        self.left_clutch = False
        self.right_clutch = False
        self.left_reset= False
        self.right_reset = False
        self.left_jaw = 0.0
        self.right_jaw = 0.0

robLeftData = RobotData()
robRightData = RobotData()
conData = ControllerData()

def measured_js_cb(msg, robData):
    robData.measured_js = msg

def measured_cp_cb(msg, robData):
    robData.measured_cp = msg

def measured_left_pos(msg, key=None):
    if isinstance(msg, Vector3):
        # If new msg > (median + scale*mad), then msg = last value, else msg = msg
        # print(msg, np.median(np.abs(conData.left_pos - np.median(conData.left_pos))))
        if len(conData.left_pos) > 5:
            conData.left_pos.pop(0)
        conData.left_pos.append(msg)
    elif isinstance(msg, Quaternion):
        conData.left_rot = msg
    elif isinstance(msg, String):
        if key.endswith('Clutch'):
            conData.left_clutch = True if msg.data == 'True' else False
        elif key.endswith('Reset'):
            conData.left_reset = True if msg.data == 'True' else False
    elif isinstance(msg, Float32):
        conData.left_jaw = 1-msg.data

def measured_right_pos(msg, key=None):
    if isinstance(msg, Vector3):
        if len(conData.right_pos) > 5:
            conData.right_pos.pop(0)
        conData.right_pos.append(msg)
    elif isinstance(msg, Quaternion):
        conData.right_rot = msg
    elif isinstance(msg, String):
        if key.endswith('Clutch'):
            conData.right_clutch = True if msg.data == 'True' else False
        elif key.endswith('Reset'):
            conData.right_reset = True if msg.data == 'True' else False
    elif isinstance(msg, Float32):
        conData.right_jaw = 1-msg.data

def sendTransform(position, rotation, jointpos = None, name='robot_left_pre_ik'):
    br = tf.TransformBroadcaster()
    rot_array = np.array((rotation[0], rotation[1], rotation[2], rotation[3]))
    br.sendTransform((position.x, position.y, position.z),
        rot_array/(np.linalg.norm(rot_array) + np.finfo(float).eps),
        rospy.Time.now(),
        name,
        "world")

rospy.init_node("sur_chal_crtk_test")

namespace = "/CRTK/"
arm_1_name = "psm1"
measured_js_1_name = namespace + arm_1_name + "/measured_js"
measured_cp_1_name = namespace + arm_1_name + "/measured_cp"
servo_jp_1_name = namespace + arm_1_name + "/servo_jp"
servo_jp_1_jaw_name = namespace + arm_1_name + "/jaw/servo_jp"
servo_cp_1_name = namespace + arm_1_name + "/servo_cp"

arm_2_name = "psm2"
measured_js_2_name = namespace + arm_2_name + "/measured_js"
measured_cp_2_name = namespace + arm_2_name + "/measured_cp"
servo_jp_2_name = namespace + arm_2_name + "/servo_jp"
servo_jp_2_jaw_name = namespace + arm_2_name + "/jaw/servo_jp"
servo_cp_2_name = namespace + arm_2_name + "/servo_cp"

measured_js_1_sub = rospy.Subscriber(
    measured_js_1_name, JointState, measured_js_cb, callback_args=robLeftData, queue_size=1)
measured_cp_1_sub = rospy.Subscriber(
    measured_cp_1_name, TransformStamped, measured_cp_cb, callback_args=robLeftData, queue_size=1)

measured_js_2_sub = rospy.Subscriber(
    measured_js_2_name, JointState, measured_js_cb, callback_args=robRightData, queue_size=1)
measured_cp_2_sub = rospy.Subscriber(
    measured_cp_2_name, TransformStamped, measured_cp_cb, callback_args=robRightData, queue_size=1)

servo_jp_1_pub = rospy.Publisher(servo_jp_1_jaw_name, JointState, queue_size=1)
servo_cp_1_pub = rospy.Publisher(servo_cp_1_name, TransformStamped, queue_size=1)

servo_jp_2_pub = rospy.Publisher(servo_jp_2_jaw_name, JointState, queue_size=1)
servo_cp_2_pub = rospy.Publisher(servo_cp_2_name, TransformStamped, queue_size=1)

servo_jaw_1_pub = rospy.Publisher(servo_jp_1_jaw_name, JointState, queue_size=1)
servo_jaw_2_pub = rospy.Publisher(servo_jp_2_jaw_name, JointState, queue_size=1)

controller_left_sub = rospy.Subscriber(
    '/QuestControllerData/leftControllerPosition', Vector3, measured_left_pos, queue_size=1)
controller_right_sub = rospy.Subscriber(
    '/QuestControllerData/rightControllerPosition', Vector3, measured_right_pos, queue_size=1)
controller_left_clutch = rospy.Subscriber(
    '/QuestControllerData/leftControllerX', String, measured_left_pos, callback_args='leftClutch', queue_size=1)
controller_right_clutch = rospy.Subscriber(
    '/QuestControllerData/rightControllerX', String, measured_right_pos, callback_args='rightClutch', queue_size=1)
controller_left_reset = rospy.Subscriber(
    '/QuestControllerData/leftControllerY', String, measured_left_pos, callback_args='leftReset', queue_size=1)
controller_right_reset = rospy.Subscriber(
    '/QuestControllerData/rightControllerY', String, measured_right_pos, callback_args='rightReset', queue_size=1)
controller_left_clutch = rospy.Subscriber(
    '/QuestControllerData/leftControllerRotation', Quaternion, measured_left_pos, queue_size=1)
controller_right_clutch = rospy.Subscriber(
    '/QuestControllerData/rightControllerRotation', Quaternion, measured_right_pos, queue_size=1)
controller_left_jaw = rospy.Subscriber(
    '/QuestControllerData/leftControllerJaw', Float32, measured_left_pos, queue_size=1)
controller_right_jaw = rospy.Subscriber(
    '/QuestControllerData/rightControllerJaw', Float32, measured_right_pos, queue_size=1)

rate = rospy.Rate(100)

servo_jp_1_msg = JointState()
servo_jp_1_msg.position = [0., 0., 1., 0., 0., 0.]
servo_jp_2_msg = JointState()
servo_jp_2_msg.position = [0., 0., 1., 0., 0., 0.]

servo_jaw_1_msg = JointState()
servo_jaw_1_msg.position = [0.]
servo_jaw_2_msg = JointState()
servo_jaw_2_msg.position = [0.]

servo_cp_1_msg = TransformStamped()
servo_cp_1_msg.transform.translation.x = -0.3
servo_cp_1_msg.transform.translation.y = 0.1
servo_cp_1_msg.transform.translation.z = -0.89
servo_cp_2_msg = TransformStamped()
servo_cp_2_msg.transform.translation.x = 0.
servo_cp_2_msg.transform.translation.y = 0.
servo_cp_2_msg.transform.translation.z = -1

R_7_0 = Rotation.RPY(3.14, 0.0, 1.57079)

servo_cp_1_msg.transform.rotation.x = R_7_0.GetQuaternion()[0]
servo_cp_1_msg.transform.rotation.y = R_7_0.GetQuaternion()[1]
servo_cp_1_msg.transform.rotation.z = R_7_0.GetQuaternion()[2]
servo_cp_1_msg.transform.rotation.w = R_7_0.GetQuaternion()[3]

servo_cp_2_msg.transform.rotation.x = R_7_0.GetQuaternion()[0]
servo_cp_2_msg.transform.rotation.y = R_7_0.GetQuaternion()[1]
servo_cp_2_msg.transform.rotation.z = R_7_0.GetQuaternion()[2]
servo_cp_2_msg.transform.rotation.w = R_7_0.GetQuaternion()[3]

temp = TransformStamped()
temp.transform.rotation.x = R_7_0.GetQuaternion()[0]
temp.transform.rotation.y = R_7_0.GetQuaternion()[1]
temp.transform.rotation.z = R_7_0.GetQuaternion()[2]
temp.transform.rotation.w = R_7_0.GetQuaternion()[3]
declutched_pose_left = None
declutched_pose_right = None
has_clutched_left = False
has_clutched_right = False
pre_transform_left = np.eye(4)
pre_transform_right = np.eye(4)

valid_key = False
key = None
# while not valid_key:
#     print("NOTE!!! For this example to work, please RUN the launch_crtk_interface.py script before hand.")
#     key = input("Press: \n"
#                 "1 - (For reading joint and Cartesian state), \n"
#                 "2 - (For joint control demo), \n"
#                 "3 - (For Cartesian control demo) \n"
#                 "4 - (For Oculus demo)) \n")
#     try:
#         key = int(key)
#     except ValueError:
#         key = None
#         pass

#     if key in [1, 2, 3, 4]:
#         valid_key = True
#     else:
#         print("Invalid Entry")

parser = argparse.ArgumentParser(description='Run the ros based controller')
parser.add_argument('-k','--key', help='The mode to run in', required=True)
args = vars(parser.parse_args())

key = int(args['key'])
valid_key = True

while not rospy.is_shutdown():
    # RUN launch_crtk_interface. Subscribe to Oculus controller position, 
    # and just add 0.1*(previour controller pos - current controller pos)
    # to the current robot position

    # ######
    # The following 3 lines display the joint positions and Cartesian pose state
    if key == 1:
        print("measured_js: ", robLeftData.measured_js)
        print("------------------------------------")
        print("measured_cp: ", robLeftData.measured_cp.transform)

    # ######
    # The following 3 lines move the first two joints in a sinusoidal pattern
    elif key == 2:
        
        servo_jp_1_msg.position[1] = 0.2 * math.sin(rospy.Time.now().to_sec())
        # servo_jp_1_msg.position[4] = 1.0
        # print(servo_jp_1_msg)
        servo_cp_1_pub.publish(servo_cp_1_msg)
        servo_jp_1_pub.publish(servo_jp_1_msg)

    # ######
    # The following 3 lines move the robot in cartesian space in sinusoidal fashion
    elif key == 3:
        # servo_cp_1_msg.transform.translation.x = 0.8 * \
        #     math.sin(rospy.Time.now().to_sec())
        sinusoidal_x = 0.8 * \
            math.cos(rospy.Time.now().to_sec())

        # Steps to debug coordintae frames:
        #   1. Get position of end-effector in base frame from a default state:
        #   2. Transform that position to the camera frame 
        #   3. Add 0.01 (so 1 cm) to the point in the camera frame
        #   4. Transform the new camera point back to th base frame of the robo
        #   5. Set the the robot position (keep same orientation) the new point and in the visualizer, the ee should move away from the camera

        # Camera Z and Robot Z are not aligned, there must be some other transform
        # Add another rotation about Z, or just use Z transformation from CameraFrame?
        p_c = np.array([0., sinusoidal_x, -1, 1])
        cTb = np.array([[0.866025, -0.321376, -0.383039, -1.02672],
               [0.321375, 0.944651, -0.0659712, 0.15093],
               [0.38304, -0.0659662, 0.921373, -0.415986],
               [0, 0, 0, 1]])
        bTc = np.linalg.inv(cTb)
        cTo = np.array([[np.cos(-3.1416), -np.sin(-3.1416), 0., 0.], [np.sin(-3.1416), np.cos(-3.1416), 0., 0.], [0., 0., 1., 0.], 
                         [0., 0., 0., 1.]]) @\
              np.array([[1., 0., 0., 0.], [0., np.cos(0.6981), -np.sin(0.6981), 0.], [0, np.sin(0.6981), np.cos(0.6981), 0.], 
                         [0., 0., 0., 1.]])
        print(cTo)
        p_e = bTc @ p_c
        servo_cp_1_msg.transform.translation.x = p_e[0]
        servo_cp_1_msg.transform.translation.y = p_e[1]
        servo_cp_1_msg.transform.translation.z = p_e[2]

        # (pi, 0, pi/2 is the neutral position. All other angles have to be
        # with respect to this?)
        R_7_0 = Rotation.RPY(3.14, 0, 1.57)
        servo_cp_1_msg.transform.rotation.x = R_7_0.GetQuaternion()[0]
        servo_cp_1_msg.transform.rotation.y = R_7_0.GetQuaternion()[1]
        servo_cp_1_msg.transform.rotation.z = R_7_0.GetQuaternion()[2]
        servo_cp_1_msg.transform.rotation.w = R_7_0.GetQuaternion()[3]

        servo_cp_1_pub.publish(servo_cp_1_msg)

        sentRot = np.array([servo_cp_1_msg.transform.rotation.x, servo_cp_1_msg.transform.rotation.y,servo_cp_1_msg.transform.rotation.z,
                               servo_cp_1_msg.transform.rotation.w])
        # sendTransform(servo_cp_1_msg.transform.translation, sentRot)
        # print(robLeftData.measured_js.position)
        robRot = np.array([robLeftData.measured_cp.transform.rotation.x, robLeftData.measured_cp.transform.rotation.y, robLeftData.measured_cp.transform.rotation.z,
                               robLeftData.measured_cp.transform.rotation.w])
        sendTransform(robLeftData.measured_cp.transform.translation, robRot, 'robot_measured')
        # print(np.array([robLeftData.measured_cp.transform.translation.x,
        #                 robLeftData.measured_cp.transform.translation.y,
        #                 robLeftData.measured_cp.transform.translation.z]) \
        #     - np.array([servo_cp_1_msg.transform.translation.x,
        #                 servo_cp_1_msg.transform.translation.y,
        #                 servo_cp_1_msg.transform.translation.z]))

    elif key == 4:
        if conData.left_pos[0].x == 0. or conData.right_pos[0].x == 0.:
            continue
        leftDelta = (conData.left_pos[0].x - conData.left_pos[1].x, 
            conData.left_pos[0].y - conData.left_pos[1].y,
            conData.left_pos[0].z - conData.left_pos[1].z)
        # conData.left_pos.pop(0)
        rightDelta = (conData.right_pos[0].x - conData.right_pos[1].x, 
            conData.right_pos[0].y - conData.right_pos[1].y,
            conData.right_pos[0].z - conData.right_pos[1].z)
        # conData.right_pos.pop(0)

        # print(conData.left_jaw)
        motion_factor = 0.3

        if not conData.left_clutch:
            rot_matrix_original = quaternion_matrix([-conData.left_rot.x, -conData.left_rot.y, 
                conData.left_rot.z, conData.left_rot.w])
            T_ec = np.array([[np.cos(-0.5236), -np.sin(-0.5236), 0], [np.sin(-0.5236), np.cos(-0.5236), 0], [0, 0, 1]])
            T_ec_h = np.eye(4)
            T_ec_h[:3, :3] = T_ec
            rot_matrix = T_ec_h @ rot_matrix_original @ np.array([[0, 1, 0, 0], [1, 0, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
            
            # Is pre_transform cumulative?
            # There is some OFFSET HAPPENING HERE!
            if declutched_pose_left is None and has_clutched_left == True:
                declutched_pose_left = rot_matrix
                current_robot_pose = robLeftData.measured_cp.transform.rotation
                current_robot_pose = quaternion_matrix([current_robot_pose.x, current_robot_pose.y,
                        current_robot_pose.z, current_robot_pose.w])
                pre_transform_left = current_robot_pose @ np.linalg.inv(declutched_pose_left)
            
            rot_matrix = pre_transform_left @ rot_matrix
            rot_quat = quaternion_from_matrix(rot_matrix)
            leftDelta = T_ec @ np.array(leftDelta)
            servo_cp_1_msg.transform.translation.x -= leftDelta[0]*motion_factor
            servo_cp_1_msg.transform.translation.y -= leftDelta[1]*motion_factor
            servo_cp_1_msg.transform.translation.z += leftDelta[2]*motion_factor
            # if conData.left_reset:
            #     R_7_0 = Rotation.RPY(3.14, 0, 1.57)
            #     rot_quat = R_7_0.GetQuaternion()
            #     pre_transform_left = np.eye(4)
            servo_cp_1_msg.transform.rotation.x = rot_quat[0]
            servo_cp_1_msg.transform.rotation.y = rot_quat[1]
            servo_cp_1_msg.transform.rotation.z = rot_quat[2]
            servo_cp_1_msg.transform.rotation.w = rot_quat[3]
            servo_jaw_1_msg.position = [conData.left_jaw]
            servo_cp_1_pub.publish(servo_cp_1_msg)
            robRot = np.array([robLeftData.measured_cp.transform.rotation.x, robLeftData.measured_cp.transform.rotation.y, robLeftData.measured_cp.transform.rotation.z,
                               robLeftData.measured_cp.transform.rotation.w])
            sendTransform(robLeftData.measured_cp.transform.translation, robRot, 'robot_measured')
            servo_jaw_1_pub.publish(servo_jaw_1_msg)
        else:
            declutched_pose_left = None
            has_clutched_left = True

        if not conData.right_clutch:
            rot_matrix_original = quaternion_matrix([-conData.right_rot.x, -conData.right_rot.y, 
                conData.right_rot.z, conData.right_rot.w])
            T_ec = np.array([[np.cos(0.5236), -np.sin(0.5236), 0], [np.sin(0.5236), np.cos(0.5236), 0], [0, 0, 1]])
            T_ec_h = np.eye(4)
            T_ec_h[:3, :3] = T_ec
            rot_matrix = T_ec_h @ rot_matrix_original @ np.array([[0, 1, 0, 0], [1, 0, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
            
            if declutched_pose_right is None and has_clutched_right:
                declutched_pose_right = rot_matrix
                current_robot_pose = robRightData.measured_cp.transform.rotation
                current_robot_pose = quaternion_matrix([current_robot_pose.x, current_robot_pose.y,
                        current_robot_pose.z, current_robot_pose.w])
                pre_transform_right = current_robot_pose @ np.linalg.inv(declutched_pose_right)
            
            # print(pre_transform_right)
            rot_matrix = pre_transform_right @ rot_matrix

            rot_quat = quaternion_from_matrix(rot_matrix)
            rightDelta = T_ec @ np.array(rightDelta)
            servo_cp_2_msg.transform.translation.x -= rightDelta[0]*motion_factor
            servo_cp_2_msg.transform.translation.y -= rightDelta[1]*motion_factor
            servo_cp_2_msg.transform.translation.z += rightDelta[2]*motion_factor
            # if conData.right_reset:
            #     R_7_0 = Rotation.RPY(3.14, 0, 1.57)
            #     rot_quat = R_7_0.GetQuaternion()
            #     pre_transform_right = np.eye(4)
            servo_cp_2_msg.transform.rotation.x = rot_quat[0]
            servo_cp_2_msg.transform.rotation.y = rot_quat[1]
            servo_cp_2_msg.transform.rotation.z = rot_quat[2]
            servo_cp_2_msg.transform.rotation.w = rot_quat[3]
            servo_jaw_2_msg.position = [conData.right_jaw]
            servo_cp_2_pub.publish(servo_cp_2_msg)
            robRot = np.array([robRightData.measured_cp.transform.rotation.x, robRightData.measured_cp.transform.rotation.y, robRightData.measured_cp.transform.rotation.z,
                               robRightData.measured_cp.transform.rotation.w])
            # sendTransform(robRightData.measured_cp.transform.translation, robRot, 'robot_measured')
            servo_jaw_2_pub.publish(servo_jaw_2_msg)
        else:
            declutched_pose_right = None
            has_clutched_right = True

    rate.sleep() 
