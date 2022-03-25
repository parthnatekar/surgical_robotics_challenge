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

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, Vector3
import rospy
import math
from PyKDL import Rotation
import numpy as np

class RobotData:
    def __init__(self):
        self.measured_js = JointState()
        self.measured_cp = TransformStamped()

class ControllerData:
    def __init__(self):
        self.left_pos = [Vector3(0.0, 0.0, 0.0)]
        self.right_pos = [Vector3(0.0, 0.0, 0.0)]

robData = RobotData()
conData = ControllerData()

def measured_js_cb(msg):
    robData.measured_js = msg


def measured_cp_cb(msg):
    robData.measured_cp = msg

def measured_left_pos(msg):
    if len(conData.left_pos) > 5:
        conData.left_pos.pop(0)
    conData.left_pos.append(msg)

def measured_right_pos(msg):
    if len(conData.right_pos) > 5:
        conData.right_pos.pop(0)
    conData.right_pos.append(msg)

rospy.init_node("sur_chal_crtk_test")

namespace = "/CRTK/"
arm_name = "psm1"
measured_js_name = namespace + arm_name + "/measured_js"
measured_cp_name = namespace + arm_name + "/measured_cp"
servo_jp_name = namespace + arm_name + "/servo_jp"
servo_cp_name = namespace + arm_name + "/servo_cp"

# measured_js_sub = rospy.Subscriber(
#     measured_js_name, JointState, measured_js_cb, queue_size=1)
measured_cp_sub = rospy.Subscriber(
    measured_cp_name, TransformStamped, measured_cp_cb, queue_size=1)

controller_left_sub = rospy.Subscriber(
    '/QuestControllerData/leftControllerPosition', Vector3, measured_left_pos, queue_size=1)
controller_right_sub = rospy.Subscriber(
    '/QuestControllerData/rightControllerPosition', Vector3, measured_right_pos, queue_size=1)

# servo_jp_pub = rospy.Publisher(servo_jp_name, JointState, queue_size=1)
servo_cp_pub = rospy.Publisher(servo_cp_name, TransformStamped, queue_size=1)

rate = rospy.Rate(50)

# servo_jp_msg = JointState()
# servo_jp_msg.position = [0., 0., 1.0, 0., 0., 0.]

servo_cp_msg = TransformStamped()
servo_cp_msg.transform.translation.z = -1.0
R_7_0 = Rotation.RPY(3.14, 0.0, 1.57079)

servo_cp_msg.transform.rotation.x = R_7_0.GetQuaternion()[0]
servo_cp_msg.transform.rotation.y = R_7_0.GetQuaternion()[1]
servo_cp_msg.transform.rotation.z = R_7_0.GetQuaternion()[2]
servo_cp_msg.transform.rotation.w = R_7_0.GetQuaternion()[3]

valid_key = False
key = None
while not valid_key:
    print("NOTE!!! For this example to work, please RUN the launch_crtk_interface.py script before hand.")
    key = input("Press: \n"
                "1 - (For reading joint and Cartesian state), \n"
                "2 - (For joint control demo), \n"
                "3 - (For Cartesian control demo) \n"
                "4 - (For Oculus demo)) \n")
    try:
        key = int(key)
    except ValueError:
        key = None
        pass

    if key in [1, 2, 3, 4]:
        valid_key = True
    else:
        print("Invalid Entry")

while not rospy.is_shutdown():
    # RUN launch_crtk_interface. Subscribe to Oculus controller position, 
    # and just add 0.1*(previour controller pos - current controller pos)
    # to the current robot position

    # ######
    # The following 3 lines display the joint positions and Cartesian pose state
    if key == 1:
        print("measured_js: ", robData.measured_js)
        print("------------------------------------")
        print("measured_cp: ", robData.measured_cp.transform)

    # ######
    # The following 3 lines move the first two joints in a sinusoidal pattern
    elif key == 2:
        servo_jp_msg.position[0] = 0.2 * math.sin(rospy.Time.now().to_sec())
        servo_jp_msg.position[1] = 0.2 * math.cos(rospy.Time.now().to_sec())
        servo_jp_pub.publish(servo_jp_msg)

    # ######
    # The following 3 lines move the robot in cartesian space in sinusoidal fashion
    elif key == 3:
        servo_cp_msg.transform.translation.x = 0.2 * \
            math.sin(rospy.Time.now().to_sec())
        servo_cp_msg.transform.translation.y = 0.2 * \
            math.cos(rospy.Time.now().to_sec())
        servo_cp_pub.publish(servo_cp_msg)

    elif key == 4:
        if conData.left_pos[0].x == 0.:
            continue
        leftDelta = (conData.left_pos[0].x - conData.left_pos[1].x, 
            conData.left_pos[0].y - conData.left_pos[1].y,
            conData.left_pos[0].z - conData.left_pos[1].z)
        # conData.left_pos.pop(0)
        rightDelta = (conData.right_pos[0].x - conData.right_pos[1].x, 
            conData.right_pos[0].y - conData.right_pos[1].y,
            conData.right_pos[0].z - conData.right_pos[1].z)
        # conData.right_pos.pop(0)

        servo_cp_msg.transform.translation.x -= leftDelta[0]*0.5
        servo_cp_msg.transform.translation.y -= leftDelta[1]*0.5
        servo_cp_msg.transform.translation.z += leftDelta[2]*0.5
        servo_cp_pub.publish(servo_cp_msg)

    rate.sleep()
