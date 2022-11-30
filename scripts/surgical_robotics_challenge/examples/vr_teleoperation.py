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
from ambf_client import Client
from surgical_robotics_challenge.camera import *
from surgical_robotics_challenge.psm_arm import PSM
from surgical_robotics_challenge.ecm_arm import ECM

# Function for sending transform to rviz for visualization
def sendTransform(position, rotation, jointpos = None, name='robot_left_pre_ik'):
	br = tf.TransformBroadcaster()
	rot_array = np.array((rotation[0], rotation[1], rotation[2], rotation[3]))
	br.sendTransform((position.x, position.y, position.z),
		rot_array/(np.linalg.norm(rot_array) + np.finfo(float).eps),
		rospy.Time.now(),
		name,
		"world")

# Init rospy node
rospy.init_node("sur_chal_crtk_test")

# Define ambf topic names
client = Client("ambf_surgical_sim_crtk_node")
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

c = Client(client)
c.connect()

# print(c.get_obj_names())

# Define classes for interfacing with the robot and VR headset
class RobotData:
	'''
	The class for interfacing with one robotic arm
	'''
	def __init__(self):
		self.measured_js = JointState()
		self.measured_cp = TransformStamped()

	def set_js(self, js):
		self.measured_js = js

	def set_cp(self, cp):
		self.measured_cp = cp

class ControllerData:
	'''
	The class for interfacing with one VR controller
	'''
	def __init__(self):
		self.pose = [Vector3(0.0, 0.0, 0.0)]
		self.rot = Quaternion(0.0, 0.0, 0.0, 0.0)
		self.clutch = False
		self.reset= False
		self.jaw = 0.0

	def set_pose(self, pose):
		# if len(self.pose) > 1:
		# 	if np.linalg.norm(np.array([pose.x, pose.y, pose.z]) - \
		# 		np.array([self.pose[-1].x, self.pose[-1].y, self.pose[-1].z])) > 0.05:
		# 		print(np.linalg.norm(np.array([pose.x, pose.y, pose.z]) - \
		# 			np.array([self.pose[-1].x, self.pose[-1].y, self.pose[-1].z])))
		# 		pose = self.pose[-1]
		# pose = self.filter_pose(pose)
		if len(self.pose) > 5:
			self.pose.pop(0)
		self.pose.append(pose)

	def filter_pose(self, pose, alpha=0.95):
		pose.x = alpha*pose.x + (1-alpha)*self.pose[-1].x
		pose.y = alpha*pose.y + (1-alpha)*self.pose[-1].y
		pose.z = alpha*pose.z + (1-alpha)*self.pose[-1].z
		return pose

	def set_rot(self, rot):
		self.rot = rot

	def set_clutch(self, clutch):
		self.clutch = clutch

	def set_reset(self, reset):
		self.reset = reset

	def set_jaw(self, jaw):
		self.jaw = jaw

	def set_values_from_subscriber(self, msg, key=None):
		if isinstance(msg, Vector3):
			self.set_pose(msg)
		elif isinstance(msg, Quaternion):
			self.set_rot(msg)
		elif isinstance(msg, String):
			if key.endswith('Clutch'):
				clutch = True if msg.data == 'True' else False
				self.set_clutch(clutch)
			elif key.endswith('Reset'):
				reset = True if msg.data == 'True' else False
				self.set_reset(reset)
		elif isinstance(msg, Float32):
			self.set_jaw(1-msg.data)

# Instantiate objects for each robot and each controller
robLeftData = RobotData()
robRightData = RobotData()
conLeftData = ControllerData()
conRightData = ControllerData()

# Define ROS publishers and subscribers
measured_js_1_sub = rospy.Subscriber(
	measured_js_1_name, JointState, robLeftData.set_js, queue_size=1)
measured_cp_1_sub = rospy.Subscriber(
	measured_cp_1_name, TransformStamped, robLeftData.set_cp, queue_size=1)

measured_js_2_sub = rospy.Subscriber(
	measured_js_2_name, JointState, robRightData.set_js, queue_size=1)
measured_cp_2_sub = rospy.Subscriber(
	measured_cp_2_name, TransformStamped, robRightData.set_cp, queue_size=1)

servo_jp_1_pub = rospy.Publisher(servo_jp_1_jaw_name, JointState, queue_size=1)
servo_cp_1_pub = rospy.Publisher(servo_cp_1_name, TransformStamped, queue_size=1)

servo_jp_2_pub = rospy.Publisher(servo_jp_2_jaw_name, JointState, queue_size=1)
servo_cp_2_pub = rospy.Publisher(servo_cp_2_name, TransformStamped, queue_size=1)

servo_jaw_1_pub = rospy.Publisher(servo_jp_1_jaw_name, JointState, queue_size=1)
servo_jaw_2_pub = rospy.Publisher(servo_jp_2_jaw_name, JointState, queue_size=1)

controller_left_sub = rospy.Subscriber(
	'/QuestControllerData/leftControllerPosition', Vector3, conLeftData.set_values_from_subscriber, queue_size=1)
controller_right_sub = rospy.Subscriber(
	'/QuestControllerData/rightControllerPosition', Vector3, conRightData.set_values_from_subscriber, queue_size=1)
controller_left_clutch = rospy.Subscriber(
	'/QuestControllerData/leftControllerX', String, conLeftData.set_values_from_subscriber, callback_args='leftClutch', queue_size=1)
controller_right_clutch = rospy.Subscriber(
	'/QuestControllerData/rightControllerX', String, conRightData.set_values_from_subscriber, callback_args='rightClutch', queue_size=1)
controller_left_reset = rospy.Subscriber(
	'/QuestControllerData/leftControllerY', String, conLeftData.set_values_from_subscriber, callback_args='leftReset', queue_size=1)
controller_right_reset = rospy.Subscriber(
	'/QuestControllerData/rightControllerY', String, conRightData.set_values_from_subscriber, callback_args='rightReset', queue_size=1)
controller_left_clutch = rospy.Subscriber(
	'/QuestControllerData/leftControllerRotation', Quaternion, conLeftData.set_values_from_subscriber, queue_size=1)
controller_right_clutch = rospy.Subscriber(
	'/QuestControllerData/rightControllerRotation', Quaternion, conRightData.set_values_from_subscriber, queue_size=1)
controller_left_jaw = rospy.Subscriber(
	'/QuestControllerData/leftControllerJaw', Float32, conLeftData.set_values_from_subscriber, queue_size=1)
controller_right_jaw = rospy.Subscriber(
	'/QuestControllerData/rightControllerJaw', Float32, conRightData.set_values_from_subscriber, queue_size=1)

# Set initial variables
rate = rospy.Rate(100)

servo_jp_1_msg = JointState()
servo_jp_1_msg.position = [0., 0., 1., 0., 0., 0.]
servo_jp_2_msg = JointState()
servo_jp_2_msg.position = [0., 0., 1., 0., 0., 0.]

servo_jaw_1_msg = JointState()
servo_jaw_1_msg.position = [0.]
servo_jaw_2_msg = JointState()
servo_jaw_2_msg.position = [0.]

cTb_left = np.array([[0.866025, -0.321376, -0.383039, -1.02672],
		    [0.321375, 0.944651, -0.0659712, 0.15093],
		    [0.38304, -0.0659662, 0.921373, -0.415986],
		    [0, 0, 0, 1]])
bTc_left = np.linalg.inv(cTb_left)  

cTb_right = [[0.866025, 0.321389, 0.383028, 1.02672],
     		 [-0.32139, 0.944646, -0.0659671, 0.15093],
    		 [-0.383027, -0.0659721, 0.921378, -0.415986],
    		 [0, 0, 0, 1]]

bTc_right = np.linalg.inv(cTb_right)  

# This gives the right matrices, but messes up something else?
# cam1 = Camera(c, 'CameraFrame')
# psm1 = PSM(c, 'psm1', add_joint_errors=False)
# bTc_right = psm1.get_T_w_b() * cam1.get_T_c_w()
# bTc_right = np.linalg.inv(np.array([[bTc_right.M[0, 0], bTc_right.M[0, 1], bTc_right.M[0, 2], bTc_right.p[0]],
# 				[bTc_right.M[1, 0], bTc_right.M[1, 1], bTc_right.M[1, 2], bTc_right.p[1]],
# 				[bTc_right.M[2, 0], bTc_right.M[2, 1], bTc_right.M[2, 2], bTc_right.p[2]],
# 				[0., 0., 0., 1.]]))
# cam2 = Camera(c, 'CameraFrame')
# psm2 = PSM(c, 'psm2', add_joint_errors=False)

# bTc_left = psm2.get_T_w_b() * cam2.get_T_c_w()
# bTc_left = np.linalg.inv(np.array([[bTc_left.M[0, 0], bTc_left.M[0, 1], bTc_left.M[0, 2], bTc_left.p[0]],
# 				[bTc_left.M[1, 0], bTc_left.M[1, 1], bTc_left.M[1, 2], bTc_left.p[1]],
# 				[bTc_left.M[2, 0], bTc_left.M[2, 1], bTc_left.M[2, 2], bTc_left.p[2]],
# 				[0., 0., 0., 1.]]))

print(bTc_right)

p_initial_left = bTc_left @ np.array([-0.3, 0.2, -1.1, 1.])

p_initial_right = bTc_right @ np.array([0.3, 0.2, -1., 1.])

servo_cp_1_msg = TransformStamped()
servo_cp_1_msg.transform.translation.x = p_initial_left[0]
servo_cp_1_msg.transform.translation.y = p_initial_left[1]
servo_cp_1_msg.transform.translation.z = p_initial_left[2]
servo_cp_2_msg = TransformStamped()
servo_cp_2_msg.transform.translation.x = p_initial_right[0]
servo_cp_2_msg.transform.translation.y = p_initial_right[1]
servo_cp_2_msg.transform.translation.z = p_initial_right[2]

R_7_0 = Rotation.RPY(3.14, 0.0, 1.57079)

servo_cp_1_msg.transform.rotation.x = R_7_0.GetQuaternion()[0]
servo_cp_1_msg.transform.rotation.y = R_7_0.GetQuaternion()[1]
servo_cp_1_msg.transform.rotation.z = R_7_0.GetQuaternion()[2]
servo_cp_1_msg.transform.rotation.w = R_7_0.GetQuaternion()[3]

servo_cp_2_msg.transform.rotation.x = R_7_0.GetQuaternion()[0]
servo_cp_2_msg.transform.rotation.y = R_7_0.GetQuaternion()[1]
servo_cp_2_msg.transform.rotation.z = R_7_0.GetQuaternion()[2]
servo_cp_2_msg.transform.rotation.w = R_7_0.GetQuaternion()[3]

declutched_pose_left = None
declutched_pose_right = None
has_clutched_left = False
has_clutched_right = False
pre_transform_left = np.eye(4)
pre_transform_right = np.eye(4)
natural_robot_pose_matrix = None
initial_pose_matrix = None

valid_key = False
key = None

# Parse user arguments
parser = argparse.ArgumentParser(description='Run the ros based controller')
parser.add_argument('-k','--key', help='The mode to run in', required=True)
args = vars(parser.parse_args())

key = int(args['key'])
valid_key = True

# Run
while not rospy.is_shutdown():

	if key == 0:
		if conLeftData.pose[0].x == 0. or conRightData.pose[0].x == 0.:
			continue
		leftDelta = (conLeftData.pose[1].x - conLeftData.pose[0].x, 
			conLeftData.pose[1].y - conLeftData.pose[0].y,
			conLeftData.pose[1].z - conLeftData.pose[0].z, 1.)
		rightDelta = (conRightData.pose[1].x - conRightData.pose[0].x, 
			conRightData.pose[1].y - conRightData.pose[0].y,
			conRightData.pose[1].z - conRightData.pose[0].z, 1.)

		motion_factor = 0.3

		if not conLeftData.clutch:
			rot_matrix_original = quaternion_matrix([conLeftData.rot.x, conLeftData.rot.y, 
				conLeftData.rot.z, conLeftData.rot.w])
			# rot_matrix_original should be o_R_h 
			# That way the coordinate transform chain is:
			#		b_R_c * c_R_o * o_R_h * h_R_e = b_R_e
			# Another check, does the sim want b_R_e or e_R_b. Probably b_R_e
			# print(rot_matrix_original)
			if natural_robot_pose_matrix is None:
				natural_robot_pose_matrix = np.array([[0, 1, 0, 0], [1, 0, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]) @ np.linalg.inv(rot_matrix_original)
			# rot_matrix = bTc_left @ natural_robot_pose_matrix @ rot_matrix_original
			uRo =  np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
			cRu =  np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
			hRe = np.array([[-1, 0, 0, 0], [0, 0, 1, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
			rot_matrix = bTc_left @ cRu @ uRo @ rot_matrix_original @ hRe
			# p_c = np.array([0., sinusoidal_x, -1, 1])
			# print(leftDelta)
			if declutched_pose_left is None and has_clutched_left:
				declutched_pose_left = rot_matrix
				current_robot_pose = robLeftData.measured_cp.transform.rotation
				current_robot_pose = quaternion_matrix([current_robot_pose.x, current_robot_pose.y,
						current_robot_pose.z, current_robot_pose.w])
				# print(current_robot_pose)
				pre_transform_left = current_robot_pose @ np.linalg.inv(declutched_pose_left)
				try:
					print(np.linalg.inv(current_robot_pose) @ prev_robot_pose)
				except Exception as e:
					print(e)
				prev_robot_pose = current_robot_pose

			rot_matrix = pre_transform_left @ rot_matrix
			rot_quat = quaternion_from_matrix(rot_matrix)
			leftDelta = bTc_left[:3, :3] @ uRo[:3, :3] @ np.array(leftDelta)[:-1]
			servo_cp_1_msg.transform.translation.x += leftDelta[0]*motion_factor
			servo_cp_1_msg.transform.translation.y += leftDelta[1]*motion_factor
			servo_cp_1_msg.transform.translation.z += leftDelta[2]*motion_factor
			servo_cp_1_msg.transform.rotation.x = rot_quat[0]
			servo_cp_1_msg.transform.rotation.y = rot_quat[1]
			servo_cp_1_msg.transform.rotation.z = rot_quat[2]
			servo_cp_1_msg.transform.rotation.w = rot_quat[3]
			servo_jaw_1_msg.position = [conLeftData.jaw]
			servo_cp_1_pub.publish(servo_cp_1_msg)
			robRot = np.array([robLeftData.measured_cp.transform.rotation.x, robLeftData.measured_cp.transform.rotation.y, robLeftData.measured_cp.transform.rotation.z,
							   robLeftData.measured_cp.transform.rotation.w])
			# print(quaternion_matrix(robRot))
			# sendTransform(robLeftData.measured_cp.transform.translation, robRot, 'robot_measured')
			servo_jaw_1_pub.publish(servo_jaw_1_msg)
		else:
			declutched_pose_left = None
			has_clutched_left = True

		if not conRightData.clutch:
			rot_matrix_original = quaternion_matrix([conRightData.rot.x, conRightData.rot.y, 
				conRightData.rot.z, conRightData.rot.w])
			# if initial_pose_matrix is None:
			# 	offset = np.array([[ 0.51055143, -0.59905804, -0.6168194,   0.],
			# 	 [-0.84867822, -0.46633213, -0.24956031,  0.],
			# 	 [-0.13814163,  0.65089494, -0.7464939,   0.],
			# 	 [ 0.,          0.,          0.,          1.]])
			# 	initial_pose_matrix = rot_matrix_original @ np.linalg.inv(offset)

			# rot_matrix_original = offset @ rot_matrix_original
			uRo =  np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
			cRu =  np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
			hRe = np.array([[-1, 0, 0, 0], [0, 0, 1, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
			rot_matrix = bTc_right @ cRu @ uRo @ rot_matrix_original #@ hRe

			rot_matrix[:, 3] = np.array([0, 0, 0, 1])

			if declutched_pose_right is None and has_clutched_right:
				declutched_pose_right = rot_matrix
				current_robot_pose = robRightData.measured_cp.transform.rotation
				current_robot_pose = quaternion_matrix([current_robot_pose.x, current_robot_pose.y,
						current_robot_pose.z, current_robot_pose.w])
				pre_transform_right = current_robot_pose @ np.linalg.inv(declutched_pose_right)
			
			rot_matrix = pre_transform_right @ rot_matrix

			rot_quat = quaternion_from_matrix(rot_matrix)
			rightDelta = bTc_right[:3, :3] @ uRo[:3, :3] @ np.array(rightDelta)[:-1]
			servo_cp_2_msg.transform.translation.x += rightDelta[0]*motion_factor
			servo_cp_2_msg.transform.translation.y += rightDelta[1]*motion_factor
			servo_cp_2_msg.transform.translation.z += rightDelta[2]*motion_factor

			servo_cp_2_msg.transform.rotation.x = rot_quat[0]
			servo_cp_2_msg.transform.rotation.y = rot_quat[1]
			servo_cp_2_msg.transform.rotation.z = rot_quat[2]
			servo_cp_2_msg.transform.rotation.w = rot_quat[3]
			servo_jaw_2_msg.position = [conRightData.jaw]
			servo_cp_2_pub.publish(servo_cp_2_msg)
			robRot = np.array([robRightData.measured_cp.transform.rotation.x, robRightData.measured_cp.transform.rotation.y, robRightData.measured_cp.transform.rotation.z,
							   robRightData.measured_cp.transform.rotation.w])
			sendTransform(robRightData.measured_cp.transform.translation, robRot, 'robot_measured')
			servo_jaw_2_pub.publish(servo_jaw_2_msg)
		else:
			declutched_pose_right = None
			has_clutched_right = True

	rate.sleep() 