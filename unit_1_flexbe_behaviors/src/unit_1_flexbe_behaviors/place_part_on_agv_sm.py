#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ariac_flexbe_states.compute_grasp_ariac_state import ComputeGraspAriacState
from ariac_flexbe_states.get_object_pose import GetObjectPoseState
from ariac_flexbe_states.moveit_to_joints_dyn_ariac_state import MoveitToJointsDynAriacState
from ariac_flexbe_states.srdf_state_to_moveit_ariac_state import SrdfStateToMoveitAriac
from ariac_flexbe_states.start_assignment_state import StartAssignment
from ariac_flexbe_states.vacuum_gripper_control_state import VacuumGripperControlState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri May 28 2021
@author: Jesper Ellens
'''
class place_part_on_agvSM(Behavior):
	'''
	het part op de desbetreffende 
AGV leggen
	'''


	def __init__(self):
		super(place_part_on_agvSM, self).__init__()
		self.name = 'place_part_on_agv'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:275, x:130 y:275
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.part = 'gear_part'
		_state_machine.userdata.robot_namespace = ''
		_state_machine.userdata.config_name_home = 'home'
		_state_machine.userdata.move_group = 'manipulator'
		_state_machine.userdata.action_topic_namespace = 'ariac/arm1'
		_state_machine.userdata.robotname = ''
		_state_machine.userdata.action_topic = '/move_group'
		_state_machine.userdata.camera_reference_frame = 'arm1_linear_arm_actuator'
		_state_machine.userdata.camera_frame = ''
		_state_machine.userdata.tool_link = 'ee_link'
		_state_machine.userdata.part_height = 0.035
		_state_machine.userdata.part_rotation = 0
		_state_machine.userdata.gripper_service = '/ariac/arm1/gripper/control'
		_state_machine.userdata.locations = []
		_state_machine.userdata.bin = ''
		_state_machine.userdata.part_pose = []
		_state_machine.userdata.zero = 0
		_state_machine.userdata.robot_config_predrop = 'AGV1PreDrop'
		_state_machine.userdata.tray = 'kit_tray_1'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:64 y:68
			OperatableStateMachine.add('Start',
										StartAssignment(),
										transitions={'continue': 'Move to pre drop'},
										autonomy={'continue': Autonomy.Off})

			# x:403 y:362
			OperatableStateMachine.add('Deactivate gripper',
										VacuumGripperControlState(enable=False),
										transitions={'continue': 'Move home', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'service_name': 'gripper_service'})

			# x:496 y:58
			OperatableStateMachine.add('Get agv pose',
										GetObjectPoseState(),
										transitions={'continue': 'Compute drop', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'ref_frame': 'camera_reference_frame', 'frame': 'tray', 'pose': 'agv_pose'})

			# x:180 y:367
			OperatableStateMachine.add('Move home',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name_home', 'move_group': 'move_group', 'action_topic_namespace': 'action_topic_namespace', 'action_topic': 'action_topic', 'robot_name': 'robotname', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:654 y:301
			OperatableStateMachine.add('Move to drop',
										MoveitToJointsDynAriacState(),
										transitions={'reached': 'Deactivate gripper', 'planning_failed': 'wait retry', 'control_failed': 'wait retry'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'action_topic_namespace': 'action_topic_namespace', 'move_group': 'move_group', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:274 y:46
			OperatableStateMachine.add('Move to pre drop',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'Get agv pose', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'robot_config_predrop', 'move_group': 'move_group', 'action_topic_namespace': 'action_topic_namespace', 'action_topic': 'action_topic', 'robot_name': 'robotname', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:677 y:391
			OperatableStateMachine.add('wait retry',
										WaitState(wait_time=2),
										transitions={'done': 'Move to drop'},
										autonomy={'done': Autonomy.Off})

			# x:667 y:57
			OperatableStateMachine.add('Compute drop',
										ComputeGraspAriacState(joint_names=['linear_arm_actuator_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']),
										transitions={'continue': 'Move to drop', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'action_topic_namespace': 'action_topic_namespace', 'tool_link': 'tool_link', 'pose': 'agv_pose', 'offset': 'part_height', 'rotation': 'part_rotation', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
