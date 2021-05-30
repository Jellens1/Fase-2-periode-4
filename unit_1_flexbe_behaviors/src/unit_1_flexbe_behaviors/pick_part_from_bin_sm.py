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
from ariac_flexbe_states.detect_part_camera_ariac_state import DetectPartCameraAriacState
from ariac_flexbe_states.lookup_from_table import LookupFromTableState
from ariac_flexbe_states.moveit_to_joints_dyn_ariac_state import MoveitToJointsDynAriacState
from ariac_flexbe_states.srdf_state_to_moveit_ariac_state import SrdfStateToMoveitAriac
from ariac_flexbe_states.start_assignment_state import StartAssignment
from ariac_flexbe_states.vacuum_gripper_control_state import VacuumGripperControlState
from ariac_logistics_flexbe_states.get_material_locations import GetMaterialLocationsState
from ariac_support_flexbe_states.get_item_from_list_state import GetItemFromListState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Apr 25 2021
@author: docent
'''
class pick_part_from_binSM(Behavior):
	'''
	pick's a specific part form a it's bin
	'''


	def __init__(self):
		super(pick_part_from_binSM, self).__init__()
		self.name = 'pick_part_from_bin'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:139 y:319, x:449 y:445
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['part', 'robot_namespace'])
		_state_machine.userdata.part = 'gasket_part'
		_state_machine.userdata.robot_namespace = ''
		_state_machine.userdata.camera_topic = ''
		_state_machine.userdata.zero = 0
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

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Start',
										StartAssignment(),
										transitions={'continue': 'Move home'},
										autonomy={'continue': Autonomy.Off})

			# x:699 y:114
			OperatableStateMachine.add('Compute pick',
										ComputeGraspAriacState(joint_names=['linear_arm_actuator_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']),
										transitions={'continue': 'Move to pick', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'action_topic_namespace': 'action_topic_namespace', 'tool_link': 'tool_link', 'pose': 'pose', 'offset': 'part_height', 'rotation': 'part_rotation', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:1104 y:113
			OperatableStateMachine.add('Detect part',
										DetectPartCameraAriacState(time_out=0.5),
										transitions={'continue': 'Move pregrasp', 'failed': 'failed', 'not_found': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'ref_frame': 'camera_reference_frame', 'camera_topic': 'camera_topic', 'camera_frame': 'camera_frame', 'part': 'part', 'pose': 'pose'})

			# x:565 y:37
			OperatableStateMachine.add('Get bin from locations',
										GetItemFromListState(),
										transitions={'done': 'Look up camera topic', 'invalid_index': 'failed'},
										autonomy={'done': Autonomy.Off, 'invalid_index': Autonomy.Off},
										remapping={'list': 'locations', 'index': 'zero', 'item': 'bin'})

			# x:372 y:35
			OperatableStateMachine.add('Get part location',
										GetMaterialLocationsState(),
										transitions={'continue': 'Get bin from locations'},
										autonomy={'continue': Autonomy.Off},
										remapping={'part': 'part', 'material_locations': 'locations'})

			# x:918 y:13
			OperatableStateMachine.add('Look up camera frame',
										LookupFromTableState(parameter_name="ariac_tables_unit1", table_name='bin_configuration', index_title='bin', column_title='camera_frame'),
										transitions={'found': 'Look up pregrasp', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'bin', 'column_value': 'camera_frame'})

			# x:761 y:24
			OperatableStateMachine.add('Look up camera topic',
										LookupFromTableState(parameter_name="ariac_tables_unit1", table_name="bin_configuration", index_title='bin', column_title='camera_topic'),
										transitions={'found': 'Look up camera frame', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'bin', 'column_value': 'camera_topic'})

			# x:1114 y:8
			OperatableStateMachine.add('Look up pregrasp',
										LookupFromTableState(parameter_name="ariac_tables_unit1", table_name='bin_configuration', index_title='bin', column_title='robot_config'),
										transitions={'found': 'Detect part', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'bin', 'column_value': 'robot_config'})

			# x:204 y:37
			OperatableStateMachine.add('Move home',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'Get part location', 'planning_failed': 'Wait retry', 'control_failed': 'Wait retry', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name_home', 'move_group': 'move_group', 'action_topic_namespace': 'action_topic_namespace', 'action_topic': 'action_topic', 'robot_name': 'robotname', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:54 y:115
			OperatableStateMachine.add('Move home2',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name_home', 'move_group': 'move_group', 'action_topic_namespace': 'action_topic_namespace', 'action_topic': 'action_topic', 'robot_name': 'robotname', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:921 y:109
			OperatableStateMachine.add('Move pregrasp',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'Compute pick', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'robot_config', 'move_group': 'move_group', 'action_topic_namespace': 'action_topic_namespace', 'action_topic': 'action_topic', 'robot_name': 'robotname', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:490 y:112
			OperatableStateMachine.add('Move to pick',
										MoveitToJointsDynAriacState(),
										transitions={'reached': 'Activate gripper', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'action_topic_namespace': 'action_topic_namespace', 'move_group': 'move_group', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:213 y:186
			OperatableStateMachine.add('Wait retry',
										WaitState(wait_time=5),
										transitions={'done': 'Move home'},
										autonomy={'done': Autonomy.Off})

			# x:306 y:110
			OperatableStateMachine.add('Activate gripper',
										VacuumGripperControlState(enable=True),
										transitions={'continue': 'Move home2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'service_name': 'gripper_service'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
