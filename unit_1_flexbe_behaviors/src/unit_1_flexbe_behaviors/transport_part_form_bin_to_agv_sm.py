#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from unit_1_flexbe_behaviors.pick_part_from_bin_sm import pick_part_from_binSM
from unit_1_flexbe_behaviors.place_part_on_agv_sm import place_part_on_agvSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri May 28 2021
@author: Jesper Ellens
'''
class transport_part_form_bin_to_agvSM(Behavior):
	'''
	bestaat uit de 
behaviors “pick_part_from_bin” en “place_part_on_agv”
	'''


	def __init__(self):
		super(transport_part_form_bin_to_agvSM, self).__init__()
		self.name = 'transport_part_form_bin_to_agv'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(pick_part_from_binSM, 'pick_part_from_bin')
		self.add_behavior(place_part_on_agvSM, 'place_part_on_agv')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:730 y:133, x:426 y:239
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.part = 'gasket_part'
		_state_machine.userdata.robot_namespace = ''

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:247 y:94
			OperatableStateMachine.add('pick_part_from_bin',
										self.use_behavior(pick_part_from_binSM, 'pick_part_from_bin'),
										transitions={'finished': 'place_part_on_agv', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'part': 'part', 'robot_namespace': 'robot_namespace'})

			# x:483 y:96
			OperatableStateMachine.add('place_part_on_agv',
										self.use_behavior(place_part_on_agvSM, 'place_part_on_agv'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
