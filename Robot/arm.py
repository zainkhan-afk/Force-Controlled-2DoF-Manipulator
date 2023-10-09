import pygame
import numpy as np
from utils import *
from state import State

from .link import Link
from .base import Base
from .kinematics import Kinematics
from .jointController import JointController
from .Dynamics import ArmDynamics


class Arm:
	def __init__(self, sim_handle, ground, position = (0, 0)):
		self.arm_width = 0.05
		self.arm_segment_length = 1.0

		self.home_position = np.array([[0, -1.0]]).T

		self.joint_0_pos =  (position[0], position[1])
		self.joint_1_pos =  (position[0], position[1])
		self.joint_2_pos =  (position[0], position[1] - 2*self.arm_segment_length)

		self.link_0_pos = (position[0], position[1] / 2)
		self.link_1_pos = (position[0], position[1] - self.arm_segment_length)
		self.link_2_pos = (position[0], position[1] - 3*self.arm_segment_length)

		self.link_1 = Link(sim_handle, self.link_1_pos, self.arm_width, self.arm_segment_length, group_index = 1)
		self.link_2 = Link(sim_handle, self.link_2_pos, self.arm_width, self.arm_segment_length, group_index = 1)


		joint_1 = sim_handle.world.CreateRevoluteJoint(
									bodyA = ground.body,
									bodyB = self.link_1.body,
									anchor = self.joint_1_pos,
									maxMotorTorque = 100.0,
									motorSpeed = 0.0,
									enableMotor = True,
									)

		joint_2 = sim_handle.world.CreateRevoluteJoint(
									bodyA = self.link_1.body,
									bodyB = self.link_2.body,
									anchor = self.joint_2_pos,
									maxMotorTorque = 100.0,
									motorSpeed = 0.0,
									enableMotor = True,
									)


		self.joint_1_controller = JointController(joint_1, name = "j1")
		self.joint_2_controller = JointController(joint_2, name = "j2")

		self.kine_model = Kinematics(self.arm_segment_length, self.arm_segment_length)
		self.dynamicsModel = ArmDynamics(self.link_1.height, self.link_2.height, self.link_1.body.mass, self.link_2.body.mass)

		self.state = State(np.array([0, 0]), np.array([0, 0]), np.array([0, 0]))
		

	def HomePosition(self):
		self.GoTo(self.home_position)
		current_pos = self.GetEEPos()

		return AlmostEqual(self.home_position, current_pos)

	def GoTo(self, position):
		desired_theta_1, desired_theta_2 = self.kine_model.IK(position)
		self.joint_1_controller.SetAngle(desired_theta_1)
		self.joint_2_controller.SetAngle(desired_theta_2)


		current_theta_1 = self.joint_1_controller.GetAngle()
		current_theta_2 = self.joint_2_controller.GetAngle()

		self.state = self.state.UpdateUsingPosition(np.array([current_theta_1, current_theta_2]))

		# print(self.state)
		# print(self.joint_1_controller.joint.speed, self.joint_2_controller.joint.speed)

	def UpdateState(self):
		current_theta_1 = self.joint_1_controller.GetAngle()
		current_theta_2 = self.joint_2_controller.GetAngle()
		
		self.state = self.state.UpdateUsingPosition(np.array([current_theta_1, current_theta_2]))
		
	def ApplyForce(self, force):
		current_theta_1 = self.joint_1_controller.GetAngle()
		current_theta_2 = self.joint_2_controller.GetAngle()


		# self.state.SetPosition(np.array([current_theta_1, current_theta_2]))
		self.state = self.state.UpdateUsingPosition(np.array([current_theta_1, current_theta_2]))

		J = self.kine_model.GetJacobian(current_theta_1, current_theta_2)
		
		theta_double_dot, _ = self.dynamicsModel.ForwardDynamics(force, J, self.state)

		self.state = self.state.UpdateUsingAcceleration(np.array([theta_double_dot[0, 0], theta_double_dot[1, 0]]))

		desired_theta_1 = self.state.theta[0]
		desired_theta_2 = self.state.theta[1]

		# theta_dot_1 = self.joint_1_controller.GetVelocity() + theta_double_dot[0, 0]*TIME_STEP
		# theta_dot_2 = self.joint_2_controller.GetVelocity() + theta_double_dot[1, 0]*TIME_STEP

		# desired_theta_1 = current_theta_1 + theta_dot_1*TIME_STEP
		# desired_theta_2 = current_theta_2 + theta_dot_2*TIME_STEP

		self.joint_1_controller.SetAngle(desired_theta_1)
		self.joint_2_controller.SetAngle(desired_theta_2)
	
	def GetJacobian(self):
		current_theta_1 = self.state.theta[0]
		current_theta_2 = self.state.theta[1]
		J = self.kine_model.GetJacobian(current_theta_1, current_theta_2)

		return J

	def ApplyState(self, desired_state):
		desired_theta_1 = desired_state.theta[0]
		desired_theta_2 = desired_state.theta[1]

		self.joint_1_controller.SetAngle(desired_theta_1)
		self.joint_2_controller.SetAngle(desired_theta_2)

	def GetState(self):
		return self.state


	def GetEEPos(self):
		current_theta_1 = self.state.theta[0]
		current_theta_2 = self.state.theta[1]

		position = self.kine_model.FK(current_theta_1, current_theta_2)

		return position
	
	def Render(self, screen, PPM):
		self.link_1.Render(screen, PPM)
		self.link_2.Render(screen, PPM)