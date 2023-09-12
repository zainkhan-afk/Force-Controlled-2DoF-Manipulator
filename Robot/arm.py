import pygame
import numpy as np
from .link import Link
from .base import Base
from .kinematics import Kinematics
from .jointController import JointController

from .Dynamics import ArmDynamics
from utils import *
class Arm:
	def __init__(self, sim_handle, position = (0, 0)):
		self.arm_width = 0.05
		self.arm_segment_length = 0.8

		self.joint_1_pos =  (position[0], position[1])
		self.joint_2_pos =  (position[0], position[1] - 2*self.arm_segment_length)


		self.link_1_pos = (position[0], position[1] - self.arm_segment_length)
		self.link_2_pos = (position[0], position[1] - 3*self.arm_segment_length)

		self.base   = Base(sim_handle, position, group_index = -1)
		self.link_1 = Link(sim_handle, self.link_1_pos, self.arm_width, self.arm_segment_length, group_index = -1)
		self.link_2 = Link(sim_handle, self.link_2_pos, self.arm_width, self.arm_segment_length, group_index = -1)

		self.joint_1 = sim_handle.world.CreateRevoluteJoint(
									bodyA = self.base.body,
									bodyB = self.link_1.body,
									anchor = self.joint_1_pos,
									maxMotorTorque = 10000.0,
									motorSpeed = 0.0,
									enableMotor = True,
									)

		self.joint_2 = sim_handle.world.CreateRevoluteJoint(
									bodyA = self.link_1.body,
									bodyB = self.link_2.body,
									anchor = self.joint_2_pos,
									maxMotorTorque = 10000.0,
									motorSpeed = 0.0,
									enableMotor = True,
									)


		self.joint_1_controller = JointController(self.joint_1)
		self.joint_2_controller = JointController(self.joint_2)

		self.kine_model = Kinematics(self.arm_segment_length, self.arm_segment_length)
		self.dynamics = ArmDynamics(self.link_1, self.link_2, self.joint_1, self.joint_2)


	def GoTo(self, position):
		theta_1, theta_2 = self.kine_model.IK(position)
		self.joint_1_controller.SetAngle(theta_1)
		self.joint_2_controller.SetAngle(theta_2)

	def ApplyForce(self, force):
		theta_1 = self.joint_1_controller.GetAngle()
		theta_2 = self.joint_2_controller.GetAngle()

		J = self.kine_model.GetJacobian(theta_1, theta_2)
		
		theta_dot_dot, _ = self.dynamics.ForwardDynamics(force, J)

		theta_1_dot_dot = theta_dot_dot[0, 0]
		theta_2_dot_dot = theta_dot_dot[1, 0]


		theta_1_dot = self.joint_1.speed + theta_1_dot_dot*TIME_STEP
		theta_2_dot = self.joint_2.speed + theta_2_dot_dot*TIME_STEP

		theta_1 = self.joint_1.angle + theta_1_dot*TIME_STEP
		theta_2 = self.joint_2.angle + theta_2_dot*TIME_STEP

		self.joint_1_controller.SetAngle(theta_1)
		self.joint_2_controller.SetAngle(theta_2)


	def Render(self, screen, PPM):
		self.base.Render(screen, PPM)
		
		self.link_1.Render(screen, PPM)
		self.link_2.Render(screen, PPM)
