import numpy as np
from state import State
from .kinematics import Kinematics
from .Dynamics import ArmDynamics

from utils import *

class Dummy:
	def __init__(self, l1, l2, m1, m2):
		self.l1 = l1
		self.l2 = l2
		self.m1 = m2
		self.m2 = m2


		self.prev_error = np.array([0, 0])
		self.error_sum = np.array([0, 0])

		self.home_position = np.array([[0, -1.0]]).T

		self.kine_model = Kinematics(self.l1, self.l2)
		self.dynamics   = ArmDynamics(self.l1, self.l2, self.m1, self.m2)

		self.state = State(np.array([0, 0]), np.array([0, 0]), np.array([0, 0]))


	def SetJointAngle(self, theta):
		P = 0.3
		I = 0
		D = 0.02

		desired_vel = (theta - self.state.theta) / TIME_STEP

		error = desired_vel - self.state.theta_dot

		# error = theta - self.state.theta

		val = P*error + D*(error - self.prev_error) + I*self.error_sum

		self.prev_error = error
		self.error_sum = self.error_sum + error

		self.state = self.state.UpdateUsingPosition(self.state.theta + val*TIME_STEP)


	def HomePosition(self):
		self.GoTo(self.home_position)
		current_pos = self.GetEEPos()

		return AlmostEqual(self.home_position, current_pos)


	def GoTo(self, position):
		desired_theta_1, desired_theta_2 = self.kine_model.IK(position)
		
		self.SetJointAngle(np.array([desired_theta_1, desired_theta_2]))
		print(self.state)

	def GetEEPos(self):
		theta_1 = self.state.theta[0]
		theta_2 = self.state.theta[1]

		# print("Angle", round(theta_1*180/np.pi, 2), round(theta_2*180/np.pi, 2))

		position = self.kine_model.FK(theta_1, theta_2)
		return position

	def ApplyForce(self, force):
		current_theta_1 = self.state.theta[0]
		current_theta_2 = self.state.theta[1]

		# self.state.SetPosition(np.array([theta_1, theta_2]))
		# self.state = self.state.UpdateUsingPosition(np.array([current_theta_1, current_theta_2]))

		J = self.kine_model.GetJacobian(current_theta_1, current_theta_2)
		
		theta_double_dot, _ = self.dynamics.ForwardDynamics(force, J, self.state)

		# self.state = self.state.UpdateUsingAcceleration(np.array([theta_double_dot[0, 0], theta_double_dot[1, 0]]))

		# desired_theta_1 = self.state.theta[0]
		# desired_theta_2 = self.state.theta[1]

		theta_dot_1 = self.state.theta_dot[0] + theta_double_dot[0, 0]*TIME_STEP
		theta_dot_2 = self.state.theta_dot[1] + theta_double_dot[1, 0]*TIME_STEP

		desired_theta_1 = current_theta_1 + theta_dot_1*TIME_STEP
		desired_theta_2 = current_theta_2 + theta_dot_2*TIME_STEP

		self.SetJointAngle(np.array([desired_theta_1, desired_theta_2]))
