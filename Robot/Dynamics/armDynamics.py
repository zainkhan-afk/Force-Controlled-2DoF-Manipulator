import numpy as np
from utils import *

class ArmDynamics:
	def __init__(self, l1, l2, m1, m2):
		self.m1 = m1
		self.m2 = m2

		self.l1 = l1
		self.l2 = l2

	def GetMassMatrix(self, state):
		theta1 = state.theta[0]
		theta2 = state.theta[1]

		# theta1 = self.joint_1.GetAngle()
		# theta2 = self.joint_2.GetAngle()

		r11 = self.m2*self.l2**2 + 2*self.l1*self.l2*self.m2*np.cos(theta2) + (self.l1**2)*(self.m2 + self.m1)
		r12 = self.m2*self.l2**2 +   self.l1*self.l2*self.m2*np.cos(theta2)
		r21 = r12
		r22 = self.m2*self.l2**2
		M = np.array([
						[r11, r12],
						[r21, r22]
					 ])

		return M

	def GetCoriolisMatrix(self, state):
		theta1 = state.theta[0]
		theta2 = state.theta[1]

		theta1_dot = state.theta_dot[0]
		theta2_dot = state.theta_dot[1]

		# theta1 = self.joint_1.GetAngle()
		# theta2 = self.joint_2.GetAngle()

		# theta1_dot = self.joint_1.GetVelocity()
		# theta2_dot = self.joint_2.GetVelocity()


		a = -  self.l1*self.l2*self.m2*np.sin(theta2)*theta2_dot**2
		b = -2*self.l1*self.l2*self.m2*np.sin(theta2)*theta2_dot*theta1_dot
		r11 = a + b
		r21 =  self.l1*self.l2*self.m2*np.sin(theta2)*theta1_dot**2
		C = np.array([
						[r11],
						[r21]
					 ])

		return C

	def GetGravityMatrix(self, state):
		theta1 = state.theta[0]
		theta2 = state.theta[1]

		theta1_dot = state.theta_dot[0]
		theta2_dot = state.theta_dot[1]

		# theta1 = self.joint_1.GetAngle()
		# theta2 = self.joint_2.GetAngle()

		# theta1_dot = self.joint_1.GetVelocity()
		# theta2_dot = self.joint_2.GetVelocity()

		a = self.l2*self.m2*gravity*np.cos(theta1 + theta2)
		b = (self.m2 + self.m1)*self.l1*gravity*np.cos(theta1)
		r11 = a + b
		r12 = self.m2*self.l2*gravity*np.cos(theta1 + theta2)
		G = np.array([
						[r11],
						[r12]
					 ])

		return G


	def ForwardDynamics(self, force, jacobian, state):
		'''
		M(q)q_dot_dot + C(q, q_dot) + g(q) = J.F
		'''
	
		M = self.GetMassMatrix(state)
		C = self.GetCoriolisMatrix(state)
		G = self.GetGravityMatrix(state)

		torques = jacobian.T@force
		# torques = force.copy()

		M_inv = GetInverseMatrix(M)


		theta_double_dot = M_inv@(torques - C - G)

		# theta_double_dot = np.zeros((2, 1))

		return theta_double_dot, torques


	def InverseDynamics(self, jacobian, state):
		'''
		M(q)q_dot_dot + C(q, q_dot) + g(q) = J.F
		'''
	
		M = self.GetMassMatrix(state)
		C = self.GetCoriolisMatrix(state)
		G = self.GetGravityMatrix(state)

		jacobian_T_inv = GetInverseMatrix(jacobian.T)
		torques = M@state.theta_double_dot.reshape((2, 1)) + C + G
		force = jacobian_T_inv@torques

		print(torques)

		return force