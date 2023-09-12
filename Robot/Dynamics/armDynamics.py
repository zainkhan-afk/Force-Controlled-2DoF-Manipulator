import numpy as np
from utils import *

class ArmDynamics:
	def __init__(self, link_1, link_2, joint_1, joint_2):
		self.link_1 = link_1
		self.link_2 = link_2
		self.joint_1 = joint_1
		self.joint_2 = joint_2


	def GetMassMatrix(self):
		r11 = self.link_2.body.mass*self.link_2.height**2 + 2*self.link_1.height*self.link_2.height*self.link_2.body.mass*np.cos(self.joint_2.angle) + self.link_1.width*(self.link_2.body.mass + self.link_1.body.mass)
		r12 = self.link_2.body.mass*self.link_2.height**2 +   self.link_1.height*self.link_2.height*self.link_2.body.mass*np.cos(self.joint_2.angle)
		r21 = self.link_2.body.mass*self.link_2.height**2 +   self.link_1.height*self.link_2.height*self.link_2.body.mass*np.cos(self.joint_2.angle)
		r22 = self.link_2.body.mass*self.link_2.height**2
		M = np.array([
						[r11, r12],
						[r21, r22]
					 ])

		return M

	def GetCoriolisMatrix(self):
		a = -self.link_1.height*self.link_2.height*self.link_2.body.mass*np.sin(self.joint_2.angle)*self.joint_2.motorSpeed**2
		b = -2*self.link_1.height*self.link_2.height*self.link_2.body.mass*np.sin(self.joint_2.angle)*self.joint_2.motorSpeed*self.joint_1.motorSpeed
		r11 = a + b
		r21 = self.link_1.height*self.link_2.height*self.link_2.body.mass*np.sin(self.joint_2.angle)*self.joint_1.motorSpeed**2
		C = np.array([
						[r11],
						[r21]
					 ])

		return C

	def GetGravityMatrix(self):
		a = self.link_2.height*self.link_2.body.mass*gravity*np.cos(self.joint_1.angle)*np.cos(self.joint_2.angle)
		b = (self.link_2.body.mass + self.link_1.body.mass)* self.link_1.height*gravity*np.cos(self.joint_1.angle)
		r11 = a + b
		r12 = self.link_2.body.mass*self.link_2.body.mass*gravity*np.cos(self.joint_1.angle)*np.cos(self.joint_2.angle)
		G = np.array([
						[r11],
						[r12]
					 ])

		return G


	def ForwardDynamics(self, force, jacobian):
		'''
		M(q)q_dot_dot + C(q, q_dot) + g(q) = J.F
		'''
	
		M = self.GetMassMatrix()
		C = self.GetCoriolisMatrix()
		G = self.GetGravityMatrix()

		torques = jacobian.T@force

		M_inv = GetInverseMatrix(M)

		theta_double_dot = M_inv@(torques - C - G)

		# print(theta_double_dot.shape)
		# print(selection_vector)

		return theta_double_dot, torques