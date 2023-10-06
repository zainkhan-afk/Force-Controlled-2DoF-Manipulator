import numpy as np
from utils import GetTransformationMatrix, GetInverseMatrix, GetRotationMatrix

class Kinematics:
	def __init__(self, l1, l2):
		self.l1 = l1
		self.l2 = l2

		self.R_world_robot = GetRotationMatrix(np.pi/2)

	def FK(self, theta_1, theta_2):
		theta_1 = theta_1 - np.pi/2

		x = self.l2*np.cos(theta_1 + theta_2) + self.l1*np.cos(theta_1)
		y = self.l2*np.sin(theta_1 + theta_2) + self.l1*np.sin(theta_1)


		position = np.array([[x, y]]).T
		# position = self.R_world_robot.T@position

		return position


	def IK(self, position):
		# position = self.R_world_robot@position
		x = position[0, 0]
		y = position[1, 0]

		temp = (x**2 + y**2 - self.l1**2 - self.l2**2)/(2*self.l1*self.l2)

		if temp>1:
			temp = 1
		if temp<-1:
			temp = -1

		theta_2 = np.arccos(temp)


		theta_1 = np.arctan2(y, x) - np.arctan2((self.l2*np.sin(theta_2)), (self.l1 + self.l2*np.cos(theta_2)))

		return np.pi/2 + theta_1, theta_2
		# return theta_1, theta_2

	def GetJacobian(self, theta_1, theta_2):
		theta_1 = theta_1 - np.pi/2

		J = np.array([
					[- self.l1*np.sin(theta_1) - self.l2*np.sin(theta_1 + theta_2), -self.l2*np.sin(theta_1 + theta_2)],
					[  self.l1*np.cos(theta_1) + self.l2*np.cos(theta_1 + theta_2),  self.l2*np.cos(theta_1 + theta_2)]
					]) # J_wrt_0


		# J = np.array([
		# 			[ self.l1*np.sin(theta_1), 0],
		# 			[ self.l1*np.cos(theta_1) + self.l2,  self.l2]
		# 			]) # J_wrt_3 

		return J