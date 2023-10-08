from utils import *

class State:
	def __init__(self, theta, theta_dot, theta_double_dot):
		self.theta = theta
		self.theta_dot = theta_dot
		self.theta_double_dot = theta_double_dot

	def SetPosition(self, theta):
		self.theta = theta

	def UpdateUsingPosition(self, theta):
		new_theta_dot = (theta - self.theta) / TIME_STEP
		new_theta_double_dot = (new_theta_dot - self.theta_dot) / TIME_STEP
		return State(theta, new_theta_dot, new_theta_double_dot)

	def UpdateUsingVelocity(self, theta_dot):
		new_theta = self.theta + theta_dot*TIME_STEP
		new_theta_double_dot = (theta_dot - self.theta_dot) / TIME_STEP
		return State(new_theta, theta_dot, new_theta_double_dot)

	def UpdateUsingAcceleration(self, theta_double_dot):
		new_theta_dot = self.theta_dot + theta_double_dot*TIME_STEP
		new_theta = self.theta + new_theta_dot*TIME_STEP
		return State(new_theta, new_theta_dot, theta_double_dot) 


	def Update(self, theta, theta_dot, theta_double_dot):
		return State(theta, theta_dot, theta_double_dot)


	def __str__(self):
		return f'''
	==========================================================
	Robot State:
		Theta: {self.theta[0], self.theta[1]}
		Theta Dot: {self.theta_dot[0], self.theta_dot[1]}
		Theta Double Dot: {self.theta_double_dot[0], self.theta_double_dot[1]}
		'''