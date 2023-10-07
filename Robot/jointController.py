import numpy as np

class JointController:
	def __init__(self, joint, name = ""):
		self.joint = joint
		self.name = name
		self.current_angle = self.joint.angle 

		self.P = 10
		self.I = 0
		self.D = 2

		self.prev_error = 0
		self.error_sum = 0

	def SetAngle(self, desired_angle):
		if self.name == "j1":
			desired_angle = desired_angle + np.pi/2
		self.current_angle = self.joint.angle
		error = desired_angle - self.current_angle

		val = self.P*error + self.D*(error - self.prev_error) + self.I*self.error_sum
		# print(self.joint.motorSpeed, self.joint.speed)


		self.joint.motorSpeed = val

		self.prev_error = error
		self.error_sum += error

	def GetVelocity(self):
		return self.joint.motorSpeed

	def SetVelocity(self, velocity):
		self.joint.motorSpeed = velocity

	def GetAngle(self):
		if self.name == "j1":
			return self.joint.angle - np.pi/2
		return self.joint.angle 
