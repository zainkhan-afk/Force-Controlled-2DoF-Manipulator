class JointController:
	def __init__(self, joint):
		self.joint = joint
		self.current_angle = self.joint.angle 

		self.P = 10
		self.I = 0.01
		self.D = 1

		self.prev_error = 0
		self.error_sum = 0

	def SetAngle(self, desired_angle):
		self.current_angle = self.joint.angle
		error = desired_angle - self.current_angle

		val = self.P*error + self.D*(error - self.prev_error) + self.I*self.error_sum

		# print(self.joint.motorSpeed, self.joint.speed)

		self.joint.motorSpeed = val

		self.prev_error = error
		self.error_sum += error

	def SetVelocity(self, velocity):
		self.joint.motorSpeed = velocity

	def GetAngle(self):
		return self.joint.angle 