import numpy as np

class PID:
	def __init__(self):
		self.P = 0;
		self.I = 0;
		self.D = 0;

		self.error = np.arra([0, 0])

	def CalculateForce(self, robot_ee_pos, goal_pos):
		pass