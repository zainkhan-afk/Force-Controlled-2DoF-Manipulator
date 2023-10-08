import numpy as np
from Robot.Dynamics.simulateDynamics import SimulateDynamics

class PID:
	def __init__(self, dynamicsModel, P = 300, I = 0, D = 50):
		self.dynamicsSimulator = SimulateDynamics(dynamicsModel)
		self.P = P;
		self.I = I;
		self.D = D;

		# self.error = np.array([0, 0])
		self.prev_error = np.array([0, 0]).astype("float32")
		self.error_sum = np.array([0, 0]).astype("float32")

	def CalculateForce(self, ee_pos, goal_pos):
		error = goal_pos - ee_pos.ravel()

		force = self.P*error + self.D*(self.prev_error - error) + self.I*self.error_sum

		self.prev_error = error
		self.error_sum += error
		# print(self.prev_error, error)
		force = force.reshape(2, 1)
		return force